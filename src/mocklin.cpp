/*
 * Copyright (C) 2020 Matthias S. Benkmann
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software, to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <errno.h>
#include <memory>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "arg.h"

#include "arg.h"
#include "fifo.h"
#include "file.h"
#include "gcode.h"
#include "marlinbuf.h"

using gcode::Line;
using std::unique_ptr;

extern const char* WELCOME_TEXT;
extern const char* WELCOME_TEXT2;

enum optionIndex
{
    UNKNOWN,
    HELP,
    RESEND
};
const option::Descriptor usage[] =

    {{UNKNOWN, 0, "", "", Arg::Unknown,
      "USAGE: mocklin [options] printdev\n\n"
      "printdev must be a path that does not exist or refers to a socket. "
      "It will be replaced by a new socket on which mocklin will listen and "
      "pretend to be a Marlin-based 3D printer."
      "\n\n"
      "Options:"},
     {HELP, 0, "", "help", Arg::None, "  \t--help  \tPrint usage and exit."},
     {RESEND, 0, "", "resend", Arg::NumberPair,
      "  \t--resend[=<when>,<what>]"
      "  \tEvery other time mocklin receives a command with line number <when>, "
      "mocklin will request a resend of line number <what>."},
     {UNKNOWN, 0, "", "", Arg::None, "\n"},
     {0, 0, 0, 0, 0, 0}};

void handle_connection(int fd);

long resend_when = LONG_MIN;
long resend_what = LONG_MIN;
bool resend_toggle = true;

int main(int argc, char* argv[])
{
    signal(SIGPIPE, SIG_IGN);

    argc -= (argc > 0);
    argv += (argc > 0); // skip program name argv[0] if present
    option::Stats stats(usage, argc, argv);

    // GCC supports C99 VLAs for C++ with proper constructor calls.
    option::Option options[stats.options_max], buffer[stats.buffer_max];

    option::Parser parse(usage, argc, argv, options, buffer);

    if (parse.error())
        return 1;

    if (options[HELP] || argc == 0 || parse.nonOptionsCount() != 1)
    {
        int columns = getenv("COLUMNS") ? atoi(getenv("COLUMNS")) : 80;
        option::printUsage(fwrite, stdout, usage, columns);
        return 0;
    }

    for (int i = 0; i < parse.optionsCount(); ++i)
    {
        option::Option& opt = buffer[i];
        switch (opt.index())
        {
            case HELP:
            // not possible, because handled further above and exits the program
            case RESEND:
                resend_when = strtol(opt.arg, 0, 10);
                resend_what = strtol(strchr(opt.arg, ',') + 1, 0, 10);
            case UNKNOWN:
                // not possible because Arg::Unknown returns ARG_ILLEGAL
                // which aborts the parse with an error
                break;
        }
    }

    const char* sockname = parse.nonOption(0);
    File sock(sockname);
    struct stat statbuf;
    if (sock.stat(&statbuf) && !S_ISSOCK(statbuf.st_mode))
    {
        fprintf(stderr, "%s exists but is not a socket.\n", sockname);
        exit(1);
    }

    sock.unlink();
    sock.clearError();
    sock.action("listening on socket");
    sock.listen();

    if (sock.hasError())
    {
        fprintf(stderr, "%s\n", sock.error());
        exit(1);
    }

    sock.action("accepting connections");
    for (;;)
    {
        int peer_fd = sock.accept();
        if (sock.hasError())
        {
            fprintf(stderr, "%s\n", sock.error());
            exit(1);
        }
        handle_connection(peer_fd);
    }
}

const char* MSG_ERRORMAGIC = "Error:";
const char* MSG_ECHOMAGIC = "echo:";
const char* MSG_OK = "ok";
const char* MSG_RESEND = "Resend: ";
const char* MSG_ERR_LINE_NO = "Line Number is not Last Line Number+1, Last Line: ";
const char* MSG_ERR_CHECKSUM_MISMATCH = "checksum mismatch, Last Line: ";
const char* MSG_ERR_NO_CHECKSUM = "No Checksum with line number, Last Line: ";
const char* MSG_UNKNOWN_COMMAND = "Unknown command: \"";

struct Command
{
    unique_ptr<Line> gcode;
    bool send_ok;
    Command(unique_ptr<Line>& gcode_, bool send_ok_) : gcode(gcode_.release()), send_ok(send_ok_) {}
};

long gcode_N = 0;
long gcode_LastN = 0;

const int BUFSIZE = 4; // maximum number of entries in cmd_fifo
FIFO<Command> cmd_fifo;

void enqueue_command(unique_ptr<Line>& gcode, bool send_ok) { cmd_fifo.put(new Command(gcode, send_ok)); }

void ok_to_send(File& peer)
{
    char buf[1024];
    int len = snprintf(buf, sizeof(buf), "%s\n", MSG_OK);
    if (len >= (int)sizeof(buf))
        len = sizeof(buf) - 1; // -1 because of 0 terminator
    peer.writeAll(buf, len);
    fprintf(stdout, "%s", buf);
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void flush_and_request_resend(File& peer)
{
    // Marlin uses ok_to_send(), here instead of always sending an ok.
    // But that seems like a bug.
    // https://github.com/MarlinFirmware/Marlin/issues/18955
    char buf[1024];
    peer.tail(buf, sizeof(buf), 0, 0);
    int len = snprintf(buf, sizeof(buf), "%s%ld\nok\n", MSG_RESEND, gcode_LastN + 1);
    if (len >= (int)sizeof(buf))
        len = sizeof(buf) - 1; // -1 because of 0 terminator
    peer.writeAll(buf, len);
    fprintf(stdout, "%s", buf);
}

void gcode_line_error(File& peer, const char* err, bool doFlush = true)
{
    char sendbuf[1024];
    int len = snprintf(sendbuf, sizeof(sendbuf), "%s%s%ld\n", MSG_ERRORMAGIC, err, gcode_LastN);
    if (len >= (int)sizeof(sendbuf))
        len = sizeof(sendbuf) - 1; // -1 because of 0 terminator
    peer.writeAll(sendbuf, len);
    fprintf(stdout, "%s", sendbuf);
    if (doFlush)
        flush_and_request_resend(peer);
}

void resend_request(File& peer, long resend_when, long resend_what)
{
    char sendbuf[1024];
    int len =
        snprintf(sendbuf, sizeof(sendbuf), "%sResend request triggered by line: %ld\n", MSG_ERRORMAGIC, resend_when);
    if (len >= (int)sizeof(sendbuf))
        len = sizeof(sendbuf) - 1; // -1 because of 0 terminator
    peer.writeAll(sendbuf, len);
    fprintf(stdout, "%s", sendbuf);
    gcode_LastN = resend_what - 1;
    flush_and_request_resend(peer);
}

void unknown_command_error(File& peer, const char* gcode)
{
    char sendbuf[1024];
    int len = snprintf(sendbuf, sizeof(sendbuf), "%s%s%s\"\n", MSG_ECHOMAGIC, MSG_UNKNOWN_COMMAND, gcode);
    if (len >= (int)sizeof(sendbuf))
        len = sizeof(sendbuf) - 1; // -1 because of 0 terminator
    peer.writeAll(sendbuf, len);
    fprintf(stdout, "%s", sendbuf);
}

void process_next_command(File& peer)
{
    if (cmd_fifo.empty())
        return;

    const int G = 0;
    const int M = 0x10000;
    const int T = 0x20000;

    unique_ptr<Command> cmd(cmd_fifo.get());

    const char* gcode = cmd->gcode->data();
    int command = -1;
    if (cmd->gcode->length() >= 2)
    {
        switch (gcode[0])
        {
            case 'G':
                command = G;
                break;
            case 'M':
                command = M;
                break;
            case 'T':
                command = T;
                break;
        }

        if (command >= 0 && gcode[1] >= '0' && gcode[1] <= '9')
            command += strtol(gcode + 1, 0, 10);
    }

    switch (command)
    {
        case G + 0: // Linear Move
        case G + 1: // Linear Move
            break;
        case G + 28: // Auto Home
            break;
        case G + 90: // Absolute Positioning
            break;
        case G + 91: // Relative Positioning
            break;
        case G + 92: // Set Position
            break;
        case M + 82: // E Absolute
            break;
        case M + 18: // Disable Steppers
        case M + 84: // Disable Steppers
            break;
        case M + 104: // Set Hotend Temperature
            break;
        case M + 105: // Report Temperatures
            break;
        case M + 106: // Set Fan Speed
            break;
        case M + 109: // Wait for Hotend Temperature
            break;
        case M + 110: // Set Line Number
            break;    // already handled
        case M + 115: // Firmware Info
            break;
        case M + 140: // Set Bed Temperature
            break;
        case M + 190: // Wait for Bed Temperature
            break;
        case M + 201: // Set Print Max Acceleration
            break;
        case M + 203: // Set Max Feedrate
            break;
        case M + 204: // Set Starting Acceleration
            break;
        case M + 205: // Set Advanced Settings
            break;
        case M + 209: // Set Auto Retract
            break;
        case M + 220: // Set Feedrate Percentage
            break;
        case M + 221: // Set Flow Percentage
            break;
        default:
            unknown_command_error(peer, gcode);
    }

    if (cmd->send_ok)
        ok_to_send(peer);
}

void handle_connection(int fd)
{

    fprintf(stdout, "New connection\n");
    File peer("remote connection", fd);
    peer.autoClose();
    peer.setNonBlock(true);
    gcode::Reader reader(peer);

    sleep(1); // Wait a little because that's what a normal printer does
    peer.writeAll(WELCOME_TEXT, strlen(WELCOME_TEXT));
    sleep(1); // wait before reporting SD card state
    peer.writeAll(WELCOME_TEXT2, strlen(WELCOME_TEXT2));

    for (;;)
    {
        // compare Marlin function get_serial_commands()
        while (cmd_fifo.size() < BUFSIZE && reader.hasNext())
        {
            unique_ptr<Line> line(reader.next());

            const char* command = line->data();
            fprintf(stdout, "%s\n", command);

            const char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line

            if (npos)
            {
                const char* cmdpos = strstr(command, "M110");
                bool M110 = cmdpos != NULL;

                if (M110)
                {
                    const char* n2pos = strchr(command + 4, 'N');
                    if (n2pos)
                        npos = n2pos;
                }

                char* endptr;
                gcode_N = strtol(npos + 1, &endptr, 10);
                if (cmdpos == 0)
                    cmdpos = endptr;

                if (gcode_N != gcode_LastN + 1 && !M110)
                {
                    gcode_line_error(peer, MSG_ERR_LINE_NO);
                    continue;
                }

                if (gcode_N == resend_when)
                {
                    resend_toggle = !resend_toggle;
                    if (!resend_toggle)
                    {
                        resend_request(peer, resend_when, resend_what);
                        continue;
                    }
                }

                const char* apos = strrchr(command, '*');
                if (apos)
                {
                    uint8_t checksum = 0;
                    uint8_t count = uint8_t(apos - command);
                    while (count)
                        checksum ^= command[--count];
                    if (strtol(apos + 1, NULL, 10) != checksum)
                    {
                        gcode_line_error(peer, MSG_ERR_CHECKSUM_MISMATCH);
                        continue;
                    }
                }
                else
                {
                    gcode_line_error(peer, MSG_ERR_NO_CHECKSUM);
                    continue;
                }

                gcode_LastN = gcode_N;

                line->slice(cmdpos - command, apos - command);
            }

            enqueue_command(line, true);
        }

        process_next_command(peer);

        if (cmd_fifo.empty())
        {
            if (peer.EndOfFile() || peer.hasError())
                break;
            usleep(1000); // sleep 1ms to save some clock cycles
        }
    }

    if (peer.hasError()) // report if we ended due to an error and not EOF
        fprintf(stderr, "%s\n", peer.error());
    else
        fprintf(stdout, "Connection closed\n");
}

const char* WELCOME_TEXT =
    "start\n"
    "echo: External Reset\n"
    "Marlin \n"
    "echo: Last Updated: 2015-12-01 12:00 | Author: (none, default config)\n"
    "Compiled: Sep  4 2017\n"
    "echo: Free Memory: 1454  PlannerBufferBytes: 1232\n"
    "echo:Hardcoded Default Settings Loaded\n"
    "echo:Steps per unit:\n"
    "echo:  M92 X80.00 Y80.00 Z400.00 E93.00\n"
    "echo:Maximum feedrates (mm/s):\n"
    "echo:  M203 X300.00 Y300.00 Z5.00 E25.00\n"
    "echo:Maximum Acceleration (mm/s2):\n"
    "echo:  M201 X1000 Y1000 Z100 E5000\n"
    "echo:Accelerations: P=printing, R=retract and T=travel\n"
    "echo:  M204 P500.00 R500.00 T1000.00\n"
    "echo:Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), "
    "X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)\n"
    "echo:  M205 S0.00 T0.00 B20000 X20.00 Z0.40 E5.00\n"
    "echo:Home offset (mm):\n"
    "echo:  M206 X0.00 Y0.00 Z0.00\n"
    "echo:Material heatup parameters:\n"
    "echo:  M145 M0 H185 B45 F0\n"
    "echo:  M145 M1 H240 B110 F0\n"
    "echo:PID settings:\n"
    "echo:  M301 P22.20 I1.08 D114.00 C100.00 L20\n"
    "echo:Filament settings: Disabled\n"
    "echo:  M200 D3.00\n"
    "echo:  M200 D0\n";

const char* WELCOME_TEXT2 = "echo:SD card ok\n"
                            "Init power off infomation.\n"
                            "size: \n"
                            "591\n";