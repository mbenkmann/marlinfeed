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
#include <sys/types.h>
#include <unistd.h>

#include "arg.h"

#include "arg.h"
#include "dirscanner.h"
#include "fifo.h"
#include "file.h"
#include "gcode.h"
#include "marlinbuf.h"
#include "millis.h"

using gcode::Line;
using std::unique_ptr;

enum optionIndex
{
    UNKNOWN,
    HELP,
    IOERROR,
    VERBOSE
};
const option::Descriptor usage[] = {
    {UNKNOWN, 0, "", "", Arg::Unknown,
     "USAGE: marlinfeed [options] [<infile> ...] <printdev>\n\n"
     "Reads all <infile> in order and sends the contained GCODE to device <printdev> "
     "which must be compatible with Marlin's serial port protocol.\n"
     "<printdev> can be either a TTY or a Unix Domain Socket.\n"
     "Pass '-' or '/dev/stdin' as <infile> to read from stdin.\n"
     "Pass an <infile> argument of the following form\n"
     "    [http://|https://][localhost][:<port>/]<uploadpath>\n"
     "to watch directory <uploadpath> for new/modified gcode files and optionally listen for "
     "Octoprint API compatible connections on <port> that will upload to <uploadpath>.\n"
     "If 'localhost' is specified, only connections from the same machine will be accepted.\n"
     "If 'https://' is specified, the connections have to be secured via TLS.\n"
     "If no <infile> is passed, '-' is assumed.\n"
     "Communication is echoed to stdout."
     "\n\n"
     "Options:"},
    {HELP, 0, "", "help", Arg::None, "  \t--help  \tPrint usage and exit."},
    {VERBOSE, 0, "v", "verbose", Arg::None, "  -v, \t--verbose  \tIncrease verbosity. Can be used multiple times."},
    {IOERROR, 0, "e", "ioerror", Arg::IOError,
     "  -e<arg>, \t--ioerror[=<arg>]"
     "  \tHow to handle an error on <infile> or <printdev>.\v'next' reinitializes communication with"
     " the printer and then tries to print the next <infile> in order.\v"
     "'quit' terminates the program.\vThe default is 'quit' if not listening on a port and 'next' if listening."},
    {UNKNOWN, 0, "", "", Arg::None,
     "\nExamples:\n"
     "  marlinfeed gcode/init.gcode gcode/benchy.gcode /dev/ttyUSB0 \n"
     "  marlinfeed --ioerror=next 1stprint.gcode 2ndprint.gcode /dev/ttyUSB0 \n"
     "  marlinfeed ./upload /dev/ttyUSB0 \n"
     "  marlinfeed --ioerror=quit :8080/upload /dev/ttyUSB0 \n"
     "  marlinfeed https://:443//var/cache/marlinfeed /dev/ttyUSB0 \n"
     "\n"},
    {0, 0, 0, 0, 0, 0}};

const char* NEW_SOCKET_CONNECTION = "New socket connection => Forking child\n";

// Maximum number of milliseconds we don't get a non-error reply from the printer.
// This aborts the current job if the printer keeps replying to everything we send
// with an error.
const int MAX_TIME_WITH_ERROR = 5000;

// Maximum number of milliseconds with no message from the printer while
// at least 1 command is not ack'd. Needs to be longer than the longest
// blocking command that is silent (e.g. G28).
const int MAX_TIME_SILENCE = 120000;

bool ioerror_next;
int verbosity = 0;
const char* upload_dir = "/dev/null/";

bool handle(File& out, File& serial, const char* infile, File* sock, const char** e, int* iop);
void handle_socket_connection(int fd);

// FIFO::filter() for removing file names with no known GCODE extension
struct GCodeExtension
{
    bool operator()(char* s)
    {
        char* ext = strrchr(s, '.');
        if (ext != 0)
        {
            if (strcmp(".gcode", ext) == 0)
                return true;
        }
        free(s);
        return false;
    }
} gcode_extension;

const char* boolStr(bool b)
{
    if (b)
        return "true";
    else
        return "false";
}

class PrinterState
{
    float tool[2][2];
    float bed[2];
    int64_t startTime;
    int64_t endTime;
    const char* printName;
    int64_t printSize;
    int64_t printedBytes;

  public:
    void clearJob()
    {
        startTime = 0;
        endTime = 0;
        free((void*)printName);
        printName = strdup("Unnamed");
        printSize = 0;
        printedBytes = 0;
    }

    enum Enum
    {
        Disconnected = 0, // Marlinfeed not currently sync'ed with printer
        Printing = 1,     // Commands are flowing from an infile to printer
        Idle = 2,         // Marlinfeed sync'ed with printer but no active infile
        Stalled = 3       // Commands are waiting because printer buffer has been full for a while
    } status;

    void operator=(Enum s)
    {
        if (s != Printing && s != Stalled)
            clearJob();
        if (s == Printing && status != Printing && status != Stalled)
            startTime = millis();
        status = s;
    }

    void setPrintName(const char* name)
    {
        free((void*)printName);
        printName = strdup(name);
    };
    void setPrintSize(int64_t bytes) { printSize = bytes; }
    void setPrintedBytes(int64_t bytes) { printedBytes = bytes; }
    void setEstimatedPrintTime(int seconds)
    {
        if (seconds > 0)
            endTime = startTime + seconds * 1000;
    }

    //"{\"job\":{\"file\":{\"name\":\"\"}},\"progress\":{\"printTime\":null,\"completion\":null}}"
    const char* jobJSON()
    {
        char* j;
        const char* text = "Operational";
        if (status == Printing || status == Stalled)
            text = "Printing";
        double deltat = 0;
        if (startTime > 0)
            deltat = (millis() - startTime);
        double completion = 0;
        if (startTime > 0 && endTime > startTime)
            completion = deltat / (endTime - startTime);
        else if (printSize > 0)
            completion = (double)printedBytes / (double)printSize;
        deltat /= 1000; // convert to seconds

        int len = asprintf(&j,
                           "{\r\n"
                           "  \"state\": \"%s\",\r\n"
                           "  \"job\": {\r\n"
                           "    \"file\": {\r\n"
                           "      \"name\": \"%s\"\r\n"
                           "    }\r\n"
                           "  },\r\n"
                           "  \"progress\": {\r\n"
                           "      \"printTime\": %f,\r\n"
                           "      \"completion\": %f\r\n"
                           "  }\r\n"
                           "}\r\n",
                           text, printName, deltat, completion);
        if (len <= 0)
            return "{}";
        return j;
    }

    const char* toJSON()
    {
        char* j;
        const char* text = "Operational";
        if (status == Printing)
            text = "Printing";
        if (status == Stalled)
            text = "Stalled";
        const char* operational = boolStr(true);
        const char* paused = boolStr(false);
        const char* printing = boolStr(status == Printing || status == Stalled);
        const char* cancelling = boolStr(false);
        const char* pausing = boolStr(false);
        const char* sdReady = boolStr(false);
        const char* error = boolStr(false);
        const char* ready = boolStr(true);
        const char* closedOrError = boolStr(false);
        int len = asprintf(&j,
                           "{\r\n"
                           "  \"sd\": {\r\n"
                           "    \"ready\": %s\r\n"
                           "  },\r\n"
                           "  \"state\": {\r\n"
                           "    \"text\": \"%s\",\r\n"
                           "    \"flags\": {\r\n"
                           "      \"operational\": %s,\r\n"
                           "      \"paused\": %s,\r\n"
                           "      \"printing\": %s,\r\n"
                           "      \"cancelling\": %s,\r\n"
                           "      \"pausing\": %s,\r\n"
                           "      \"sdReady\": %s,\r\n"
                           "      \"error\": %s,\r\n"
                           "      \"ready\": %s,\r\n"
                           "      \"closedOrError\": %s\r\n"
                           "    }\r\n"
                           "  },\r\n"
                           "  \"temperature\": {\r\n"
                           "    \"tool0\": {\r\n"
                           "      \"actual\": %f,\r\n"
                           "      \"target\": %f,\r\n"
                           "      \"offset\": 0\r\n"
                           "    },\r\n"
                           "    \"tool1\": {\r\n"
                           "      \"actual\": %f,\r\n"
                           "      \"target\": %f,\r\n"
                           "      \"offset\": 0\r\n"
                           "    },\r\n"
                           "    \"bed\": {\r\n"
                           "      \"actual\": %f,\r\n"
                           "      \"target\": %f,\r\n"
                           "      \"offset\": 0\r\n"
                           "    }\r\n"
                           "  }\r\n"
                           "}\r\n",
                           sdReady, text, operational, paused, printing, cancelling, pausing, sdReady, error, ready,
                           closedOrError, tool[0][0], tool[0][1], tool[1][0], tool[1][1], bed[0], bed[1]);
        if (len <= 0)
            return "{}";
        return j;
    }

    PrinterState()
    {
        printName = 0;
        clearJob();
        tool[0][0] = 0;
        tool[0][1] = 0;
        tool[1][0] = 0;
        tool[1][1] = 0;
        bed[0] = 0;
        bed[1] = 0;
    }
} printerState;

File out("stdout", 1);

int main(int argc, char* argv[])
{
    signal(SIGCHLD, SIG_IGN); // automatic zombie removal
    signal(SIGPIPE, SIG_IGN); // handle as EPIPE on write()

    argc -= (argc > 0);
    argv += (argc > 0); // skip program name argv[0] if present
    option::Stats stats(usage, argc, argv);

    // GCC supports C99 VLAs for C++ with proper constructor calls.
    option::Option options[stats.options_max], buffer[stats.buffer_max];

    option::Parser parse(usage, argc, argv, options, buffer);

    if (parse.error())
        return 1;

    if (options[HELP] || argc == 0)
    {
        int columns = getenv("COLUMNS") ? atoi(getenv("COLUMNS")) : 80;
        option::printUsage(fwrite, stdout, usage, columns);
        return 0;
    }

    if (parse.nonOptionsCount() == 0)
    {
        fprintf(stderr, "%s\n", "You must provide a path to your printer device!");
        exit(1);
    }

    verbosity = options[VERBOSE].count();

    out.setNonBlock(true);
    // We don't exit for errors on stdout. It's just used for echoing.

    File serial(parse.nonOption(parse.nonOptionsCount() - 1));

    File* sock = 0;

    FIFO<char> infile_queue;
    DirScanner dirScanner;
    // Set last scan time to now so that we only detect new files and not
    // whatever is currently in the upload directories.
    dirScanner.refill(infile_queue);

    // If we don't have any infile arguments, assume "-" (i.e. stdin)
    if (parse.nonOptionsCount() == 1)
        infile_queue.put(strdup("-"));

    for (int i = 0; i < parse.nonOptionsCount() - 1; ++i)
    {
        const char* inf = parse.nonOption(i);
        if (inf[0] == '-' && inf[1] == 0)
        {
            infile_queue.put(strdup(inf));
            continue;
        }
        bool http = strncmp(inf, "http://", 7) == 0;
        bool https = strncmp(inf, "https://", 8) == 0;
        const char* host = "";
        if (http)
            inf += 7;
        if (https)
            inf += 8;
        bool localhost = strncmp(inf, "localhost", 9) == 0;
        if (localhost)
        {
            inf += 9;
            host = "localhost";
        }
        int port = 0;
        if (http)
            port = 80;
        if (https)
            port = 443;
        if (inf[0] == ':')
        {
            char* endptr;
            port = strtol(inf + 1, &endptr, 10);
            if (port < 10)
            {
                fprintf(stderr, "%s %d\n", "Illegal port specified for listening:", port);
                exit(1);
            }
            inf = endptr;
            if (inf[0] == '/')
                inf++;
            else
                inf = ""; // Cause a "Don't understand..." error to trigger further down
        }

        struct stat statbuf;

        if (0 > stat(inf, &statbuf))
        {
            fprintf(stderr, "Don't understand this argument: %s\n", parse.nonOption(i));
            exit(1);
        }

        if (port > 0 && !S_ISDIR(statbuf.st_mode))
        {
            fprintf(stderr, "Not a directory: %s\n", inf);
            exit(1);
        }

        if (S_ISDIR(statbuf.st_mode))
            dirScanner.addDir(inf);
        else
            infile_queue.put(strdup(inf));

        if (port > 0)
        {
            if (sock != 0)
            {
                fprintf(stderr, "%s\n", "Listening on multiple ports is not supported at this time!");
                exit(1);
            }

            upload_dir = strdup(inf);

            char* listen_address;
            assert(0 <= asprintf(&listen_address, "%s:%d", host, port));
            sock = new File(listen_address);
            sock->action("listening on");
            sock->listen();
            sock->setNonBlock(true);
            if (sock->hasError())
            {
                fprintf(stderr, "%s\n", sock->error());
                exit(1);
            }
            sock->action("accepting connections on");
        }
    }

    ioerror_next = false; // default
    if (sock != 0)
        ioerror_next = true; // default if we are listening
    if (options[IOERROR] && options[IOERROR].arg[0] == 'q')
        ioerror_next = false; // override default if --ioerror=quit on command line

    printerState = PrinterState::Disconnected;

    for (;;)
    {
        // If we're done with all infiles and there is no chance of any additional
        // infiles coming in, then exit.
        if (infile_queue.empty() && (sock == 0 || sock->hasError()) && dirScanner.empty())
            break;

        if (infile_queue.empty())
        {
            if (sock)
            {
                sock->poll(POLLIN, 1000);
                // Accept as socket connection if any is pending, then fork
                // and handle it in a child process.
                int connfd = sock->accept();
                if (connfd >= 0)
                {
                    if (verbosity > 0)
                        out.writeAll(NEW_SOCKET_CONNECTION, strlen(NEW_SOCKET_CONNECTION));
                    pid_t childpid = fork();
                    if (childpid < 0)
                        perror("fork");
                    if (childpid == 0)
                    {
                        sock->close();
                        handle_socket_connection(connfd);
                        _exit(0);
                    }
                    close(connfd);
                }
                else
                {
                    if (sock->errNo() == EWOULDBLOCK)
                        sock->clearError();
                }
            }
            else
                sleep(1); // to make sure we don't burn cycles waiting for files

            dirScanner.refill(infile_queue);
            infile_queue.filter(gcode_extension);
            if (infile_queue.empty())
                continue;
        }

        const char* error = 0;   // error message returned by handle()
        int in_out_printer = -1; // 0: error occurred on infile,
                                 // 1: error occured on stdout,
                                 // 2: error occurred on printer device, do not reconnect
                                 // 3: error occurred on printer device, try reconnecting

        char* infile = infile_queue.get();

        if (!handle(out, serial, infile, sock, &error, &in_out_printer))
        {
            fprintf(stderr, "%s\n", error);
            if (!ioerror_next)
                exit(1);
            if (in_out_printer == 2) // hard error on printer (e.g. USB unplugged)
                sleep(5);            // wait for it to go away (e.g. USB cable to be replugged)
            if (in_out_printer == 2 || in_out_printer == 3)
            {
                serial.close();
                printerState = PrinterState::Disconnected;
            }
        }
        else
            printerState = PrinterState::Idle;

        free(infile);
    }
}

bool handle_error(const char** e, const char* err_msg, int* iop, int which)
{
    *e = err_msg;
    *iop = which;
    return false;
}

bool handle(File& out, File& serial, const char* infile, File* sock, const char** e, int* iop)
{
    out.writeAll("\n>>> ", 5);
    out.writeAll(infile, strlen(infile));
    out.writeAll("\n", 1);

    // (Re-)connect to printer if necessary.
    bool hard_reconnect = (serial.isClosed() || serial.EndOfFile() || serial.hasError());

    if (hard_reconnect)
    {
    do_hard_reconnect:
        hard_reconnect = true;
        serial.close();
        serial.clearError();
        serial.action("opening printer device");
        struct stat statbuf;
        if (!serial.stat(&statbuf))
            return handle_error(e, serial.error(), iop, 2);

        if (S_ISSOCK(statbuf.st_mode))
        { // connect to socket
            serial.connect();
        }
        else
        { // not a socket? Treat it as a TTY.
            serial.open();
            serial.setupTTY();
            if (serial.hasError())
                return handle_error(e, serial.error(), iop, 2);
        }
    }

    serial.action("connecting to printer");
    serial.setNonBlock(true);
    const int MAX_ATTEMPTS = 4;
    int attempt = 0;

    // On hard reconnect, start by waiting up to 3s for something to appear on the line
    // because Marlin spams some stuff over the line when a new connection is established.
    if (hard_reconnect)
        serial.poll(POLLIN, 3000);

    for (; attempt < MAX_ATTEMPTS; attempt++)
    {
        char buffy[2048];
        int idx = serial.tail(buffy, sizeof(buffy) - 1 /* -1 for appending \n if nec. */, 500);
        if (idx < 0)
        {
            if (hard_reconnect)
                return handle_error(e, serial.error(), iop, 2);
            else
                goto do_hard_reconnect;
        }

        int n = idx;
        if (idx > 0 && buffy[idx - 1] == '\n')
            idx--; // make sure buffy[idx-1] is not the \n terminating the buffer
        else
            buffy[n++] = '\n'; // make sure buffy ends in a \n

        // Put idx at start of last line
        while (idx > 0 && buffy[idx - 1] != '\n')
            --idx;

        // buffy[idx] is the first character of the last line
        // buffy[n-1] is \n terminating the last line
        // buffy[n] is out of bounds

        if (verbosity > 0)
            out.writeAll(buffy, n);

        // When we get here for attempt 0, we haven't yet sent WRAP_AROUND_STRING, so any ok
        // we may see is unrelated. Therefore we don't break for attempt == 0.
        if (attempt > 0 && (buffy[idx] == 'o' && buffy[idx + 1] == 'k' && buffy[idx + 2] <= ' '))
            break;

        if (verbosity > 0)
            out.writeAll(MarlinBuf::WRAP_AROUND_STRING, MarlinBuf::WRAP_AROUND_STRING_LENGTH);

        if (!serial.writeAll(MarlinBuf::WRAP_AROUND_STRING, MarlinBuf::WRAP_AROUND_STRING_LENGTH))
        {
            if (hard_reconnect)
                return handle_error(e, serial.error(), iop, 2);
            else
                goto do_hard_reconnect;
        }

        // give the printer some time to reset itself
        if (hard_reconnect)
            usleep(1500000); // give the printer some time to reset itself
        else
            usleep(100000);
    }

    if (out.hasError())
    {
        if (out.errNo() == EWOULDBLOCK)
            out.clearError(); // we don't care if we could not echo all
        else
        {
            // Guess we won't be doing any echoing to stdout, anymore. Too bad.
        }
    }

    serial.action("");

    if (attempt == MAX_ATTEMPTS)
    {
        if (hard_reconnect)
            return handle_error(e, "Failed to establish connection with printer", iop, 2);
        else
            goto do_hard_reconnect;
    }

    printerState = PrinterState::Idle;

    gcode::Reader gcode_serial(serial);
    gcode_serial.whitespaceCompression(1);

    unique_ptr<File> in;
    if (infile[0] == '-' && infile[1] == 0)
    {
        in.reset(new File("stdin", 0));
    }
    else
    {
        printerState.setPrintName(infile);
        in.reset(new File(infile));
        in->open(O_RDONLY);
    }

    in->setNonBlock(true);
    struct stat statbuf;
    if (in->stat(&statbuf))
        printerState.setPrintSize(statbuf.st_size);
    if (in->hasError())
        return handle_error(e, in->error(), iop, 0);

    in->action("reading source gcode");
    gcode::Reader gcode_in(*in);
    gcode_in.whitespaceCompression(1); // CR-10's stock version of Marlin requires a space between command and params
    gcode::Line* next_gcode = 0;

    // stdoutbuf is a FIFO buffer that stores output lines for pushing to stdout
    // the buffer has no size limit and just grows if stdout blocks
    FIFO<gcode::Line> stdoutbuf;

    MarlinBuf marlinbuf;
    int idx;

    int nfds = 3;
    pollfd fds[4];
    fds[0].fd = serial.fileDescriptor();
    fds[0].events = POLLIN | POLLOUT;
    fds[1].fd = out.fileDescriptor();
    fds[1].events = POLLIN | POLLOUT;
    fds[2].fd = in->fileDescriptor();
    fds[2].events = POLLIN | POLLOUT;
    if (sock != 0)
    {
        fds[nfds].fd = sock->fileDescriptor();
        fds[nfds].events = POLLIN | POLLOUT;
        ++nfds;
    }

    printerState = PrinterState::Printing;
    int stall_counter = 0;
    bool have_time = false; // if we have extracted an estimated print time from slicer comments
    int resend_count = 0;
    int64_t last_error = 0;
    int64_t last_lifesign = 0; // 0 => we're not waiting for a lifesign

    for (;;)
    {
        // Save CPU cycles by doing a poll() on the involved file descriptors
        poll(fds, nfds, 1000);

        /*
          Handle all action on the serial interface before doing other stuff.
          This prioritizes communication with the printer because it is time-sensitive.
        */
        bool action_on_printer = true;
        while (action_on_printer)
        {
            action_on_printer = false;

            serial.action("reading printer response");
            gcode::Line* input;
            bool ignore_ok = false;
            while (0 != (input = gcode_serial.next()))
            {
                last_lifesign = millis();
                action_on_printer = true;
                if (input->startsWith("ok\b"))
                {
                    if (verbosity > 1)
                        stdoutbuf.put(input); // echo to stdout
                    stall_counter = 0;
                    if (ignore_ok)
                        ignore_ok = false;
                    else
                    {
                        resend_count = 0;
                        last_error = 0;
                        if (!marlinbuf.ack())
                            stdoutbuf.put( // Don't exit for this error. The user knows best.
                                new gcode::Line("WARNING! Spurious 'ok'! Is a user manually controlling the printer?"));
                    }
                }
                else if (input->startsWith("Error:"))
                {
                    if (last_error == 0)
                        last_error = millis();
                    stdoutbuf.put(input); // echo to stdout
                    // Give printer a little bit of time to send more errors if any, so that we
                    // don't leave this loop too early, start sending and trigger more errors.
                    usleep(100000);
                }
                else if (0 != (idx = input->startsWith("Resend:\b")))
                {
                    if (last_error == 0)
                        last_error = millis();
                    ++resend_count;
                    input->slice(idx);
                    stdoutbuf.put(new Line("Resend: ")); // print the sliced away part
                    stdoutbuf.put(input);                // echo to stdout
                    long line = input->number();
                    if (line < 0 || line > 2147483647)
                        line = -1;

                    if (!marlinbuf.seek(line))
                        return handle_error(e, "Illegal 'Resend' received from printer", iop, 3);

                    ignore_ok = true; // ignore the ok that accompanies the Resend

                    // Give printer a little bit of time to send more errors if any, so that we
                    // don't leave this loop too early, start sending and trigger more errors.
                    usleep(100000);
                }
                else
                {
                    last_error = 0;
                    stdoutbuf.put(input); // echo to stdout
                }

                if (last_error > 0 && millis() - last_error > MAX_TIME_WITH_ERROR)
                    return handle_error(e, "Persistent error state on printer => abort current job", iop, 3);
            }

            for (;;)
            {
                if (next_gcode == 0)
                    next_gcode = gcode_in.next(); // may still be null if no data available

                if (!have_time)
                {
                    if (gcode_in.estimatedPrintTime() > 0)
                    {
                        have_time = true;
                        printerState.setEstimatedPrintTime(gcode_in.estimatedPrintTime());
                    }
                    else
                        printerState.setPrintedBytes(gcode_in.totalBytesRead());
                }

                if (next_gcode != 0)
                {
                    if (next_gcode->length() <= marlinbuf.maxAppendLen())
                    {
                        action_on_printer = true;
                        marlinbuf.append(next_gcode->data());
                        delete next_gcode;
                        next_gcode = 0;
                    }
                    else
                    {
                        ++stall_counter;
                        break;
                    }
                }
                else
                    break;
            }

            serial.action("sending gcode to printer");
            while (marlinbuf.hasNext() && !serial.hasError())
            {
                action_on_printer = true;
                gcode::Line* gcode_to_send = new gcode::Line(marlinbuf.next());
                if (verbosity > 1)
                    stdoutbuf.put(gcode_to_send); // echo to stdout
                serial.writeAll(gcode_to_send->data(), gcode_to_send->length());
            }

            printerState = stall_counter > 2 ? PrinterState::Stalled : PrinterState::Printing;
        } // while(action_on_printer)

        // Accept as socket connection if any is pending, then fork
        // and handle it in a child process.
        if (sock != 0)
        {
            int connfd = sock->accept();
            if (connfd >= 0)
            {
                if (verbosity > 0)
                    stdoutbuf.put(new gcode::Line(NEW_SOCKET_CONNECTION));
                pid_t childpid = fork();
                if (childpid < 0)
                {
                    perror("fork");
                }
                if (childpid == 0)
                {
                    serial.close();
                    in->close();
                    sock->close();
                    handle_socket_connection(connfd);
                    _exit(0);
                }
                close(connfd);
            }
            else
            {
                if (sock->errNo() == EWOULDBLOCK)
                    sock->clearError();
            }
        }

        while (!out.hasError() && !stdoutbuf.empty())
        {
            gcode::Line& outline = stdoutbuf.peek();
            size_t nrest;
            out.writeAll(outline.data(), outline.length(), &nrest);
            if (nrest == 0)
                delete stdoutbuf.get();
            else
                outline.slice(-nrest);
        }

        if (out.errNo() == EWOULDBLOCK)
            out.clearError(); // Try again later.
                              // we don't exit for errors on stdout because it's only for echoing.

        if (resend_count > 3)
            return handle_error(e, "Too many 'Resend's received from printer", iop, 3);

        if (in->hasError())
            return handle_error(e, in->error(), iop, 0);

        if (marlinbuf.needsAck())
        {
            if (last_lifesign == 0)
                last_lifesign = millis();
            if (millis() - last_lifesign > MAX_TIME_SILENCE)
                return handle_error(e, "Printer timeout waiting for ack", iop, 3);
        }
        else
        {
            last_lifesign = 0;
            if (in->EndOfFile() && next_gcode == 0)
            {
                *iop = 0;
                *e = "EOF on GCode source";
                return true;
            }
        }

        if (serial.hasError())
            return handle_error(e, serial.error(), iop, 3);

        if (serial.EndOfFile())
            return handle_error(e, "EOF on printer connection", iop, 3);
    }
}

const char* HTTP_HEADERS = "HTTP/1.1 %d %s\r\n"
                           "Cache-Control: no-store\r\n"
                           "Content-Length: %d\r\n"
                           "Content-Type: %s\r\n"
                           "\r\n%s";

enum HTTPCode
{
    OK = 0,
    NotFound = 1
};
int HTTPCodeNum[] = {200, 404};
const char* HTTPCodeDesc[] = {"OK", "Not Found"};

const char* VERSION_JSON = "{\r\n"
                           "  \"api\": \"0.1\",\r\n"
                           "  \"server\": \"1.0.0\",\r\n"
                           "  \"text\": \"Marlinfeed 1.0.0\"\r\n"
                           "}\r\n";

const char* SETTINGS_JSON = "{\r\n"
                            "  \"feature\":\r\n"
                            "  {\r\n"
                            "    \"sdSupport\": false\r\n"
                            "  },\r\n"
                            "  \"webcam\":\r\n"
                            "  {\r\n"
                            "    \"webcamEnabled\": false,\r\n"
                            "    \"streamUrl\": \"\"\r\n"
                            "  }\r\n"
                            "}\r\n";

const char* HTTP_LOGIN = "{\r\n"
                         "  \"_is_external_client\": false,\r\n"
                         "  \"active\": true,\r\n"
                         "  \"admin\": true,\r\n"
                         "  \"apikey\": null,\r\n"
                         "  \"groups\": [\"admins\",\"users\"],\r\n"
                         "  \"name\": \"_api\""
                         "}\r\n";

int wait_empty_line(gcode::Reader& client_reader)
{
    int contentlength = 0;
    client_reader.whitespaceCompression(2);
    for (;;)
    {
        auto line = client_reader.next();
        if (line == 0)
            break;
        if (line->length() == 1)
            break;
        int idx;
        if (0 < (idx = line->startsWith("Content-Length:\b")))
        {
            line->slice(idx);
            contentlength = line->number();
        }
        delete line;
    }
    return contentlength;
}

void http_error(File& client, gcode::Reader& client_reader, HTTPCode code)
{
    int len = wait_empty_line(client_reader);
    if (len < 65536)
    {
        char buffy[len];
        client.read(buffy, len, 1000);
    }

    const char* content = "<!DOCTYPE html><html><head><title>Error</title></head><body><h1>Error</h1></body></html>";
    char* reply;
    len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[code], HTTPCodeDesc[code], strlen(content), "text/html", content);
    if (len > 0)
    {
        client.writeAll(reply, len);
        out.writeAll(reply, len);
    }
    _exit(1);
}

const char* login_json()
{
    char* login;
    if (0 < asprintf(&login, "%s", HTTP_LOGIN))
        return login;
    return "";
}

void http_json(const char* json, File& client, gcode::Reader& client_reader, HTTPCode code)
{
    int len = wait_empty_line(client_reader);
    if (len < 65536)
    {
        char buffy[len];
        client.read(buffy, len, 1000);
    }
    char* reply;
    len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[code], HTTPCodeDesc[code], strlen(json), "application/json", json);
    if (len > 0)
    {
        client.writeAll(reply, len);
        if (verbosity > 0)
            out.writeAll(reply, len);
    }
    _exit(1);
}

void handle_socket_connection(int fd)
{
    File client("API request", fd);
    gcode::Reader client_reader(client);
    client_reader.whitespaceCompression(1);
    Line* request = client_reader.next();
    if (request == 0)
        _exit(0);

    if (verbosity > 0)
        out.writeAll(request->data(), request->length());

    int idx;
    if (0 < (idx = (request->startsWith("get\b") + request->startsWith("GET\b"))))
    {
        request->slice(idx);
        if (request->startsWith("/plugin/appkeys/probe\b"))
            http_error(client, client_reader, NotFound);

        if (request->startsWith("/api/"))
        {
            request->slice(5);
            if (request->startsWith("version\b"))
                http_json(VERSION_JSON, client, client_reader, OK);
            if (request->startsWith("settings\b"))
                http_json(SETTINGS_JSON, client, client_reader, OK);
            if (request->startsWith("printer\b"))
                http_json(printerState.toJSON(), client, client_reader, OK);
            if (request->startsWith("job\b"))
                http_json(printerState.jobJSON(), client, client_reader, OK);
        }
    }
    else if (0 < (idx = (request->startsWith("post\b") + request->startsWith("POST\b"))))
    {
        request->slice(idx);
        if (request->startsWith("/api/"))
        {
            request->slice(5);
            if (request->startsWith("login\b"))
                http_json(login_json(), client, client_reader, OK);
        }
    }

    http_error(client, client_reader, NotFound);
}
