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

#include <arpa/inet.h>
#include <errno.h>
#include <memory>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <utime.h>

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
    VERBOSE,
    PORT,
    LOCALHOST,
    API
};
const option::Descriptor usage[] = {
    {UNKNOWN, 0, "", "", Arg::Unknown,
     "USAGE: marlinfeed [options] [<infile> ...] <printdev>\n\n"
     "Reads all <infile> in order and sends the contained GCODE to device <printdev> "
     "which must be compatible with Marlin's serial port protocol.\n"
     "<printdev> can be either a TTY or a Unix Domain Socket.\n"
     "Pass '-' or '/dev/stdin' as <infile> to read from stdin.\n"
     "If an <infile> is a directory, it will be watched for "
     "new/modified gcode files that will automatically be printed. Files with a timestamp "
     "older than the time Marlinfeed is started won't be printed.\n"
     "If no --api and no <infile> is passed, '-' is assumed.\n"
     "Communication is echoed to stdout.\n"
     "\n"
     "Octoprint API:\n"
     "Use the --api switch to make Marlinfeed listen for incoming connections and serve them "
     "with an Octoprint compatible API. Uploaded print files will be stored in the first watch "
     "directory in the <infile> ... list. If no directories are listed, a temporary directory under /tmp "
     "will be created and used.\n"
     "\n"
     "Security:\n"
     "Marlinfeed offers no access control features other than the --localhost switch. To make Marlinfeed "
     "available over an insecure network, use something like haproxy(1). A very good authentication "
     "solution is the use of client certificates. A quick and easy tool to create a set of "
     "client, server and CA certificates is certificate-assembler(1) which is part of the certifidog "
     "package.\n"
     "\n\n"
     "Options:"},
    {HELP, 0, "", "help", Arg::None, "  \t--help  \tPrint usage and exit."},
    {VERBOSE, 0, "v", "verbose", Arg::None,
     "  -v, \t--verbose  \tIncrease verbosity. Can be used multiple times. At level 4+ erroneous requests will be "
     "written to /tmp."},
    {API, 0, "", "api", Arg::Required,
     " \t--api=<base-url>  \tListen for incoming connections with an Octoprint compatible API that clients will access "
     "as <base-url>/api . If --port is not specified and <base-url> contains a port, the latter port will be the port "
     "Marlinfeed will listen on."},
    {PORT, 0, "p", "port", Arg::Numeric,
     "  -p<num>, \t--port=<num>  \tPort to listen on for API connections. Defaults to 8080 unless derived from "
     "<base-url> (see above)."},
    {LOCALHOST, 0, "", "localhost", Arg::None,
     " \t--localhost  \tLimit API connections to connections from the same machine Marlinfeed is running on. "
     "Most useful when combined with something like haproxy(1) to make Marlinfeed available to a wider network with "
     "access controls."},
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
     "  marlinfeed --ioerror=quit --api=http://my-printer:80 upload /dev/ttyUSB0 \n"
     "      listens for connections on port 80.\n\n"
     "  marlinfeed --localhost --api=https://my-printer /dev/ttyUSB0 \n"
     "      listens for localhost connections on port 8080. Needs some form of proxy to implement TLS.\n\n"
     "  marlinfeed -p 6000 --api=https://my-printer:443/ /var/cache/marlinfeed /dev/ttyUSB0 \n"
     "      listens for connections on port 6000. Needs some form of proxy.\n"
     "\n"},
    {0, 0, 0, 0, 0, 0}};

const char* NEW_SOCKET_CONNECTION = "New socket connection => Handled by child with PID %d\n";

// Maximum number of milliseconds we don't get a non-error reply from the printer.
// This aborts the current job if the printer keeps replying to everything we send
// with an error.
const int MAX_TIME_WITH_ERROR = 5000;

// Maximum number of milliseconds with no message from the printer while
// at least 1 command is not ack'd. Needs to be longer than the longest
// blocking command that is silent (e.g. G28).
const int MAX_TIME_SILENCE = 120000;

// If this number of milliseconds pass with a gcode ready to be sent to the printer
// but no ok received to free up space in the buffer, the printer state will change to
// Stalled. This indicates a long running command like G28.
const int STALL_TIME = 2000;

bool ioerror_next;
int verbosity = 0;
volatile sig_atomic_t interrupt = 0;

bool handle(File& out, File& serial, const char* infile, File* sock, const char** e, int* iop);
void handle_socket_connection(int fd);
void socketTest();

void signal_handler(int signum, siginfo_t*, void*)
{
    switch (signum)
    {
        case SIGUSR1:
            interrupt ^= 1;
            break;
    }
}

bool isPaused() { return (interrupt & 1) != 0; }

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
    int64_t pauseStartTime;
    int64_t pauseTime;
    const char* printName;
    int64_t printSize;
    int64_t printedBytes;

  public:
    void clearJob()
    {
        startTime = 0;
        endTime = 0;
        pauseTime = 0;
        pauseStartTime = 0;
        free((void*)printName);
        printName = strdup("None");
        printSize = 0;
        printedBytes = 0;
    }

    enum Enum
    {
        Disconnected = 0, // Marlinfeed not currently sync'ed with printer
        Printing = 1,     // Commands are flowing from an infile to printer
        Idle = 2,         // Marlinfeed sync'ed with printer but no active infile
        Stalled = 3,      // Commands are waiting because printer buffer has been full for a while
        Paused = 4        // Paused by user
    } status;

    void operator=(Enum s)
    {
        if (s != Printing && s != Stalled && s != Paused)
            clearJob();
        if (s == Printing && status != Printing && status != Stalled && status != Paused)
            startTime = millis();
        if (s == Paused && status != Paused)
            pauseStartTime = millis();
        if (status == Paused && s != Paused)
        {
            pauseTime += millis() - pauseStartTime;
            pauseStartTime = 0;
        }
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

    void parseTemperatureReport(const char* p)
    {
        /*  Temperature report format:
        M190
        T:25.91 E:0 B:48.1
        ok
        If waiting, ok is only returned when temp is reached

        M105
        ok T:25.9 /0.0 B:50.0 /50.0 T0:25.9 /0.0 @:0 B@:0
           ^^^    ^^^^        ^^^^  ^^^^^^^^^^^
        active hotend target      target  all hotends T0, T1,...
        The @ and B@ stuff are a number specifying the current power applied to the heaters

        M109 S104
        T:100.0 E:0 W:?
        ...
        T:100.0 E:0 W:0
        ok
        */

        float* component = 0;
        int idx = 0;
        while (*p != 0)
        {
            if (p[0] == 'T' && p[1] == ':')
            {
                p += 2;
                component = &tool[0][0];
                idx = 0;
            }
            else if (p[0] == 'T' && p[1] == '0' && p[2] == ':')
            {
                p += 3;
                component = &tool[0][0];
                idx = 0;
            }
            else if (p[0] == 'T' && p[1] == '1' && p[2] == ':')
            {
                p += 3;
                component = &tool[1][0];
                idx = 0;
            }
            else if (p[0] == 'B' && p[1] == ':')
            {
                p += 2;
                component = &bed[0];
                idx = 0;
            }
            else if (p[0] == '/')
            {
                idx = 1;
                p++;
            }
            else
            {
                while (*p != 0 && *p != ':')
                    p++;
                if (p[0] != 0)
                    p++;
                component = 0;
            }

            char* endptr;
            double d = strtod(p, &endptr);
            p = endptr;

            while (isspace(*p))
                p++;

            if (component != 0)
                component[idx] = d;
        }
    }

    //"{\"job\":{\"file\":{\"name\":\"\"}},\"progress\":{\"printTime\":null,\"completion\":null}}"
    const char* jobJSON()
    {
        char* j;
        const char* text = "Operational";
        if (status == Printing || status == Stalled)
            text = "Printing";
        else if (status == Paused)
            text = "Paused";
        double deltat = 0;
        if (startTime > 0)
        {
            if (pauseStartTime > 0)
                deltat = pauseStartTime - startTime;
            else
                deltat = millis() - startTime;

            deltat -= pauseTime;
        }
        double completion = 0;
        if (startTime > 0 && endTime > startTime)
            completion = 100.0 * deltat / (endTime - startTime);
        else if (printSize > 0)
            completion = 100.0 * (double)printedBytes / (double)printSize;
        deltat /= 1000; // convert to seconds

        const char* nameOnly = strrchr(printName, '/');
        if (nameOnly == 0)
            nameOnly = printName;
        else
            nameOnly++; // skip the '/'

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
                           "      \"printTimeLeft\": null,\r\n"
                           "      \"completion\": %f\r\n"
                           "  }\r\n"
                           "}\r\n",
                           text, nameOnly, deltat, completion);
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
        else if (status == Stalled)
            text = "Stalled";
        else if (status == Paused)
            text = "Paused";
        const char* operational = boolStr(true);
        const char* paused = boolStr(status == Paused);
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

const char* api_base_url = 0;
const char* upload_dir = 0;
int cmd_inject[2]; // socketpair, cmd_inject[0] is the write end for child processes
gcode::Reader* inject_in;

pid_t MainProcess = getpid();

int main(int argc, char* argv[])
{
    signal(SIGCHLD, SIG_IGN); // automatic zombie removal
    signal(SIGPIPE, SIG_IGN); // handle as EPIPE on write()
    struct sigaction sigact;
    sigact.sa_sigaction = signal_handler;
    sigact.sa_flags = SA_SIGINFO;
    sigfillset(&sigact.sa_mask);
    assert(0 == sigaction(SIGUSR1, &sigact, 0));
    /*assert(0 == sigaction(SIGHUP, &sigact, 0));
    assert(0 == sigaction(SIGINT, &sigact, 0));
    assert(0 == sigaction(SIGTERM, &sigact, 0));
    assert(0 == sigaction(SIGQUIT, &sigact, 0));*/

    argc -= (argc > 0);
    argv += (argc > 0); // skip program name argv[0] if present
    option::Stats stats(true, usage, argc, argv);

    // GCC supports C99 VLAs for C++ with proper constructor calls.
    option::Option options[stats.options_max], buffer[stats.buffer_max];

    option::Parser parse(true, usage, argc, argv, options, buffer);

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

    assert(0 == socketpair(AF_UNIX, SOCK_SEQPACKET, 0, cmd_inject));
    File inject("Command Injector", cmd_inject[1]);
    inject.setNonBlock(true);
    inject_in = new gcode::Reader(inject);
    inject_in->whitespaceCompression(1);

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

    long port = 8080;
    if (options[API])
    {
        api_base_url = options[API].last()->arg;

        const char* p = strstr(api_base_url, ":/");
        if (p == 0)
            p = api_base_url;
        else
            ++p;
        p = strchr(p, ':');

        if (p)
            port = strtol(p + 1, 0, 10);

        if (options[PORT])
            port = strtol(options[PORT].last()->arg, 0, 10);

        if (port < 10 || port > 65535)
        {
            fprintf(stderr, "%s %ld\n", "Illegal port specified:", port);
            exit(1);
        }
    }
    else
    {
        if (options[LOCALHOST] || options[PORT])
        {
            fprintf(stderr, "%s\n", "--localhost and --port don't work without --api!");
            exit(1);
        }
    }

    for (int i = 0; i < parse.nonOptionsCount() - 1; ++i)
    {
        const char* inf = parse.nonOption(i);
        if (inf[0] == '-' && inf[1] == 0)
        {
            infile_queue.put(strdup(inf));
            continue;
        }

        struct stat statbuf;

        if (0 > stat(inf, &statbuf))
        {
            fprintf(stderr, "Don't understand this argument: %s\n", parse.nonOption(i));
            exit(1);
        }

        if (S_ISDIR(statbuf.st_mode))
        {
            dirScanner.addDir(inf);
            if (upload_dir == 0)
                upload_dir = strdup(inf);
        }
        else
            infile_queue.put(strdup(inf));
    }

    if (api_base_url != 0)
    {
        char* listen_address;
        const char* host = "";
        if (options[LOCALHOST])
            host = "localhost";
        assert(0 <= asprintf(&listen_address, "%s:%ld", host, port));
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

        if (upload_dir == 0)
        {
            // Create temporary directory
            upload_dir = File::createDirectory("/tmp/marlinfeed-????", 0700);
            if (upload_dir == 0)
            {
                perror("mkdir");
                exit(1);
            }
            dirScanner.addDir(upload_dir);
        }

        fprintf(stdout, "Listening on port %ld. Uploading to %s. API base: %s\n", port, upload_dir, api_base_url);
    }
    else // If we're not listening
    {
        // If we don't have any infile arguments, assume "-" (i.e. stdin)
        if (parse.nonOptionsCount() == 1)
            infile_queue.put(strdup("-"));
    }

    ioerror_next = false; // default
    if (sock != 0)
        ioerror_next = true; // default if we are listening
    if (options[IOERROR] && options[IOERROR].arg[0] == 'q')
        ioerror_next = false; // override default if --ioerror=quit on command line

    printerState = PrinterState::Disconnected;

    if (api_base_url != 0 && strcmp(api_base_url, "Debug") == 0)
        socketTest();

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
                sock->poll(POLLIN, 250);
                // Accept as socket connection if any is pending, then fork
                // and handle it in a child process.
                int connfd = sock->accept();
                if (connfd >= 0)
                {
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
                    if (verbosity > 0)
                        fprintf(stdout, NEW_SOCKET_CONNECTION, childpid);
                }
                else
                {
                    if (sock->errNo() == EWOULDBLOCK)
                        sock->clearError();
                }
            }
            else if (!inject_in->hasNext())
                usleep(250000); // to make sure we don't burn cycles waiting for files

            dirScanner.refill(infile_queue);
            infile_queue.filter(gcode_extension);
            if (infile_queue.empty() && !inject_in->hasNext())
                continue;
        }

        const char* error = 0;   // error message returned by handle()
        int in_out_printer = -1; // 0: error occurred on infile,
                                 // 1: error occured on stdout,
                                 // 2: error occurred on printer device, do not reconnect
                                 // 3: error occurred on printer device, try reconnecting

        char* infile = infile_queue.get();
        if (infile == 0) // can only happen if we have something in inject_in
            infile = strdup("/dev/null");

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
    if (verbosity > 0)
    {
        out.writeAll("\n>>> ", 5);
        out.writeAll(infile, strlen(infile));
        out.writeAll("\n", 1);
    }

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

    printerState = PrinterState::Printing;
    int64_t last_ok_time = 0;
    bool have_time = false; // if we have extracted an estimated print time from slicer comments
    int resend_count = 0;
    int64_t last_error = 0;
    int64_t last_lifesign = 0; // 0 => we're not waiting for a lifesign

    for (;;)
    {
        // Save CPU cycles by doing a poll() on the involved file descriptors
        {
            pollfd fds[5];
            int nfds = 0;

            fds[nfds].fd = serial.fileDescriptor();
            fds[nfds].events = POLLIN; // always interested in what the printer has to say
            if (marlinbuf.hasNext())
                fds[nfds].events |= POLLOUT;

            fds[++nfds].fd = cmd_inject[1]; // always interested in injections
            fds[nfds].events = POLLIN;

            if (!out.hasError() && !stdoutbuf.empty())
            {
                fds[++nfds].fd = out.fileDescriptor();
                fds[nfds].events = POLLOUT;
            }

            if (next_gcode == 0 && !isPaused())
            {
                fds[++nfds].fd = in->fileDescriptor();
                fds[nfds].events = POLLIN;
            }

            if (sock != 0)
            {
                fds[++nfds].fd = sock->fileDescriptor();
                fds[nfds].events = POLLIN;
            }

            ++nfds;
            poll(fds, nfds, -1);
        }

        /*
          Handle all action on the serial interface before doing other stuff.
          This prioritizes communication with the printer because it is time-sensitive.
        */
        bool action_on_printer = true;
        while (action_on_printer)
        {
            action_on_printer = false;

            serial.action("reading printer response");
            serial.setNonBlock(true);
            gcode::Line* input;
            bool ignore_ok = false;
            while (0 != (input = gcode_serial.next()))
            {
                last_lifesign = millis();
                action_on_printer = true;
            reparse:
                if (0 != (idx = input->startsWith("ok\b")))
                {
                    if (verbosity > 2)
                        stdoutbuf.put(new gcode::Line("ok\n")); // echo to stdout

                    last_ok_time = millis();
                    if (ignore_ok)
                        ignore_ok = false;
                    else
                    {
                        resend_count = 0;
                        last_error = 0;
                        if (!marlinbuf.ack())
                            stdoutbuf.put( // Don't exit for this error. The user knows best.
                                new gcode::Line(
                                    "WARNING! Spurious 'ok'! Is a user manually controlling the printer?\n"));
                    }

                    input->slice(idx);
                    if (input->length() > 0)
                        goto reparse; // in case something follows ok, such as an M105 temperature report
                    else
                        delete input;
                }
                else if (input->startsWith("T:"))
                {
                    printerState.parseTemperatureReport(input->data());

                    if (verbosity > 1)
                        stdoutbuf.put(input);
                    else
                        delete input;
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
                    next_gcode = inject_in->next(); // may still be null if no data available
                if (next_gcode == 0 && !isPaused())
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
                        break;
                    }
                }
                else
                    break;
            }

            serial.action("sending gcode to printer");
            serial.setNonBlock(false);
            while (marlinbuf.hasNext() && !serial.hasError())
            {
                action_on_printer = true;
                gcode::Line* gcode_to_send = new gcode::Line(marlinbuf.next());
                serial.writeAll(gcode_to_send->data(), gcode_to_send->length());
                if (verbosity > 2)
                    stdoutbuf.put(gcode_to_send); // echo to stdout
                else
                    delete gcode_to_send;
            }

            if (isPaused())
                printerState = PrinterState::Paused;
            else
                printerState = (next_gcode != 0 && millis() - last_ok_time > STALL_TIME) ? PrinterState::Stalled
                                                                                         : PrinterState::Printing;
        } // while(action_on_printer)

        // Accept as socket connection if any is pending, then fork
        // and handle it in a child process.
        if (sock != 0)
        {
            int connfd = sock->accept();
            if (connfd >= 0)
            {
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
                if (verbosity > 0)
                    fprintf(stdout, NEW_SOCKET_CONNECTION, childpid);
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

void* raw;
int rawsize;

const char* HTTP_HEADERS = "HTTP/1.1 %d %s\r\n"
                           "%sCache-Control: no-store\r\n"
                           "Content-Length: %d\r\n"
                           "Content-Type: %s\r\n"
                           "\r\n%s";

enum HTTPCode
{
    OK = 0,
    NotFound = 1,
    Created = 2,
    NoContent = 3
};
int HTTPCodeNum[] = {200, 404, 201, 204};
const char* HTTPCodeDesc[] = {"OK", "Not Found", "Created", "No Content"};

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

const char* HTTP_CREATED_JSON = "{\r\n"
                                "  \"done\": true,\r\n"
                                "  \"files\": {\r\n"
                                "    \"local\": {\r\n"
                                "      \"origin\": \"local\",\r\n"
                                "      \"refs\": {\r\n"
                                "      }\r\n"
                                "    }\r\n"
                                "  }\r\n"
                                "}\r\n";

int wait_empty_line(gcode::Reader& client_reader)
{
    int contentlength = 0;
    for (;;)
    {
        auto line = client_reader.next();
        if (line == 0 || line->length() == 0)
            break;
        if (verbosity > 1)
            out.writeAll(line->data(), line->length());
        if (verbosity > 3)
        {
            raw = realloc(raw, rawsize + line->length());
            memcpy((char*)raw + rawsize, line->data(), line->length());
            rawsize += line->length();
        }
        if (line->data()[0] == '\n' || (line->data()[0] == '\r' && line->data()[1] == '\n'))
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

void http_error(const char* message, int echo_verbosity, File& client, gcode::Reader& client_reader, HTTPCode code)
{
    int len = wait_empty_line(client_reader);
    if (len < 65536)
    {
        char buffy[len];
        int i = client_reader.raw(buffy, len);
        len = client.read(buffy + i, len - i, 1000);
        if (len >= 0 && verbosity > 3)
        {
            len += i;
            raw = realloc(raw, rawsize + len);
            memcpy((char*)raw + rawsize, buffy, len);
            rawsize += len;
            const char* fname = File::createFile("/tmp/raw-request-????", 0600);
            if (fname)
            {
                File f(fname);
                f.open(O_WRONLY);
                f.writeAll(raw, rawsize);
                f.close();
            }
        }
    }

    char* reply;
    char* content;
    len = asprintf(
        &content,
        "<!DOCTYPE html><html><head><title>Error</title></head><body><h1>Unsupported Request: %s</h1></body></html>",
        message);
    if (len > 0)
        len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[code], HTTPCodeDesc[code], "", strlen(content), "text/html",
                       content);
    if (len > 0)
    {
        client.writeAll(reply, len);
        if (verbosity >= echo_verbosity)
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

void http_json(const char* json, File& client, gcode::Reader& client_reader, HTTPCode code,
               const char* extra_headers = "")
{
    int len = wait_empty_line(client_reader);
    len -= client_reader.discard();
    if (len > 0 && len < 65536)
    {
        char buffy[len];
        client.read(buffy, len, 1000);
    }
    char* reply;
    len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[code], HTTPCodeDesc[code], extra_headers, strlen(json),
                   "application/json", json);
    if (len > 0)
    {
        client.writeAll(reply, len);
        if (verbosity > 1)
            out.writeAll(reply, len);
    }
    _exit(1);
}

void upload(File& client, gcode::Reader& client_reader)
{
    client_reader.whitespaceCompression(0); // preserve whitespace
    client_reader.commentChar('\n');        // do not handle comments
    int contentlength = wait_empty_line(client_reader);
    int contentread = 0;

    const char* boundary = 0;
    int len;

    char* fname = 0;
    char* file_line = 0;
    char* finished_fname = 0;

    bool wait_for_file_start = false;

    // create temporary file
    char fpath[1024];
    snprintf(fpath, sizeof(fpath), "%s/upload-????", upload_dir);
    const char* tempname = File::createFile(fpath, 0644);
    if (tempname == 0)
    {
        perror(fpath);
        _exit(1);
    }

    File tmp(tempname);

    while (client_reader.hasNext())
    {
        unique_ptr<gcode::Line> line(client_reader.next());
        contentread += line->length();

        if (boundary == 0 && line->startsWith("--"))
        {
            if (verbosity > 1)
                out.writeAll(line->data(), line->length());
            boundary = strdup(line->data());
        }
        else if (0 != (len = line->startsWith(boundary)) && (line->length() == len))
        {
            if (verbosity > 1)
                out.writeAll(line->data(), line->length());

            if (file_line)
            {
                out.clearError(); // In case we wrote too fast and ran into EWOULDBLOCK

                // Translate evil characters to _
                finished_fname = strdup(fname);
                for (unsigned char* p = (unsigned char*)finished_fname; *p != 0; p++)
                    if (!(*p > 127 || isalnum(*p) || *p == '_' || *p == '-' || *p == '+' || *p == '.' || *p == ','))
                        *p = '_';

                if (verbosity > 0)
                {
                    char msg[1024];
                    int len =
                        snprintf(msg, sizeof(msg), "Renaming temporary file '%s' => '%s'\n", tempname, finished_fname);
                    if (len >= (int)sizeof(msg))
                        len = sizeof(msg) - 1;
                    out.writeAll(msg, len);
                }

                char* newpath;
                assert(0 < asprintf(&newpath, "%s/%s", upload_dir, finished_fname));
                tmp.move(newpath);
                tmp.close();

                if (tmp.hasError())
                    fprintf(stderr, "%s\n", tmp.error());

                free(fname);
                free(file_line);
                fname = 0;
                file_line = 0;
                break;
            }
        }
        else if (fname != 0)
        {
            if (wait_for_file_start)
            {
                if (verbosity > 1)
                    out.writeAll(line->data(), line->length());
                wait_for_file_start = !(line->data()[0] == '\r' && line->data()[1] == '\n');
            }
            else
            {
                if (file_line)
                {
                    if (verbosity > 2)
                        out.writeAll(".", 1); // one dot per line
                    tmp.writeAll(file_line, strlen(file_line));
                }
                else
                {
                    if (verbosity > 0)
                    {
                        char msg[512];
                        int len = snprintf(msg, sizeof(msg), "Storing upload data in temporary file '%s'\n", tempname);
                        if (len >= 512)
                            len = 511;
                        out.writeAll(msg, len);
                    }

                    tmp.open(O_WRONLY);
                }
                free(file_line);
                file_line = strdup(line->data());
            }
        }
        else if ((finished_fname == 0) && line->startsWith("Content-Disposition:\bform-data\b"))
        {
            if (verbosity > 1)
                out.writeAll(line->data(), line->length());
            fname = line->getString("filename");
            wait_for_file_start = (fname != 0);
        }
        else if (verbosity > 1)
            out.writeAll(line->data(), line->length());
    }

    if (tmp.fileDescriptor() >= 0)
        fprintf(stderr, "Premature end of upload data\n");

    if (finished_fname)
    {
        // Read remainder of multipart MIME data
        int dis = client_reader.discard();
        contentread += dis;
        contentlength -= contentread;
        if (contentlength > 0 && contentlength < 65536)
        {
            char buf[contentlength];
            client.read(buf, sizeof(buf), 200, 2000, 200);
        }

        const char* location;
        if (0 >= asprintf((char**)&location, "Location: %s/api/files/local/%s\r\n", api_base_url, finished_fname))
            location = "";
        char* reply;
        int len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[Created], HTTPCodeDesc[Created], location,
                           strlen(HTTP_CREATED_JSON), "application/json", HTTP_CREATED_JSON);
        if (len > 0)
        {
            client.writeAll(reply, len);
            if (verbosity == 1)
                out.writeAll(location, strlen(location));
            if (verbosity > 1)
                out.writeAll(reply, len);
        }
    }
    _exit(1);
}

void touch_file(gcode::Line& request, File& client, gcode::Reader& client_reader)
{
    int contentlength = wait_empty_line(client_reader);
    if (contentlength > 0 && contentlength < 65536)
    {
        char buf[contentlength + 1];
        int i = client_reader.raw(buf, contentlength);
        contentlength = client.read(buf + i, contentlength - i, 200, 2000);

        if (contentlength >= 0)
        {
            contentlength += i;
            buf[contentlength] = 0;
            request.slice(strlen("files/local/"));
            const char* space = strchr(request.data(), ' ');
            if (space)
            {
                int len = space - request.data();
                request.slice(0, len);

                if (0 != strstr(buf, "\"print\""))
                {
                    char* finished_fname = strdup(request.data());
                    for (unsigned char* p = (unsigned char*)finished_fname; *p != 0; p++)
                        if (!(*p > 127 || isalnum(*p) || *p == '_' || *p == '-' || *p == '+' || *p == '.' || *p == ','))
                            *p = '_';

                    char* fpath;
                    if (0 < asprintf(&fpath, "%s/%s", upload_dir, finished_fname))
                    {
                        File f(fpath);
                        struct stat statbuf;
                        if (f.stat(&statbuf) && S_ISREG(statbuf.st_mode))
                        {
                            utime(fpath, 0);
                            char* reply;
                            len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[NoContent], HTTPCodeDesc[NoContent], "", 0,
                                           "text/html", "");
                            if (len > 0)
                            {
                                client.writeAll(reply, len);
                                if (verbosity > 1)
                                    out.writeAll(reply, len);
                            }
                            _exit(0);
                        }
                    }
                }
            }
        }
    }

    const char* content =
        "<!DOCTYPE html><html><head><title>Error</title></head><body><h1>Touch Error</h1></body></html>";
    char* reply;
    int len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[NotFound], HTTPCodeDesc[NotFound], "", strlen(content),
                       "text/html", content);
    if (len > 0)
    {
        client.writeAll(reply, len);
        out.writeAll(reply, len);
    }
    _exit(1);
}

void inject(File& client, gcode::Reader& client_reader)
{
    int contentlength = wait_empty_line(client_reader);
    if (contentlength > 0 && contentlength < 65536)
    {
        char buf[contentlength + 1];
        int i = client_reader.raw(buf, contentlength);
        contentlength = client.read(buf + i, contentlength - i, 200, 2000);

        if (contentlength >= 0)
        {
            contentlength += i;
            buf[contentlength] = 0;

            char* commands = strstr(buf, "\"commands\"");
            if (commands)
                commands = strchr(commands, '[');
            if (commands)
            {
                File injector("Command Injector", cmd_inject[0]);
                char* cmd = 0;
                commands++;
                char* p = commands;
                for (; *p != ']' && *p != 0; p++)
                {
                    if (*p == '"')
                    {
                        if (cmd == 0)
                            cmd = p + 1;
                        else
                            cmd = 0;
                        *p = '\n';
                    }
                    else if (*p == ',')
                        *p = ' ';
                }

                if (*p == ']')
                {
                    if (verbosity > 1)
                    {
                        *p = 0;
                        fprintf(stdout, "Injecting \"%s\"\n", commands);
                    }
                    *p = '\n';
                    injector.writeAll(commands, p + 1 - commands);

                    char* reply;
                    int len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[NoContent], HTTPCodeDesc[NoContent], "", 0,
                                       "text/html", "");
                    if (len > 0)
                    {
                        client.writeAll(reply, len);
                        if (verbosity > 1)
                            out.writeAll(reply, len);
                    }
                    _exit(0);
                }
            }
        }
    }

    const char* content =
        "<!DOCTYPE html><html><head><title>Error</title></head><body><h1>Inject Error</h1></body></html>";
    char* reply;
    int len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[NotFound], HTTPCodeDesc[NotFound], "", strlen(content),
                       "text/html", content);
    if (len > 0)
    {
        client.writeAll(reply, len);
        out.writeAll(reply, len);
    }
    _exit(1);
}

void job_command(File& client, gcode::Reader& client_reader)
{
    client_reader.whitespaceCompression(0); // preserve whitespace
    client_reader.commentChar('\n');        // do not handle comments
    int contentlength = wait_empty_line(client_reader);

    char buf[contentlength + 1];
    int i = client_reader.raw(buf, contentlength);
    contentlength = client.read(buf + i, contentlength - i, 200, 2000);

    int command = 0;
    int action = 0;

    if (contentlength >= 0)
    {
        contentlength += i;
        buf[contentlength] = 0;

        gcode::Line line(buf);

        char* cmd = line.getString("\"command\"");
        if (cmd)
        {
            if (strcmp(cmd, "pause") == 0)
                command = 1;
            else if (strcmp(cmd, "cancel") == 0)
                command = 2;
            free(cmd);
        }

        char* act = line.getString("\"action\"");
        if (act)
        {
            if (strcmp(act, "pause") == 0)
                action = 1;
            else if (strcmp(act, "resume") == 0)
                action = 2;
            free(act);
        }
    }

    if (command > 0)
    {
        if (command == 1)
        {                    // pause
            if (action == 0) // toggle
                kill(MainProcess, SIGUSR1);
        }

        char* reply;
        int len =
            asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[NoContent], HTTPCodeDesc[NoContent], "", 0, "text/html", "");
        if (len > 0)
        {
            client.writeAll(reply, len);
            if (verbosity > 1)
                out.writeAll(reply, len);
        }
        _exit(0);
    }

    const char* content =
        "<!DOCTYPE html><html><head><title>Error</title></head><body><h1>Unsupported Job Action</h1></body></html>";
    char* reply;
    int len = asprintf(&reply, HTTP_HEADERS, HTTPCodeNum[NotFound], HTTPCodeDesc[NotFound], "", strlen(content),
                       "text/html", content);
    if (len > 0)
    {
        client.writeAll(reply, len);
        out.writeAll(reply, len);
    }
    _exit(1);
}

void handle_socket_connection(int fd)
{
    // union {
    //     struct sockaddr sock;
    //     struct sockaddr_in6 ipv6;
    //     struct sockaddr_in ipv4;
    // } addr;

    // socklen_t socklen = sizeof(addr);
    // if (getsockname(fd, &addr.sock, &socklen) != 0)
    // {
    //     perror("getsockname");
    //     addr.sock.sa_family = 0;
    // }

    // char buffy[128];
    // const char* addr_str;
    // char* url;
    // switch (addr.sock.sa_family)
    // {
    //     case AF_INET:
    //         addr_str = inet_ntop(addr.ipv4.sin_family, &addr.ipv4.sin_addr, buffy, sizeof(buffy));
    //         asprintf(&url, "%s%s:%d", location_prefix, addr_str, location_port);
    //         break;
    //     case AF_INET6:
    //         addr_str = inet_ntop(addr.ipv6.sin6_family, &addr.ipv6.sin6_addr, buffy, sizeof(buffy));
    //         asprintf(&url, "%s[%s]:%d", location_prefix, addr_str, location_port);
    //         break;
    //     default:
    //         asprintf(&url, "%slocalhost:%d", location_prefix, location_port);
    // }

    // File f("/tmp/upload.raw");
    // f.open(O_CREAT | O_EXCL | O_WRONLY, 0644);
    // char buf[65536];
    // int sz = client_reader.raw(buf, sizeof(buf));
    // f.writeAll(buf, sz);
    // for (;;)
    // {
    //     sz = client.read(buf, sizeof(buf), 5000, -1, 5000);
    //     if (sz <= 0)
    //         break;
    //     f.writeAll(buf, sz);
    // }
    // f.close();
    // _exit(1);

    File client("API request", fd);
    gcode::Reader client_reader(client);
    client_reader.whitespaceCompression(1);
    Line* request = client_reader.next();
    if (request == 0)
        _exit(0);

    if (verbosity > 0)
        out.writeAll(request->data(), request->length());

    if (verbosity > 3)
    {
        raw = strdup(request->data());
        rawsize = request->length();
    }

    int idx;
    if (0 < (idx = (request->startsWith("get\b") + request->startsWith("GET\b"))))
    {
        request->slice(idx);
        if (request->startsWith("/plugin/appkeys/probe\b"))
            http_error("/plugin/appkeys/probe", 2, client, client_reader, NotFound);

        if (request->startsWith("/api/"))
        {
            request->slice(5);
            if (request->startsWith("version\b"))
                http_json(VERSION_JSON, client, client_reader, OK);
            else if (request->startsWith("settings\b"))
                http_json(SETTINGS_JSON, client, client_reader, OK);
            else if (request->startsWith("printer\b"))
                http_json(printerState.toJSON(), client, client_reader, OK);
            else if (request->startsWith("job\b"))
                http_json(printerState.jobJSON(), client, client_reader, OK);
            else if (request->startsWith("printerprofiles\b"))
                http_error("/api/printerprofiles", 2, client, client_reader, NotFound);
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
            else if (request->startsWith("job\b"))
                job_command(client, client_reader);
            else if (request->startsWith("files/local/"))
                touch_file(*request, client, client_reader);
            else if (request->startsWith("files/local\b"))
                upload(client, client_reader);
            else if (request->startsWith("printer/command\b"))
                inject(client, client_reader);
        }
    }

    http_error(request->data(), 0, client, client_reader, NotFound);
}

void socketTest()
{
    int fd[2];
    assert(0 == socketpair(AF_LOCAL, SOCK_STREAM, 0, fd));
    pid_t childpid = fork();
    assert(childpid >= 0);
    if (childpid == 0)
    {
        close(fd[0]);
        File uploadData("test/upload.raw");
        uploadData.open(O_RDONLY);
        File sock("Teststream", fd[1]);
        char buf[65536];
        for (;;)
        {
            int sz = uploadData.read(buf, sizeof(buf));
            if (sz <= 0)
                break;
            sock.writeAll(buf, sz);
        }

        out.setNonBlock(false);
        for (;;)
        {
            int sz = sock.read(buf, sizeof(buf));
            if (sz <= 0)
                break;
            out.writeAll(buf, sz);
        }

        fprintf(stdout, "Connection closed.\n");

        _exit(0);
    }

    close(fd[1]);
    handle_socket_connection(fd[0]);
    _exit(0);
}
