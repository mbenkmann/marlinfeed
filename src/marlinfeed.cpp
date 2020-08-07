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
#include <stdio.h>
#include <stdlib.h>

#include "arg.h"

#include "arg.h"
#include "fifo.h"
#include "file.h"
#include "gcode.h"
#include "marlinbuf.h"

using std::unique_ptr;

enum optionIndex
{
    UNKNOWN,
    HELP,
    IOERROR,
};
const option::Descriptor usage[] = {
    {UNKNOWN, 0, "", "", Arg::Unknown,
     "USAGE: marlinfeed [options] [infile ...] printdev\n\n"
     "Reads all infile in order and sends the contained GCODE to device printdev "
     "which must be compatible with Marlin's serial port protocol. printdev can "
     "be either a TTY or a Unix Domain Socket.\n"
     "Pass '-' or '/dev/stdin' to read from stdin.\n"
     "Communication is echoed to stdout."
     "\n\n"
     "Options:"},
    {HELP, 0, "", "help", Arg::None, "  \t--help  \tPrint usage and exit."},
    {IOERROR, 0, "e", "ioerror", Arg::IOError,
     "  -e<arg>, \t--ioerror[=<arg>]"
     "  \tHow to handle an error on infile or printdev.\v'next' reinitializes communication with"
     " the printer and then tries to print the next infile in order.\v"
     "'quit' (the default) terminates the program."},
    {UNKNOWN, 0, "", "", Arg::None,
     "\nExamples:\n"
     "  marlinfeed gcode/init.gcode gcode/benchy.gcode /dev/ttyUSB0 \n"
     "  marlinfeed --ioerror=next 1stprint.gcode 2ndprint.gcode /dev/ttyUSB0 \n"
     "\n"},
    {0, 0, 0, 0, 0, 0}};

bool ioerror_next = false;

bool handle(const char* infile, const char* printerdev, const char** e, int* iop);

int main(int argc, char* argv[])
{
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

    for (int i = 0; i < parse.optionsCount(); ++i)
    {
        option::Option& opt = buffer[i];
        switch (opt.index())
        {
            case HELP:
                // not possible, because handled further above and exits the program
            case IOERROR:
                ioerror_next = (opt.arg[0] == 'n');
                break;
            case UNKNOWN:
                // not possible because Arg::Unknown returns ARG_ILLEGAL
                // which aborts the parse with an error
                break;
        }
    }

    if (parse.nonOptionsCount() == 0)
    {
        fprintf(stderr, "%s\n", "You must provide a path to your printer device!");
        exit(1);
    }

    for (int i = 0; i < parse.nonOptionsCount() - 1; ++i)
    {
        const char* error = 0;   // error message returned by handle()
        int in_out_printer = -1; // 0: error occured on infile, 1: error occured on stdout,
                                 // 2: error occured on printer device
        if (!handle(parse.nonOption(i), parse.nonOption(parse.nonOptionsCount() - 1), &error, &in_out_printer))
        {
            fprintf(stderr, "%s\n", error);
            if (!ioerror_next)
                exit(1);
        }
    }
}

bool handle_error(const char** e, const char* err_msg, int* iop, int which)
{
    *e = err_msg;
    *iop = which;
    return false;
}

bool handle(const char* infile, const char* printerdev, const char** e, int* iop)
{
    unique_ptr<File> in;
    if (infile[0] == '-' && infile[1] == 0)
    {
        in.reset(new File("stdin", 0));
    }
    else
    {
        in.reset(new File(infile));
        in->open(O_RDONLY);
    }

    in->setNonBlock(true);
    if (in->hasError())
        return handle_error(e, in->error(), iop, 0);

    unique_ptr<File> out;
    out.reset(new File("stdout", 1));
    out->setNonBlock(true);
    // We don't exit for errors on stdout. It's just used for echoing.

    unique_ptr<File> serial(new File(printerdev));
    serial->action("opening printer device");
    struct stat statbuf;
    if (!serial->stat(&statbuf))
        return handle_error(e, serial->error(), iop, 2);

    if (S_ISSOCK(statbuf.st_mode))
    {
        // connect to socket
    }
    else
    { // not a socket? Treat it as a TTY.
        serial->open();
        serial->setupTTY();
        if (serial->hasError())
            return handle_error(e, serial->error(), iop, 2);
    }

    serial->action("connecting to printer");
    serial->setNonBlock(true);
    const int MAX_ATTEMPTS = 4;
    int attempt = 0;
    for (; attempt < MAX_ATTEMPTS; attempt++)
    {
        char buffy[1024];
        int idx = serial->tail(buffy, sizeof(buffy) - 1 /* -1 for appending \n if nec. */, 100, 10000);
        if (idx < 0)
            return handle_error(e, serial->error(), iop, 2);

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

        out->writeAll(buffy, n);

        // When we get here for attempt 0, we haven't yet sent WRAP_AROUND_STRING, so any ok
        // we may see is unrelated. Therefore we don't break for attempt == 0.
        if (attempt > 0 && (buffy[idx] == 'o' && buffy[idx + 1] == 'k' && buffy[idx + 2] <= ' '))
            break;

        out->writeAll(MarlinBuf::WRAP_AROUND_STRING, MarlinBuf::WRAP_AROUND_STRING_LENGTH);

        if (!serial->writeAll(MarlinBuf::WRAP_AROUND_STRING, MarlinBuf::WRAP_AROUND_STRING_LENGTH))
            return handle_error(e, serial->error(), iop, 2);
    }

    if (out->hasError())
    {
        if (out->errNo() == EWOULDBLOCK)
            out->clearError(); // we don't care if we could not echo all
        else
        {
            // Guess we won't be doing any echoing to stdout, anymore. Too bad.
        }
    }

    serial->action("");

    if (attempt == MAX_ATTEMPTS)
        return handle_error(e, "Failed to establish connection with printer", iop, 2);

    gcode::Reader gcode_serial(*serial);
    gcode_serial.whitespaceCompression(1);

    in->action("reading source gcode");
    gcode::Reader gcode_in(*in);
    gcode::Line* next_gcode = 0;

    // stdoutbuf is a FIFO buffer that stores output lines for pushing to stdout
    // the buffer has no size limit and just grows if stdout blocks
    FIFO<gcode::Line> stdoutbuf;

    MarlinBuf marlinbuf;
    int idx;

    for (;;)
    {
        // TODO: Save CPU cycles by doing a poll() on the involved file descriptors

        serial->action("reading printer response");
        gcode::Line* input;
        while (0 != (input = gcode_serial.next()))
        {
            if (input->startsWith("ok\b"))
            {
                if (!marlinbuf.ack())
                    return handle_error(e, "Spurious 'ok' received from printer", iop, 2);
            }
            else if (input->startsWith("Error:"))
            {
                // Add code to handle specific errors.
            }
            else if (0 != (idx = input->startsWith("Resend:\b")))
            {
                input->slice(idx);
                long line = input->number();
                if (line < 0 || line > 2147483647)
                    line = -1;

                if (!marlinbuf.seek(line))
                    return handle_error(e, "Illegal 'Resend' received from printer", iop, 2);
            }

            stdoutbuf.put(input); // echo to stdout
        }

        for (;;)
        {
            if (next_gcode == 0)
                next_gcode = gcode_in.next(); // may still be null if no data available

            if (next_gcode != 0)
            {
                if (next_gcode->length() <= marlinbuf.maxAppendLen())
                {
                    marlinbuf.append(next_gcode->data());
                    delete next_gcode;
                    next_gcode = 0;
                }
                else
                    break;
            }
            else
                break;
        }

        serial->action("sending gcode to printer");
        while (marlinbuf.hasNext() && !serial->hasError())
        {
            gcode::Line* gcode_to_send = new gcode::Line(marlinbuf.next());
            stdoutbuf.put(gcode_to_send); // echo to stdout
            serial->writeAll(gcode_to_send->data(), gcode_to_send->length());
        }

        while (!out->hasError() && !stdoutbuf.empty())
        {
            gcode::Line& outline = stdoutbuf.peek();
            size_t nrest;
            out->writeAll(outline.data(), outline.length(), &nrest);
            if (nrest == 0)
                delete stdoutbuf.get();
            else
                outline.slice(-nrest);
        }

        if (out->errNo() == EWOULDBLOCK)
            out->clearError(); // Try again later.
        // we don't exit for errors on stdout because it's only for echoing.

        if (in->hasError())
            return handle_error(e, in->error(), iop, 0);

        if (in->EndOfFile() && next_gcode == 0)
        {
            *iop = 0;
            *e = "EOF on GCode source";
            return true;
        }

        if (serial->hasError())
            return handle_error(e, serial->error(), iop, 2);

        if (serial->EndOfFile())
            return handle_error(e, "EOF on printer connection", iop, 2);
    }
}
