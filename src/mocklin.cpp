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
};
const option::Descriptor usage[] = {{UNKNOWN, 0, "", "", Arg::Unknown,
                                     "USAGE: mocklin [options] printdev\n\n"
                                     "printdev must be a path that does not exist or refers to a socket. "
                                     "It will be replaced by a new socket on which mocklin will listen and "
                                     "pretend to be a Marlin-based 3D printer."
                                     "\n\n"
                                     "Options:"},
                                    {HELP, 0, "", "help", Arg::None, "  \t--help  \tPrint usage and exit."},
                                    {UNKNOWN, 0, "", "", Arg::None, "\n"},
                                    {0, 0, 0, 0, 0, 0}};

void handle_connection(int fd);

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

void handle_connection(int fd)
{
    File peer("remote connection", fd);
    peer.autoClose();
    gcode::Reader reader(peer);
    gcode::Line* line;
    while (0 != (line = reader.next()))
    {
        fprintf(stdout, "%s\n", line->data());
    }

    if (peer.hasError()) // report if we ended due to an error and not EOF
        fprintf(stderr, "%s\n", peer.error());
}
