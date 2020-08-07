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

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "fifo.h"
#include "file.h"
#include "gcode.h"
#include "marlinbuf.h"

const char* SIGCHILD_MSG = "...\n";
const char* WELCOME_MSG = "Running unit tests...\n";
const char* BYE_MSG = "...all tests successful\n";
const char* RESUME_MSG1 = "Checking writeAll resume after partial writes\n";
const char* RESUME_MSG2 = "Checking writeAll resume after EINTR\n";

char bigblock[655360];

void sigchild(int)
{
    (void)!write(1, SIGCHILD_MSG, strlen(SIGCHILD_MSG));
    return;
};

void file_tests();
void gcode_tests();
void fifo_tests();
void marlinbuf_tests();

File out("stdout", 1);

int main()
{
    out.writeAll(WELCOME_MSG, strlen(WELCOME_MSG));

    gcode_tests();
    marlinbuf_tests();
    file_tests();
    fifo_tests();

    out.writeAll(BYE_MSG, strlen(BYE_MSG));
};

void gcode_tests()
{
    gcode::Line empty;
    assert(empty.length() == 0);
    assert(empty.data()[0] == 0);

    const char* TESTSTR = "   -6.0";
    gcode::Line str(TESTSTR);
    assert(str.length() == (int)strlen(TESTSTR));
    assert(strcmp(str.data(), TESTSTR) == 0);
    int valid = -1;
    assert(str.number(&valid) == -6);
    assert(valid == (int)strlen(TESTSTR) - 2);
    str = "0xFF";
    assert(str.length() == 4);
    assert(str.number(&valid, 16) == 255);
    assert(valid == 4);
    str = "011";
    assert(str.number(0, 0) == 9);
    assert(str.number(0, 10) == 11);
    assert(str.number() == 11);
    assert(str.number(0, 16) == 17);

    str = "Match me";
    assert(str.startsWith("Match") == 5);
    assert(str.startsWith("match") == 0);
    assert(str.startsWith("") == 0);
    str = "";
    assert(str.startsWith("") == 0);
    assert(str.startsWith("foo") == 0);
    str = "   Foo    bar    ";
    assert(str.startsWith("Foo") == 0);
    assert(str.startsWith("\bFoo") == 6);
    assert(str.startsWith("\bFoo\bbar") == 13);
    str = "Foobar";
    assert(str.startsWith("\bFoo\bbar") == 0);
    assert(str.startsWith("\bFoobar\b") == 6);
    str = "Foobar ";
    assert(str.startsWith("\bFoobar\b") == 7);

    str = "12345";
    str.slice(-100, -80);
    assert(str.length() == 0);
    assert(str.data()[0] == 0);

    str = "12345";
    str.slice(100, 80);
    assert(str.length() == 0);
    assert(str.data()[0] == 0);

    str = "12345";
    str.slice(1, 1);
    assert(str.length() == 0);
    assert(str.data()[0] == 0);

    str = "12345";
    str.slice(3, 1);
    assert(str.length() == 0);
    assert(str.data()[0] == 0);

    str = "12345";
    str.slice(0);
    assert(str.length() == 5);
    assert(strcmp(str.data(), "12345") == 0);

    str = "12345";
    str.slice(-2);
    assert(str.length() == 2);
    assert(strcmp(str.data(), "45") == 0);

    str = "12345";
    str.slice(2, -2);
    assert(str.length() == 1);
    assert(strcmp(str.data(), "3") == 0);

    File f("test/unit-test.gcode");
    f.open();
    f.setNonBlock(true);
    gcode::Reader reader(f);
    reader.commentChar('\n'); // preserve comments
    assert(reader.hasNext());
    gcode::Line* line = reader.next();
    assert(strcmp(line->data(), "G28;Thisisacomment") == 0);
    reader.whitespaceCompression(2);
    delete line;
    line = reader.next();
    assert(strcmp(line->data(), "G1X2Y3\n") == 0);
    delete line;
    line = reader.next();
    assert(strcmp(line->data(), "\n") == 0);
    delete line;
    reader.whitespaceCompression(1);
    line = reader.next();
    assert(strcmp(line->data(), "; Just a comment with an embedded ; semicolon\n") == 0);
    delete line;
    reader.whitespaceCompression(0);
    line = reader.next();
    assert(strcmp(line->data(), "; Just another   comment\n") == 0);
    delete line;
    line = reader.next();
    assert(line->data()[line->length()] == 0);
    assert(line->data()[line->length() - 1] != '\n'); // overlong line cut in middle so no \n
    delete line;

    f.open();                        // restart reading
    reader.commentChar(';');         // strip comments
    reader.whitespaceCompression(3); // strip all whitespace
    assert(reader.hasNext());
    line = reader.next();
    assert(strcmp(line->data(), "G28") == 0);
    delete line;
    line = reader.next();
    assert(strcmp(line->data(), "G1X2Y3") == 0);
    delete line;
    line = reader.next();
    assert(strcmp(line->data(), "M115") == 0);
    delete line;
    line = reader.next();
    assert(line == 0);
    assert(!reader.hasNext());
}

void marlinbuf_tests()
{
    MarlinBuf buf;
    buf.setBufSize(1000);
    assert(!buf.hasNext());
    assert(!buf.ack());
    assert(!buf.seek(0));
    int maxAppendLen = buf.maxAppendLen();
    assert(maxAppendLen > 100);
    for (int i = 0; i < 98; i++)
    {
        int l1 = buf.maxAppendLen();
        assert(l1 > 10);
        char gc[8];
        snprintf(gc, sizeof(gc), "G%d", i);
        buf.append(gc);
        int l2 = l1 - buf.maxAppendLen();
        assert(l2 >= 7 && l2 <= l1);

        assert(buf.hasNext());
        assert(buf.seek(i));
    }
    assert(buf.maxAppendLen() == 0);

    assert(buf.seek(0));
    assert(!buf.ack());

    int l;
    assert(strcmp(buf.next(&l), "N0G0*9\n") == 0);
    assert(l == 7);
    assert(strcmp(buf.next(&l), "N1G1*9\n") == 0);
    assert(l == 7);
    assert(buf.seek(0));
    assert(strcmp(buf.next(&l), "N0G0*9\n") == 0);
    assert(l == 7);
    assert(strcmp(buf.next(&l), "N1G1*9\n") == 0);
    assert(l == 7);
    assert(buf.maxAppendLen() == 0);
    assert(buf.ack());
    assert(buf.maxAppendLen() > 10);
    assert(!buf.seek(0));
    buf.append("G98");
    assert(buf.maxAppendLen() == 0);
    assert(!buf.seek(0));

    assert(buf.seek(99));
    assert(buf.seek(1));

    for (int i = 1; i <= 98; i++)
        buf.next();

    assert(strcmp(buf.next(), "N99M110N-1*97\n") == 0);

    for (int i = 1; i <= 99; i++)
        assert(buf.ack());

    assert(!buf.hasNext());
    assert(maxAppendLen == buf.maxAppendLen());

    buf.append("   G452   \n\n");
    buf.append("   G452   ; This is a comment");
    buf.append("G452");
    int m = buf.maxAppendLen();
    buf.append("    ");
    assert(m == buf.maxAppendLen());

    assert(strncmp(buf.next() + 2, buf.next() + 2, 5) == 0);
    assert(strcmp(buf.next(), "N2G452*8\n") == 0);
};

void fifo_tests()
{
    FIFO<int> fifi;
    assert(fifi.empty());
    fifi.put(new int(5));
    assert(!fifi.empty());
    assert(fifi.peek() = 5);
    int* ip = fifi.get();
    assert(*ip == 5);
    assert(fifi.empty());
    assert(fifi.get() == 0);
    delete ip;

    ip = new int(777);
    fifi.put(ip);
    fifi.put(new int(666));
    assert(fifi.peek() == 777);
    ip = fifi.get();
    assert(*ip == 777);
    delete ip;
    ip = fifi.get();
    assert(*ip == 666);
    delete ip;
    assert(fifi.empty());
    assert(fifi.get() == 0);
};

void file_tests()
{
    File noperm("/etc/shadow");
    noperm.action("opening");
    assert(!noperm.hasError());
    assert(noperm.errNo() == 0);
    assert(noperm.error()[0] == 0);
    assert(noperm.open(O_RDONLY) == false);
    assert(noperm.hasError());
    assert(noperm.errNo() == EACCES);
    assert(strncmp(noperm.error(), "Error opening /etc/shadow: ", 27) == 0);
    noperm.clearError();
    assert(!noperm.hasError());
    assert(noperm.errNo() == 0);
    assert(noperm.error()[0] == 0);
    assert(!noperm.EndOfFile());

    int pipefd[2];
    assert(pipe(pipefd) == 0);
    File pw("pipe write end", pipefd[1]);
    File pr("pipe read end", pipefd[0]);

    struct sigaction sigact;
    memset(&sigact, 0, sizeof(sigact));
    sigact.sa_handler = sigchild;
    assert(0 == sigaction(SIGCHLD, &sigact, 0));

    /*
      We write a large block to the pipe in blocking mode in the parent process.
      The child waits a bit, then signals the parent which interrupts the
      parent's write() to test the resume code.
      Then the child reads all the data from the read end to unblock the parent.
    */
    pid_t parentpid = getpid();
    pid_t childpid = fork();
    assert(childpid >= 0);
    if (childpid == 0)
    {
        sleep(2);
        kill(parentpid, SIGCHLD);
        sleep(1);
        assert(pr.read(bigblock, sizeof(bigblock)) == sizeof(bigblock));
        exit(1);
    };

    out.writeAll(RESUME_MSG1, strlen(RESUME_MSG1));
    const void* bufrest = 0;
    size_t nrest = 9999;
    assert(pw.writeAll(bigblock, sizeof(bigblock), &nrest, &bufrest));
    assert(nrest == 0);
    assert(bufrest == (const void*)&bigblock[sizeof(bigblock)]);
    assert(!pw.hasError());

    assert(pw.setNonBlock(true));
    assert(!pw.writeAll(bigblock, sizeof(bigblock), &nrest, &bufrest));
    assert(nrest > 0 && nrest < sizeof(bigblock));
    assert(bufrest == (const void*)&bigblock[sizeof(bigblock) - nrest]);
    assert(pw.hasError());
    assert(pw.errNo() == EWOULDBLOCK);
    pw.clearError();
    int pre_fill_size = sizeof(bigblock) - nrest;

    assert(pw.setNonBlock(false));
    // The next writeAll will block because the buffer is full. We will
    // interrupt the blocked call before anything was written by a signal
    // from a child process. This causes EINTR.
    out.writeAll(RESUME_MSG2, strlen(RESUME_MSG2));

    childpid = fork();
    assert(childpid >= 0);
    if (childpid == 0)
    {
        sleep(2);
        kill(parentpid, SIGCHLD);
        sleep(1);
        assert(pr.read(bigblock, pre_fill_size) == pre_fill_size);
        pr.setNonBlock(true);
        assert(pr.read(bigblock, sizeof(bigblock), 1000, 2000) == sizeof(bigblock));
        exit(1);
    };

    assert(pw.writeAll(bigblock, sizeof(bigblock), &nrest, &bufrest));
    assert(!pw.hasError());
    assert(nrest == 0);
    assert(bufrest == (const void*)&bigblock[sizeof(bigblock)]);

    childpid = fork();
    assert(childpid >= 0);
    if (childpid == 0)
    {
        const char* seq = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";

        struct timespec waittime
        {
            0, 100000000
        };

        const char* p = seq;
        for (int i = 0; i < 10; i++)
        {
            pw.writeAll(p, 5);
            p += 5;
            nanosleep(&waittime, 0);
        }
        exit(1);
    };

    char buffy[4];

    assert(pr.tail(buffy, sizeof(buffy), 200, 420) == sizeof(buffy));
    assert(strncmp(buffy, "lmno", sizeof(buffy)) == 0);
    assert(!pr.EndOfFile());

    struct stat statbuf;

    File fi("test/sequence");
    statbuf.st_mode = 0;
    assert(fi.stat(&statbuf));
    assert(S_ISREG(statbuf.st_mode));
    assert(fi.open());
    statbuf.st_mode = 0;
    assert(fi.stat(&statbuf));
    assert(S_ISREG(statbuf.st_mode));
    char filebuf[1024];
    assert(fi.read(filebuf, sizeof(filebuf)) == 62);
    assert(fi.EndOfFile());

    void* illptr = (void*)LONG_LONG_MAX;
    assert(fi.open());
    assert(!fi.EndOfFile());
    assert(fi.read(illptr, 1) < 0);
    assert(fi.errNo() == EFAULT);
    assert(fi.close());
    statbuf.st_mode = 0;
    assert(fi.stat(&statbuf));
    assert(S_ISREG(statbuf.st_mode));

    assert(pr.setNonBlock(true));
    pr.read(bigblock, sizeof(bigblock), 200); // drain the pipe
    pr.clearError();
    assert(pr.read(bigblock, sizeof(bigblock)) < 0);
    assert(pr.hasError());
    assert(pr.errNo() == EWOULDBLOCK);
    pr.clearError();
    assert(pr.tail(bigblock, sizeof(bigblock)) == 0);
    assert(!pr.hasError());
    assert(pr.errNo() == 0);
    assert(pr.setNonBlock(false));
    pr.clearError();
    assert(pr.read(bigblock, sizeof(bigblock), 0, 0) < 0);
    assert(pr.hasError());
    assert(pr.errNo() == EWOULDBLOCK);
    pr.clearError();
    assert(pr.tail(bigblock, sizeof(bigblock), 0, 0) == 0);
    assert(!pr.hasError());
    assert(pr.errNo() == 0);

    File doesnotexist("test/doesnotexist");
    assert(!doesnotexist.stat(&statbuf));
    assert(doesnotexist.errNo() == ENOENT);

    File stdo("__stdout__", 1);
    assert(fi.stat(&statbuf));

    File unlink_test("test/unlinktest");
    assert(!unlink_test.unlink());
    assert(unlink_test.errNo() == ENOENT);
    assert(unlink_test.open(O_RDONLY | O_CREAT));
    assert(unlink_test.stat(&statbuf));
    assert(unlink_test.unlink());
    assert(unlink_test.close());
    assert(!unlink_test.stat(&statbuf));
};
