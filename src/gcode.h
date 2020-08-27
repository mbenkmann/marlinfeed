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

#ifndef GCODE_H
#define GCODE_H

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "file.h"

namespace gcode
{

/* Copied from https://github.com/OctoPrint/OctoPrint/blob/master/src/octoprint/settings.py
At this point we have no special handling for different types of GCode commands, this
may become useful in the future. Of particular interest are the "longRunningCommands", which
are commands that are acknowledged with 'ok' only when they are finished, whereas normally
an 'ok' is sent when the command has been transferred to the planner buffer.

                "longRunningCommands": ["G4", "G28", "G29", "G30", "G32", "M400", "M226", "M600"],
                "pausingCommands": ["M0", "M1", "M25"],
                "emergencyCommands": ["M112", "M108", "M410"],
*/

// A line of GCode.
class Line
{
    // strlen(text)
    int len;

    // line data. Never NULL.
    char* text;

    Line(const Line& orig);
    Line& operator=(const Line& orig);

  public:
    // Creates a new empty Line.
    Line() : len(0) { text = strdup(""); };

    // Creates a Line initialized with the string str.
    // The string is copied.
    // DON'T FORGET TO FREE str IF NECESSARY.
    Line(const char* str)
    {
        len = strlen(str);
        text = strndup(str, len);
    }

    // Destructor.
    ~Line()
    {
        free(text);
        text = 0;
    };

    // Returns the length of the line (not counting the always present 0-terminator).
    int length() const { return len; }

    // Returns a pointer to the internal data storage for the line.
    // Do not free or modify. The pointer may become invalid by the
    // next operation on the line. Do not store it.
    // The only thing you should do with this pointer is copy its contents
    // to some other place.
    // The number of bytes valid at the returned address is length()+1 because
    // the string is always 0-terminated.
    const char* data() const { return text; }

    // Changes the contents of this Line.
    // The string str is copied.
    // DON'T FORGET TO FREE str IF NECESSARY.
    Line& operator=(const char* str)
    {
        len = strlen(str);
        free(text);
        text = strndup(str, len);
        return *this;
    }

    // Converts the beginning of the line as per strtol() and returns the result.
    // If valid is non-null, the number of valid characters parsed is stored.
    long number(int* valid = 0, int base = 10) const
    {
        char* endptr;
        long l = strtol(text, &endptr, base);
        if (valid != 0)
            *valid = endptr - text;
        return l;
    }

    // Searches the line for the 1st occurence of id preceded and followed
    // by a non-alpha character. Then skips over whitespace, ':' and '=' and
    // converts the following characters to double via strtod().
    // If id is not found, base is returned. If id is found and add==true,
    // base is added to the converted number; otherwise the converted number
    // is returned as strtod() produces it.
    double getDouble(const char* id, double base = 0, bool add = false)
    {
        double found = base;
        if (add)
            found = 0;

        int idlen = strlen(id);

        const char* line = data();
        for (;;)
        {
            const char* s = strstr(line, id);
            if (s == 0)
                break;

            if ((s == data() || !isalpha(s[-1])) && !isalpha(s[idlen]))
            {
                s += idlen;
                while (s[0] != 0 && (isspace(s[0]) || s[0] == ':' || s[0] == '='))
                    ++s;
                found = strtod(s, 0);
                break;
            }
            else
            {
                line++;
            }
        }

        if (add)
            found += base;
        return found;
    }

    // Searches the line for the 1st occurence of id preceded and followed
    // by a non-alpha character. Then skips over whitespace, ':' and '=' and
    // extracts a string surrounded by single or double quotes.
    // If id is not found or there is no pair of quotes where expected,
    // default_ is returned.
    // If non-null, the returned string is always a copy allocated with strdup(),
    // so you need to free() it when done.
    char* getString(const char* id, const char* default_ = 0)
    {
        char* found = 0;

        int idlen = strlen(id);

        const char* line = data();
        for (;;)
        {
            const char* location = strstr(line, id);
            if (location == 0)
                break;

            const char* s = location;

            if ((s == data() || !isalpha(s[-1])) && !isalpha(s[idlen]))
            {
                s += idlen;
                while (s[0] != 0 && (isspace(s[0]) || s[0] == ':' || s[0] == '='))
                    ++s;
                if (s[0] == '"' || s[0] == '\'')
                {
                    const char* endquote = strchr(s + 1, s[0]);
                    if (endquote != 0)
                    {
                        found = strndup(s + 1, (endquote - s) - 1);
                        break;
                    }
                }
            }
            line = location + 1;
        }

        if (found == 0 && default_ != 0)
            found = strdup(default_);
        return found;
    }

    // Returns 0 if the line does not start with prefix; otherwise
    // returns the length of the matched prefix (which is strlen(prefix), unless
    // you use any of the below special characters).
    //
    // The following characters have a special meaning in prefix:
    //   \b (backspace)  matches a word-boundary, i.e. either before or after
    //                   the \b position has to be something other than
    //                   isalnum().
    //                   Start of string and end of string are boundaries.
    //                   If the boundary includes a sequence of whitespace,
    //                   its length will be included in the return value.
    int startsWith(const char* prefix) const
    {
        int equal = 0;
        int remain = len;
        const char* a = text;
        const char* b = prefix;

        for (;;)
        {
            if (*b == '\b')
            {
                b++;
                if (equal == 0)
                {
                    // start of string => OK
                    goto count_whitespace;
                }
                else if (remain == 0)
                {
                    // end of string => OK
                }
                else if (isalnum(*a) == 0 || isalnum(*(a - 1)) == 0)
                {
                count_whitespace:
                    // skip and count whitespace
                    while (remain > 0 && isspace(*a))
                    {
                        ++a;
                        ++equal;
                        --remain;
                    }
                }
                else
                    break;
            }
            else if (*b == 0)
            {
                return equal;
            }
            else
            {
                if (remain > 0 && *b == *a)
                {
                    ++equal;
                    ++b;
                    ++a;
                    --remain;
                }
                else
                    break;
            }
        }

        return 0;
    }

    // Replaces the line with the slice between characters at index idx1 (incl.) and
    // index idx2 (excl.).
    //
    // NOTES:
    //  * The first character has index 0.
    //  * Indexes are allowed to be larger than the line length.
    //  * Negative values for idx1 and idx2 are translated by adding the line length,
    //    i.e. -1 translates to the index of the last character in the line.
    //  * If translation of a negative value still results in a negative value it is
    //    considered 0.
    //  * If position idx2 is at or before idx1 (after translation of negative values)
    //    the result is the empty string.
    void slice(int idx1, int idx2 = INT_MAX)
    {
        if (idx1 < 0)
            if ((idx1 += len) < 0)
                idx1 = 0;
        if (idx2 < 0)
            if ((idx2 += len) < 0)
                idx2 = 0;
        if (idx1 > len)
            idx1 = len;
        if (idx2 > len)
            idx2 = len;

        if (idx2 <= idx1)
            len = 0;
        else
        {
            len = idx2 - idx1;
            memmove(&text[0], &text[idx1], len);
        }
        text[len] = 0;
    }
};

// A buffered wrapper around a File that extracts and preps GCODE lines.
class Reader
{
    // Size of the buffer for reading and preparing the next GCODE line.
    // If a line exceeds this size (e.g. too much whitespace), it will be split
    // which will probably create at least 1 illegal line.
    static const int BUFSIZE = 1024;

    File& in;

    char buf[BUFSIZE + 1]; // +1 for 0 terminator

    // Size of buffer for Slicer comments that we parse.
    static const int COMMENT_BUFSIZE = 64;

    // Slicer comment to be parsed.
    char combuf[64];

    // index in combuf[]
    int comidx;

    // index of the next empty spot in buf, i.e. where reading will continue.
    // always <= BUFSIZE, meaning buf[bufidx] is not out of bounds.
    int bufidx;

    // If > 0, buf[0:ready] is a complete GCODE line, without 0 terminator
    // (because buf[ready] is potentially the 1st character of the next line)
    int ready;

    // see whitespaceCompression()
    int wsComp;

    // Set to true after next() to tell tryRead() that it needs to scan
    // from the very beginning of the buffer.
    bool full_scan;

    // Everything after this character up to the next \n is discarded.
    // Setting this to \n preserves comments.
    char comment = ';';

    // Set to true while bufidx is within a comment block
    bool in_comment;

    // Total number of bytes received from the underlying file.
    int64_t bytesRead;

    // Print time extracted from slicer comments; 0 if not parsed (yet)
    int printTime;

    // Try to extract information from a slicer comment.
    void parseComment()
    {
        combuf[comidx] = 0;
        if (strncmp("TIME:", combuf, 5) == 0)
        {
            long l = strtol(combuf + 5, 0, 10);
            if (l > 0 && l < 8640000)
            {
                printTime = l;
            }
        }
        comidx = 0;
    }

    // DO NOT CALL if ready > 0!
    void tryRead()
    {
        for (;;)
        {
            int retval = 0;
            if (!full_scan)
                retval = in.read(buf + bufidx, BUFSIZE - bufidx);

            if (retval > 0 || full_scan)
            {
                int i = bufidx;
                if (full_scan)
                {
                    in_comment = false;
                    i = 0;
                    if (retval < 0)
                        retval = 0;
                }
                full_scan = false;

                bytesRead += retval;
                bufidx += retval;
                for (int k = i; k < bufidx;)
                {
                    char ch = buf[k++];

                    if (ch == '\n')
                    {
                        if (in_comment)
                            parseComment();

                        in_comment = false;
                        if (wsComp == 1 && i > 0 && buf[i - 1] == ' ')
                            --i;
                        if (wsComp < 3)
                            buf[i++] = ch;

                        if (i == 0) // empty line (not even a terminating \n)
                            continue;

                        ready = i;
                        memmove(buf + i, buf + k, bufidx - k);
                        i += bufidx - k;
                        break;
                    }

                    if (in_comment || ch == comment)
                    {
                        if (in_comment && comidx < COMMENT_BUFSIZE - 1) // -1 for 0 terminator
                            combuf[comidx++] = ch;
                        else
                            comidx = 0;
                        in_comment = true;
                        continue;
                    }

                    if (wsComp <= 0 || !isspace(ch))
                        buf[i++] = ch;
                    else
                    {
                        if (wsComp == 1 && i > 0)
                        {
                            if (buf[i - 1] != ' ')
                                buf[i++] = ' ';
                        }
                    }
                }

                bufidx = i;

                if (ready != 0)
                    break;
            }

            if (bufidx == BUFSIZE || in.EndOfFile() || in.hasError())
            {
                if (in.errNo() == EWOULDBLOCK)
                    in.clearError();
                else
                    ready = bufidx;

                break;
            }
        }
    };

  public:
    // Wraps a Reader around in. If the file is in blocking mode, hasNext() will block
    // until data is available.
    // in has to be open already.
    Reader(File& _in)
        : in(_in), comidx(0), bufidx(0), ready(0), wsComp(3), full_scan(false), comment(';'), in_comment(false),
          bytesRead(0), printTime(0){};

    // Discard all data currently buffered by the reader. The next attempt to
    // read will start a new line at whatever file position the underlying
    // file is at.
    // Returns the number of bytes discarded.
    int discard()
    {
        int i = bufidx;
        comidx = 0;
        bufidx = 0;
        ready = 0;
        full_scan = false;
        in_comment = false;
        return i;
    }

    // Extracts up to sz bytes of buffered data from this reader and stores them
    // into dest. Returns number of bytes extracted.
    // If raw() returns 0, the state is as if discard() had been called.
    // raw() can be called repeatedly until it returns 0 to extract all
    // buffered data.
    int raw(char* dest, int sz)
    {
        if (sz > bufidx)
            sz = bufidx;
        memcpy(dest, buf, sz);
        memmove(buf, buf + sz, bufidx - sz);
        bufidx -= sz;
        comidx = 0;
        ready = 0;
        full_scan = (bufidx > 0);
        in_comment = false;
        return sz;
    }

    // Sets the whitespace compression level. See also commentChar().
    // 0: keep all whitespace
    // 1: convert sequences of whitespace to a single space
    //    strip whitespace at the beginning and end of line (except for a single \n)
    // 2: remove all whitespace except for a single \n at line end.
    // 3(default): remove all whitespace
    void whitespaceCompression(int level) { wsComp = level; };

    // Sets the character starting a comment to be stripped. Everything starting
    // with this character up to the next \n is stripped. Set this to '\n' to
    // preserve comments.
    // Default is ';'.
    void commentChar(char ch) { comment = ch; }

    // Returns the number of bytes read from the underlying file (regardless of
    // whether or not they have been stripped as whitespace and regardless of
    // whether they have been extracted via next()).
    int64_t totalBytesRead() { return bytesRead; }

    // Returns the estimated print time as parsed from slicer comments; or 0 if
    // no such comment has been parsed yet.
    int estimatedPrintTime() { return printTime; }

    // Returns true if a complete line of GCODE has been read and is ready for
    // extraction via next(). If a line is not already available when hasNext()
    // is called, it will first try to read more data from the input source.
    // An EWOULDBLOCK error encountered during a read attempt will be cleared
    // right away, so you will never see it when testing the in File for an error.
    // Other errors will be kept, so you can test the in File to see if everything
    // is still working.
    // You probably don't want to use hasNext() if the underlying file is in
    // blocking mode.
    bool hasNext()
    {
        if (ready == 0)
            tryRead();
        return ready > 0;
    };

    // See hasNext(). If hasNext()==false, null is returned.
    // Like hasNext() this function tries to read more data if necessary.
    // You own the return value and must use delete to free it.
    Line* next()
    {
        if (!hasNext())
            return 0;

        char saved_char = buf[ready];
        buf[ready] = 0;
        Line* ret = new Line(buf);
        buf[ready] = saved_char;
        memmove(buf, buf + ready, bufidx - ready);
        bufidx -= ready;
        ready = 0;
        full_scan = true;
        return ret;
    };
};

}; // namespace gcode

#endif
