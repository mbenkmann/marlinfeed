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

#ifndef MARLINBUF_H
#define MARLINBUF_H

#include <assert.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// A buffer for GCode commands to be sent to Marlin. Performs the following
// functions:
//  * line numbering and checksumming
//  * keeps track of which lines are acknowledged by 'ok'
//  * rewind to an already sent (but not ack'd line) for Resend support
//  * keep track of the serial buffer fill state to prevent overflowing it
class MarlinBuf
{
    // Size of the serial port transfer buffer. This is the limiting factor,
    // because pushing more than this will cause data loss and force resends.
    // The buffers Marlin manages internally (e.g. planner buffer) are less
    // relevant because filling them does not cause data loss, because if they are
    // full Marlin will stop sending 'ok' and stop reading from the serial connection
    // with no data loss (as long as the serial buffer is not overflowed).
    // 128 is the buffer size of the FTDI FT232R USB->UART interface chip
    // commonly used on 8-bit boards.
    int buf_size = 128;

    // Each line in the buffer is 1 GCODE command prefixed by "Nxx" where
    // xx is the line number (0 to 99), and suffixed by "*chk" where chk is
    // the checksum as per Marlin protocol (i.e. the XOR of all bytes preceding
    // the "*", including the Nxx). Each line ends in '\n'.
    // line[99] is always "N99M110N-1*97\n" so that line numbers roll around to 0
    // after 99.
    // When line 98 is added to the buffer, line 99 is automatically added after it.
    char* line[100];

    // String length (excluding 0-terminator) of corresponding line[].
    int lineLen[100];

    // The next line appended to the buffer will become line[i_in].
    int i_in = 0;

    // line[i_out] is the next line to be transmitted over the wire.
    // If i_out == i_in, nothing is queued for transmission.
    int i_out = 0;

    // The next buffer entry to have its memory freed is line[i_free].
    // Note that this trails behind i_out, because lines get freed when
    // Marlin ACKs them with "ok".
    int i_free = 0;

    // The sum of line lengths of unACK'd lines in the buffer.
    int sz = 0;

  public:
    static const char* const WRAP_AROUND_STRING;
    static const int WRAP_AROUND_STRING_LENGTH = 14;

    MarlinBuf()
    {
        line[99] = strdup(WRAP_AROUND_STRING);
        lineLen[99] = WRAP_AROUND_STRING_LENGTH;

        // Pre-fill line[] with line numbers.
        // We never free the memory, only use realloc, so the line
        // numbers stay forever.
        for (int i = 0; i < 99; i++)
        {
            lineLen[i] = 0;
            line[i] = (char*)malloc(3);
            line[i][0] = 'N';
            if (i < 10)
            {
                line[i][1] = '0' + i;
            }
            else
            {
                line[i][1] = '0' + i / 10;
                line[i][2] = '0' + (i % 10);
            }
        }
    }

    // Changes the size of the assumed Marlin buffer. This will affect future calls
    // to maxAppendLen(). If you reduce the buffer size below what's currently
    // stored, maxAppendLen() will return a negative value, because changing the
    // buffer size does not actually remove anything from the buffer.
    // NOTE: Independent of the buffer size there is a fixed upper limit of 99 lines
    // that can be stored in the buffer.
    void setBufSize(int new_buf_size) { buf_size = new_buf_size; }

    // Returns the maximum length of GCODE command that still fits in the buffer.
    // Takes into account the line number, checksum and '\n' that will be added
    // as well as a potential line number wrap-around.
    // NOTE: Independent of the free space in the buffer there is a fixed upper limit
    // of 98 lines that can be stored in the buffer (the 99th line is auto-generated to
    // roll around the line numbers from 99 to 0). Once that limit is reached,
    // maxAppendLen() will return 0.
    int maxAppendLen()
    {
        // Return 0 if all line spots are taken, regardless of size.
        // The number is 99, not 100, because of the special line[99].
        if ((i_in + 1) % 99 == i_free)
            return 0;

        int remain = buf_size - sz;
        if (i_in < 10)
            remain -= 2; // Nx
        else
            remain -= 3; // Nxx

        if (i_in == 98)
            remain -= WRAP_AROUND_STRING_LENGTH;

        remain -= 4; // *chk
        remain--;    // \n

        return remain;
    }

    // Appends gcode to the buffer. gcode must not include line number or checksum.
    // These will be added automatically. A comment will be stripped.
    // A trailing \n is optional and will be added if missing.
    // Make sure you check maxAppendLen() first and don't forget to free
    // the memory of gcode (because this function creates its own copy).
    // If gcode is the empty string (after stripping whitespace) nothing is done.
    void append(const char* gcode)
    {
        // strip leading whitespace
        while (isspace(*gcode))
            gcode++;

        int len = 0;
        int chk = line[i_in][0] ^ line[i_in][1];

        int N_len = 2; // Nx
        if (i_in >= 10)
        {
            N_len++; // Nxx
            chk ^= line[i_in][2];
        }

        const char* p = gcode;

        while (*p != 0 && *p != ';')
        {
            chk ^= *p;
            len++;
            p++;
        }

        // strip trailing whitespace
        while (p != gcode && isspace(p[-1]))
        {
            p--;
            len--;
            chk ^= *p;
        }

        if (len == 0)
            return;

        char lend[6]; // line end: * checksum \n 0
        int endlen = 0;
        lend[endlen++] = '*';
        if (chk < 10)
        {
            lend[endlen++] = chk + '0';
        }
        else if (chk < 100)
        {
            lend[endlen++] = chk / 10 + '0';
            lend[endlen++] = chk % 10 + '0';
        }
        else
        {
            lend[endlen++] = chk / 100 + '0';
            chk = chk % 100;
            lend[endlen++] = chk / 10 + '0';
            lend[endlen++] = chk % 10 + '0';
        }
        lend[endlen++] = '\n';
        lend[endlen++] = 0;

        line[i_in] = (char*)realloc(line[i_in], N_len + len + endlen);
        memcpy(line[i_in] + N_len, gcode, len);
        memcpy(line[i_in] + N_len + len, lend, endlen);

        lineLen[i_in] = N_len + len + endlen - 1; // -1 because we don't count the 0 terminator
        sz += lineLen[i_in];
        i_in++;

        // if we just appended line 98, automatically append the wraparound M110
        if (i_in == 99)
        {
            i_in = 0;
            sz += WRAP_AROUND_STRING_LENGTH;
        }

        assert(i_in != i_free);
        assert(i_in != i_out);
        assert(sz <= buf_size);
    }

    // Returns true if there is a line to be sent over the wire.
    bool hasNext() { return i_out != i_in; }

    // Returns true if there is still a line that has been sent but not ack()d.
    bool needsAck() { return i_free != i_out; }

    // Returns the next line to be sent over the wire.
    // If the pointer len is passed as non-null, the length of the
    // returned string will be stored there (including the \n but excluding
    // the 0 terminator).
    // ATTENTION! DO NOT FREE THE RETURNED POINTER. It belongs to MarlinBuf.
    const char* next(int* len = 0)
    {
        assert(hasNext());
        const char* p = line[i_out];
        if (len != 0)
            *len = lineLen[i_out];
        if (++i_out == 100)
            i_out = 0;

        return p;
    }

    // Remove the oldest line from the buffer.
    // Must not be called before the line has been retrieved with next().
    // Returns false if called in an illegal situation, i.e. there is no line
    // to be ack'd.
    bool ack()
    {
        if (i_free == i_out)
            return false;
        sz -= lineLen[i_free];
        assert(sz >= 0);
        if (++i_free == 100)
            i_free = 0;
        return true;
    }

    // Makes line l the next line to be returned by next().
    // The line must actually be in the buffer and not have been ack()d, yet.
    // Returns false if l is not a valid line to seek to.
    bool seek(int l)
    {
        // buffer empty
        if (i_free == i_in)
            return false;

        if (i_free < i_in)
        {
            if (l < i_free || l >= i_in)
                return false;
        }
        else // i_in < i_free
        {
            if (l >= 100 || l < 0 || (l < i_free && l >= i_in))
                return false;
        }
        i_out = l;
        return true;
    }
};

const char* const MarlinBuf::WRAP_AROUND_STRING = "N99M110N-1*97\n";

#endif
