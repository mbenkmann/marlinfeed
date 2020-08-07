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

#ifndef FIFO_H
#define FIFO_H

// A FIFO buffer.
template <typename T> class FIFO
{
    struct Node
    {
        T* obj;
        Node* next;
        Node() : obj(0), next(0){};
        ~Node()
        {
            // We DO NOT delete obj here, because its ownership is transferred on get()
        }
    };

    Node* entry;
    Node* exit;

  public:
    // Creates a new, empty FIFO.
    FIFO() : entry(0), exit(0){};

    // Returns true if the buffer is empty.
    bool empty() { return exit == 0; };

    // Puts obj into the buffer.
    // ATTENTION! The pointer is used directly! Ownership transfers to the FIFO!
    void put(T* obj)
    {
        if (entry == 0)
        {
            entry = new Node();
            exit = entry;
        }
        else
        {
            entry->next = new Node();
            entry = entry->next;
        }
        entry->obj = obj;
    };

    // Removes and returns the oldest object in the buffer or NULL if the buffer is empty.
    // The returned pointer is the same you passed to put() when adding the object.
    // You get ownership of the returned pointer back and have to free it using the
    // appropriate manner matching how you initially created it.
    T* get()
    {
        if (exit == 0)
            return 0;

        Node* n = exit;
        exit = n->next;
        if (exit == 0)
            entry = 0;
        T* ret = n->obj;
        n->obj = 0;
        delete n;
        return ret;
    };

    // Returns a reference to the oldest object in the buffer. This is the actual
    // object that the next call to get() will return, so any modifications you
    // make will be reflected in that.
    // Calling this on an empty buffer will crash the program.
    T& peek() { return *(exit->obj); }
};

#endif
