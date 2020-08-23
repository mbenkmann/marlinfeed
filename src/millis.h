#ifndef MILLIS_H
#define MILLIS_H

#include <sys/time.h>

// Unix timestamp in milliseconds.
int64_t millis()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (int64_t)tv.tv_sec * 1000 + (tv.tv_usec + 500) / 1000;
}

#endif
