#ifndef DIRSCANNER_H
#define DIRSCANNER_H

#include <assert.h>
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "fifo.h"

// Scans and watches directories for new files.
class DirScanner
{
    struct StringChecker
    {
        bool found = false;
        const char* cmp;
        StringChecker(const char* s) : cmp(s){};
        operator bool() { return found; }
        bool operator()(char* s)
        {
            found = found || (0 == strcmp(cmp, s));
            return !found;
        }
    };

    // If the 1st character of an entry is 0, it's a one-shot entry that
    // is scanned and then removed. Otherwise the directory is scanned
    // and then requeued.
    FIFO<char> dirs;

    // Paths of files found during the last scan. They will not be used
    // until their modification time is at least MIN_AGE milliseconds in the
    // past. This makes sure that files that are still being written to
    // won't be used until their contents are complete.
    FIFO<char> candidates;

    int64_t last_scan = 0;

    // WARNING! Discards the nanosecond part because I observed issues of the
    // modification time appearing to lie before the last scan even if the
    // actual touch was afterwards. I blame a lower resolution of the mtime
    // compared to the realtime clock.
    int64_t nano(const timespec& tp) { return (int64_t)tp.tv_sec * 1000000000 /*+ (int64_t)tp.tv_nsec*/; }

    int64_t now()
    {
        struct timespec tp;
        if (0 > clock_gettime(CLOCK_REALTIME, &tp))
            perror("clock_gettime");
        return nano(tp);
    }

    // Returns true iff fpath can be statted and its last modification time is
    // at least MIN_AGE milliseconds before now.
    bool ripe(const char* fpath)
    {
        struct stat statbuf;
        if (0 <= stat(fpath, &statbuf))
        {
            return (nano(statbuf.st_mtim) + 1000000 * MIN_AGE <= now());
        }
        return false;
    }

    // Scans directories for files that have been changed since the last scan
    // and adds them to candidates.
    void scan()
    {
        int64_t last = last_scan;
        last_scan = now();
        if (last == last_scan) // prevent discovering the same files again
            return;
        int64_t cur = last_scan;

        for (int i = dirs.size(); i > 0; i--)
        {
            char* dir = dirs.get();
            int once = dir[0] == 0 ? 1 : 0;

            DIR* dp = opendir(dir + once);

            if (dp == 0)
            {
                perror(dir + once);
                once = 1;
            }
            else
            {
                int dfd = dirfd(dp);
                assert(dfd >= 0);
                errno = 0;
                for (;;)
                {
                    auto f = readdir(dp);
                    if (f == 0)
                        break;
                    struct stat statbuf;

                    if (0 > fstatat(dfd, f->d_name, &statbuf, 0))
                        break;

                    if (!S_ISREG(statbuf.st_mode))
                        continue;
                    int64_t ftim = nano(statbuf.st_mtim);
                    if (ftim < last || ftim >= cur)
                        continue;
                    char* fpath;
                    assert(0 < asprintf(&fpath, "%s/%s", dir + once, f->d_name));
                    StringChecker found(fpath);
                    if (candidates.visit(found))
                        free(fpath);
                    else
                        candidates.put(fpath);
                }

                if (errno != 0)
                    perror("readdir");
                closedir(dp);
            }

            if (once)
                free(dir);
            else
                dirs.put(dir);
        }
    }

  public:
    // minimum time in milliseconds that has to pass since the last modification
    // for a file to be considered ripe for being reported by refill().
    static const int MIN_AGE = 2000;

    // Adds directory dpath to the list of directories to scan.
    // If once, the directory will be scanned only once, otherwise it will
    // be watched for changes.
    // NOTE: The string dpath is copied.
    // WARNING! once==true is not useful at this time because files are not
    // sorted.
    void addDir(const char* dpath, bool once = false)
    {
        if (dpath[0] == 0)
            return; // ignore empty string

        char* st = (char*)malloc(strlen(dpath) + 2);
        if (once)
        {
            strcpy(st + 1, dpath);
            st[0] = 0;
        }
        else
            strcpy(st, dpath);

        dirs.put(st);
    }

    // Returns true iff there is no chance for refill() to produce
    // additional entries. If this returns false, it is still not
    // guaranteed that refill() will produce entries.
    bool empty() { return candidates.empty() && dirs.empty(); }

    // Adds "ripe" files from the watched directories to files. A file is ripe
    // if it has been last modified a few seconds ago.
    // Paths added have been alloc'd by malloc and must be freed with free().
    void refill(FIFO<char>& files)
    {
        scan();

        // If we have candidates, check if they have aged enough and
        // use those that have.
        for (int i = candidates.size(); i > 0; --i)
        {
            char* can = candidates.get();
            if (ripe(can))
                files.put(can);
            else
                candidates.put(can);
        }
    }
};

#endif
