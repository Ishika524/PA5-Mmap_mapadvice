#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

#define AS_LIMIT (1U << 23)
#define MAX_SQRTS (1U << 23)
#define BACKING_FILE "sqrts.bin"
#define ACCESSES 500000
#define TABLE_BYTES (MAX_SQRTS * sizeof(double))

#ifndef CHUNK_PAGES
#define CHUNK_PAGES 16
#endif

#ifndef RESIDENT_CHUNKS
#define RESIDENT_CHUNKS 4
#endif

enum mode { BASELINE, SEQUENTIAL, RANDOM, WILLNEED };
enum workload { MIXED, RANDOM_ONLY, SEQUENTIAL_ONLY };

static enum mode mode;
static enum workload workload;
static int backing_fd;
_Static_assert(CHUNK_PAGES > 0, "CHUNK_PAGES must be positive");
_Static_assert(RESIDENT_CHUNKS > 0, "RESIDENT_CHUNKS must be positive");

static size_t chunk_bytes;
static double *sqrts;
static uintptr_t fifo[RESIDENT_CHUNKS];
static size_t oldest;
static size_t count;
static volatile sig_atomic_t sequential_access;
static volatile sig_atomic_t random_faults;
static volatile sig_atomic_t sequential_faults;

#define HANDLER_FAIL(message) do {                         \
    static const char diagnostic[] = message "\n";         \
    (void)write(STDERR_FILENO, diagnostic,                  \
                sizeof diagnostic - 1);                    \
    _exit(EXIT_FAILURE);                                   \
} while (0)

static void fail(const char *operation)
{
    perror(operation);
    exit(EXIT_FAILURE);
}

static void generate_file(void)
{
    double values[4096];
    int fd = open(BACKING_FILE, O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (fd == -1)
        fail("create sqrts.bin");

    for (size_t first = 0; first < MAX_SQRTS; first += 4096) {
        for (size_t i = 0; i < 4096; i++)
            values[i] = sqrt((double)(first + i));

        const char *data = (const char *)values;
        size_t remaining = sizeof values;
        while (remaining > 0) {
            ssize_t written = write(fd, data, remaining);
            if (written == -1 && errno == EINTR)
                continue;
            if (written <= 0) {
                if (written == 0)
                    errno = EIO;
                fail("write sqrts.bin");
            }
            data += written;
            remaining -= (size_t)written;
        }
    }

    if (fsync(fd) == -1 || close(fd) == -1)
        fail("finish sqrts.bin");
    puts("Created 64 MiB sqrts.bin");
}

static size_t mapping_length(uintptr_t chunk_addr)
{
    size_t remaining = TABLE_BYTES - (chunk_addr - (uintptr_t)sqrts);
    return remaining < chunk_bytes ? remaining : chunk_bytes;
}
