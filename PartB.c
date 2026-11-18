#include <errno.h>
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
#define MAX_SQRTS (1U << 27)
#define TABLE_BYTES (MAX_SQRTS * sizeof(double))

#ifndef CHUNK_PAGES
#define CHUNK_PAGES 1
#endif

#ifndef RESIDENT_CHUNKS
#define RESIDENT_CHUNKS 1
#endif

_Static_assert(CHUNK_PAGES > 0, "CHUNK_PAGES must be positive");
_Static_assert(RESIDENT_CHUNKS > 0, "RESIDENT_CHUNKS must be positive");

static size_t chunk_bytes;
static double *sqrts;
static uintptr_t fifo[RESIDENT_CHUNKS];
static size_t oldest;
static size_t count;
static volatile sig_atomic_t fault_count;

#define HANDLER_FAIL(message) do {                         \
    static const char diagnostic[] = message "\n";         \
    (void)write(STDERR_FILENO, diagnostic,                  \
                sizeof diagnostic - 1);                    \
    _exit(EXIT_FAILURE);                                   \
} while (0)

static void calculate_sqrts(double *dest, size_t first, size_t entries)
{
    for (size_t i = 0; i < entries; i++)
        dest[i] = sqrt((double)(first + i));
}

static size_t mapping_length(uintptr_t chunk_addr)
{
    size_t remaining = TABLE_BYTES - (chunk_addr - (uintptr_t)sqrts);
    return remaining < chunk_bytes ? remaining : chunk_bytes;
}

static void handle_sigsegv(int sig, siginfo_t *si, void *ctx)
{
    (void)sig;
    (void)ctx;

    int saved_errno = errno;
    uintptr_t addr = (uintptr_t)si->si_addr;
    uintptr_t base = (uintptr_t)sqrts;

    if (addr < base || addr - base >= TABLE_BYTES ||
        si->si_code != SEGV_MAPERR)
        HANDLER_FAIL("Unexpected SIGSEGV");

    fault_count++;
    size_t offset = addr - base;
    uintptr_t chunk_addr = base + (offset / chunk_bytes) * chunk_bytes;

    if (count == RESIDENT_CHUNKS) {
        uintptr_t victim = fifo[oldest];
        if (munmap((void *)victim, mapping_length(victim)) == -1)
            HANDLER_FAIL("Could not unmap the oldest chunk");

        oldest = (oldest + 1) % RESIDENT_CHUNKS;
        count--;
    }

    size_t len = mapping_length(chunk_addr);
    void *chunk = mmap((void *)chunk_addr, len, PROT_READ | PROT_WRITE,
                       MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (chunk == MAP_FAILED) {
        if (errno == ENOMEM)
            HANDLER_FAIL("mmap failed with ENOMEM under RLIMIT_AS");
        HANDLER_FAIL("Could not map the faulting chunk");
    }

    size_t first_index = (chunk_addr - base) / sizeof(double);
    calculate_sqrts(chunk, first_index, len / sizeof(double));

    fifo[(oldest + count) % RESIDENT_CHUNKS] = chunk_addr;
    count++;
    errno = saved_errno;
}

static void setup_sqrt_region(void)
{
    size_t probe_size = TABLE_BYTES + AS_LIMIT;
    struct rlimit lim = { AS_LIMIT, AS_LIMIT };
    struct sigaction act = {0};

    sqrts = mmap(NULL, probe_size, PROT_NONE,
                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (sqrts == MAP_FAILED) {
        perror("mmap table region");
        exit(EXIT_FAILURE);
    }

    if (munmap(sqrts, probe_size) == -1) {
        perror("munmap table region");
        exit(EXIT_FAILURE);
    }

    if (setrlimit(RLIMIT_AS, &lim) == -1) {
        perror("setrlimit");
        exit(EXIT_FAILURE);
    }

    act.sa_sigaction = handle_sigsegv;
    act.sa_flags = SA_SIGINFO;
    if (sigemptyset(&act.sa_mask) == -1 ||
        sigaction(SIGSEGV, &act, NULL) == -1) {
        perror("install SIGSEGV handler");
        exit(EXIT_FAILURE);
    }
}

static void test_sqrt_region(void)
{
    size_t pos = 0;
    srand(0xDEADBEEF);

    for (int i = 0; i < 500000; i++) {
        if (i % 2 == 0)
            pos = rand() % (MAX_SQRTS - 1);
        else
            pos++;

        double expected;
        calculate_sqrts(&expected, pos, 1);
        double actual = sqrts[pos];

        if (actual != expected) {
            fprintf(stderr, "Index %zu: expected %f, got %f\n",
                    pos, expected, actual);
            exit(EXIT_FAILURE);
        }
    }
}

int main(void)
{
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0 || TABLE_BYTES % (size_t)page_size != 0 ||
        (size_t)CHUNK_PAGES > TABLE_BYTES / (size_t)page_size) {
        fprintf(stderr, "Invalid hardware page or chunk size\n");
        return EXIT_FAILURE;
    }

    chunk_bytes = (size_t)CHUNK_PAGES * (size_t)page_size;
    setup_sqrt_region();

    struct timespec start, end;
    struct rusage usage;

    printf("Validating square root table contents...\n");
    if (clock_gettime(CLOCK_MONOTONIC, &start) == -1) {
        perror("clock_gettime start");
        return EXIT_FAILURE;
    }

    test_sqrt_region();

    if (clock_gettime(CLOCK_MONOTONIC, &end) == -1 ||
        getrusage(RUSAGE_SELF, &usage) == -1) {
        perror("collect benchmark measurements");
        return EXIT_FAILURE;
    }

    double seconds = (end.tv_sec - start.tv_sec) +
                     (end.tv_nsec - start.tv_nsec) / 1e9;

    while (count > 0) {
        uintptr_t chunk = fifo[oldest];
        if (munmap((void *)chunk, mapping_length(chunk)) == -1) {
            perror("munmap final chunk");
            return EXIT_FAILURE;
        }
        oldest = (oldest + 1) % RESIDENT_CHUNKS;
        count--;
    }

    printf("Pager faults: %d\n", (int)fault_count);
    printf("Wall time: %.6f s\n", seconds);
    printf("Peak RSS: %ld KiB\n", usage.ru_maxrss);
    printf("Minor faults: %ld, major faults: %ld\n",
           usage.ru_minflt, usage.ru_majflt);
    printf("All tests passed!\n");
    return EXIT_SUCCESS;
}
