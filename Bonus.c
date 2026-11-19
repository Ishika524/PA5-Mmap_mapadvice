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
#define MAX_SQRTS (1U << 23)       /* 64 MiB slice of the table */
#define BACKING_FILE "sqrts.bin"
#define ACCESSES 500000
#define TABLE_BYTES (MAX_SQRTS * sizeof(double))

#ifndef CHUNK_PAGES
#define CHUNK_PAGES 16
#endif

#ifndef RESIDENT_CHUNKS
#define RESIDENT_CHUNKS 4
#endif

enum mode { BASELINE, SEQ_HINT, RAND_HINT, WILLNEED };

_Static_assert(CHUNK_PAGES > 0, "CHUNK_PAGES must be positive");
_Static_assert(RESIDENT_CHUNKS > 0, "RESIDENT_CHUNKS must be positive");

static enum mode hint_mode = BASELINE;
static int backing_fd = -1;
static size_t chunk_bytes;
static double *sqrts;
static uintptr_t fifo[RESIDENT_CHUNKS];
static size_t oldest;
static size_t count;
static volatile sig_atomic_t fault_count;
static volatile sig_atomic_t random_faults;
static volatile sig_atomic_t sequential_faults;
static volatile sig_atomic_t sequential_access;

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

/* ------------------------------------------------------------------ */
/*  Generate the backing file with precomputed square roots.           */
/* ------------------------------------------------------------------ */
static void generate_file(void)
{
    struct stat st;
    /* Skip if file already exists with the right size. */
    if (stat(BACKING_FILE, &st) == 0 && (size_t)st.st_size == TABLE_BYTES) {
        puts("sqrts.bin already exists, skipping generation.");
        return;
    }

    double values[4096];
    int fd = open(BACKING_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd == -1)
        fail("create sqrts.bin");

    for (size_t first = 0; first < MAX_SQRTS; first += 4096) {
        size_t batch = MAX_SQRTS - first;
        if (batch > 4096) batch = 4096;

        for (size_t i = 0; i < batch; i++)
            values[i] = sqrt((double)(first + i));

        const char *data = (const char *)values;
        size_t remaining = batch * sizeof(double);
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
    printf("Created %zu MiB sqrts.bin\n", TABLE_BYTES / (1024 * 1024));
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */
static size_t mapping_length(uintptr_t chunk_addr)
{
    size_t remaining = TABLE_BYTES - (chunk_addr - (uintptr_t)sqrts);
    return remaining < chunk_bytes ? remaining : chunk_bytes;
}

/* Check if a chunk is already in the FIFO. */
static int chunk_is_resident(uintptr_t chunk_addr)
{
    for (size_t i = 0; i < count; i++) {
        if (fifo[(oldest + i) % RESIDENT_CHUNKS] == chunk_addr)
            return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Signal handler — maps chunks from the backing file.                */
/* ------------------------------------------------------------------ */
static void handle_sigsegv(int sig, siginfo_t *si, void *ctx)
{
    (void)sig;
    (void)ctx;

    int saved_errno = errno;
    uintptr_t addr = (uintptr_t)si->si_addr;
    uintptr_t base = (uintptr_t)sqrts;

    /* Only handle faults inside the table. */
    if (addr < base || addr - base >= TABLE_BYTES ||
        si->si_code != SEGV_MAPERR)
        HANDLER_FAIL("Unexpected SIGSEGV outside the sqrt table");

    fault_count++;
    if (sequential_access)
        sequential_faults++;
    else
        random_faults++;

    /* Align down to chunk boundary. */
    size_t offset = addr - base;
    uintptr_t chunk_addr = base + (offset / chunk_bytes) * chunk_bytes;

    /* Evict the oldest chunk if the FIFO is full. */
    if (count == RESIDENT_CHUNKS) {
        uintptr_t victim = fifo[oldest];
        if (munmap((void *)victim, mapping_length(victim)) == -1)
            HANDLER_FAIL("Could not unmap the oldest chunk");
        oldest = (oldest + 1) % RESIDENT_CHUNKS;
        count--;
    }

    /* Map the faulting chunk from the backing file. */
    size_t len = mapping_length(chunk_addr);
    off_t file_off = (off_t)(chunk_addr - base);

    void *chunk = mmap((void *)chunk_addr, len, PROT_READ,
                       MAP_PRIVATE | MAP_FIXED, backing_fd, file_off);
    if (chunk == MAP_FAILED) {
        if (errno == ENOMEM)
            HANDLER_FAIL("mmap failed with ENOMEM under RLIMIT_AS");
        HANDLER_FAIL("Could not map the faulting chunk from file");
    }

    /* Apply madvise hint based on selected mode. */
    if (hint_mode == SEQ_HINT)
        madvise(chunk, len, MADV_SEQUENTIAL);
    else if (hint_mode == RAND_HINT)
        madvise(chunk, len, MADV_RANDOM);

    /* Record in FIFO. */
    fifo[(oldest + count) % RESIDENT_CHUNKS] = chunk_addr;
    count++;

    /* WILLNEED mode: prefetch-hint the next chunk. */
    if (hint_mode == WILLNEED &&
        TABLE_BYTES - (chunk_addr - base) > chunk_bytes) {
        uintptr_t next = chunk_addr + chunk_bytes;

        if (!chunk_is_resident(next)) {
            /* Evict if needed to make room for prefetch. */
            if (count == RESIDENT_CHUNKS) {
                uintptr_t victim = fifo[oldest];
                if (munmap((void *)victim, mapping_length(victim)) == -1)
                    HANDLER_FAIL("Could not unmap chunk for prefetch");
                oldest = (oldest + 1) % RESIDENT_CHUNKS;
                count--;
            }

            size_t next_len = mapping_length(next);
            off_t next_off = (off_t)(next - base);

            void *next_map = mmap((void *)next, next_len, PROT_READ,
                                  MAP_PRIVATE | MAP_FIXED,
                                  backing_fd, next_off);
            if (next_map == MAP_FAILED)
                HANDLER_FAIL("Could not map next chunk for prefetch");

            madvise(next_map, next_len, MADV_WILLNEED);

            fifo[(oldest + count) % RESIDENT_CHUNKS] = next;
            count++;
        }
    }

    errno = saved_errno;
}

/* ------------------------------------------------------------------ */
/*  Setup: reserve address space, set rlimit, install handler.         */
/* ------------------------------------------------------------------ */
static void setup_sqrt_region(void)
{
    size_t probe_size = TABLE_BYTES + AS_LIMIT;
    struct rlimit lim = { AS_LIMIT, AS_LIMIT };
    struct sigaction act = {0};

    /* Find an available address range. */
    sqrts = mmap(NULL, probe_size, PROT_NONE,
                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (sqrts == MAP_FAILED) {
        perror("mmap table region");
        exit(EXIT_FAILURE);
    }

    /* Release it before imposing the AS limit. */
    if (munmap(sqrts, probe_size) == -1) {
        perror("munmap table region");
        exit(EXIT_FAILURE);
    }

    /* Enforce the 8 MiB virtual-memory limit. */
    if (setrlimit(RLIMIT_AS, &lim) == -1) {
        perror("setrlimit");
        exit(EXIT_FAILURE);
    }

    /* Install SIGSEGV handler. */
    act.sa_sigaction = handle_sigsegv;
    act.sa_flags = SA_SIGINFO;
    if (sigemptyset(&act.sa_mask) == -1 ||
        sigaction(SIGSEGV, &act, NULL) == -1) {
        perror("install SIGSEGV handler");
        exit(EXIT_FAILURE);
    }
}

/* ------------------------------------------------------------------ */
/*  Validation — same pattern as the main assignment.                  */
/* ------------------------------------------------------------------ */
static void calculate_sqrts(double *dest, size_t first, size_t entries)
{
    for (size_t i = 0; i < entries; i++)
        dest[i] = sqrt((double)(first + i));
}

static void test_sqrt_region(void)
{
    size_t pos = 0;
    srand(0xDEADBEEF);

    for (int i = 0; i < ACCESSES; i++) {
        sequential_access = i % 2;
        if (!sequential_access)
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

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */
int main(int argc, char *argv[])
{
    /* Parse optional mode argument. */
    if (argc > 1) {
        if (strcmp(argv[1], "sequential") == 0)
            hint_mode = SEQ_HINT;
        else if (strcmp(argv[1], "random") == 0)
            hint_mode = RAND_HINT;
        else if (strcmp(argv[1], "willneed") == 0)
            hint_mode = WILLNEED;
        else if (strcmp(argv[1], "baseline") == 0)
            hint_mode = BASELINE;
        else {
            fprintf(stderr,
                    "Usage: %s [baseline|sequential|random|willneed]\n",
                    argv[0]);
            return EXIT_FAILURE;
        }
    }

    static const char *mode_names[] = {
        "baseline", "MADV_SEQUENTIAL", "MADV_RANDOM", "MADV_WILLNEED"
    };

    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0 || TABLE_BYTES % (size_t)page_size != 0 ||
        (size_t)CHUNK_PAGES > TABLE_BYTES / (size_t)page_size) {
        fprintf(stderr, "Invalid hardware page or chunk size\n");
        return EXIT_FAILURE;
    }
    chunk_bytes = (size_t)CHUNK_PAGES * (size_t)page_size;

    /* Step 1: Generate backing file (before rlimit). */
    generate_file();

    /* Open the backing file. */
    backing_fd = open(BACKING_FILE, O_RDONLY);
    if (backing_fd == -1)
        fail("open sqrts.bin");

    /* Step 2: Setup region and handler. */
    setup_sqrt_region();

    /* Step 3: Run validation with timing. */
    struct timespec start, end;
    struct rusage usage;

    printf("Mode: %s | CHUNK_PAGES=%d | RESIDENT_CHUNKS=%d\n",
           mode_names[hint_mode], CHUNK_PAGES, RESIDENT_CHUNKS);
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

    /* Clean up resident chunks. */
    while (count > 0) {
        uintptr_t chunk = fifo[oldest];
        if (munmap((void *)chunk, mapping_length(chunk)) == -1) {
            perror("munmap final chunk");
            return EXIT_FAILURE;
        }
        oldest = (oldest + 1) % RESIDENT_CHUNKS;
        count--;
    }

    close(backing_fd);

    /* Step 4: Print results. */
    printf("\n--- Results ---\n");
    printf("Pager faults : %d (random: %d, sequential: %d)\n",
           (int)fault_count, (int)random_faults, (int)sequential_faults);
    printf("Wall time    : %.6f s\n", seconds);
    printf("Peak RSS     : %ld KiB\n", usage.ru_maxrss);
    printf("Minor faults : %ld\n", usage.ru_minflt);
    printf("Major faults : %ld\n", usage.ru_majflt);
    printf("All tests passed!\n");

    return EXIT_SUCCESS;
}
