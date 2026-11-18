#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#define AS_LIMIT (1U << 23)
#define MAX_SQRTS (1U << 27)
#define TABLE_BYTES (MAX_SQRTS * sizeof(double))

#define align_down(x, a) ((x) & ~((uintptr_t)(a) - 1))

static size_t page_size;
static double *sqrts;
static void *resident_page = MAP_FAILED;

static void calculate_sqrts(double *sqrt_pos, size_t start, size_t nr)
{
    for (size_t i = 0; i < nr; i++)
        sqrt_pos[i] = sqrt((double)(start + i));
}

/* Handler errors use write() and _exit() instead of stdio. */
#define HANDLER_FAIL(message)                          \
    do {                                               \
        static const char diagnostic[] = message "\n"; \
        (void)write(STDERR_FILENO, diagnostic,          \
                    sizeof diagnostic - 1);            \
        _exit(EXIT_FAILURE);                           \
    } while (0)

static void handle_sigsegv(int sig, siginfo_t *si, void *ctx)
{
    (void)sig;
    (void)ctx;

    int saved_errno = errno;
    uintptr_t fault_addr = (uintptr_t)si->si_addr;
    uintptr_t table_base = (uintptr_t)sqrts;

    /* Handle only missing mappings inside the square-root table. */
    if (fault_addr < table_base || fault_addr - table_base >= TABLE_BYTES)
        HANDLER_FAIL("SIGSEGV outside the square-root table");

    if (si->si_code != SEGV_MAPERR)
        HANDLER_FAIL("Unexpected SIGSEGV inside the square-root table");

    uintptr_t page_addr = align_down(fault_addr, page_size);

    /* Part A keeps only one table page mapped at a time. */
    if (resident_page != MAP_FAILED &&
        munmap(resident_page, page_size) == -1)
        HANDLER_FAIL("Could not unmap the previous page");

    void *page = mmap((void *)page_addr, page_size,
                      PROT_READ | PROT_WRITE,
                      MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS,
                      -1, 0);
    if (page == MAP_FAILED)
        HANDLER_FAIL("Could not map the faulting page");

    size_t first_index = (page_addr - table_base) / sizeof(double);
    size_t entries_per_page = page_size / sizeof(double);

    calculate_sqrts(page, first_index, entries_per_page);
    resident_page = page;

    errno = saved_errno;
    /* Returning lets the CPU retry the original read. */
}

static void setup_sqrt_region(void)
{
    size_t probe_size = TABLE_BYTES + AS_LIMIT;
    struct rlimit lim = { AS_LIMIT, AS_LIMIT };
    struct sigaction act = {0};

    /* Find an available address range for the table. */
    sqrts = mmap(NULL, probe_size, PROT_NONE,
                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (sqrts == MAP_FAILED) {
        perror("mmap table region");
        exit(EXIT_FAILURE);
    }

    /* Release the range before imposing the 8 MiB address-space limit. */
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
    int pos = 0;
    double correct_sqrt;

    printf("Validating square root table contents...\n");
    srand(0xDEADBEEF);

    for (int i = 0; i < 500000; i++) {
        if (i % 2 == 0)
            pos = rand() % (MAX_SQRTS - 1);
        else
            pos += 1;

        calculate_sqrts(&correct_sqrt, pos, 1);
        double actual = sqrts[pos];

        if (actual != correct_sqrt) {
            fprintf(stderr,
                    "Square root is incorrect. Expected %f, got %f.\n",
                    correct_sqrt, actual);
            exit(EXIT_FAILURE);
        }
    }
}

int main(void)
{
    long system_page_size = sysconf(_SC_PAGESIZE);

    if (system_page_size <= 0 ||
        TABLE_BYTES % (size_t)system_page_size != 0) {
        fprintf(stderr, "Invalid hardware page size\n");
        return EXIT_FAILURE;
    }

    page_size = (size_t)system_page_size;
    printf("page_size is %zu\n", page_size);

    setup_sqrt_region();
    test_sqrt_region();

    if (resident_page != MAP_FAILED &&
        munmap(resident_page, page_size) == -1) {
        perror("munmap final page");
        return EXIT_FAILURE;
    }

    printf("All tests passed!\n");
    return EXIT_SUCCESS;
}
