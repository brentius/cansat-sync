/* GPL-3.0-only. Minimal host unit-test harness for the STM32 core library.
 *
 * Host only: uses stdio. Each tests/test_*.c defines tests with CS_TEST and
 * ends with CS_MAIN(test_a, test_b, ...). tools/host_test.py builds and runs
 * them. A test file may name the core sources it needs on a comment line:
 *     CS_SOURCES: cs_crc16.c cs_cobs.c
 * so a half-finished module elsewhere in core/ does not break unrelated tests.
 */
#ifndef CS_TEST_H
#define CS_TEST_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int cs_test_failures;
static int cs_test_checks;
static const char *cs_test_current = "";

#define CS_TEST(name) static void name(void)

#define CS_FAIL_AT(file, line, ...)                                      \
    do {                                                                 \
        cs_test_failures++;                                              \
        printf("FAIL %s (%s:%d): ", cs_test_current, file, line);        \
        printf(__VA_ARGS__);                                             \
        printf("\n");                                                    \
    } while (0)

#define CS_CHECK(cond)                                                   \
    do {                                                                 \
        cs_test_checks++;                                                \
        if (!(cond)) CS_FAIL_AT(__FILE__, __LINE__, "%s", #cond);        \
    } while (0)

#define CS_CHECK_MSG(cond, ...)                                          \
    do {                                                                 \
        cs_test_checks++;                                                \
        if (!(cond)) CS_FAIL_AT(__FILE__, __LINE__, __VA_ARGS__);        \
    } while (0)

#define CS_CHECK_EQ(a, b)                                                \
    do {                                                                 \
        long long cs_a_ = (long long)(a), cs_b_ = (long long)(b);        \
        cs_test_checks++;                                                \
        if (cs_a_ != cs_b_)                                              \
            CS_FAIL_AT(__FILE__, __LINE__, "%s == %s: %lld != %lld",     \
                       #a, #b, cs_a_, cs_b_);                            \
    } while (0)

#define CS_CHECK_EQ_U(a, b)                                              \
    do {                                                                 \
        unsigned long long cs_a_ = (unsigned long long)(a);              \
        unsigned long long cs_b_ = (unsigned long long)(b);              \
        cs_test_checks++;                                                \
        if (cs_a_ != cs_b_)                                              \
            CS_FAIL_AT(__FILE__, __LINE__, "%s == %s: %llu != %llu",     \
                       #a, #b, cs_a_, cs_b_);                            \
    } while (0)

#define CS_CHECK_NEAR(a, b, tol)                                         \
    do {                                                                 \
        double cs_a_ = (double)(a), cs_b_ = (double)(b);                 \
        cs_test_checks++;                                                \
        if (!(fabs(cs_a_ - cs_b_) <= (double)(tol)))                     \
            CS_FAIL_AT(__FILE__, __LINE__, "%s ~ %s: %.12g vs %.12g "    \
                       "(tol %.3g)", #a, #b, cs_a_, cs_b_, (double)(tol)); \
    } while (0)

#define CS_CHECK_MEM(a, b, n)                                            \
    do {                                                                 \
        const uint8_t *cs_a_ = (const uint8_t *)(a);                     \
        const uint8_t *cs_b_ = (const uint8_t *)(b);                     \
        size_t cs_n_ = (size_t)(n), cs_i_;                               \
        cs_test_checks++;                                                \
        for (cs_i_ = 0; cs_i_ < cs_n_; cs_i_++)                          \
            if (cs_a_[cs_i_] != cs_b_[cs_i_]) break;                     \
        if (cs_i_ < cs_n_)                                               \
            CS_FAIL_AT(__FILE__, __LINE__, "%s vs %s differ at byte %u " \
                       "(0x%02x != 0x%02x)", #a, #b, (unsigned)cs_i_,    \
                       cs_a_[cs_i_], cs_b_[cs_i_]);                      \
    } while (0)

typedef void (*cs_test_fn)(void);

static int cs_test_run(const char *file, cs_test_fn *fns, const char **names, int n)
{
    int i, failed_tests = 0;
    for (i = 0; i < n; i++) {
        int before = cs_test_failures;
        cs_test_current = names[i];
        fns[i]();
        if (cs_test_failures != before) failed_tests++;
    }
    printf("%s: %d tests, %d checks, %d failed tests, %d failed checks\n",
           file, n, cs_test_checks, failed_tests, cs_test_failures);
    return cs_test_failures ? 1 : 0;
}

/* Stringise each test name so failures can be reported by name. */
#define CS_STR_(x) #x
#define CS_MAIN(...)                                                     \
    int main(void)                                                       \
    {                                                                    \
        cs_test_fn fns[] = {__VA_ARGS__};                                \
        static const char *all = CS_STR_((__VA_ARGS__));                 \
        const char *names[sizeof fns / sizeof fns[0]];                   \
        static char buf[4096];                                           \
        int n = (int)(sizeof fns / sizeof fns[0]), k = 0;                \
        size_t j = 0, len = strlen(all);                                 \
        char *p = buf;                                                   \
        if (len >= sizeof buf) len = sizeof buf - 1;                     \
        memcpy(buf, all, len);                                           \
        buf[len] = 0;                                                    \
        for (j = 0; j < len && k < n; j++) {                             \
            char c = buf[j];                                             \
            if (c == '(' || c == ' ') { p = buf + j + 1; continue; }     \
            if (c == ',' || c == ')') {                                  \
                buf[j] = 0;                                              \
                names[k++] = p;                                          \
                p = buf + j + 1;                                         \
            }                                                            \
        }                                                                \
        while (k < n) names[k++] = "?";                                  \
        return cs_test_run(__FILE__, fns, names, n);                     \
    }

#endif
