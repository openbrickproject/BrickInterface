// Minimal test framework — zero external dependencies.
//
// Usage:
//   TEST(name) { ASSERT_EQ(2 + 2, 4); }
//   int main(void) { RUN_ALL_TESTS(); }
//
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

extern int test_failed;
extern int test_total;
extern int test_passed;
extern const char *current_test_name;

typedef void (*TestFn)(void);

struct TestEntry {
    const char *name;
    TestFn fn;
};
extern struct TestEntry g_tests[256];
extern int g_test_count;

void test_register(const char *name, TestFn fn);

// TEST(name) declares + auto-registers a test
#define TEST(name)                                                            \
    static void name(void);                                                   \
    static void __attribute__((constructor)) register_##name(void) {          \
        test_register(#name, name);                                           \
    }                                                                         \
    static void name(void)

#define _TEST_FAIL(msg, ...)                                                  \
    do {                                                                     \
        printf("  FAIL %s:%d: " msg "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
        test_failed = 1;                                                      \
    } while (0)

#define ASSERT_EQ(a, b)                                                       \
    do {                                                                      \
        long long _a = (long long)(a), _b = (long long)(b);                   \
        if (_a != _b) _TEST_FAIL("%s == %s : got %lld, expected %lld",        \
                                  #a, #b, _a, _b);                            \
    } while (0)

#define ASSERT_TRUE(x)                                                        \
    do { if (!(x)) _TEST_FAIL("expected truthy: %s", #x); } while (0)

#define ASSERT_FALSE(x)                                                       \
    do { if ((x)) _TEST_FAIL("expected falsy: %s", #x); } while (0)

#define ASSERT_BYTES_EQ(a, b, len)                                            \
    do {                                                                      \
        if (memcmp((a), (b), (len)) != 0) {                                   \
            _TEST_FAIL("byte arrays differ over %zu bytes", (size_t)(len));   \
            printf("    got: ");                                              \
            for (size_t _i = 0; _i < (size_t)(len); _i++)                     \
                printf("%02X ", ((const uint8_t *)(a))[_i]);                  \
            printf("\n    exp: ");                                            \
            for (size_t _i = 0; _i < (size_t)(len); _i++)                     \
                printf("%02X ", ((const uint8_t *)(b))[_i]);                  \
            printf("\n");                                                     \
        }                                                                     \
    } while (0)

#define RUN_ALL_TESTS()                                                       \
    do {                                                                      \
        for (int _i = 0; _i < g_test_count; _i++) {                           \
            current_test_name = g_tests[_i].name;                             \
            test_failed = 0;                                                  \
            printf("  %s ... ", g_tests[_i].name);                            \
            fflush(stdout);                                                   \
            g_tests[_i].fn();                                                 \
            test_total++;                                                     \
            if (!test_failed) { test_passed++; printf("ok\n"); }              \
            else { printf("\n"); }                                            \
        }                                                                     \
        printf("\n%d/%d passed\n", test_passed, test_total);                  \
        return test_passed == test_total ? 0 : 1;                             \
    } while (0)

#endif
