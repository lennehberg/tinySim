#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "Vec2.h"

// A minimal zero-dependency test harness.
//
// Tests self-register at static-init time via the TEST macro, so adding a new
// test file only requires adding it to the Makefile's TESTS list.

struct TestCase {
    const char *suite;
    const char *name;
    std::function<void()> fn;
};

std::vector<TestCase> &registry();

struct Registrar {
    Registrar(const char *suite, const char *name, std::function<void()> fn);
};

// Thrown by a failing assertion; caught and reported by the runner.
struct TestFailure {
    std::string message;
};

#define TEST(suite, name)                                                     \
    static void suite##_##name##_body();                                      \
    static Registrar suite##_##name##_reg(#suite, #name,                      \
                                          suite##_##name##_body);             \
    static void suite##_##name##_body()

std::string testLocation(const char *file, int line);

#define CHECK(cond)                                                           \
    do {                                                                      \
        if (!(cond)) {                                                        \
            throw TestFailure{testLocation(__FILE__, __LINE__) +              \
                              "CHECK failed: " #cond};                        \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, eps)                                     \
    do {                                                                      \
        const float a_ = (actual);                                            \
        const float e_ = (expected);                                          \
        if (!(std::fabs(a_ - e_) <= (eps))) {                                 \
            throw TestFailure{testLocation(__FILE__, __LINE__) +              \
                              "CHECK_NEAR failed: " #actual " = " +           \
                              std::to_string(a_) + ", expected " +            \
                              std::to_string(e_)};                            \
        }                                                                     \
    } while (0)

// Default tolerance for float comparisons throughout the suite.
constexpr float kEps = 1e-4f;

#define CHECK_FLOAT_EQ(actual, expected) CHECK_NEAR(actual, expected, kEps)

#define CHECK_VEC_NEAR(actual, ex, ey)                                        \
    do {                                                                      \
        const Vec2 v_ = (actual);                                             \
        CHECK_NEAR(v_.getX(), (ex), kEps);                                    \
        CHECK_NEAR(v_.getY(), (ey), kEps);                                    \
    } while (0)

#define CHECK_THROWS(expr, exception_type)                                    \
    do {                                                                      \
        bool threw_ = false;                                                  \
        try {                                                                 \
            (void)(expr);                                                     \
        } catch (const exception_type &) {                                    \
            threw_ = true;                                                    \
        }                                                                     \
        if (!threw_) {                                                        \
            throw TestFailure{testLocation(__FILE__, __LINE__) +              \
                              "CHECK_THROWS failed: " #expr                   \
                              " did not throw " #exception_type};             \
        }                                                                     \
    } while (0)

#endif  // TEST_UTILS_H
