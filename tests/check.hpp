// A very small test helper: TEST(name) registers a function and CHECK macros record failures.
#pragma once

#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace check {

void add_test(const char* name, std::function<void()> body);
void fail(const char* file, int line, const std::string& message);

template <typename T>
std::string show(const T& value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

struct Registrar {
    Registrar(const char* name, std::function<void()> body) { add_test(name, std::move(body)); }
};

}  // namespace check

#define TEST(name)                                          \
    static void name();                                     \
    static const check::Registrar registrar_##name(#name, name); \
    static void name()

#define CHECK(condition)                                                 \
    do {                                                                 \
        if (!(condition)) check::fail(__FILE__, __LINE__, #condition);   \
    } while (false)

#define CHECK_EQ(actual, expected)                                                                     \
    do {                                                                                               \
        const auto check_a = (actual);                                                                 \
        const auto check_b = (expected);                                                               \
        if (!(check_a == check_b)) {                                                                   \
            check::fail(__FILE__, __LINE__, #actual " is " + check::show(check_a) + ", expected " + check::show(check_b)); \
        }                                                                                              \
    } while (false)

#define CHECK_NEAR(actual, expected, margin)                                                           \
    do {                                                                                               \
        double check_a = (actual);                                                                     \
        double check_b = (expected);                                                                   \
        if (!(std::fabs(check_a - check_b) <= (margin))) {                                             \
            check::fail(__FILE__, __LINE__, #actual " is " + check::show(check_a) + ", expected " + check::show(check_b)); \
        }                                                                                              \
    } while (false)

#define CHECK_THROWS(expression, ExceptionType)                                              \
    do {                                                                                     \
        bool check_thrown = false;                                                           \
        try {                                                                                \
            (void)(expression);                                                              \
        } catch (const ExceptionType&) {                                                     \
            check_thrown = true;                                                             \
        }                                                                                    \
        if (!check_thrown) check::fail(__FILE__, __LINE__, #expression " did not throw " #ExceptionType); \
    } while (false)
