#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "support/test_utils.h"

std::vector<TestCase> &registry() {
    // Function-local static: avoids static-init order issues with the
    // Registrar objects in the individual test translation units.
    static std::vector<TestCase> tests;
    return tests;
}

Registrar::Registrar(const char *suite, const char *name,
                     std::function<void()> fn) {
    registry().push_back(TestCase{suite, name, std::move(fn)});
}

std::string testLocation(const char *file, int line) {
    return std::string(file) + ":" + std::to_string(line) + ": ";
}

int main() {
    int passed = 0;
    std::vector<std::string> failures;

    for (const TestCase &test : registry()) {
        const std::string label =
            std::string(test.suite) + "." + test.name;
        try {
            test.fn();
            ++passed;
            std::printf("  ok   %s\n", label.c_str());
        } catch (const TestFailure &failure) {
            failures.push_back(label + "\n         " + failure.message);
            std::printf("  FAIL %s\n           %s\n", label.c_str(),
                        failure.message.c_str());
        } catch (const std::exception &e) {
            failures.push_back(label + "\n         unexpected exception: " +
                               e.what());
            std::printf("  FAIL %s\n           unexpected exception: %s\n",
                        label.c_str(), e.what());
        }
    }

    std::printf("\n%d passed, %zu failed, %zu total\n", passed,
                failures.size(), registry().size());
    return failures.empty() ? 0 : 1;
}
