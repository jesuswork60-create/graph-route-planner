#include "check.hpp"

#include <exception>
#include <iostream>
#include <vector>

namespace check {

namespace {

struct Entry {
    const char* name;
    std::function<void()> body;
};

std::vector<Entry>& entries() {
    static std::vector<Entry> list;
    return list;
}

int failures_in_test = 0;

}  // namespace

void add_test(const char* name, std::function<void()> body) {
    entries().push_back(Entry{name, std::move(body)});
}

void fail(const char* file, int line, const std::string& message) {
    ++failures_in_test;
    std::cerr << file << ':' << line << ": " << message << '\n';
}

}  // namespace check

int main() {
    int failed = 0;
    for (auto& entry : check::entries()) {
        check::failures_in_test = 0;
        try {
            entry.body();
        } catch (const std::exception& e) {
            check::fail(entry.name, 0, std::string("unexpected exception: ") + e.what());
        }
        if (check::failures_in_test > 0) {
            ++failed;
            std::cerr << "FAILED " << entry.name << '\n';
        }
    }
    std::cout << check::entries().size() - static_cast<std::size_t>(failed) << " of " << check::entries().size() << " tests passed\n";
    return failed == 0 ? 0 : 1;
}
