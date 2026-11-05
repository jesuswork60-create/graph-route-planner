// The route-plan command line tool, as a function so tests can call it directly.
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace routeplan {

// Exit codes: 0 done, 1 no route or the check found a problem, 2 bad usage or a bad graph file.
int run_cli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace routeplan
