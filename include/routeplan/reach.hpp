// What can be reached from a node within a budget.
#pragma once

#include <vector>

#include "routeplan/router.hpp"

namespace routeplan {

struct ReachedNode {
    NodeId node = 0;
    double cost = 0;
};

// An edge that the budget runs out on: `fraction` of the way along it, between 0 and 1.
struct PartialEdge {
    EdgeId edge = 0;
    double fraction = 0;
};

struct ReachResult {
    std::vector<ReachedNode> nodes;  // Cheapest first; ties in node order.
    std::vector<PartialEdge> frontier;  // In edge order.
};

ReachResult reachable_within(const Router& router, NodeId source, double budget);

}  // namespace routeplan
