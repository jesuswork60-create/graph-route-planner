// Several different routes between two nodes, using Yen's algorithm.
#pragma once

#include <vector>

#include "routeplan/router.hpp"

namespace routeplan {

// Up to `count` different routes in order of rising cost; fewer come back when fewer exist.
// A deviation never returns to a node on the stretch it shares with an earlier route, but a
// turn rule can still force any one route through the same node twice.
std::vector<Route> k_shortest_routes(const Router& router, NodeId source, NodeId target, int count);

}  // namespace routeplan
