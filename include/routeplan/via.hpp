// Routes through a list of waypoints.
#pragma once

#include <optional>
#include <vector>

#include "routeplan/router.hpp"

namespace routeplan {

struct ViaResult {
    std::vector<Route> legs;
    Route total;
    // When some leg has no route, its index; `legs` then holds the ones before it.
    std::optional<std::size_t> failed_leg;

    bool ok() const { return !failed_leg; }
};

// Each leg is the best given how the one before it ended, so a turn rule at a waypoint is
// honoured. Legs are planned one at a time, which can miss a dearer early leg that would
// have made the next one cheaper. Needs at least one waypoint.
ViaResult route_via(const Router& router, const std::vector<NodeId>& waypoints);

}  // namespace routeplan
