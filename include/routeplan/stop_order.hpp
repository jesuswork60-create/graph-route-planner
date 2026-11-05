// Cost tables between nodes and choosing a good order to visit stops in.
#pragma once

#include <optional>
#include <vector>

#include "routeplan/router.hpp"

namespace routeplan {

// One row per source and one column per target; infinity where there is no route.
std::vector<std::vector<double>> cost_matrix(const Router& router, const std::vector<NodeId>& sources,
                                             const std::vector<NodeId>& targets);

struct StopOrder {
    std::vector<NodeId> order;  // Starts with `start`, ends with `end` when given.
    double cost = 0;
    bool exact = false;  // True when the order is proven best for the cost table.
};

inline constexpr int kExactStopLimit = 12;

// Visits every stop once, using an exact search up to kExactStopLimit stops and nearest
// neighbour plus 2-opt above that. Returns nothing when no order connects them all. Costs
// come from cost_matrix, which ignores how each leg arrives, so a turn rule at a stop can
// make the real route dearer than this total.
std::optional<StopOrder> plan_stop_order(const Router& router, NodeId start, const std::vector<NodeId>& stops,
                                         std::optional<NodeId> end = std::nullopt);

}  // namespace routeplan
