// Least-cost paths over the graph, honouring turn restrictions and the cost profile.
#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "routeplan/graph.hpp"
#include "routeplan/profile.hpp"

namespace routeplan {

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Route {
    std::vector<NodeId> nodes;  // Always starts with the source, even for an empty route.
    std::vector<EdgeId> edges;
    double cost = 0;
    double length = 0;
    double time = 0;
};

struct RouteQuery {
    NodeId source = 0;
    NodeId target = 0;
    // The edge already travelled to reach the source. Turn rules apply to the first step.
    std::optional<EdgeId> arrived_by;
    bool use_heuristic = true;
    double max_cost = kInfinity;
    // Optional masks with one entry per edge or node; non-zero means off limits.
    const std::vector<char>* banned_edges = nullptr;
    const std::vector<char>* banned_nodes = nullptr;
};

struct SearchStats {
    long settled = 0;
    long relaxed = 0;
    long pushed = 0;
};

// The cheapest way to arrive anywhere from one source.
struct Exploration {
    std::vector<double> node_cost;   // By node; infinity when not reached.
    std::vector<double> state_cost;  // By edge: cost through the end of that edge; infinity when not used.
};

class Router {
public:
    // Throws ProfileError if the profile is invalid.
    Router(const Graph& graph, CostProfile profile);

    const Graph& graph() const { return graph_; }
    const CostProfile& profile() const { return profile_; }

    double edge_cost(EdgeId edge) const { return edge_costs_[static_cast<std::size_t>(edge)]; }

    // The extra cost of going from one edge straight into the next, or infinity if not allowed.
    double transition_cost(EdgeId previous, EdgeId next) const;

    // The heuristic is a lower bound, so results match plain Dijkstra's in cost.
    std::optional<Route> route(const RouteQuery& query, SearchStats* stats = nullptr) const;
    std::optional<Route> route(NodeId source, NodeId target) const;

    // Stops at the budget when one is given.
    Exploration explore(NodeId source, double budget = kInfinity) const;

    // Lower bound on the cost from `from` to `to`. Zero when the graph has no coordinates.
    double heuristic(NodeId from, NodeId to) const;

    // Cost of following these edges, or infinity if any step is closed or not allowed.
    double path_cost(const std::vector<EdgeId>& edges, std::optional<EdgeId> arrived_by = std::nullopt) const;

    Route make_route(NodeId source, std::vector<EdgeId> edges, double cost) const;

private:
    const Graph& graph_;
    CostProfile profile_;
    std::vector<double> edge_costs_;
    double min_cost_per_unit_ = 0;
};

}  // namespace routeplan
