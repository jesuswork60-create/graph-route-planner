#include "routeplan/reach.hpp"

#include <algorithm>
#include <map>

namespace routeplan {

ReachResult reachable_within(const Router& router, NodeId source, double budget) {
    const Graph& graph = router.graph();
    ReachResult result;
    if (budget < 0) return result;
    Exploration explored = router.explore(source, budget);
    for (NodeId node = 0; node < graph.node_count(); ++node) {
        double cost = explored.node_cost[static_cast<std::size_t>(node)];
        if (cost != kInfinity) result.nodes.push_back(ReachedNode{node, cost});
    }
    std::stable_sort(result.nodes.begin(), result.nodes.end(),
                     [](const ReachedNode& a, const ReachedNode& b) { return a.cost < b.cost; });

    std::map<EdgeId, double> best;
    auto consider = [&](EdgeId edge, double spent) {
        double cost = router.edge_cost(edge);
        if (cost == kInfinity || cost <= 0 || spent + cost <= budget || spent > budget) return;
        double fraction = (budget - spent) / cost;
        double& known = best[edge];
        known = std::max(known, fraction);
    };
    for (EdgeId next : graph.out_edges(source)) consider(next, 0);
    for (EdgeId edge = 0; edge < graph.edge_count(); ++edge) {
        double spent = explored.state_cost[static_cast<std::size_t>(edge)];
        if (spent == kInfinity) continue;
        for (EdgeId next : graph.out_edges(graph.edge(edge).to)) {
            consider(next, spent + router.transition_cost(edge, next));
        }
    }
    for (const auto& entry : best) {
        // Some other approach may have covered the whole edge, which then isn't a frontier.
        bool covered = explored.state_cost[static_cast<std::size_t>(entry.first)] != kInfinity;
        if (!covered && entry.second > 0) result.frontier.push_back(PartialEdge{entry.first, entry.second});
    }
    return result;
}

}  // namespace routeplan
