#include "routeplan/alternatives.hpp"

#include <algorithm>
#include <set>
#include <tuple>

namespace routeplan {

std::vector<Route> k_shortest_routes(const Router& router, NodeId source, NodeId target, int count) {
    std::vector<Route> accepted;
    if (count <= 0) return accepted;
    auto first = router.route(source, target);
    if (!first) return accepted;
    accepted.push_back(std::move(*first));
    if (source == target) return accepted;

    const Graph& graph = router.graph();
    using Candidate = std::tuple<double, std::vector<EdgeId>>;
    std::set<Candidate> candidates;
    std::set<std::vector<EdgeId>> seen;
    seen.insert(accepted[0].edges);
    std::vector<char> banned_edges(static_cast<std::size_t>(graph.edge_count()));
    std::vector<char> banned_nodes(static_cast<std::size_t>(graph.node_count()));

    while (static_cast<int>(accepted.size()) < count) {
        const Route previous = accepted.back();
        for (std::size_t spur = 0; spur < previous.edges.size(); ++spur) {
            std::fill(banned_edges.begin(), banned_edges.end(), 0);
            std::fill(banned_nodes.begin(), banned_nodes.end(), 0);
            std::vector<EdgeId> root(previous.edges.begin(), previous.edges.begin() + static_cast<std::ptrdiff_t>(spur));
            for (const Route& route : accepted) {
                if (route.edges.size() > spur && std::equal(root.begin(), root.end(), route.edges.begin())) {
                    banned_edges[static_cast<std::size_t>(route.edges[spur])] = 1;
                }
            }
            for (std::size_t i = 0; i < spur; ++i) banned_nodes[static_cast<std::size_t>(previous.nodes[i])] = 1;
            NodeId spur_node = previous.nodes[spur];
            banned_nodes[static_cast<std::size_t>(spur_node)] = 0;

            RouteQuery query;
            query.source = spur_node;
            query.target = target;
            if (!root.empty()) query.arrived_by = root.back();
            query.banned_edges = &banned_edges;
            query.banned_nodes = &banned_nodes;
            auto tail = router.route(query);
            if (!tail) continue;
            std::vector<EdgeId> whole = root;
            whole.insert(whole.end(), tail->edges.begin(), tail->edges.end());
            if (seen.insert(whole).second) {
                candidates.emplace(router.path_cost(root) + tail->cost, std::move(whole));
            }
        }
        if (candidates.empty()) break;
        auto best = candidates.begin();
        accepted.push_back(router.make_route(source, std::get<1>(*best), std::get<0>(*best)));
        candidates.erase(best);
    }
    return accepted;
}

}  // namespace routeplan
