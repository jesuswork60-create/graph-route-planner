#include "routeplan/router.hpp"

#include <algorithm>
#include <queue>
#include <tuple>

namespace routeplan {

namespace {

// State 0 is "at the source, before moving"; state e + 1 is "just travelled edge e". Searching
// over edges instead of nodes is what lets a turn rule depend on how a node was reached.
using QueueItem = std::tuple<double, int, double>;  // Priority, state, cost so far.
using Queue = std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>;

bool is_masked(const std::vector<char>* mask, int id) {
    return mask != nullptr && (*mask)[static_cast<std::size_t>(id)] != 0;
}

}  // namespace

Router::Router(const Graph& graph, CostProfile profile) : graph_(graph), profile_(std::move(profile)) {
    profile_.validate();
    edge_costs_.reserve(static_cast<std::size_t>(graph.edge_count()));
    double best = kInfinity;
    for (EdgeId id = 0; id < graph.edge_count(); ++id) {
        const Edge& edge = graph.edge(id);
        double cost = profile_.edge_cost(edge);
        edge_costs_.push_back(cost);
        if (cost == kInfinity) continue;
        double straight = graph.straight_distance(edge.from, edge.to);
        if (straight > 0) best = std::min(best, cost / straight);
    }
    // No usable edge means there is nothing to scale by, and zero is always safe.
    min_cost_per_unit_ = best == kInfinity ? 0 : best;
}

double Router::heuristic(NodeId from, NodeId to) const {
    return min_cost_per_unit_ * graph_.straight_distance(from, to);
}

double Router::transition_cost(EdgeId previous, EdgeId next) const {
    if (!graph_.turn_allowed(previous, next)) return kInfinity;
    const Edge& before = graph_.edge(previous);
    const Edge& after = graph_.edge(next);
    bool u_turn = after.to == before.from && after.from == before.to;
    if (!u_turn) return 0;
    if (profile_.u_turns == UTurnPolicy::Allow) return profile_.u_turn_penalty;
    if (profile_.u_turns == UTurnPolicy::Forbid) return kInfinity;
    for (EdgeId other : graph_.out_edges(before.to)) {
        if (other == next || edge_cost(other) == kInfinity || !graph_.turn_allowed(previous, other)) continue;
        if (graph_.edge(other).to != before.from) return kInfinity;
    }
    return profile_.u_turn_penalty;
}

std::optional<Route> Router::route(NodeId source, NodeId target) const {
    RouteQuery query;
    query.source = source;
    query.target = target;
    return route(query);
}

std::optional<Route> Router::route(const RouteQuery& query, SearchStats* stats) const {
    std::vector<double> best(static_cast<std::size_t>(graph_.edge_count()) + 1, kInfinity);
    std::vector<int> parent(best.size(), -1);
    SearchStats local;
    Queue queue;
    auto estimate = [&](NodeId node) { return query.use_heuristic ? heuristic(node, query.target) : 0.0; };

    best[0] = 0;
    queue.emplace(estimate(query.source), 0, 0.0);
    ++local.pushed;
    int found = -1;
    while (!queue.empty()) {
        auto [priority, state, cost] = queue.top();
        (void)priority;
        queue.pop();
        if (cost > best[static_cast<std::size_t>(state)]) continue;
        ++local.settled;
        NodeId node = state == 0 ? query.source : graph_.edge(state - 1).to;
        if (node == query.target) {
            found = state;
            break;
        }
        std::optional<EdgeId> previous = query.arrived_by;
        if (state != 0) previous = state - 1;
        for (EdgeId next : graph_.out_edges(node)) {
            double step = edge_cost(next);
            const Edge& edge = graph_.edge(next);
            if (step == kInfinity || is_masked(query.banned_edges, next) || is_masked(query.banned_nodes, edge.to)) continue;
            ++local.relaxed;
            if (previous) step += transition_cost(*previous, next);
            double total = cost + step;
            if (total == kInfinity || total > query.max_cost) continue;
            auto next_state = static_cast<std::size_t>(next) + 1;
            if (total < best[next_state]) {
                best[next_state] = total;
                parent[next_state] = state;
                queue.emplace(total + estimate(edge.to), next + 1, total);
                ++local.pushed;
            }
        }
    }
    if (stats != nullptr) *stats = local;
    if (found < 0) return std::nullopt;
    std::vector<EdgeId> edges;
    for (int state = found; state != 0; state = parent[static_cast<std::size_t>(state)]) {
        edges.push_back(state - 1);
    }
    std::reverse(edges.begin(), edges.end());
    return make_route(query.source, std::move(edges), best[static_cast<std::size_t>(found)]);
}

Exploration Router::explore(NodeId source, double budget) const {
    Exploration result;
    result.node_cost.assign(static_cast<std::size_t>(graph_.node_count()), kInfinity);
    std::vector<double> best(static_cast<std::size_t>(graph_.edge_count()) + 1, kInfinity);
    Queue queue;
    best[0] = 0;
    queue.emplace(0.0, 0, 0.0);
    while (!queue.empty()) {
        auto [priority, state, cost] = queue.top();
        (void)priority;
        queue.pop();
        if (cost > best[static_cast<std::size_t>(state)]) continue;
        NodeId node = state == 0 ? source : graph_.edge(state - 1).to;
        double& known = result.node_cost[static_cast<std::size_t>(node)];
        known = std::min(known, cost);
        for (EdgeId next : graph_.out_edges(node)) {
            double total = cost + edge_cost(next);
            if (state != 0) total += transition_cost(state - 1, next);
            if (total == kInfinity || total > budget) continue;
            auto next_state = static_cast<std::size_t>(next) + 1;
            if (total < best[next_state]) {
                best[next_state] = total;
                queue.emplace(total, next + 1, total);
            }
        }
    }
    result.state_cost.assign(best.begin() + 1, best.end());
    return result;
}

double Router::path_cost(const std::vector<EdgeId>& edges, std::optional<EdgeId> arrived_by) const {
    double total = 0;
    std::optional<EdgeId> previous = arrived_by;
    for (EdgeId edge : edges) {
        total += edge_cost(edge);
        if (previous) total += transition_cost(*previous, edge);
        previous = edge;
    }
    return total;
}

Route Router::make_route(NodeId source, std::vector<EdgeId> edges, double cost) const {
    Route route;
    route.cost = cost;
    route.nodes.reserve(edges.size() + 1);
    route.nodes.push_back(source);
    for (EdgeId id : edges) {
        const Edge& edge = graph_.edge(id);
        route.nodes.push_back(edge.to);
        route.length += edge.length;
        route.time += edge.time;
    }
    route.edges = std::move(edges);
    return route;
}

}  // namespace routeplan
