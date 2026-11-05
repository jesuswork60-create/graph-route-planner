#include "routeplan/analysis.hpp"

#include <algorithm>
#include <limits>
#include <set>

namespace routeplan {

namespace {

// Tarjan's algorithm with an explicit stack, because a long road can be deeper than the call stack allows.
class TarjanScc {
public:
    TarjanScc(const Graph& graph, const CostProfile& profile) : graph_(graph), profile_(profile) {
        auto count = static_cast<std::size_t>(graph.node_count());
        index_.assign(count, -1);
        low_.assign(count, 0);
        on_stack_.assign(count, 0);
    }

    std::vector<std::vector<NodeId>> run() {
        for (NodeId node = 0; node < graph_.node_count(); ++node) {
            if (index_[static_cast<std::size_t>(node)] < 0) visit(node);
        }
        return std::move(components_);
    }

private:
    struct Frame {
        NodeId node;
        std::size_t next_edge;
    };

    void enter(NodeId node, std::vector<Frame>& frames) {
        auto at = static_cast<std::size_t>(node);
        index_[at] = low_[at] = counter_++;
        on_stack_[at] = 1;
        stack_.push_back(node);
        frames.push_back(Frame{node, 0});
    }

    void visit(NodeId root) {
        std::vector<Frame> frames;
        enter(root, frames);
        while (!frames.empty()) {
            Frame& frame = frames.back();
            NodeId node = frame.node;
            auto at = static_cast<std::size_t>(node);
            const std::vector<EdgeId>& out = graph_.out_edges(node);
            if (frame.next_edge < out.size()) {
                const Edge& edge = graph_.edge(out[frame.next_edge++]);
                if (profile_.edge_cost(edge) == std::numeric_limits<double>::infinity()) continue;
                auto to = static_cast<std::size_t>(edge.to);
                if (index_[to] < 0) {
                    enter(edge.to, frames);
                } else if (on_stack_[to] != 0) {
                    low_[at] = std::min(low_[at], index_[to]);
                }
                continue;
            }
            if (low_[at] == index_[at]) close_component(node);
            frames.pop_back();
            if (!frames.empty()) {
                auto parent = static_cast<std::size_t>(frames.back().node);
                low_[parent] = std::min(low_[parent], low_[at]);
            }
        }
    }

    void close_component(NodeId root) {
        std::vector<NodeId> component;
        NodeId node;
        do {
            node = stack_.back();
            stack_.pop_back();
            on_stack_[static_cast<std::size_t>(node)] = 0;
            component.push_back(node);
        } while (node != root);
        std::sort(component.begin(), component.end());
        components_.push_back(std::move(component));
    }

    const Graph& graph_;
    const CostProfile& profile_;
    std::vector<int> index_;
    std::vector<int> low_;
    std::vector<char> on_stack_;
    std::vector<NodeId> stack_;
    std::vector<std::vector<NodeId>> components_;
    int counter_ = 0;
};

}  // namespace

std::vector<std::vector<NodeId>> strongly_connected_components(const Graph& graph, const CostProfile& profile) {
    std::vector<std::vector<NodeId>> components = TarjanScc(graph, profile).run();
    std::sort(components.begin(), components.end(), [](const auto& a, const auto& b) {
        if (a.size() != b.size()) return a.size() > b.size();
        return a.front() < b.front();
    });
    return components;
}

GraphSummary summarize(const Graph& graph, const CostProfile& profile) {
    GraphSummary summary;
    summary.nodes = graph.node_count();
    summary.edges = graph.edge_count();
    std::set<std::pair<NodeId, NodeId>> pairs;
    for (EdgeId id = 0; id < graph.edge_count(); ++id) {
        const Edge& edge = graph.edge(id);
        if (profile.edge_cost(edge) == std::numeric_limits<double>::infinity()) ++summary.closed_edges;
        if (edge.from == edge.to) ++summary.self_loops;
        if (!pairs.emplace(edge.from, edge.to).second) ++summary.parallel_edges;
        summary.total_length += edge.length;
        for (const std::string& tag : edge.tags) ++summary.tag_counts[tag];
    }
    auto components = strongly_connected_components(graph, profile);
    summary.components = static_cast<int>(components.size());
    summary.largest_component = components.empty() ? 0 : static_cast<int>(components.front().size());
    return summary;
}

std::vector<Finding> find_issues(const Graph& graph, const CostProfile& profile) {
    std::vector<Finding> findings;
    auto add = [&](Severity severity, const char* code, std::string message) {
        findings.push_back(Finding{severity, code, std::move(message)});
    };
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto is_open = [&](EdgeId id) { return profile.edge_cost(graph.edge(id)) != kInf; };

    for (NodeId node = 0; node < graph.node_count(); ++node) {
        const std::string& key = graph.node(node).key;
        const auto& out = graph.out_edges(node);
        const auto& in = graph.in_edges(node);
        if (out.empty() && in.empty()) {
            add(Severity::Warning, "isolated-node", "node " + key + " has no edges");
            continue;
        }
        bool any_out = std::any_of(out.begin(), out.end(), is_open);
        bool any_in = std::any_of(in.begin(), in.end(), is_open);
        if (any_in && !any_out) add(Severity::Warning, "dead-end", "node " + key + " can be entered but not left");
        if (any_out && !any_in) add(Severity::Warning, "no-entry", "node " + key + " can be left but not entered");
    }

    auto components = strongly_connected_components(graph, profile);
    if (components.size() > 1) {
        add(Severity::Warning, "split-network",
            std::to_string(components.size()) + " strongly connected components, the largest with " +
                std::to_string(components.front().size()) + " of " + std::to_string(graph.node_count()) + " nodes");
    }

    for (EdgeId id = 0; id < graph.edge_count(); ++id) {
        const Edge& edge = graph.edge(id);
        double straight = graph.straight_distance(edge.from, edge.to);
        // A little room for rounding in coordinates that were typed by hand.
        if (edge.length < straight * 0.999) {
            add(Severity::Warning, "short-edge", "edge " + edge.key + " is shorter than the distance between its ends");
        }
        if (profile.edge_cost(edge) == 0) {
            add(Severity::Warning, "zero-cost-edge", "edge " + edge.key + " costs nothing under profile " + profile.name);
        }
    }

    for (EdgeId id = 0; id < graph.edge_count(); ++id) {
        if (!is_open(id)) continue;
        const auto& exits = graph.out_edges(graph.edge(id).to);
        bool any_open = false;
        bool any_allowed = false;
        for (EdgeId next : exits) {
            if (!is_open(next)) continue;
            any_open = true;
            if (graph.turn_allowed(id, next)) any_allowed = true;
        }
        if (any_open && !any_allowed) {
            add(Severity::Problem, "turn-trap", "turn rules leave no way out after edge " + graph.edge(id).key);
        }
    }
    return findings;
}

}  // namespace routeplan
