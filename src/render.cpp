#include "routeplan/render.hpp"

#include <cstdio>
#include <set>

namespace routeplan {

using json::Value;

std::string format_cost(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.6g", value);
    return buffer;
}

std::string route_text(const Graph& graph, const Route& route) {
    std::string text;
    for (std::size_t i = 0; i < route.nodes.size(); ++i) {
        if (i > 0) text += " -> ";
        text += graph.node(route.nodes[i]).key;
    }
    text += "\ncost " + format_cost(route.cost) + ", length " + format_cost(route.length) + ", time " +
            format_cost(route.time) + ", " + std::to_string(route.edges.size()) + " edges\n";
    return text;
}

Value route_json(const Graph& graph, const Route& route) {
    Value::Array nodes;
    for (NodeId node : route.nodes) nodes.emplace_back(graph.node(node).key);
    Value::Array edges;
    for (EdgeId edge : route.edges) edges.emplace_back(graph.edge(edge).key);
    Value::Object object;
    object.emplace_back("cost", route.cost);
    object.emplace_back("length", route.length);
    object.emplace_back("time", route.time);
    object.emplace_back("nodes", std::move(nodes));
    object.emplace_back("edges", std::move(edges));
    return Value(std::move(object));
}

Value itinerary_json(const Graph& graph, const std::vector<Step>& steps) {
    Value::Array items;
    for (const Step& step : steps) {
        Value::Object object;
        object.emplace_back("maneuver", maneuver_name(step.maneuver));
        object.emplace_back("street", step.street);
        object.emplace_back("length", step.length);
        object.emplace_back("time", step.time);
        object.emplace_back("to", graph.node(step.end_node).key);
        object.emplace_back("text", describe_step(graph, step));
        items.emplace_back(std::move(object));
    }
    return Value(std::move(items));
}

Value reach_json(const Graph& graph, const ReachResult& reach) {
    Value::Array nodes;
    for (const ReachedNode& reached : reach.nodes) {
        Value::Object object;
        object.emplace_back("node", graph.node(reached.node).key);
        object.emplace_back("cost", reached.cost);
        nodes.emplace_back(std::move(object));
    }
    Value::Array frontier;
    for (const PartialEdge& partial : reach.frontier) {
        Value::Object object;
        object.emplace_back("edge", graph.edge(partial.edge).key);
        object.emplace_back("fraction", partial.fraction);
        frontier.emplace_back(std::move(object));
    }
    Value::Object result;
    result.emplace_back("nodes", std::move(nodes));
    result.emplace_back("frontier", std::move(frontier));
    return Value(std::move(result));
}

Value stop_order_json(const Graph& graph, const StopOrder& order) {
    Value::Array nodes;
    for (NodeId node : order.order) nodes.emplace_back(graph.node(node).key);
    Value::Object result;
    result.emplace_back("order", std::move(nodes));
    result.emplace_back("cost", order.cost);
    result.emplace_back("exact", order.exact);
    return Value(std::move(result));
}

Value findings_json(const std::vector<Finding>& findings) {
    Value::Array items;
    for (const Finding& finding : findings) {
        Value::Object object;
        object.emplace_back("severity", finding.severity == Severity::Problem ? "problem" : "warning");
        object.emplace_back("code", finding.code);
        object.emplace_back("message", finding.message);
        items.emplace_back(std::move(object));
    }
    return Value(std::move(items));
}

Value summary_json(const GraphSummary& summary) {
    Value::Object tags;
    for (const auto& entry : summary.tag_counts) tags.emplace_back(entry.first, entry.second);
    Value::Object object;
    object.emplace_back("nodes", summary.nodes);
    object.emplace_back("edges", summary.edges);
    object.emplace_back("closed_edges", summary.closed_edges);
    object.emplace_back("self_loops", summary.self_loops);
    object.emplace_back("parallel_edges", summary.parallel_edges);
    object.emplace_back("components", summary.components);
    object.emplace_back("largest_component", summary.largest_component);
    object.emplace_back("total_length", summary.total_length);
    object.emplace_back("tags", std::move(tags));
    return Value(std::move(object));
}

namespace {

std::string dot_quote(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    out += '"';
    return out;
}

}  // namespace

std::string graph_dot(const Graph& graph, const std::vector<EdgeId>& highlight) {
    std::set<EdgeId> marked(highlight.begin(), highlight.end());
    std::string text = "digraph routes {\n";
    for (NodeId id = 0; id < graph.node_count(); ++id) {
        text += "  " + dot_quote(graph.node(id).key) + ";\n";
    }
    for (EdgeId id = 0; id < graph.edge_count(); ++id) {
        const Edge& edge = graph.edge(id);
        text += "  " + dot_quote(graph.node(edge.from).key) + " -> " + dot_quote(graph.node(edge.to).key);
        text += " [label=" + dot_quote(format_cost(edge.length));
        if (marked.count(id) > 0) text += ", color=red, penwidth=2";
        text += "];\n";
    }
    text += "}\n";
    return text;
}

}  // namespace routeplan
