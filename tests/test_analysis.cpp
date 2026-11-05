#include <algorithm>

#include "check.hpp"
#include "routeplan/analysis.hpp"

using namespace routeplan;

namespace {

bool has_code(const std::vector<Finding>& findings, const std::string& code) {
    return std::any_of(findings.begin(), findings.end(), [&](const Finding& f) { return f.code == code; });
}

}  // namespace

TEST(finds_components) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    NodeId d = graph.add_node("d");
    NodeId e = graph.add_node("e");
    graph.add_edge(a, b, 1);
    graph.add_edge(b, c, 1);
    graph.add_edge(c, a, 1);
    graph.add_edge(c, d, 1);
    graph.add_edge(d, e, 1);
    graph.add_edge(e, d, 1);
    auto components = strongly_connected_components(graph, CostProfile::distance());
    CHECK_EQ(components.size(), std::size_t{2});
    CHECK(components[0] == (std::vector<NodeId>{a, b, c}));
    CHECK(components[1] == (std::vector<NodeId>{d, e}));
}

TEST(a_closed_edge_can_split_a_component) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 1);
    Edge back;
    back.from = b;
    back.to = a;
    back.length = 1;
    back.tags = {"closed"};
    graph.add_edge(back);
    CHECK_EQ(strongly_connected_components(graph, CostProfile::distance()).size(), std::size_t{1});
    CostProfile profile = CostProfile::distance();
    profile.avoid_tags = {"closed"};
    CHECK_EQ(strongly_connected_components(graph, profile).size(), std::size_t{2});
}

TEST(a_very_long_road_does_not_overflow_the_stack) {
    Graph graph;
    constexpr int kCount = 200000;
    for (int i = 0; i < kCount; ++i) graph.add_node("n" + std::to_string(i));
    for (int i = 0; i + 1 < kCount; ++i) graph.add_edge(i, i + 1, 1);
    graph.add_edge(kCount - 1, 0, 1);
    auto components = strongly_connected_components(graph, CostProfile::distance());
    CHECK_EQ(components.size(), std::size_t{1});
    CHECK_EQ(components[0].size(), static_cast<std::size_t>(kCount));
}

TEST(summary_counts) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 2);
    graph.add_edge(a, b, 3);
    graph.add_edge(b, b, 4);
    Edge tagged;
    tagged.from = b;
    tagged.to = a;
    tagged.length = 1;
    tagged.tags = {"x", "y"};
    graph.add_edge(tagged);
    CostProfile profile = CostProfile::distance();
    profile.avoid_tags = {"y"};
    GraphSummary summary = summarize(graph, profile);
    CHECK_EQ(summary.nodes, 2);
    CHECK_EQ(summary.edges, 4);
    CHECK_EQ(summary.closed_edges, 1);
    CHECK_EQ(summary.self_loops, 1);
    CHECK_EQ(summary.parallel_edges, 1);
    CHECK_EQ(summary.components, 2);
    CHECK_EQ(summary.largest_component, 1);
    CHECK_EQ(summary.total_length, 10.0);
    CHECK_EQ(summary.tag_counts.at("x"), 1);
}

TEST(empty_graph_summary) {
    Graph graph;
    GraphSummary summary = summarize(graph, CostProfile::distance());
    CHECK_EQ(summary.components, 0);
    CHECK_EQ(summary.largest_component, 0);
    CHECK(find_issues(graph, CostProfile::distance()).empty());
}

TEST(a_clean_ring_has_no_issues) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 3, 0);
    NodeId c = graph.add_node("c", 0, 4);
    graph.add_edge(a, b, 3);
    graph.add_edge(b, c, 5);
    graph.add_edge(c, a, 4);
    CHECK(find_issues(graph, CostProfile::distance()).empty());
}

TEST(finds_isolated_dead_end_and_no_entry_nodes) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_node("alone");
    graph.add_edge(a, b, 1);
    auto findings = find_issues(graph, CostProfile::distance());
    CHECK(has_code(findings, "isolated-node"));
    CHECK(has_code(findings, "dead-end"));
    CHECK(has_code(findings, "no-entry"));
    CHECK(has_code(findings, "split-network"));
}

TEST(finds_short_and_free_edges) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 10, 0);
    graph.add_edge(a, b, 5);
    graph.add_edge(b, a, 10);
    Edge free_edge;
    free_edge.from = a;
    free_edge.to = a;
    free_edge.length = 0;
    graph.add_edge(free_edge);
    auto findings = find_issues(graph, CostProfile::distance());
    CHECK(has_code(findings, "short-edge"));
    CHECK(has_code(findings, "zero-cost-edge"));
}

TEST(finds_turn_traps_as_problems) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId ba = graph.add_edge(b, a, 1);
    graph.ban_turn(ab, ba);
    auto findings = find_issues(graph, CostProfile::distance());
    CHECK(has_code(findings, "turn-trap"));
    auto trap = std::find_if(findings.begin(), findings.end(), [](const Finding& f) { return f.code == "turn-trap"; });
    CHECK(trap->severity == Severity::Problem);
}

TEST(no_exit_at_all_is_a_dead_end_and_not_a_trap) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 1);
    CHECK(!has_code(find_issues(graph, CostProfile::distance()), "turn-trap"));
}
