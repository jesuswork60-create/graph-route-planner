#include "check.hpp"
#include "routeplan/reach.hpp"

using namespace routeplan;

namespace {

struct Fixture {
    Graph graph;
    NodeId a, b, c, d;
    EdgeId ab, bc, ad;
    Fixture() {
        a = graph.add_node("a");
        b = graph.add_node("b");
        c = graph.add_node("c");
        d = graph.add_node("d");
        ab = graph.add_edge(a, b, 2);
        bc = graph.add_edge(b, c, 4);
        ad = graph.add_edge(a, d, 10);
    }
};

}  // namespace

TEST(lists_nodes_cheapest_first) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ReachResult result = reachable_within(router, f.a, 6);
    CHECK_EQ(result.nodes.size(), std::size_t{3});
    CHECK_EQ(result.nodes[0].node, f.a);
    CHECK_EQ(result.nodes[1].node, f.b);
    CHECK_EQ(result.nodes[2].node, f.c);
    CHECK_EQ(result.nodes[2].cost, 6.0);
}

TEST(frontier_shows_how_far_along_the_budget_runs_out) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ReachResult result = reachable_within(router, f.a, 3);
    CHECK_EQ(result.nodes.size(), std::size_t{2});
    CHECK_EQ(result.frontier.size(), std::size_t{2});
    CHECK_EQ(result.frontier[0].edge, f.bc);
    CHECK_NEAR(result.frontier[0].fraction, 0.25, 1e-9);
    CHECK_EQ(result.frontier[1].edge, f.ad);
    CHECK_NEAR(result.frontier[1].fraction, 0.3, 1e-9);
}

TEST(a_budget_ending_exactly_on_a_node_has_no_partial_edge_beyond_it) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ReachResult result = reachable_within(router, f.a, 2);
    CHECK_EQ(result.nodes.size(), std::size_t{2});
    CHECK_EQ(result.frontier.size(), std::size_t{1});
    CHECK_EQ(result.frontier[0].edge, f.ad);
}

TEST(zero_budget_reaches_only_the_source) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ReachResult result = reachable_within(router, f.a, 0);
    CHECK_EQ(result.nodes.size(), std::size_t{1});
    CHECK(result.frontier.empty());
}

TEST(negative_budget_reaches_nothing) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    CHECK(reachable_within(router, f.a, -1).nodes.empty());
}

TEST(an_edge_covered_another_way_is_not_on_the_frontier) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    EdgeId long_way = graph.add_edge(a, b, 5);
    graph.add_edge(a, c, 1);
    EdgeId from_c = graph.add_edge(c, b, 8);
    EdgeId onward = graph.add_edge(b, c, 1);
    Router router(graph, CostProfile::distance());
    ReachResult result = reachable_within(router, a, 6);
    (void)long_way;
    for (const PartialEdge& partial : result.frontier) CHECK(partial.edge != onward);
    CHECK_EQ(result.frontier.size(), std::size_t{1});
    CHECK_EQ(result.frontier[0].edge, from_c);
}

TEST(a_banned_turn_keeps_a_node_out_of_reach) {
    Fixture f;
    f.graph.ban_turn(f.ab, f.bc);
    Router router(f.graph, CostProfile::distance());
    ReachResult result = reachable_within(router, f.a, 100);
    CHECK_EQ(result.nodes.size(), std::size_t{3});
    for (const ReachedNode& reached : result.nodes) CHECK(reached.node != f.c);
}

TEST(closed_edges_are_not_on_the_frontier) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    Edge closed;
    closed.from = a;
    closed.to = b;
    closed.length = 10;
    closed.tags = {"gate"};
    graph.add_edge(closed);
    CostProfile profile = CostProfile::distance();
    profile.avoid_tags = {"gate"};
    Router router(graph, profile);
    CHECK(reachable_within(router, a, 5).frontier.empty());
}
