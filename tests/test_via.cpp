#include "check.hpp"
#include "routeplan/via.hpp"

using namespace routeplan;

namespace {

struct Fixture {
    Graph graph;
    NodeId a, b, c, d;
    EdgeId ab, bc, bd, dc;
    Fixture() {
        a = graph.add_node("a");
        b = graph.add_node("b");
        c = graph.add_node("c");
        d = graph.add_node("d");
        ab = graph.add_edge(a, b, 1);
        bc = graph.add_edge(b, c, 1);
        bd = graph.add_edge(b, d, 1);
        dc = graph.add_edge(d, c, 1);
    }
};

}  // namespace

TEST(joins_legs_in_order) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ViaResult result = route_via(router, {f.a, f.b, f.d, f.c});
    CHECK(result.ok());
    CHECK_EQ(result.legs.size(), std::size_t{3});
    CHECK_EQ(result.total.cost, 3.0);
    CHECK(result.total.edges == (std::vector<EdgeId>{f.ab, f.bd, f.dc}));
    CHECK(result.total.nodes == (std::vector<NodeId>{f.a, f.b, f.d, f.c}));
}

TEST(needs_a_waypoint) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    CHECK_THROWS(route_via(router, {}), std::invalid_argument);
}

TEST(a_single_waypoint_is_an_empty_route) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ViaResult result = route_via(router, {f.b});
    CHECK(result.ok());
    CHECK(result.legs.empty());
    CHECK(result.total.nodes == std::vector<NodeId>{f.b});
}

TEST(reports_the_failed_leg) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ViaResult result = route_via(router, {f.a, f.c, f.a, f.b});
    CHECK(!result.ok());
    CHECK_EQ(*result.failed_leg, std::size_t{1});
    CHECK_EQ(result.legs.size(), std::size_t{1});
}

TEST(a_turn_rule_applies_across_a_waypoint) {
    Fixture f;
    f.graph.ban_turn(f.ab, f.bc);
    Router router(f.graph, CostProfile::distance());
    ViaResult result = route_via(router, {f.a, f.b, f.c});
    CHECK(result.ok());
    CHECK(result.total.edges == (std::vector<EdgeId>{f.ab, f.bd, f.dc}));
    CHECK_EQ(result.total.cost, 3.0);
}

TEST(a_turn_rule_can_make_the_next_leg_impossible) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId bc = graph.add_edge(b, c, 1);
    graph.ban_turn(ab, bc);
    Router router(graph, CostProfile::distance());
    ViaResult result = route_via(router, {a, b, c});
    CHECK(!result.ok());
    CHECK_EQ(*result.failed_leg, std::size_t{1});
}

TEST(repeated_waypoints_add_nothing) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    ViaResult result = route_via(router, {f.a, f.a, f.b, f.b});
    CHECK(result.ok());
    CHECK_EQ(result.total.cost, 1.0);
    CHECK_EQ(result.total.edges.size(), std::size_t{1});
}

TEST(u_turn_penalty_counts_across_a_waypoint) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 1);
    graph.add_edge(b, a, 1);
    CostProfile profile = CostProfile::distance();
    profile.u_turn_penalty = 5;
    Router router(graph, profile);
    CHECK_EQ(route_via(router, {a, b, a}).total.cost, 7.0);
}
