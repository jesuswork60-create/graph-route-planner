#include <set>

#include "check.hpp"
#include "routeplan/alternatives.hpp"

using namespace routeplan;

namespace {

// Three ways from a to d, costing 3, 4 and 5, with a cross link making a fourth.
struct Fixture {
    Graph graph;
    NodeId a, b, c, d;
    EdgeId ab, bd, ac, cd, ad, bc;
    Fixture() {
        a = graph.add_node("a");
        b = graph.add_node("b");
        c = graph.add_node("c");
        d = graph.add_node("d");
        ab = graph.add_edge(a, b, 1);
        bd = graph.add_edge(b, d, 2);
        ac = graph.add_edge(a, c, 2);
        cd = graph.add_edge(c, d, 2);
        ad = graph.add_edge(a, d, 5);
        bc = graph.add_edge(b, c, 3);
    }
};

}  // namespace

TEST(lists_routes_in_cost_order) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    auto routes = k_shortest_routes(router, f.a, f.d, 10);
    CHECK_EQ(routes.size(), std::size_t{4});
    CHECK_EQ(routes[0].cost, 3.0);
    CHECK_EQ(routes[1].cost, 4.0);
    CHECK_EQ(routes[2].cost, 5.0);
    CHECK_EQ(routes[3].cost, 6.0);
    CHECK(routes[3].edges == (std::vector<EdgeId>{f.ab, f.bc, f.cd}));
}

TEST(stops_at_the_count) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    CHECK_EQ(k_shortest_routes(router, f.a, f.d, 2).size(), std::size_t{2});
    CHECK(k_shortest_routes(router, f.a, f.d, 0).empty());
    CHECK(k_shortest_routes(router, f.a, f.d, -1).empty());
}

TEST(no_route_gives_an_empty_list) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    CHECK(k_shortest_routes(router, f.d, f.a, 3).empty());
}

TEST(same_node_gives_one_empty_route) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    auto routes = k_shortest_routes(router, f.a, f.a, 3);
    CHECK_EQ(routes.size(), std::size_t{1});
    CHECK(routes[0].edges.empty());
}

TEST(parallel_edges_count_as_different_routes) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 1);
    graph.add_edge(a, b, 2);
    Router router(graph, CostProfile::distance());
    auto routes = k_shortest_routes(router, a, b, 5);
    CHECK_EQ(routes.size(), std::size_t{2});
    CHECK_EQ(routes[1].cost, 2.0);
}

TEST(honours_turn_rules_in_deviations) {
    Fixture f;
    f.graph.ban_turn(f.ab, f.bc);
    Router router(f.graph, CostProfile::distance());
    auto routes = k_shortest_routes(router, f.a, f.d, 10);
    CHECK_EQ(routes.size(), std::size_t{3});
    for (const Route& route : routes) CHECK(router.path_cost(route.edges) != kInfinity);
}

TEST(costs_match_the_edges_including_penalties) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    graph.add_edge(a, b, 1);
    graph.add_edge(b, a, 1);
    graph.add_edge(a, c, 3);
    graph.add_edge(b, c, 3);
    CostProfile profile = CostProfile::distance();
    profile.u_turn_penalty = 2;
    Router router(graph, profile);
    for (const Route& route : k_shortest_routes(router, a, c, 5)) {
        CHECK_NEAR(route.cost, router.path_cost(route.edges), 1e-9);
    }
}

TEST(routes_are_all_different_on_a_grid) {
    Graph graph;
    constexpr int kSide = 4;
    for (int i = 0; i < kSide * kSide; ++i) graph.add_node("n" + std::to_string(i));
    for (int y = 0; y < kSide; ++y) {
        for (int x = 0; x < kSide; ++x) {
            int here = y * kSide + x;
            if (x + 1 < kSide) graph.add_edge(here, here + 1, 1 + (x + y) % 3);
            if (y + 1 < kSide) graph.add_edge(here, here + kSide, 1 + (x * y) % 2);
        }
    }
    Router router(graph, CostProfile::distance());
    auto routes = k_shortest_routes(router, 0, kSide * kSide - 1, 20);
    // A 4 by 4 grid heading only right and down has exactly 20 monotone paths.
    CHECK_EQ(routes.size(), std::size_t{20});
    std::set<std::vector<EdgeId>> distinct;
    double previous = 0;
    for (const Route& route : routes) {
        distinct.insert(route.edges);
        CHECK(route.cost >= previous);
        previous = route.cost;
    }
    CHECK_EQ(distinct.size(), routes.size());
}
