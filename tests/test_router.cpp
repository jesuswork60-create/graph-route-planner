#include <cstdint>

#include "check.hpp"
#include "routeplan/router.hpp"

using namespace routeplan;

namespace {

// A plain Bellman-Ford over edge states, as an independent answer to check searches against.
double slow_best_cost(const Router& router, NodeId source, NodeId target) {
    const Graph& graph = router.graph();
    auto edges = static_cast<std::size_t>(graph.edge_count());
    std::vector<double> best(edges, kInfinity);
    double answer = source == target ? 0 : kInfinity;
    for (EdgeId edge : graph.out_edges(source)) best[static_cast<std::size_t>(edge)] = router.edge_cost(edge);
    for (std::size_t round = 0; round <= edges; ++round) {
        for (EdgeId edge = 0; edge < graph.edge_count(); ++edge) {
            double known = best[static_cast<std::size_t>(edge)];
            if (known == kInfinity) continue;
            for (EdgeId next : graph.out_edges(graph.edge(edge).to)) {
                double total = known + router.edge_cost(next) + router.transition_cost(edge, next);
                if (total < best[static_cast<std::size_t>(next)]) best[static_cast<std::size_t>(next)] = total;
            }
        }
    }
    for (EdgeId edge = 0; edge < graph.edge_count(); ++edge) {
        if (graph.edge(edge).to == target) answer = std::min(answer, best[static_cast<std::size_t>(edge)]);
    }
    return answer;
}

class Random {
public:
    explicit Random(std::uint32_t seed) : state_(seed) {}
    std::uint32_t next() {
        state_ = state_ * 1664525u + 1013904223u;
        return state_ >> 8;
    }
    int below(int limit) { return static_cast<int>(next() % static_cast<std::uint32_t>(limit)); }

private:
    std::uint32_t state_;
};

Graph random_graph(std::uint32_t seed, int nodes, int edges) {
    Random random(seed);
    Graph graph(Coordinates::Plane);
    for (int i = 0; i < nodes; ++i) {
        graph.add_node("n" + std::to_string(i), random.below(1000), random.below(1000));
    }
    std::vector<EdgeId> ids;
    for (int i = 0; i < edges; ++i) {
        NodeId from = random.below(nodes);
        NodeId to = random.below(nodes);
        Edge edge;
        edge.from = from;
        edge.to = to;
        // Longer than the crow flies by a random factor, so the graph is plausible.
        edge.length = graph.straight_distance(from, to) * (1 + random.below(100) / 50.0) + 1;
        edge.time = edge.length / (1 + random.below(5));
        if (random.below(4) == 0) edge.tags.push_back("rough");
        ids.push_back(graph.add_edge(std::move(edge)));
    }
    for (int i = 0; i < edges / 5; ++i) {
        EdgeId first = ids[static_cast<std::size_t>(random.below(edges))];
        const auto& exits = graph.out_edges(graph.edge(first).to);
        if (exits.empty()) continue;
        graph.ban_turn(first, exits[static_cast<std::size_t>(random.below(static_cast<int>(exits.size())))]);
    }
    return graph;
}

struct Fixture {
    Graph graph;
    NodeId a, b, c, d;
    EdgeId ab, bc, ad, dc, ac;
    Fixture() {
        a = graph.add_node("a");
        b = graph.add_node("b");
        c = graph.add_node("c");
        d = graph.add_node("d");
        ab = graph.add_edge(a, b, 1);
        bc = graph.add_edge(b, c, 1);
        ad = graph.add_edge(a, d, 2);
        dc = graph.add_edge(d, c, 2);
        ac = graph.add_edge(a, c, 10);
    }
};

}  // namespace

TEST(finds_the_cheapest_route) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    auto route = router.route(f.a, f.c);
    CHECK(route.has_value());
    CHECK_EQ(route->cost, 2.0);
    CHECK_EQ(route->length, 2.0);
    CHECK(route->nodes == (std::vector<NodeId>{f.a, f.b, f.c}));
    CHECK(route->edges == (std::vector<EdgeId>{f.ab, f.bc}));
}

TEST(a_route_to_the_same_node_is_empty) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    auto route = router.route(f.a, f.a);
    CHECK(route.has_value());
    CHECK(route->edges.empty());
    CHECK(route->nodes == std::vector<NodeId>{f.a});
    CHECK_EQ(route->cost, 0.0);
}

TEST(no_route_against_the_direction) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    CHECK(!router.route(f.c, f.a).has_value());
}

TEST(takes_the_cheaper_parallel_edge) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 9);
    EdgeId cheap = graph.add_edge(a, b, 2);
    graph.add_edge(a, b, 5);
    Router router(graph, CostProfile::distance());
    CHECK(router.route(a, b)->edges == std::vector<EdgeId>{cheap});
}

TEST(a_banned_turn_changes_the_route) {
    Fixture f;
    f.graph.ban_turn(f.ab, f.bc);
    Router router(f.graph, CostProfile::distance());
    auto route = router.route(f.a, f.c);
    CHECK(route->edges == (std::vector<EdgeId>{f.ad, f.dc}));
    CHECK_EQ(route->cost, 4.0);
}

TEST(an_only_rule_changes_the_route) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    NodeId d = graph.add_node("d");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId bc = graph.add_edge(b, c, 1);
    EdgeId bd = graph.add_edge(b, d, 1);
    EdgeId dc = graph.add_edge(d, c, 1);
    graph.require_turn(ab, bd);
    Router router(graph, CostProfile::distance());
    CHECK(router.route(a, c)->edges == (std::vector<EdgeId>{ab, bd, dc}));
    (void)bc;
}

TEST(a_turn_ban_can_make_a_node_be_visited_twice) {
    // Reaching b from a may not go straight to c, but looping round b first is fine.
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    NodeId loop = graph.add_node("loop");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId bc = graph.add_edge(b, c, 1);
    EdgeId out = graph.add_edge(b, loop, 1);
    EdgeId back = graph.add_edge(loop, b, 1);
    graph.ban_turn(ab, bc);
    Router router(graph, CostProfile::distance());
    auto route = router.route(a, c);
    CHECK(route.has_value());
    CHECK(route->edges == (std::vector<EdgeId>{ab, out, back, bc}));
    CHECK(route->nodes == (std::vector<NodeId>{a, b, loop, b, c}));
}

TEST(arriving_edge_applies_to_the_first_step) {
    Fixture f;
    f.graph.ban_turn(f.ab, f.bc);
    Router router(f.graph, CostProfile::distance());
    RouteQuery query;
    query.source = f.b;
    query.target = f.c;
    CHECK(router.route(query).has_value());
    query.arrived_by = f.ab;
    CHECK(!router.route(query).has_value());
}

TEST(u_turns_cost_the_penalty_when_allowed) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId ba = graph.add_edge(b, a, 1);
    CostProfile profile = CostProfile::distance();
    profile.u_turn_penalty = 7;
    Router router(graph, profile);
    CHECK_EQ(router.transition_cost(ab, ba), 7.0);
    CHECK_EQ(router.path_cost({ab, ba}), 9.0);
}

TEST(u_turns_can_be_forbidden) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId ba = graph.add_edge(b, a, 1);
    graph.add_edge(a, c, 1);
    CostProfile profile = CostProfile::distance();
    profile.u_turns = UTurnPolicy::Forbid;
    Router router(graph, profile);
    CHECK_EQ(router.transition_cost(ab, ba), kInfinity);
    RouteQuery query;
    query.source = b;
    query.target = c;
    query.arrived_by = ab;
    CHECK(!router.route(query).has_value());
}

TEST(dead_end_u_turns_only_at_dead_ends) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId ba = graph.add_edge(b, a, 1);
    EdgeId ac = graph.add_edge(a, c, 1);
    EdgeId ca = graph.add_edge(c, a, 1);
    graph.add_edge(a, b, 3);
    CostProfile profile = CostProfile::distance();
    profile.u_turns = UTurnPolicy::DeadEndOnly;
    profile.u_turn_penalty = 4;
    Router router(graph, profile);
    // b only leads back to a, so turning there is fine; a has other ways on.
    CHECK_EQ(router.transition_cost(ab, ba), 4.0);
    CHECK_EQ(router.transition_cost(ba, ab), kInfinity);
    CHECK_EQ(router.transition_cost(ac, ca), 4.0);
}

TEST(a_closed_exit_makes_a_dead_end) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId ba = graph.add_edge(b, a, 1);
    Edge closed;
    closed.from = b;
    closed.to = c;
    closed.length = 1;
    closed.tags = {"private"};
    graph.add_edge(closed);
    CostProfile profile = CostProfile::distance();
    profile.u_turns = UTurnPolicy::DeadEndOnly;
    profile.avoid_tags = {"private"};
    Router router(graph, profile);
    CHECK_EQ(router.transition_cost(ab, ba), 0.0);
}

TEST(avoid_tags_close_edges) {
    Fixture f;
    Edge toll;
    toll.from = f.a;
    toll.to = f.c;
    toll.length = 0.5;
    toll.tags = {"toll"};
    EdgeId id = f.graph.add_edge(toll);
    CostProfile profile = CostProfile::distance();
    CHECK(Router(f.graph, profile).route(f.a, f.c)->edges == std::vector<EdgeId>{id});
    profile.avoid_tags = {"toll"};
    CHECK_EQ(Router(f.graph, profile).route(f.a, f.c)->cost, 2.0);
}

TEST(time_profile_prefers_the_fast_road) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    Edge slow;
    slow.from = a;
    slow.to = b;
    slow.length = 10;
    slow.time = 20;
    Edge fast;
    fast.from = a;
    fast.to = b;
    fast.length = 30;
    fast.time = 5;
    EdgeId slow_id = graph.add_edge(slow);
    EdgeId fast_id = graph.add_edge(fast);
    CHECK(Router(graph, CostProfile::distance()).route(a, b)->edges == std::vector<EdgeId>{slow_id});
    auto timed = Router(graph, CostProfile::travel_time()).route(a, b);
    CHECK(timed->edges == std::vector<EdgeId>{fast_id});
    CHECK_EQ(timed->cost, 5.0);
    CHECK_EQ(timed->length, 30.0);
}

TEST(masks_exclude_edges_and_nodes) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    std::vector<char> edges(static_cast<std::size_t>(f.graph.edge_count()));
    edges[static_cast<std::size_t>(f.ab)] = 1;
    RouteQuery query;
    query.source = f.a;
    query.target = f.c;
    query.banned_edges = &edges;
    CHECK_EQ(router.route(query)->cost, 4.0);
    std::vector<char> nodes(static_cast<std::size_t>(f.graph.node_count()));
    nodes[static_cast<std::size_t>(f.d)] = 1;
    query.banned_nodes = &nodes;
    CHECK_EQ(router.route(query)->cost, 10.0);
}

TEST(max_cost_cuts_the_search_off) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    RouteQuery query;
    query.source = f.a;
    query.target = f.c;
    query.max_cost = 2;
    CHECK(router.route(query).has_value());
    query.max_cost = 1.5;
    CHECK(!router.route(query).has_value());
}

TEST(rejects_an_invalid_profile) {
    Fixture f;
    CostProfile profile = CostProfile::distance();
    profile.distance_weight = -1;
    CHECK_THROWS(Router(f.graph, profile), ProfileError);
}

TEST(explore_gives_costs_and_respects_budget) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    Exploration all = router.explore(f.a);
    CHECK_EQ(all.node_cost[static_cast<std::size_t>(f.a)], 0.0);
    CHECK_EQ(all.node_cost[static_cast<std::size_t>(f.c)], 2.0);
    CHECK_EQ(all.state_cost[static_cast<std::size_t>(f.ac)], 10.0);
    Exploration near = router.explore(f.a, 1);
    CHECK_EQ(near.node_cost[static_cast<std::size_t>(f.b)], 1.0);
    CHECK_EQ(near.node_cost[static_cast<std::size_t>(f.c)], kInfinity);
}

TEST(path_cost_spots_a_banned_step) {
    Fixture f;
    f.graph.ban_turn(f.ab, f.bc);
    Router router(f.graph, CostProfile::distance());
    CHECK_EQ(router.path_cost({f.ab, f.bc}), kInfinity);
    CHECK_EQ(router.path_cost({f.ad, f.dc}), 4.0);
    CHECK_EQ(router.path_cost({}), 0.0);
}

TEST(heuristic_never_overestimates_scaled_costs) {
    // A profile that makes a long road cheap would break a heuristic based on raw distance.
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 100, 0);
    NodeId c = graph.add_node("c", 50, 10);
    Edge motorway;
    motorway.from = a;
    motorway.to = c;
    motorway.length = 60;
    motorway.tags = {"fast"};
    Edge rest;
    rest.from = c;
    rest.to = b;
    rest.length = 60;
    rest.tags = {"fast"};
    Edge direct;
    direct.from = a;
    direct.to = b;
    direct.length = 100;
    graph.add_edge(motorway);
    graph.add_edge(rest);
    graph.add_edge(direct);
    CostProfile profile = CostProfile::distance();
    profile.tag_multipliers = {{"fast", 0.1}};
    Router router(graph, profile);
    CHECK(router.heuristic(a, b) <= 12.0);
    CHECK_NEAR(router.route(a, b)->cost, 12.0, 1e-9);
}

TEST(heuristic_is_zero_without_coordinates) {
    Fixture f;
    Router router(f.graph, CostProfile::distance());
    CHECK_EQ(router.heuristic(f.a, f.c), 0.0);
}

TEST(heuristic_saves_work_on_a_grid) {
    Graph graph(Coordinates::Plane);
    constexpr int kSide = 20;
    for (int y = 0; y < kSide; ++y) {
        for (int x = 0; x < kSide; ++x) graph.add_node(std::to_string(x) + "," + std::to_string(y), x, y);
    }
    auto id = [&](int x, int y) { return y * kSide + x; };
    for (int y = 0; y < kSide; ++y) {
        for (int x = 0; x < kSide; ++x) {
            if (x + 1 < kSide) {
                graph.add_edge(id(x, y), id(x + 1, y), 1);
                graph.add_edge(id(x + 1, y), id(x, y), 1);
            }
            if (y + 1 < kSide) {
                graph.add_edge(id(x, y), id(x, y + 1), 1);
                graph.add_edge(id(x, y + 1), id(x, y), 1);
            }
        }
    }
    Router router(graph, CostProfile::distance());
    RouteQuery query;
    query.source = id(5, 5);
    query.target = id(15, 5);
    SearchStats guided;
    SearchStats plain;
    auto first = router.route(query, &guided);
    query.use_heuristic = false;
    auto second = router.route(query, &plain);
    CHECK_EQ(first->cost, 10.0);
    CHECK_EQ(second->cost, 10.0);
    CHECK(guided.settled < plain.settled);
}

TEST(geographic_heuristic_keeps_the_result) {
    Graph graph(Coordinates::Geographic);
    NodeId a = graph.add_node("a", 10.00, 50.00);
    NodeId b = graph.add_node("b", 10.01, 50.00);
    NodeId c = graph.add_node("c", 10.02, 50.00);
    NodeId d = graph.add_node("d", 10.01, 50.02);
    auto link = [&](NodeId from, NodeId to) { graph.add_edge(from, to, graph.straight_distance(from, to)); };
    link(a, b);
    link(b, c);
    link(a, d);
    link(d, c);
    Router router(graph, CostProfile::distance());
    RouteQuery query;
    query.source = a;
    query.target = c;
    double with = router.route(query)->cost;
    query.use_heuristic = false;
    CHECK_NEAR(with, router.route(query)->cost, 1e-6);
    CHECK_NEAR(with, graph.straight_distance(a, c), 1e-3);
}

TEST(agrees_with_an_independent_search_on_random_graphs) {
    for (std::uint32_t seed = 1; seed <= 12; ++seed) {
        Graph graph = random_graph(seed, 25, 80);
        for (int variant = 0; variant < 3; ++variant) {
            CostProfile profile = CostProfile::distance();
            if (variant == 1) profile = CostProfile::travel_time();
            if (variant == 2) {
                profile.tag_multipliers = {{"rough", 3}};
                profile.u_turn_penalty = 50;
            }
            Router router(graph, profile);
            Random pick(seed * 31);
            for (int trial = 0; trial < 40; ++trial) {
                NodeId from = pick.below(25);
                NodeId to = pick.below(25);
                double expected = slow_best_cost(router, from, to);
                RouteQuery query;
                query.source = from;
                query.target = to;
                auto guided = router.route(query);
                query.use_heuristic = false;
                auto plain = router.route(query);
                CHECK_EQ(guided.has_value(), expected != kInfinity);
                CHECK_EQ(plain.has_value(), expected != kInfinity);
                if (guided && plain) {
                    CHECK_NEAR(guided->cost, expected, 1e-6);
                    CHECK_NEAR(plain->cost, expected, 1e-6);
                    CHECK_NEAR(router.path_cost(guided->edges), expected, 1e-6);
                    CHECK_NEAR(router.explore(from).node_cost[static_cast<std::size_t>(to)], expected, 1e-6);
                }
            }
        }
    }
}
