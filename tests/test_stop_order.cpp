#include <algorithm>
#include <cstdint>
#include <cstdlib>

#include "check.hpp"
#include "routeplan/stop_order.hpp"

using namespace routeplan;

namespace {

// A row of towns along a line, so the best order is obvious.
Graph line_graph(int count) {
    Graph graph;
    for (int i = 0; i < count; ++i) graph.add_node("t" + std::to_string(i));
    for (int i = 0; i + 1 < count; ++i) {
        graph.add_edge(i, i + 1, 1);
        graph.add_edge(i + 1, i, 1);
    }
    return graph;
}

}  // namespace

TEST(matrix_has_a_row_per_source) {
    Graph graph = line_graph(3);
    Router router(graph, CostProfile::distance());
    auto matrix = cost_matrix(router, {0, 2}, {0, 1, 2});
    CHECK_EQ(matrix.size(), std::size_t{2});
    CHECK(matrix[0] == (std::vector<double>{0, 1, 2}));
    CHECK(matrix[1] == (std::vector<double>{2, 1, 0}));
}

TEST(matrix_marks_unreachable_with_infinity) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    graph.add_edge(a, b, 1);
    Router router(graph, CostProfile::distance());
    auto matrix = cost_matrix(router, {b}, {a});
    CHECK_EQ(matrix[0][0], kInfinity);
}

TEST(orders_stops_along_a_line) {
    Graph graph = line_graph(6);
    Router router(graph, CostProfile::distance());
    auto order = plan_stop_order(router, 0, {3, 1, 5, 2});
    CHECK(order.has_value());
    CHECK(order->exact);
    CHECK_EQ(order->cost, 5.0);
    CHECK(order->order == (std::vector<NodeId>{0, 1, 2, 3, 5}));
}

TEST(a_fixed_end_changes_the_order) {
    Graph graph = line_graph(5);
    Router router(graph, CostProfile::distance());
    auto order = plan_stop_order(router, 2, {1, 3}, NodeId{2});
    CHECK(order.has_value());
    CHECK_EQ(order->cost, 4.0);
    CHECK_EQ(order->order.front(), 2);
    CHECK_EQ(order->order.back(), 2);
    CHECK_EQ(order->order.size(), std::size_t{4});
}

TEST(no_stops_is_fine) {
    Graph graph = line_graph(3);
    Router router(graph, CostProfile::distance());
    auto alone = plan_stop_order(router, 1, {});
    CHECK(alone.has_value());
    CHECK(alone->order == std::vector<NodeId>{1});
    CHECK_EQ(alone->cost, 0.0);
    auto to_end = plan_stop_order(router, 0, {}, NodeId{2});
    CHECK_EQ(to_end->cost, 2.0);
}

TEST(returns_nothing_when_a_stop_cannot_be_reached) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    graph.add_edge(a, b, 1);
    Router router(graph, CostProfile::distance());
    CHECK(!plan_stop_order(router, a, {b, c}).has_value());
    CHECK(!plan_stop_order(router, a, {}, c).has_value());
}

TEST(respects_one_way_streets) {
    // A one-way ring: only one order works cheaply.
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    graph.add_edge(a, b, 1);
    graph.add_edge(b, c, 1);
    graph.add_edge(c, a, 1);
    Router router(graph, CostProfile::distance());
    auto order = plan_stop_order(router, a, {c, b});
    CHECK(order->order == (std::vector<NodeId>{a, b, c}));
    CHECK_EQ(order->cost, 2.0);
}

TEST(exact_search_beats_every_other_order) {
    Graph graph;
    constexpr int kCount = 7;
    for (int i = 0; i < kCount; ++i) graph.add_node("n" + std::to_string(i));
    std::uint32_t state = 12345;
    for (int i = 0; i < kCount; ++i) {
        for (int j = 0; j < kCount; ++j) {
            if (i == j) continue;
            state = state * 1664525u + 1013904223u;
            graph.add_edge(i, j, 1 + (state >> 16) % 50);
        }
    }
    Router router(graph, CostProfile::distance());
    std::vector<NodeId> stops{1, 2, 3, 4, 5, 6};
    auto table = cost_matrix(router, {0, 1, 2, 3, 4, 5, 6}, {0, 1, 2, 3, 4, 5, 6});
    double best = kInfinity;
    std::vector<NodeId> permutation = stops;
    do {
        double total = table[0][static_cast<std::size_t>(permutation[0])];
        for (std::size_t i = 0; i + 1 < permutation.size(); ++i) {
            total += table[static_cast<std::size_t>(permutation[i])][static_cast<std::size_t>(permutation[i + 1])];
        }
        best = std::min(best, total);
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    auto order = plan_stop_order(router, 0, stops);
    CHECK_NEAR(order->cost, best, 1e-9);
}

TEST(many_stops_use_the_heuristic_and_visit_each_once) {
    Graph graph = line_graph(kExactStopLimit + 5);
    Router router(graph, CostProfile::distance());
    std::vector<NodeId> stops;
    for (int i = kExactStopLimit + 4; i >= 1; --i) stops.push_back(i);
    auto order = plan_stop_order(router, 0, stops);
    CHECK(order.has_value());
    CHECK(!order->exact);
    CHECK_EQ(order->cost, static_cast<double>(kExactStopLimit + 4));
    std::vector<NodeId> seen(order->order.begin() + 1, order->order.end());
    std::sort(seen.begin(), seen.end());
    std::sort(stops.begin(), stops.end());
    CHECK(seen == stops);
}

TEST(two_opt_untangles_a_zigzag) {
    // Each nearest stop lies on the far side of the start, so nearest neighbour alone goes back and forth.
    Graph graph = line_graph(121);
    Router router(graph, CostProfile::distance());
    const std::vector<NodeId> stops{61, 58, 65, 53, 70, 47, 77, 40, 85, 32, 94, 22, 105, 10, 118};
    NodeId here = 60;
    std::vector<NodeId> left = stops;
    double zigzag = 0;
    while (!left.empty()) {
        auto nearest = std::min_element(left.begin(), left.end(), [&](NodeId a, NodeId b) { return std::abs(a - here) < std::abs(b - here); });
        zigzag += std::abs(*nearest - here);
        here = *nearest;
        left.erase(nearest);
    }
    auto order = plan_stop_order(router, 60, stops);
    CHECK(order.has_value());
    CHECK(!order->exact);
    CHECK(order->cost < zigzag);
    // Nothing can beat going to the nearer end first and then all the way to the other.
    CHECK(order->cost >= 50 + 108);
}

TEST(the_heuristic_can_miss_the_best_order) {
    // From the middle of a line the best tour starts towards the nearer end, costing 43. Nearest
    // neighbour picks the other way on a tie and no single reversal fixes it. The README lists this.
    Graph graph = line_graph(30);
    Router router(graph, CostProfile::distance());
    std::vector<NodeId> stops;
    for (int i = 0; i < 30; ++i) {
        if (i != 15) stops.push_back(i);
    }
    auto order = plan_stop_order(router, 15, stops);
    CHECK(order.has_value());
    CHECK(!order->exact);
    CHECK_EQ(order->cost, 44.0);
}
