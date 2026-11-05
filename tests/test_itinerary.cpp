#include "check.hpp"
#include "routeplan/itinerary.hpp"

using namespace routeplan;

namespace {

EdgeId street(Graph& graph, NodeId from, NodeId to, const std::string& name) {
    Edge edge;
    edge.from = from;
    edge.to = to;
    edge.length = graph.straight_distance(from, to);
    edge.time = edge.length / 2;
    edge.name = name;
    return graph.add_edge(std::move(edge));
}

Route route_of(const Graph& graph, NodeId source, std::vector<EdgeId> edges) {
    return Router(graph, CostProfile::distance()).make_route(source, std::move(edges), 0);
}

}  // namespace

TEST(bearings_on_the_plane) {
    Graph graph(Coordinates::Plane);
    NodeId o = graph.add_node("o", 0, 0);
    NodeId n = graph.add_node("n", 0, 1);
    NodeId e = graph.add_node("e", 1, 0);
    NodeId s = graph.add_node("s", 0, -1);
    NodeId w = graph.add_node("w", -1, 0);
    CHECK_NEAR(edge_bearing(graph, graph.edge(graph.add_edge(o, n, 1))), 0, 1e-9);
    CHECK_NEAR(edge_bearing(graph, graph.edge(graph.add_edge(o, e, 1))), 90, 1e-9);
    CHECK_NEAR(edge_bearing(graph, graph.edge(graph.add_edge(o, s, 1))), 180, 1e-9);
    CHECK_NEAR(edge_bearing(graph, graph.edge(graph.add_edge(o, w, 1))), 270, 1e-9);
}

TEST(bearings_on_the_globe) {
    Graph graph(Coordinates::Geographic);
    NodeId o = graph.add_node("o", 10, 50);
    NodeId north = graph.add_node("n", 10, 51);
    NodeId east = graph.add_node("e", 10.01, 50);
    CHECK_NEAR(edge_bearing(graph, graph.edge(graph.add_edge(o, north, 1))), 0, 1e-6);
    CHECK_NEAR(edge_bearing(graph, graph.edge(graph.add_edge(o, east, 1))), 90, 0.01);
}

TEST(turn_angle_wraps) {
    CHECK_EQ(turn_angle(0, 90), 90.0);
    CHECK_EQ(turn_angle(0, 270), -90.0);
    CHECK_EQ(turn_angle(350, 10), 20.0);
    CHECK_EQ(turn_angle(10, 350), -20.0);
    CHECK_EQ(turn_angle(0, 180), 180.0);
    CHECK_EQ(turn_angle(180, 0), 180.0);
}

TEST(classifies_turns) {
    CHECK(classify_turn(5) == Maneuver::Continue);
    CHECK(classify_turn(-14) == Maneuver::Continue);
    CHECK(classify_turn(30) == Maneuver::SlightRight);
    CHECK(classify_turn(-30) == Maneuver::SlightLeft);
    CHECK(classify_turn(90) == Maneuver::Right);
    CHECK(classify_turn(-90) == Maneuver::Left);
    CHECK(classify_turn(150) == Maneuver::SharpRight);
    CHECK(classify_turn(-150) == Maneuver::SharpLeft);
    CHECK(classify_turn(175) == Maneuver::UTurn);
    CHECK(classify_turn(-180) == Maneuver::UTurn);
}

TEST(merges_a_street_with_several_edges) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 0, 100);
    NodeId c = graph.add_node("c", 10, 200);
    EdgeId first = street(graph, a, b, "Mill Road");
    EdgeId second = street(graph, b, c, "Mill Road");
    auto steps = build_itinerary(graph, route_of(graph, a, {first, second}));
    CHECK_EQ(steps.size(), std::size_t{1});
    CHECK(steps[0].maneuver == Maneuver::Depart);
    CHECK_EQ(steps[0].edges.size(), std::size_t{2});
    CHECK_EQ(steps[0].end_node, c);
    CHECK(steps[0].length > 200);
}

TEST(reports_a_turn_onto_a_new_street) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 0, 100);
    NodeId c = graph.add_node("c", 100, 100);
    NodeId d = graph.add_node("d", 100, 200);
    auto steps = build_itinerary(graph, route_of(graph, a, {street(graph, a, b, "One"), street(graph, b, c, "Two"), street(graph, c, d, "Three")}));
    CHECK_EQ(steps.size(), std::size_t{3});
    CHECK(steps[1].maneuver == Maneuver::Right);
    CHECK(steps[2].maneuver == Maneuver::Left);
    CHECK_EQ(steps[1].street, std::string("Two"));
}

TEST(a_hard_turn_on_the_same_street_is_its_own_step) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 0, 100);
    NodeId c = graph.add_node("c", 100, 100);
    auto steps = build_itinerary(graph, route_of(graph, a, {street(graph, a, b, "Bend"), street(graph, b, c, "Bend")}));
    CHECK_EQ(steps.size(), std::size_t{2});
    CHECK(steps[1].maneuver == Maneuver::Right);
}

TEST(unnamed_edges_merge_only_going_straight) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 0, 100);
    NodeId c = graph.add_node("c", 0, 200);
    NodeId d = graph.add_node("d", 30, 300);
    auto steps = build_itinerary(graph, route_of(graph, a, {street(graph, a, b, ""), street(graph, b, c, ""), street(graph, c, d, "")}));
    CHECK_EQ(steps.size(), std::size_t{2});
    CHECK(steps[1].maneuver == Maneuver::SlightRight);
}

TEST(reversing_along_an_edge_is_a_u_turn) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 0, 100);
    auto steps = build_itinerary(graph, route_of(graph, a, {street(graph, a, b, "Lane"), street(graph, b, a, "Lane")}));
    CHECK_EQ(steps.size(), std::size_t{2});
    CHECK(steps[1].maneuver == Maneuver::UTurn);
}

TEST(no_geometry_still_gives_steps_by_name) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    NodeId d = graph.add_node("d");
    auto steps = build_itinerary(graph, route_of(graph, a, {street(graph, a, b, "One"), street(graph, b, c, "One"), street(graph, c, d, "Two")}));
    CHECK_EQ(steps.size(), std::size_t{2});
    CHECK(steps[1].maneuver == Maneuver::Continue);
}

TEST(an_edge_of_no_length_keeps_the_previous_bearing) {
    Graph graph(Coordinates::Plane);
    NodeId a = graph.add_node("a", 0, 0);
    NodeId b = graph.add_node("b", 0, 100);
    NodeId same = graph.add_node("same", 0, 100);
    NodeId c = graph.add_node("c", 100, 100);
    auto steps = build_itinerary(graph, route_of(graph, a, {street(graph, a, b, "One"), street(graph, b, same, "One"), street(graph, same, c, "Two")}));
    CHECK_EQ(steps.size(), std::size_t{2});
    CHECK(steps[1].maneuver == Maneuver::Right);
}

TEST(an_empty_route_has_no_steps) {
    Graph graph;
    NodeId a = graph.add_node("a");
    CHECK(build_itinerary(graph, route_of(graph, a, {})).empty());
}

TEST(formats_lengths) {
    Graph geographic(Coordinates::Geographic);
    CHECK_EQ(format_length(geographic, 400), std::string("400 m"));
    CHECK_EQ(format_length(geographic, 999.6), std::string("1000 m"));
    CHECK_EQ(format_length(geographic, 2500), std::string("2.5 km"));
    Graph plane(Coordinates::Plane);
    CHECK_EQ(format_length(plane, 12.5), std::string("12.5"));
}

TEST(describes_steps) {
    Graph graph(Coordinates::Geographic);
    Step step;
    step.street = "Mill Road";
    step.length = 400;
    step.maneuver = Maneuver::Left;
    CHECK_EQ(describe_step(graph, step), std::string("Turn left onto Mill Road and go 400 m"));
    step.maneuver = Maneuver::Depart;
    step.street = "";
    CHECK_EQ(describe_step(graph, step), std::string("Start on the unnamed road and go 400 m"));
}
