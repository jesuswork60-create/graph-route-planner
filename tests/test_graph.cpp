#include "check.hpp"
#include "routeplan/graph.hpp"
#include "routeplan/graph_io.hpp"

using namespace routeplan;

TEST(adds_nodes_and_edges) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    EdgeId e = graph.add_edge(a, b, 5);
    CHECK_EQ(graph.node_count(), 2);
    CHECK_EQ(graph.edge_count(), 1);
    CHECK_EQ(graph.out_edges(a).size(), std::size_t{1});
    CHECK_EQ(graph.in_edges(b).size(), std::size_t{1});
    CHECK_EQ(graph.edge(e).key, std::string("#0"));
    CHECK_EQ(graph.edge(e).time, 5.0);
    CHECK(graph.find_node("a").has_value());
    CHECK(!graph.find_node("z").has_value());
    CHECK(graph.find_edge("#0").has_value());
}

TEST(rejects_bad_elements) {
    Graph graph;
    NodeId a = graph.add_node("a");
    CHECK_THROWS(graph.add_node("a"), GraphError);
    CHECK_THROWS(graph.add_node(""), GraphError);
    CHECK_THROWS(graph.add_edge(a, 7, 1), GraphError);
    CHECK_THROWS(graph.add_edge(a, a, -1), GraphError);
    CHECK_THROWS(graph.require_node("z"), GraphError);
    Edge edge;
    edge.key = "same";
    graph.add_edge(edge);
    CHECK_THROWS(graph.add_edge(edge), GraphError);
}

TEST(checks_geographic_ranges) {
    Graph graph(Coordinates::Geographic);
    graph.add_node("ok", 179, 89);
    CHECK_THROWS(graph.add_node("lat", 0, 91), GraphError);
    CHECK_THROWS(graph.add_node("lon", 181, 0), GraphError);
}

TEST(turn_rules_need_edges_that_meet) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId bc = graph.add_edge(b, c, 1);
    EdgeId ca = graph.add_edge(c, a, 1);
    CHECK_THROWS(graph.ban_turn(ab, ca), GraphError);
    CHECK_THROWS(graph.require_turn(ab, 99), GraphError);
    CHECK(graph.turn_allowed(ab, bc));
    graph.ban_turn(ab, bc);
    CHECK(!graph.turn_allowed(ab, bc));
}

TEST(only_rules_exclude_the_others) {
    Graph graph;
    NodeId a = graph.add_node("a");
    NodeId b = graph.add_node("b");
    NodeId c = graph.add_node("c");
    NodeId d = graph.add_node("d");
    EdgeId ab = graph.add_edge(a, b, 1);
    EdgeId bc = graph.add_edge(b, c, 1);
    EdgeId bd = graph.add_edge(b, d, 1);
    graph.require_turn(ab, bc);
    CHECK(graph.turn_allowed(ab, bc));
    CHECK(!graph.turn_allowed(ab, bd));
}

TEST(straight_distance_by_coordinate_system) {
    Graph none;
    CHECK_EQ(none.straight_distance(none.add_node("a", 1, 1), none.add_node("b", 4, 5)), 0.0);
    Graph plane(Coordinates::Plane);
    CHECK_NEAR(plane.straight_distance(plane.add_node("a", 1, 1), plane.add_node("b", 4, 5)), 5.0, 1e-9);
}

TEST(haversine_matches_known_distance) {
    // One degree of latitude is about 111.2 km.
    CHECK_NEAR(haversine_metres(10, 50, 10, 51), 111195, 50);
    CHECK_EQ(haversine_metres(10, 50, 10, 50), 0.0);
    // Halfway around the world along the equator.
    CHECK_NEAR(haversine_metres(0, 0, 180, 0), 20015115, 100);
}

TEST(loads_a_full_document) {
    GraphDocument doc = load_graph_text(R"({
        "coordinates": "plane",
        "nodes": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 3, "y": 4}, {"id": "c", "x": 3, "y": 8}],
        "edges": [
            {"id": "ab", "from": "a", "to": "b", "speed": 2, "name": "First", "tags": ["x"], "two_way": true},
            {"id": "bc", "from": "b", "to": "c", "length": 9, "time": 3}
        ],
        "turn_restrictions": [{"type": "no", "from": "ab", "to": "bc"}],
        "profiles": {"mine": {"avoid": ["x"], "u_turns": "forbid"}}
    })");
    const Graph& graph = doc.graph;
    CHECK_EQ(graph.edge_count(), 3);
    const Edge& ab = graph.edge(*graph.find_edge("ab"));
    CHECK_NEAR(ab.length, 5.0, 1e-9);
    CHECK_NEAR(ab.time, 2.5, 1e-9);
    CHECK_EQ(ab.name, std::string("First"));
    CHECK(ab.has_tag("x"));
    const Edge& reverse = graph.edge(*graph.find_edge("ab~rev"));
    CHECK_EQ(reverse.from, ab.to);
    CHECK_EQ(reverse.to, ab.from);
    CHECK(!graph.turn_allowed(*graph.find_edge("ab"), *graph.find_edge("bc")));
    CHECK_EQ(doc.profiles.size(), std::size_t{3});
    CHECK(doc.profiles.at("mine").u_turns == UTurnPolicy::Forbid);
}

TEST(geographic_nodes_use_lon_and_lat) {
    GraphDocument doc = load_graph_text(R"({
        "coordinates": "geographic",
        "nodes": [{"id": "a", "lon": 10, "lat": 50}, {"id": "b", "lon": 10, "lat": 51}],
        "edges": [{"from": "a", "to": "b"}]
    })");
    CHECK_EQ(doc.graph.node(0).x, 10.0);
    CHECK_EQ(doc.graph.node(0).y, 50.0);
    CHECK_NEAR(doc.graph.edge(0).length, 111195, 50);
}

TEST(reports_bad_documents) {
    const char* bad[] = {
        "not json",
        "[]",
        R"({})",
        R"({"nodes": 1})",
        R"({"nodes": [], "surprise": 1})",
        R"({"nodes": [{"x": 1}]})",
        R"({"nodes": [{"id": "a", "colour": "red"}]})",
        R"({"coordinates": "polar", "nodes": []})",
        R"({"coordinates": "plane", "nodes": [{"id": "a"}]})",
        R"({"nodes": [{"id": "a"}, {"id": "a"}]})",
        R"({"nodes": [{"id": "a"}], "edges": [{"from": "a", "to": "z", "length": 1}]})",
        R"({"nodes": [{"id": "a"}], "edges": [{"from": "a", "to": "a"}]})",
        R"({"nodes": [{"id": "a"}], "edges": [{"from": "a", "to": "a", "length": -1}]})",
        R"({"nodes": [{"id": "a"}], "edges": [{"from": "a", "to": "a", "length": 1, "time": 1, "speed": 1}]})",
        R"({"nodes": [{"id": "a"}], "edges": [{"from": "a", "to": "a", "length": 1, "speed": 0}]})",
        R"({"nodes": [{"id": "a"}], "edges": [{"from": "a", "to": "a", "length": 1, "two_way": true}]})",
        R"({"nodes": [{"id": "a"}], "turn_restrictions": [{"type": "no", "from": "x", "to": "y"}]})",
        R"({"nodes": [{"id": "a"}], "profiles": {"p": {"distance_weight": -1}}})",
        R"({"nodes": [{"id": "a"}], "profiles": {"p": 3}})",
    };
    for (const char* text : bad) CHECK_THROWS(load_graph_text(text), FormatError);
}

TEST(restriction_type_is_checked) {
    const char* text = R"({"nodes": [{"id": "a"}], "edges": [{"id": "e", "from": "a", "to": "a", "length": 1}],
        "turn_restrictions": [{"type": "maybe", "from": "e", "to": "e"}]})";
    CHECK_THROWS(load_graph_text(text), FormatError);
}

TEST(a_profile_in_the_file_can_replace_a_builtin) {
    GraphDocument doc = load_graph_text(R"({"nodes": [], "profiles": {"distance": {"time_weight": 2}}})");
    CHECK_EQ(doc.profiles.at("distance").time_weight, 2.0);
}

TEST(missing_file_is_a_format_error) {
    CHECK_THROWS(load_graph_file("this-file-does-not-exist.json"), FormatError);
}

TEST(json_output_loads_back_to_the_same_graph) {
    GraphDocument doc = load_graph_text(R"({
        "coordinates": "plane",
        "nodes": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 3, "y": 4}],
        "edges": [{"id": "ab", "from": "a", "to": "b", "time": 7, "name": "N", "tags": ["t"]},
                  {"id": "ba", "from": "b", "to": "a", "length": 6}],
        "turn_restrictions": [{"type": "only", "from": "ab", "to": "ba"}]
    })");
    std::string first = json::dump(graph_to_json(doc.graph));
    GraphDocument again = load_graph_text(first);
    CHECK_EQ(json::dump(graph_to_json(again.graph)), first);
    CHECK_EQ(again.graph.edge(0).time, 7.0);
    CHECK(!again.graph.required_turns().empty());
}
