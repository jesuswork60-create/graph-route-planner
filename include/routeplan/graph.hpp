// The road network: a directed multigraph with named nodes and edges, plus turn restrictions.
#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace routeplan {

using NodeId = int;
using EdgeId = int;

enum class Coordinates {
    None,        // Nodes have no position, so A* falls back to plain Dijkstra.
    Plane,       // x and y in the same unit as edge lengths.
    Geographic,  // x is longitude and y latitude in degrees; lengths are in metres.
};

struct Node {
    std::string key;
    double x = 0;
    double y = 0;
};

struct Edge {
    std::string key;
    NodeId from = 0;
    NodeId to = 0;
    double length = 0;
    double time = 0;
    std::string name;  // Street name for directions; may be empty.
    std::vector<std::string> tags;

    bool has_tag(const std::string& tag) const;
};

class GraphError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Graph {
public:
    explicit Graph(Coordinates coordinates = Coordinates::None) : coordinates_(coordinates) {}

    // Keys must be unique. Lengths and times must be finite and not negative.
    NodeId add_node(const std::string& key, double x = 0, double y = 0);
    EdgeId add_edge(Edge edge);
    EdgeId add_edge(NodeId from, NodeId to, double length);

    // Travelling from `from_edge` straight into `to_edge` is not allowed.
    void ban_turn(EdgeId from_edge, EdgeId to_edge);
    // From `from_edge`, only the edges named this way may be taken at its end node.
    void require_turn(EdgeId from_edge, EdgeId to_edge);
    bool turn_allowed(EdgeId from_edge, EdgeId to_edge) const;

    int node_count() const { return static_cast<int>(nodes_.size()); }
    int edge_count() const { return static_cast<int>(edges_.size()); }
    const Node& node(NodeId id) const { return nodes_.at(static_cast<std::size_t>(id)); }
    const Edge& edge(EdgeId id) const { return edges_.at(static_cast<std::size_t>(id)); }
    const std::vector<EdgeId>& out_edges(NodeId id) const { return out_.at(static_cast<std::size_t>(id)); }
    const std::vector<EdgeId>& in_edges(NodeId id) const { return in_.at(static_cast<std::size_t>(id)); }

    std::optional<NodeId> find_node(const std::string& key) const;
    std::optional<EdgeId> find_edge(const std::string& key) const;
    NodeId require_node(const std::string& key) const;

    Coordinates coordinates() const { return coordinates_; }

    // Straight-line distance in the coordinate system's unit; 0 when there are no coordinates.
    double straight_distance(NodeId a, NodeId b) const;

    const std::unordered_map<EdgeId, std::unordered_set<EdgeId>>& banned_turns() const { return banned_; }
    const std::unordered_map<EdgeId, std::unordered_set<EdgeId>>& required_turns() const { return required_; }

private:
    void check_turn(EdgeId from_edge, EdgeId to_edge) const;

    Coordinates coordinates_;
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::vector<std::vector<EdgeId>> out_;
    std::vector<std::vector<EdgeId>> in_;
    std::unordered_map<std::string, NodeId> node_keys_;
    std::unordered_map<std::string, EdgeId> edge_keys_;
    std::unordered_map<EdgeId, std::unordered_set<EdgeId>> banned_;
    std::unordered_map<EdgeId, std::unordered_set<EdgeId>> required_;
};

// Great-circle distance in metres between two (longitude, latitude) points in degrees.
double haversine_metres(double lon1, double lat1, double lon2, double lat2);

}  // namespace routeplan
