#include "routeplan/graph.hpp"

#include <algorithm>
#include <cmath>

namespace routeplan {

bool Edge::has_tag(const std::string& tag) const {
    return std::find(tags.begin(), tags.end(), tag) != tags.end();
}

NodeId Graph::add_node(const std::string& key, double x, double y) {
    if (key.empty()) throw GraphError("a node needs a non-empty key");
    if (!std::isfinite(x) || !std::isfinite(y)) throw GraphError("node " + key + " has a position that is not finite");
    if (coordinates_ == Coordinates::Geographic && (std::fabs(y) > 90 || std::fabs(x) > 180)) {
        throw GraphError("node " + key + " is outside the valid range of longitude and latitude");
    }
    auto id = static_cast<NodeId>(nodes_.size());
    if (!node_keys_.emplace(key, id).second) throw GraphError("duplicate node " + key);
    nodes_.push_back(Node{key, x, y});
    out_.emplace_back();
    in_.emplace_back();
    return id;
}

EdgeId Graph::add_edge(Edge edge) {
    if (edge.from < 0 || edge.from >= node_count() || edge.to < 0 || edge.to >= node_count()) {
        throw GraphError("edge " + edge.key + " refers to a node that does not exist");
    }
    if (!std::isfinite(edge.length) || edge.length < 0) throw GraphError("edge " + edge.key + " has a bad length");
    if (!std::isfinite(edge.time) || edge.time < 0) throw GraphError("edge " + edge.key + " has a bad time");
    auto id = static_cast<EdgeId>(edges_.size());
    if (edge.key.empty()) edge.key = "#" + std::to_string(id);
    if (!edge_keys_.emplace(edge.key, id).second) throw GraphError("duplicate edge " + edge.key);
    out_[static_cast<std::size_t>(edge.from)].push_back(id);
    in_[static_cast<std::size_t>(edge.to)].push_back(id);
    edges_.push_back(std::move(edge));
    return id;
}

EdgeId Graph::add_edge(NodeId from, NodeId to, double length) {
    Edge edge;
    edge.from = from;
    edge.to = to;
    edge.length = length;
    edge.time = length;
    return add_edge(std::move(edge));
}

void Graph::check_turn(EdgeId from_edge, EdgeId to_edge) const {
    if (from_edge < 0 || from_edge >= edge_count() || to_edge < 0 || to_edge >= edge_count()) {
        throw GraphError("turn restriction refers to an edge that does not exist");
    }
    if (edge(from_edge).to != edge(to_edge).from) {
        throw GraphError("turn restriction from " + edge(from_edge).key + " to " + edge(to_edge).key +
                         ": the edges do not meet");
    }
}

void Graph::ban_turn(EdgeId from_edge, EdgeId to_edge) {
    check_turn(from_edge, to_edge);
    banned_[from_edge].insert(to_edge);
}

void Graph::require_turn(EdgeId from_edge, EdgeId to_edge) {
    check_turn(from_edge, to_edge);
    required_[from_edge].insert(to_edge);
}

bool Graph::turn_allowed(EdgeId from_edge, EdgeId to_edge) const {
    auto banned = banned_.find(from_edge);
    if (banned != banned_.end() && banned->second.count(to_edge) > 0) return false;
    auto required = required_.find(from_edge);
    return required == required_.end() || required->second.count(to_edge) > 0;
}

std::optional<NodeId> Graph::find_node(const std::string& key) const {
    auto it = node_keys_.find(key);
    if (it == node_keys_.end()) return std::nullopt;
    return it->second;
}

std::optional<EdgeId> Graph::find_edge(const std::string& key) const {
    auto it = edge_keys_.find(key);
    if (it == edge_keys_.end()) return std::nullopt;
    return it->second;
}

NodeId Graph::require_node(const std::string& key) const {
    auto id = find_node(key);
    if (!id) throw GraphError("no node called " + key);
    return *id;
}

double Graph::straight_distance(NodeId a, NodeId b) const {
    const Node& first = node(a);
    const Node& second = node(b);
    switch (coordinates_) {
        case Coordinates::None: return 0;
        case Coordinates::Plane: return std::hypot(first.x - second.x, first.y - second.y);
        case Coordinates::Geographic: return haversine_metres(first.x, first.y, second.x, second.y);
    }
    return 0;
}

double haversine_metres(double lon1, double lat1, double lon2, double lat2) {
    constexpr double kEarthRadius = 6371008.8;
    constexpr double kRadians = 3.14159265358979323846 / 180.0;
    double phi1 = lat1 * kRadians;
    double phi2 = lat2 * kRadians;
    double dphi = (lat2 - lat1) * kRadians;
    double dlambda = (lon2 - lon1) * kRadians;
    double a = std::sin(dphi / 2) * std::sin(dphi / 2) +
               std::cos(phi1) * std::cos(phi2) * std::sin(dlambda / 2) * std::sin(dlambda / 2);
    return 2 * kEarthRadius * std::asin(std::min(1.0, std::sqrt(a)));
}

}  // namespace routeplan
