// Turning a route into directions a person could follow.
#pragma once

#include <string>
#include <vector>

#include "routeplan/graph.hpp"
#include "routeplan/router.hpp"

namespace routeplan {

enum class Maneuver {
    Depart,
    Continue,  // A new street with no geometry to say which way, or straight on.
    SlightLeft,
    Left,
    SharpLeft,
    SlightRight,
    Right,
    SharpRight,
    UTurn,
};

const char* maneuver_name(Maneuver maneuver);

struct Step {
    Maneuver maneuver = Maneuver::Depart;
    std::string street;  // Empty when the edges have no name.
    double length = 0;
    double time = 0;
    std::vector<EdgeId> edges;
    NodeId end_node = 0;
};

// Compass bearing of an edge in degrees from north or +y, clockwise, in [0, 360).
// Meaningless when the edge's ends coincide or the graph has no coordinates.
double edge_bearing(const Graph& graph, const Edge& edge);

// The turn between two bearings; right is positive and the range is (-180, 180].
double turn_angle(double from_bearing, double to_bearing);

Maneuver classify_turn(double angle);

// Consecutive edges merge when they share a non-empty street name and the turn between them
// is gentle. Unnamed edges merge only when going straight on.
std::vector<Step> build_itinerary(const Graph& graph, const Route& route);

// Metres and kilometres for geographic graphs, a bare number otherwise.
std::string format_length(const Graph& graph, double length);

// For example "Turn left onto Mill Road and go 400 m".
std::string describe_step(const Graph& graph, const Step& step);

}  // namespace routeplan
