#include "routeplan/itinerary.hpp"

#include <cmath>
#include <cstdio>

namespace routeplan {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kStraightLimit = 15;
constexpr double kSlightLimit = 45;
constexpr double kNormalLimit = 135;
constexpr double kSharpLimit = 170;

bool same_place(const Graph& graph, const Edge& edge) {
    const Node& a = graph.node(edge.from);
    const Node& b = graph.node(edge.to);
    return a.x == b.x && a.y == b.y;
}

std::string format_number(double value, const char* pattern) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, pattern, value);
    return buffer;
}

}  // namespace

const char* maneuver_name(Maneuver maneuver) {
    switch (maneuver) {
        case Maneuver::Depart: return "depart";
        case Maneuver::Continue: return "continue";
        case Maneuver::SlightLeft: return "slight-left";
        case Maneuver::Left: return "left";
        case Maneuver::SharpLeft: return "sharp-left";
        case Maneuver::SlightRight: return "slight-right";
        case Maneuver::Right: return "right";
        case Maneuver::SharpRight: return "sharp-right";
        case Maneuver::UTurn: return "u-turn";
    }
    return "continue";
}

double edge_bearing(const Graph& graph, const Edge& edge) {
    const Node& a = graph.node(edge.from);
    const Node& b = graph.node(edge.to);
    double degrees = 0;
    if (graph.coordinates() == Coordinates::Geographic) {
        double phi1 = a.y * kPi / 180;
        double phi2 = b.y * kPi / 180;
        double dlambda = (b.x - a.x) * kPi / 180;
        degrees = std::atan2(std::sin(dlambda) * std::cos(phi2),
                             std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlambda)) *
                  180 / kPi;
    } else {
        degrees = std::atan2(b.x - a.x, b.y - a.y) * 180 / kPi;
    }
    return degrees < 0 ? degrees + 360 : degrees;
}

double turn_angle(double from_bearing, double to_bearing) {
    double angle = std::fmod(to_bearing - from_bearing, 360.0);
    if (angle <= -180) angle += 360;
    if (angle > 180) angle -= 360;
    return angle;
}

Maneuver classify_turn(double angle) {
    double size = std::fabs(angle);
    bool right = angle > 0;
    if (size < kStraightLimit) return Maneuver::Continue;
    if (size < kSlightLimit) return right ? Maneuver::SlightRight : Maneuver::SlightLeft;
    if (size < kNormalLimit) return right ? Maneuver::Right : Maneuver::Left;
    if (size < kSharpLimit) return right ? Maneuver::SharpRight : Maneuver::SharpLeft;
    return Maneuver::UTurn;
}

std::vector<Step> build_itinerary(const Graph& graph, const Route& route) {
    std::vector<Step> steps;
    bool has_geometry = graph.coordinates() != Coordinates::None;
    double bearing = 0;
    bool have_bearing = false;
    const Edge* previous = nullptr;
    for (EdgeId id : route.edges) {
        const Edge& edge = graph.edge(id);
        Maneuver maneuver = Maneuver::Continue;
        bool gentle = true;
        bool straight = true;
        bool measurable = has_geometry && !same_place(graph, edge);
        double this_bearing = measurable ? edge_bearing(graph, edge) : bearing;
        if (previous != nullptr && measurable && have_bearing) {
            double angle = turn_angle(bearing, this_bearing);
            maneuver = classify_turn(angle);
            straight = maneuver == Maneuver::Continue;
            gentle = std::fabs(angle) < kSlightLimit;
        }
        if (previous != nullptr && edge.from == previous->to && edge.to == previous->from) {
            maneuver = Maneuver::UTurn;
            gentle = false;
            straight = false;
        }
        bool merges = false;
        if (!steps.empty()) {
            if (edge.name.empty() && previous->name.empty()) merges = straight;
            else merges = edge.name == previous->name && gentle;
        }
        if (merges) {
            Step& step = steps.back();
            step.length += edge.length;
            step.time += edge.time;
            step.edges.push_back(id);
            step.end_node = edge.to;
        } else {
            Step step;
            step.maneuver = steps.empty() ? Maneuver::Depart : maneuver;
            step.street = edge.name;
            step.length = edge.length;
            step.time = edge.time;
            step.edges.push_back(id);
            step.end_node = edge.to;
            steps.push_back(std::move(step));
        }
        if (measurable) {
            bearing = this_bearing;
            have_bearing = true;
        }
        previous = &edge;
    }
    return steps;
}

std::string format_length(const Graph& graph, double length) {
    if (graph.coordinates() == Coordinates::Geographic) {
        if (length < 1000) return format_number(std::round(length), "%.0f m");
        return format_number(length / 1000, "%.1f km");
    }
    return format_number(length, "%g");
}

std::string describe_step(const Graph& graph, const Step& step) {
    std::string street = step.street.empty() ? "the unnamed road" : step.street;
    std::string length = format_length(graph, step.length);
    switch (step.maneuver) {
        case Maneuver::Depart: return "Start on " + street + " and go " + length;
        case Maneuver::Continue: return "Continue onto " + street + " and go " + length;
        case Maneuver::SlightLeft: return "Bear left onto " + street + " and go " + length;
        case Maneuver::Left: return "Turn left onto " + street + " and go " + length;
        case Maneuver::SharpLeft: return "Turn sharp left onto " + street + " and go " + length;
        case Maneuver::SlightRight: return "Bear right onto " + street + " and go " + length;
        case Maneuver::Right: return "Turn right onto " + street + " and go " + length;
        case Maneuver::SharpRight: return "Turn sharp right onto " + street + " and go " + length;
        case Maneuver::UTurn: return "Make a U-turn onto " + street + " and go " + length;
    }
    return "";
}

}  // namespace routeplan
