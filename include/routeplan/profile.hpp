// How edges are priced for a particular kind of traveller.
#pragma once

#include <map>
#include <set>
#include <stdexcept>
#include <string>

#include "routeplan/graph.hpp"
#include "routeplan/json.hpp"

namespace routeplan {

enum class UTurnPolicy {
    Allow,        // Always possible, at the cost of u_turn_penalty.
    DeadEndOnly,  // Only where no other way out exists.
    Forbid,
};

class ProfileError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct CostProfile {
    std::string name = "distance";
    double distance_weight = 1;
    double time_weight = 0;
    std::map<std::string, double> tag_multipliers;  // Multiply together for all of an edge's tags.
    std::set<std::string> avoid_tags;               // An edge with any of these is closed.
    UTurnPolicy u_turns = UTurnPolicy::Allow;
    double u_turn_penalty = 0;

    static CostProfile distance();
    static CostProfile travel_time();

    // Reads an object with any of these keys: distance_weight, time_weight, tag_multipliers,
    // avoid, u_turns ("allow", "dead_end_only", "forbid") and u_turn_penalty.
    static CostProfile from_json(const std::string& name, const json::Value& value);

    // Throws ProfileError for weights or penalties that would allow negative or useless costs.
    void validate() const;

    // Infinity when the edge is closed to this profile.
    double edge_cost(const Edge& edge) const;
};

}  // namespace routeplan
