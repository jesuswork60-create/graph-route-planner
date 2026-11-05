#include "routeplan/profile.hpp"

#include <cmath>
#include <limits>

namespace routeplan {

CostProfile CostProfile::distance() {
    return CostProfile{};
}

CostProfile CostProfile::travel_time() {
    CostProfile profile;
    profile.name = "time";
    profile.distance_weight = 0;
    profile.time_weight = 1;
    return profile;
}

CostProfile CostProfile::from_json(const std::string& name, const json::Value& value) {
    CostProfile profile;
    profile.name = name;
    try {
        for (const json::Value::Member& member : value.as_object()) {
            const std::string& key = member.first;
            const json::Value& item = member.second;
            if (key == "distance_weight") {
                profile.distance_weight = item.as_number();
            } else if (key == "time_weight") {
                profile.time_weight = item.as_number();
            } else if (key == "u_turn_penalty") {
                profile.u_turn_penalty = item.as_number();
            } else if (key == "tag_multipliers") {
                for (const json::Value::Member& tag : item.as_object()) {
                    profile.tag_multipliers[tag.first] = tag.second.as_number();
                }
            } else if (key == "avoid") {
                for (const json::Value& tag : item.as_array()) {
                    profile.avoid_tags.insert(tag.as_string());
                }
            } else if (key == "u_turns") {
                const std::string& policy = item.as_string();
                if (policy == "allow") profile.u_turns = UTurnPolicy::Allow;
                else if (policy == "dead_end_only") profile.u_turns = UTurnPolicy::DeadEndOnly;
                else if (policy == "forbid") profile.u_turns = UTurnPolicy::Forbid;
                else throw ProfileError("profile " + name + ": unknown u_turns value " + policy);
            } else {
                throw ProfileError("profile " + name + ": unknown key " + key);
            }
        }
    } catch (const json::TypeError& e) {
        throw ProfileError("profile " + name + ": " + e.what());
    }
    profile.validate();
    return profile;
}

void CostProfile::validate() const {
    if (!std::isfinite(distance_weight) || distance_weight < 0) throw ProfileError("profile " + name + ": distance_weight must not be negative");
    if (!std::isfinite(time_weight) || time_weight < 0) throw ProfileError("profile " + name + ": time_weight must not be negative");
    if (distance_weight == 0 && time_weight == 0) throw ProfileError("profile " + name + ": at least one weight must be above zero");
    if (!std::isfinite(u_turn_penalty) || u_turn_penalty < 0) throw ProfileError("profile " + name + ": u_turn_penalty must not be negative");
    for (const auto& entry : tag_multipliers) {
        if (!std::isfinite(entry.second) || entry.second <= 0) {
            throw ProfileError("profile " + name + ": the multiplier for " + entry.first + " must be above zero");
        }
    }
}

double CostProfile::edge_cost(const Edge& edge) const {
    double multiplier = 1;
    for (const std::string& tag : edge.tags) {
        if (avoid_tags.count(tag) > 0) return std::numeric_limits<double>::infinity();
        auto it = tag_multipliers.find(tag);
        if (it != tag_multipliers.end()) multiplier *= it->second;
    }
    return (distance_weight * edge.length + time_weight * edge.time) * multiplier;
}

}  // namespace routeplan
