#include "check.hpp"
#include "routeplan/profile.hpp"

using namespace routeplan;

namespace {

Edge sample_edge() {
    Edge edge;
    edge.length = 100;
    edge.time = 10;
    edge.tags = {"steep", "cobbled"};
    return edge;
}

}  // namespace

TEST(builtin_profiles) {
    CHECK_EQ(CostProfile::distance().edge_cost(sample_edge()), 100.0);
    CHECK_EQ(CostProfile::travel_time().edge_cost(sample_edge()), 10.0);
}

TEST(weights_combine) {
    CostProfile profile;
    profile.distance_weight = 0.5;
    profile.time_weight = 3;
    CHECK_EQ(profile.edge_cost(sample_edge()), 80.0);
}

TEST(multipliers_stack) {
    CostProfile profile;
    profile.tag_multipliers = {{"steep", 2}, {"cobbled", 1.5}, {"unused", 100}};
    CHECK_EQ(profile.edge_cost(sample_edge()), 300.0);
}

TEST(avoid_closes_the_edge) {
    CostProfile profile;
    profile.avoid_tags = {"steep"};
    CHECK_EQ(profile.edge_cost(sample_edge()), std::numeric_limits<double>::infinity());
}

TEST(reads_json) {
    CostProfile profile = CostProfile::from_json("p", json::parse(R"({
        "distance_weight": 2, "time_weight": 1, "tag_multipliers": {"a": 3}, "avoid": ["b", "c"],
        "u_turns": "dead_end_only", "u_turn_penalty": 9})"));
    CHECK_EQ(profile.name, std::string("p"));
    CHECK_EQ(profile.distance_weight, 2.0);
    CHECK_EQ(profile.time_weight, 1.0);
    CHECK_EQ(profile.tag_multipliers.at("a"), 3.0);
    CHECK_EQ(profile.avoid_tags.size(), std::size_t{2});
    CHECK(profile.u_turns == UTurnPolicy::DeadEndOnly);
    CHECK_EQ(profile.u_turn_penalty, 9.0);
}

TEST(rejects_bad_json) {
    const char* bad[] = {
        R"({"distance_weight": -1})",
        R"({"distance_weight": 0})",
        R"({"u_turn_penalty": -2})",
        R"({"tag_multipliers": {"a": 0}})",
        R"({"tag_multipliers": {"a": -1}})",
        R"({"u_turns": "sometimes"})",
        R"({"speed": 1})",
        R"({"avoid": "a"})",
        R"([])",
    };
    for (const char* text : bad) CHECK_THROWS(CostProfile::from_json("p", json::parse(text)), ProfileError);
}
