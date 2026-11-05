#include <cstdio>
#include <fstream>
#include <sstream>

#include "check.hpp"
#include "routeplan/cli.hpp"
#include "routeplan/json.hpp"

using routeplan::run_cli;
namespace json = routeplan::json;

namespace {

const char kGraph[] = R"({
  "coordinates": "plane",
  "nodes": [
    {"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 100, "y": 0},
    {"id": "c", "x": 200, "y": 0}, {"id": "d", "x": 100, "y": 100},
    {"id": "lost", "x": 500, "y": 500}
  ],
  "edges": [
    {"id": "ab", "from": "a", "to": "b", "name": "First", "two_way": true},
    {"id": "bc", "from": "b", "to": "c", "name": "First", "tags": ["toll"], "two_way": true},
    {"id": "bd", "from": "b", "to": "d", "name": "Second", "two_way": true},
    {"id": "dc", "from": "d", "to": "c", "name": "Third", "time": 10, "two_way": true}
  ],
  "profiles": {"no-toll": {"avoid": ["toll"]}}
})";

// One graph file shared by the tests in this process, removed when the process exits.
class GraphFile {
public:
    GraphFile() : path_("route-plan-cli-test-graph.json") {
        std::ofstream out(path_);
        out << kGraph;
    }
    ~GraphFile() { std::remove(path_.c_str()); }
    const std::string& path() const { return path_; }

private:
    std::string path_;
};

const std::string& graph_path() {
    static GraphFile file;
    return file.path();
}

struct Run {
    int code;
    std::string out;
    std::string err;
};

Run run(const std::string& command, std::vector<std::string> rest = {}) {
    std::vector<std::string> args{command, graph_path()};
    args.insert(args.end(), rest.begin(), rest.end());
    std::ostringstream out;
    std::ostringstream err;
    int code = run_cli(args, out, err);
    return Run{code, out.str(), err.str()};
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}  // namespace

TEST(route_prints_nodes_and_totals) {
    Run result = run("route", {"a", "c"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "a -> b -> c"));
    CHECK(contains(result.out, "cost 200"));
}

TEST(route_as_json) {
    Run result = run("route", {"a", "c", "--json"});
    json::Value value = json::parse(result.out);
    CHECK_EQ(value.find("cost")->as_number(), 200.0);
    CHECK_EQ(value.find("edges")->as_array()[1].as_string(), std::string("bc"));
}

TEST(a_profile_from_the_file_is_used) {
    Run result = run("route", {"a", "c", "--profile", "no-toll"});
    CHECK(contains(result.out, "a -> b -> d -> c"));
}

TEST(avoid_adds_to_the_profile) {
    Run result = run("route", {"a", "c", "--avoid", "toll"});
    CHECK(contains(result.out, "a -> b -> d -> c"));
}

TEST(time_profile_uses_edge_times) {
    Run result = run("route", {"d", "c", "--profile", "time"});
    CHECK(contains(result.out, "cost 10"));
}

TEST(unknown_profile_is_a_usage_error) {
    Run result = run("route", {"a", "c", "--profile", "boat"});
    CHECK_EQ(result.code, 2);
    CHECK(contains(result.err, "boat"));
}

TEST(no_route_exits_one) {
    Run result = run("route", {"a", "lost"});
    CHECK_EQ(result.code, 1);
    CHECK(contains(result.err, "no route"));
}

TEST(unknown_node_exits_two) {
    Run result = run("route", {"a", "nowhere"});
    CHECK_EQ(result.code, 2);
    CHECK(contains(result.err, "nowhere"));
}

TEST(stats_go_to_stderr) {
    Run result = run("route", {"a", "c", "--stats"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.err, "settled"));
    CHECK(!contains(result.out, "settled"));
}

TEST(no_heuristic_gives_the_same_answer) {
    CHECK_EQ(run("route", {"a", "c", "--no-heuristic"}).out, run("route", {"a", "c"}).out);
}

TEST(directions_read_like_directions) {
    Run result = run("directions", {"a", "d"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "1. Start on First and go 100"));
    CHECK(contains(result.out, "2. Turn left onto Second and go 100"));
    CHECK(contains(result.out, "Arrive at d"));
}

TEST(directions_as_json) {
    json::Value value = json::parse(run("directions", {"a", "d", "--json"}).out);
    CHECK_EQ(value.as_array().size(), std::size_t{2});
    CHECK_EQ(value.as_array()[1].find("maneuver")->as_string(), std::string("left"));
}

TEST(alternatives_are_numbered) {
    Run result = run("alternatives", {"a", "c", "--count", "2"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "#1 a -> b -> c"));
    CHECK(contains(result.out, "#2 "));
    CHECK(!contains(result.out, "#3 "));
}

TEST(alternatives_count_must_be_positive) {
    CHECK_EQ(run("alternatives", {"a", "c", "--count", "0"}).code, 2);
    CHECK_EQ(run("alternatives", {"a", "c", "--count", "x"}).code, 2);
}

TEST(reach_lists_nodes_and_partial_edges) {
    Run result = run("reach", {"a", "150"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "b 100"));
    CHECK(contains(result.out, "partly along bc 0.5"));
}

TEST(reach_as_json_and_bad_budget) {
    json::Value value = json::parse(run("reach", {"a", "150", "--json"}).out);
    CHECK_EQ(value.find("nodes")->as_array().size(), std::size_t{2});
    CHECK_EQ(run("reach", {"a", "lots"}).code, 2);
}

TEST(via_follows_the_waypoints) {
    Run result = run("via", {"a", "d", "c"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "a -> b -> d -> c"));
    CHECK_EQ(run("via", {"a"}).code, 2);
    CHECK_EQ(run("via", {"a", "lost"}).code, 1);
}

TEST(via_json_has_legs) {
    json::Value value = json::parse(run("via", {"a", "d", "c", "--json"}).out);
    CHECK_EQ(value.find("legs")->as_array().size(), std::size_t{2});
}

TEST(order_finds_a_sequence) {
    Run result = run("order", {"a", "c", "b"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "a -> b -> c"));
    CHECK(contains(result.out, "best order"));
}

TEST(order_with_an_end_and_failure) {
    Run ended = run("order", {"a", "c", "--end", "d", "--json"});
    json::Value value = json::parse(ended.out);
    CHECK_EQ(value.find("order")->as_array().back().as_string(), std::string("d"));
    CHECK_EQ(run("order", {"a", "lost"}).code, 1);
    CHECK_EQ(run("order", {}).code, 2);
}

TEST(matrix_prints_a_table_with_dashes) {
    Run result = run("matrix", {"a,b", "c,lost"});
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "\tc\tlost\n"));
    CHECK(contains(result.out, "a\t200\t-\n"));
}

TEST(matrix_json_uses_null_for_unreachable) {
    json::Value value = json::parse(run("matrix", {"a", "c,lost", "--json"}).out);
    CHECK_EQ(value.as_array()[0].as_array()[0].as_number(), 200.0);
    CHECK(value.as_array()[0].as_array()[1].is_null());
}

TEST(check_lists_warnings_but_passes) {
    Run result = run("check");
    CHECK_EQ(result.code, 0);
    CHECK(contains(result.out, "[isolated-node]"));
    json::Value value = json::parse(run("check", {"--json"}).out);
    CHECK(value.as_array().size() >= 1);
}

TEST(summary_prints_counts) {
    Run result = run("summary");
    CHECK(contains(result.out, "nodes 5"));
    CHECK(contains(result.out, "edges 8"));
    CHECK(contains(result.out, "tag toll 2"));
    json::Value value = json::parse(run("summary", {"--json"}).out);
    CHECK_EQ(value.find("components")->as_number(), 2.0);
}

TEST(dot_marks_the_route) {
    Run plain = run("dot");
    CHECK(contains(plain.out, "digraph"));
    CHECK(!contains(plain.out, "color=red"));
    Run marked = run("dot", {"a", "b"});
    CHECK(contains(marked.out, "color=red"));
    CHECK_EQ(run("dot", {"a"}).code, 2);
}

TEST(usage_errors_exit_two) {
    std::ostringstream out;
    std::ostringstream err;
    CHECK_EQ(run_cli({}, out, err), 2);
    CHECK(contains(err.str(), "usage:"));
    CHECK_EQ(run_cli({"route", graph_path(), "--wings"}, out, err), 2);
    CHECK_EQ(run_cli({"fly", graph_path()}, out, err), 2);
    CHECK_EQ(run_cli({"route", "missing-file.json", "a", "b"}, out, err), 2);
    CHECK_EQ(run_cli({"route", graph_path(), "a", "c", "--profile"}, out, err), 2);
}

TEST(help_prints_usage) {
    std::ostringstream out;
    std::ostringstream err;
    CHECK_EQ(run_cli({"--help"}, out, err), 0);
    CHECK(contains(out.str(), "usage:"));
}
