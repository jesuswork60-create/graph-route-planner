// Checks and summaries for spotting mistakes in a graph file.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "routeplan/graph.hpp"
#include "routeplan/profile.hpp"

namespace routeplan {

// Strongly connected components over the edges this profile leaves open, ignoring turn
// rules. Each is sorted by node id; the list is sorted largest first, then by first node.
std::vector<std::vector<NodeId>> strongly_connected_components(const Graph& graph, const CostProfile& profile);

struct GraphSummary {
    int nodes = 0;
    int edges = 0;
    int closed_edges = 0;
    int self_loops = 0;
    int parallel_edges = 0;  // Edges beyond the first between the same ordered pair of nodes.
    int components = 0;
    int largest_component = 0;
    double total_length = 0;
    std::map<std::string, int> tag_counts;
};

GraphSummary summarize(const Graph& graph, const CostProfile& profile);

enum class Severity { Warning, Problem };

struct Finding {
    Severity severity = Severity::Warning;
    std::string code;
    std::string message;
};

// Things that are legal but probably wrong, in a stable order:
//   isolated-node   no edges at all
//   dead-end        can be entered but has no way out
//   no-entry        can be left but never entered
//   split-network   more than one strongly connected component
//   short-edge      length below the straight-line distance between its ends
//   zero-cost-edge  costs nothing under this profile
//   turn-trap       turn rules leave no way out of the edge's end, though edges leave it
std::vector<Finding> find_issues(const Graph& graph, const CostProfile& profile);

}  // namespace routeplan
