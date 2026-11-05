// Reading and writing the JSON graph format.
#pragma once

#include <map>
#include <stdexcept>
#include <string>

#include "routeplan/graph.hpp"
#include "routeplan/json.hpp"
#include "routeplan/profile.hpp"

namespace routeplan {

class FormatError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct GraphDocument {
    Graph graph;
    std::map<std::string, CostProfile> profiles;  // The file's own, plus "distance" and "time".
};

// Throws FormatError, naming the element at fault, for anything malformed.
GraphDocument load_graph(const json::Value& document);
GraphDocument load_graph_text(const std::string& text);
GraphDocument load_graph_file(const std::string& path);

json::Value graph_to_json(const Graph& graph);

}  // namespace routeplan
