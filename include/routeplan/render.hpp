// Output formats shared by the command line tool.
#pragma once

#include <string>
#include <vector>

#include "routeplan/analysis.hpp"
#include "routeplan/graph.hpp"
#include "routeplan/itinerary.hpp"
#include "routeplan/json.hpp"
#include "routeplan/reach.hpp"
#include "routeplan/router.hpp"
#include "routeplan/stop_order.hpp"

namespace routeplan {

// Shortest form that keeps six significant digits, such as 12.5 or 1200.
std::string format_cost(double value);

std::string route_text(const Graph& graph, const Route& route);
json::Value route_json(const Graph& graph, const Route& route);

json::Value itinerary_json(const Graph& graph, const std::vector<Step>& steps);
json::Value reach_json(const Graph& graph, const ReachResult& reach);
json::Value stop_order_json(const Graph& graph, const StopOrder& order);
json::Value findings_json(const std::vector<Finding>& findings);
json::Value summary_json(const GraphSummary& summary);

// Graphviz source. Edges on `highlight` are drawn thicker and in red.
std::string graph_dot(const Graph& graph, const std::vector<EdgeId>& highlight = {});

}  // namespace routeplan
