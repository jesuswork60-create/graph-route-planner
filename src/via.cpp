#include "routeplan/via.hpp"

#include <stdexcept>

namespace routeplan {

ViaResult route_via(const Router& router, const std::vector<NodeId>& waypoints) {
    if (waypoints.empty()) throw std::invalid_argument("a route needs at least one waypoint");
    ViaResult result;
    std::vector<EdgeId> edges;
    double cost = 0;
    std::optional<EdgeId> arrived_by;
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
        RouteQuery query;
        query.source = waypoints[i];
        query.target = waypoints[i + 1];
        query.arrived_by = arrived_by;
        auto leg = router.route(query);
        if (!leg) {
            result.failed_leg = i;
            break;
        }
        cost += leg->cost;
        edges.insert(edges.end(), leg->edges.begin(), leg->edges.end());
        if (!leg->edges.empty()) arrived_by = leg->edges.back();
        result.legs.push_back(std::move(*leg));
    }
    result.total = router.make_route(waypoints.front(), std::move(edges), cost);
    return result;
}

}  // namespace routeplan
