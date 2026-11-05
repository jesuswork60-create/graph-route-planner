#include "routeplan/stop_order.hpp"

#include <algorithm>
#include <cstdint>
#include <numeric>

namespace routeplan {

std::vector<std::vector<double>> cost_matrix(const Router& router, const std::vector<NodeId>& sources,
                                             const std::vector<NodeId>& targets) {
    std::vector<std::vector<double>> matrix;
    matrix.reserve(sources.size());
    for (NodeId source : sources) {
        Exploration explored = router.explore(source);
        std::vector<double> row;
        row.reserve(targets.size());
        for (NodeId target : targets) row.push_back(explored.node_cost[static_cast<std::size_t>(target)]);
        matrix.push_back(std::move(row));
    }
    return matrix;
}

namespace {

using Table = std::vector<std::vector<double>>;

// Index 0 of the table is the start; 1..n are the stops; n + 1 is the end when there is one.
struct Problem {
    const Table& table;
    int stops;
    bool has_end;

    double closing_cost(int last) const {
        return has_end ? table[static_cast<std::size_t>(last)][static_cast<std::size_t>(stops) + 1] : 0.0;
    }
    double cost(int a, int b) const { return table[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)]; }
};

// Held-Karp over subsets of stops.
std::optional<std::pair<std::vector<int>, double>> solve_exact(const Problem& problem) {
    int n = problem.stops;
    if (n == 0) {
        double closing = problem.closing_cost(0);
        if (closing == kInfinity) return std::nullopt;
        return std::make_pair(std::vector<int>{}, closing);
    }
    std::size_t subsets = std::size_t{1} << n;
    std::vector<double> best(subsets * static_cast<std::size_t>(n), kInfinity);
    std::vector<std::int8_t> from(best.size(), -1);
    auto at = [&](std::size_t subset, int last) { return subset * static_cast<std::size_t>(n) + static_cast<std::size_t>(last); };
    for (int i = 0; i < n; ++i) best[at(std::size_t{1} << i, i)] = problem.cost(0, i + 1);
    for (std::size_t subset = 1; subset < subsets; ++subset) {
        for (int last = 0; last < n; ++last) {
            double known = best[at(subset, last)];
            if (known == kInfinity || (subset & (std::size_t{1} << last)) == 0) continue;
            for (int next = 0; next < n; ++next) {
                std::size_t bit = std::size_t{1} << next;
                if ((subset & bit) != 0) continue;
                double total = known + problem.cost(last + 1, next + 1);
                if (total < best[at(subset | bit, next)]) {
                    best[at(subset | bit, next)] = total;
                    from[at(subset | bit, next)] = static_cast<std::int8_t>(last);
                }
            }
        }
    }
    std::size_t all = subsets - 1;
    int end = -1;
    double best_total = kInfinity;
    for (int last = 0; last < n; ++last) {
        double total = best[at(all, last)] + problem.closing_cost(last + 1);
        if (total < best_total) {
            best_total = total;
            end = last;
        }
    }
    if (end < 0) return std::nullopt;
    std::vector<int> order;
    std::size_t subset = all;
    for (int last = end; last >= 0;) {
        order.push_back(last + 1);
        int previous = from[at(subset, last)];
        subset &= ~(std::size_t{1} << last);
        last = previous;
    }
    std::reverse(order.begin(), order.end());
    return std::make_pair(std::move(order), best_total);
}

double tour_cost(const Problem& problem, const std::vector<int>& order) {
    double total = 0;
    int previous = 0;
    for (int stop : order) {
        total += problem.cost(previous, stop);
        previous = stop;
    }
    return total + problem.closing_cost(previous);
}

std::vector<int> nearest_neighbour(const Problem& problem) {
    std::vector<char> used(static_cast<std::size_t>(problem.stops) + 1, 0);
    std::vector<int> order;
    int current = 0;
    for (int step = 0; step < problem.stops; ++step) {
        int choice = -1;
        double cheapest = kInfinity;
        for (int stop = 1; stop <= problem.stops; ++stop) {
            if (used[static_cast<std::size_t>(stop)] == 0 && problem.cost(current, stop) < cheapest) {
                cheapest = problem.cost(current, stop);
                choice = stop;
            }
        }
        if (choice < 0) {
            // Everything left is unreachable from here; 2-opt may still repair the order.
            for (int stop = 1; stop <= problem.stops; ++stop) {
                if (used[static_cast<std::size_t>(stop)] == 0) {
                    choice = stop;
                    break;
                }
            }
        }
        used[static_cast<std::size_t>(choice)] = 1;
        order.push_back(choice);
        current = choice;
    }
    return order;
}

// Tries reversing each stretch, costing the whole tour because reversal matters on one-way streets.
void improve_two_opt(const Problem& problem, std::vector<int>& order) {
    double best = tour_cost(problem, order);
    bool improved = true;
    while (improved) {
        improved = false;
        for (std::size_t i = 0; i + 1 < order.size(); ++i) {
            for (std::size_t j = i + 1; j < order.size(); ++j) {
                std::reverse(order.begin() + static_cast<std::ptrdiff_t>(i), order.begin() + static_cast<std::ptrdiff_t>(j) + 1);
                double cost = tour_cost(problem, order);
                if (cost < best) {
                    best = cost;
                    improved = true;
                } else {
                    std::reverse(order.begin() + static_cast<std::ptrdiff_t>(i), order.begin() + static_cast<std::ptrdiff_t>(j) + 1);
                }
            }
        }
    }
}

}  // namespace

std::optional<StopOrder> plan_stop_order(const Router& router, NodeId start, const std::vector<NodeId>& stops,
                                         std::optional<NodeId> end) {
    std::vector<NodeId> places;
    places.reserve(stops.size() + 2);
    places.push_back(start);
    places.insert(places.end(), stops.begin(), stops.end());
    if (end) places.push_back(*end);
    Table table = cost_matrix(router, places, places);
    Problem problem{table, static_cast<int>(stops.size()), end.has_value()};

    std::vector<int> order;
    double cost = 0;
    bool exact = problem.stops <= kExactStopLimit;
    if (exact) {
        auto solved = solve_exact(problem);
        if (!solved) return std::nullopt;
        order = std::move(solved->first);
        cost = solved->second;
    } else {
        order = nearest_neighbour(problem);
        improve_two_opt(problem, order);
        cost = tour_cost(problem, order);
        if (cost == kInfinity) return std::nullopt;
    }
    StopOrder result;
    result.cost = cost;
    result.exact = exact;
    result.order.push_back(start);
    for (int stop : order) result.order.push_back(places[static_cast<std::size_t>(stop)]);
    if (end) result.order.push_back(*end);
    return result;
}

}  // namespace routeplan
