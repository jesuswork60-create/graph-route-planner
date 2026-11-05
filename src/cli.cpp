#include "routeplan/cli.hpp"

#include <charconv>
#include <ostream>
#include <stdexcept>

#include "routeplan/alternatives.hpp"
#include "routeplan/analysis.hpp"
#include "routeplan/graph_io.hpp"
#include "routeplan/itinerary.hpp"
#include "routeplan/reach.hpp"
#include "routeplan/render.hpp"
#include "routeplan/router.hpp"
#include "routeplan/stop_order.hpp"
#include "routeplan/via.hpp"

namespace routeplan {

namespace {

const char kUsage[] =
    "usage: route-plan COMMAND GRAPH.json [arguments] [options]\n"
    "\n"
    "commands:\n"
    "  route FROM TO             the cheapest route\n"
    "  directions FROM TO        the cheapest route as turn by turn directions\n"
    "  alternatives FROM TO      several routes (--count N, default 3)\n"
    "  reach FROM BUDGET         what can be reached within a budget\n"
    "  via A B [C...]            a route through waypoints in the given order\n"
    "  order START STOP...       the best order to visit stops (--end NODE to finish there)\n"
    "  matrix A,B,... C,D,...    costs from each of the first list to each of the second\n"
    "  check                     report likely mistakes in the graph\n"
    "  summary                   counts of nodes, edges, components and tags\n"
    "  dot [FROM TO]             Graphviz source, marking a route when one is asked for\n"
    "\n"
    "options:\n"
    "  --profile NAME   cost profile: distance (default), time, or one from the file\n"
    "  --avoid TAG      close edges with this tag; may be repeated\n"
    "  --json           machine-readable output\n"
    "  --no-heuristic   search with plain Dijkstra\n"
    "  --stats          with route, also print how much searching was done\n";

class UsageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct Options {
    std::vector<std::string> positional;
    std::vector<std::string> avoid;
    std::string profile = "distance";
    std::string end;
    int count = 3;
    bool as_json = false;
    bool no_heuristic = false;
    bool stats = false;
    bool help = false;
};

double parse_number(const std::string& text) {
    double value = 0;
    const char* last = text.data() + text.size();
    auto result = std::from_chars(text.data(), last, value);
    if (result.ec != std::errc() || result.ptr != last) throw UsageError("expected a number, got " + text);
    return value;
}

int parse_count(const std::string& text) {
    int value = 0;
    const char* last = text.data() + text.size();
    auto result = std::from_chars(text.data(), last, value);
    if (result.ec != std::errc() || result.ptr != last || value < 1) {
        throw UsageError("expected a positive whole number, got " + text);
    }
    return value;
}

Options parse_options(const std::vector<std::string>& args) {
    Options options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        auto value = [&]() -> const std::string& {
            if (i + 1 >= args.size()) throw UsageError(arg + " needs a value");
            return args[++i];
        };
        if (arg == "-h" || arg == "--help") options.help = true;
        else if (arg == "--json") options.as_json = true;
        else if (arg == "--no-heuristic") options.no_heuristic = true;
        else if (arg == "--stats") options.stats = true;
        else if (arg == "--profile") options.profile = value();
        else if (arg == "--avoid") options.avoid.push_back(value());
        else if (arg == "--end") options.end = value();
        else if (arg == "--count") options.count = parse_count(value());
        else if (arg.rfind("--", 0) == 0) throw UsageError("unknown option " + arg);
        else options.positional.push_back(arg);
    }
    return options;
}

std::vector<std::string> split_list(const std::string& text) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        std::size_t comma = text.find(',', start);
        parts.push_back(text.substr(start, comma == std::string::npos ? comma : comma - start));
        if (comma == std::string::npos) return parts;
        start = comma + 1;
    }
}

class Session {
public:
    Session(const Options& options, GraphDocument document, std::ostream& out, std::ostream& err)
        : options_(options),
          document_(std::move(document)),
          router_(document_.graph, choose_profile(options, document_)),
          out_(out),
          err_(err) {}

    int run(const std::string& command) {
        // The command and file take the first two positions.
        std::vector<std::string> args(options_.positional.begin() + 2, options_.positional.end());
        if (command == "route") return cmd_route(args);
        if (command == "directions") return cmd_directions(args);
        if (command == "alternatives") return cmd_alternatives(args);
        if (command == "reach") return cmd_reach(args);
        if (command == "via") return cmd_via(args);
        if (command == "order") return cmd_order(args);
        if (command == "matrix") return cmd_matrix(args);
        if (command == "check") return cmd_check(args);
        if (command == "summary") return cmd_summary(args);
        if (command == "dot") return cmd_dot(args);
        throw UsageError("unknown command " + command);
    }

private:
    static CostProfile choose_profile(const Options& options, const GraphDocument& document) {
        auto found = document.profiles.find(options.profile);
        if (found == document.profiles.end()) throw UsageError("no profile called " + options.profile);
        CostProfile profile = found->second;
        profile.avoid_tags.insert(options.avoid.begin(), options.avoid.end());
        return profile;
    }

    const Graph& graph() const { return document_.graph; }
    NodeId node(const std::string& key) const { return graph().require_node(key); }

    static void need(const std::vector<std::string>& args, std::size_t count, const char* shape) {
        if (args.size() != count) throw UsageError(std::string("expected ") + shape);
    }

    void print_json(const json::Value& value) { out_ << json::dump(value, 2) << '\n'; }

    int no_route(const std::string& from, const std::string& to) {
        err_ << "route-plan: no route from " << from << " to " << to << '\n';
        return 1;
    }

    int cmd_route(const std::vector<std::string>& args) {
        need(args, 2, "FROM TO");
        RouteQuery query;
        query.source = node(args[0]);
        query.target = node(args[1]);
        query.use_heuristic = !options_.no_heuristic;
        SearchStats stats;
        auto route = router_.route(query, &stats);
        if (options_.stats) {
            err_ << "settled " << stats.settled << ", relaxed " << stats.relaxed << ", pushed " << stats.pushed << '\n';
        }
        if (!route) return no_route(args[0], args[1]);
        if (options_.as_json) print_json(route_json(graph(), *route));
        else out_ << route_text(graph(), *route);
        return 0;
    }

    int cmd_directions(const std::vector<std::string>& args) {
        need(args, 2, "FROM TO");
        auto route = router_.route(node(args[0]), node(args[1]));
        if (!route) return no_route(args[0], args[1]);
        std::vector<Step> steps = build_itinerary(graph(), *route);
        if (options_.as_json) {
            print_json(itinerary_json(graph(), steps));
            return 0;
        }
        int number = 1;
        for (const Step& step : steps) out_ << number++ << ". " << describe_step(graph(), step) << '\n';
        out_ << "Arrive at " << args[1] << '\n';
        return 0;
    }

    int cmd_alternatives(const std::vector<std::string>& args) {
        need(args, 2, "FROM TO");
        std::vector<Route> routes = k_shortest_routes(router_, node(args[0]), node(args[1]), options_.count);
        if (routes.empty()) return no_route(args[0], args[1]);
        if (options_.as_json) {
            json::Value::Array items;
            for (const Route& route : routes) items.push_back(route_json(graph(), route));
            print_json(json::Value(std::move(items)));
            return 0;
        }
        int number = 1;
        for (const Route& route : routes) out_ << '#' << number++ << ' ' << route_text(graph(), route);
        return 0;
    }

    int cmd_reach(const std::vector<std::string>& args) {
        need(args, 2, "FROM BUDGET");
        ReachResult reach = reachable_within(router_, node(args[0]), parse_number(args[1]));
        if (options_.as_json) {
            print_json(reach_json(graph(), reach));
            return 0;
        }
        for (const ReachedNode& reached : reach.nodes) {
            out_ << graph().node(reached.node).key << ' ' << format_cost(reached.cost) << '\n';
        }
        for (const PartialEdge& partial : reach.frontier) {
            out_ << "partly along " << graph().edge(partial.edge).key << ' ' << format_cost(partial.fraction) << '\n';
        }
        return 0;
    }

    int cmd_via(const std::vector<std::string>& args) {
        if (args.size() < 2) throw UsageError("expected at least two waypoints");
        std::vector<NodeId> waypoints;
        for (const std::string& key : args) waypoints.push_back(node(key));
        ViaResult result = route_via(router_, waypoints);
        if (!result.ok()) return no_route(args[*result.failed_leg], args[*result.failed_leg + 1]);
        if (options_.as_json) {
            json::Value::Array legs;
            for (const Route& leg : result.legs) legs.push_back(route_json(graph(), leg));
            json::Value::Object object;
            object.emplace_back("total", route_json(graph(), result.total));
            object.emplace_back("legs", std::move(legs));
            print_json(json::Value(std::move(object)));
            return 0;
        }
        out_ << route_text(graph(), result.total);
        return 0;
    }

    int cmd_order(const std::vector<std::string>& args) {
        if (args.empty()) throw UsageError("expected START and any stops");
        std::vector<NodeId> stops;
        for (std::size_t i = 1; i < args.size(); ++i) stops.push_back(node(args[i]));
        std::optional<NodeId> end;
        if (!options_.end.empty()) end = node(options_.end);
        auto order = plan_stop_order(router_, node(args[0]), stops, end);
        if (!order) {
            err_ << "route-plan: no order connects all of the stops\n";
            return 1;
        }
        if (options_.as_json) {
            print_json(stop_order_json(graph(), *order));
            return 0;
        }
        for (std::size_t i = 0; i < order->order.size(); ++i) {
            out_ << (i > 0 ? " -> " : "") << graph().node(order->order[i]).key;
        }
        out_ << "\ncost " << format_cost(order->cost) << (order->exact ? " (best order)\n" : " (good order, not proven best)\n");
        return 0;
    }

    int cmd_matrix(const std::vector<std::string>& args) {
        need(args, 2, "A,B,... C,D,...");
        std::vector<std::string> from = split_list(args[0]);
        std::vector<std::string> to = split_list(args[1]);
        std::vector<NodeId> sources;
        std::vector<NodeId> targets;
        for (const std::string& key : from) sources.push_back(node(key));
        for (const std::string& key : to) targets.push_back(node(key));
        auto matrix = cost_matrix(router_, sources, targets);
        if (options_.as_json) {
            json::Value::Array rows;
            for (const auto& row : matrix) {
                json::Value::Array cells;
                // JSON has no infinity, so unreachable pairs come out as null.
                for (double cost : row) cells.emplace_back(cost);
                rows.emplace_back(std::move(cells));
            }
            print_json(json::Value(std::move(rows)));
            return 0;
        }
        for (const std::string& key : to) out_ << '\t' << key;
        out_ << '\n';
        for (std::size_t i = 0; i < matrix.size(); ++i) {
            out_ << from[i];
            for (double cost : matrix[i]) out_ << '\t' << (cost == kInfinity ? "-" : format_cost(cost));
            out_ << '\n';
        }
        return 0;
    }

    int cmd_check(const std::vector<std::string>& args) {
        need(args, 0, "no arguments");
        std::vector<Finding> findings = find_issues(graph(), router_.profile());
        bool problem = false;
        for (const Finding& finding : findings) problem = problem || finding.severity == Severity::Problem;
        if (options_.as_json) {
            print_json(findings_json(findings));
        } else if (findings.empty()) {
            out_ << "no issues found\n";
        } else {
            for (const Finding& finding : findings) {
                out_ << (finding.severity == Severity::Problem ? "problem" : "warning") << ": [" << finding.code << "] "
                     << finding.message << '\n';
            }
        }
        return problem ? 1 : 0;
    }

    int cmd_summary(const std::vector<std::string>& args) {
        need(args, 0, "no arguments");
        GraphSummary summary = summarize(graph(), router_.profile());
        if (options_.as_json) {
            print_json(summary_json(summary));
            return 0;
        }
        out_ << "nodes " << summary.nodes << "\nedges " << summary.edges << " (" << summary.closed_edges
             << " closed)\nself loops " << summary.self_loops << "\nparallel edges " << summary.parallel_edges
             << "\ncomponents " << summary.components << " (largest " << summary.largest_component
             << ")\ntotal length " << format_cost(summary.total_length) << '\n';
        for (const auto& entry : summary.tag_counts) out_ << "tag " << entry.first << ' ' << entry.second << '\n';
        return 0;
    }

    int cmd_dot(const std::vector<std::string>& args) {
        std::vector<EdgeId> highlight;
        if (!args.empty()) {
            need(args, 2, "FROM TO, or nothing");
            auto route = router_.route(node(args[0]), node(args[1]));
            if (!route) return no_route(args[0], args[1]);
            highlight = route->edges;
        }
        out_ << graph_dot(graph(), highlight);
        return 0;
    }

    const Options& options_;
    GraphDocument document_;
    Router router_;
    std::ostream& out_;
    std::ostream& err_;
};

}  // namespace

int run_cli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    try {
        Options options = parse_options(args);
        if (options.help) {
            out << kUsage;
            return 0;
        }
        if (options.positional.size() < 2) throw UsageError("a command and a graph file are required");
        Session session(options, load_graph_file(options.positional[1]), out, err);
        return session.run(options.positional[0]);
    } catch (const UsageError& e) {
        err << "route-plan: " << e.what() << '\n' << kUsage;
    } catch (const FormatError& e) {
        err << "route-plan: " << e.what() << '\n';
    } catch (const GraphError& e) {
        err << "route-plan: " << e.what() << '\n';
    } catch (const ProfileError& e) {
        err << "route-plan: " << e.what() << '\n';
    }
    return 2;
}

}  // namespace routeplan
