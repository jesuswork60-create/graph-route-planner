#include "routeplan/graph_io.hpp"

#include <fstream>
#include <set>
#include <sstream>

namespace routeplan {

namespace {

using json::Value;

void check_keys(const Value& object, const std::set<std::string>& allowed, const std::string& where) {
    for (const Value::Member& member : object.as_object()) {
        if (allowed.count(member.first) == 0) throw FormatError(where + ": unknown key \"" + member.first + "\"");
    }
}

const Value& required(const Value& object, const char* key, const std::string& where) {
    const Value* found = object.find(key);
    if (found == nullptr) throw FormatError(where + ": missing \"" + std::string(key) + "\"");
    return *found;
}

Coordinates read_coordinates(const Value& document) {
    const Value* value = document.find("coordinates");
    if (value == nullptr) return Coordinates::None;
    const std::string& text = value->as_string();
    if (text == "none") return Coordinates::None;
    if (text == "plane") return Coordinates::Plane;
    if (text == "geographic") return Coordinates::Geographic;
    throw FormatError("unknown coordinates value \"" + text + "\"");
}

void read_node(Graph& graph, const Value& item, std::size_t index) {
    std::string where = "node " + std::to_string(index);
    check_keys(item, {"id", "x", "y", "lat", "lon"}, where);
    std::string key = required(item, "id", where).as_string();
    where = "node " + key;
    double x = 0;
    double y = 0;
    if (graph.coordinates() == Coordinates::Plane) {
        x = required(item, "x", where).as_number();
        y = required(item, "y", where).as_number();
    } else if (graph.coordinates() == Coordinates::Geographic) {
        x = required(item, "lon", where).as_number();
        y = required(item, "lat", where).as_number();
    }
    graph.add_node(key, x, y);
}

std::vector<std::string> read_tags(const Value& item) {
    std::vector<std::string> tags;
    if (const Value* value = item.find("tags")) {
        for (const Value& tag : value->as_array()) tags.push_back(tag.as_string());
    }
    return tags;
}

void read_edge(Graph& graph, const Value& item, std::size_t index) {
    std::string where = "edge " + std::to_string(index);
    check_keys(item, {"id", "from", "to", "length", "time", "speed", "name", "tags", "two_way"}, where);
    Edge edge;
    if (const Value* id = item.find("id")) {
        edge.key = id->as_string();
        where = "edge " + edge.key;
    }
    auto from = graph.find_node(required(item, "from", where).as_string());
    auto to = graph.find_node(required(item, "to", where).as_string());
    if (!from) throw FormatError(where + ": unknown node " + item.find("from")->as_string());
    if (!to) throw FormatError(where + ": unknown node " + item.find("to")->as_string());
    edge.from = *from;
    edge.to = *to;

    if (const Value* length = item.find("length")) {
        edge.length = length->as_number();
    } else if (graph.coordinates() != Coordinates::None) {
        edge.length = graph.straight_distance(*from, *to);
    } else {
        throw FormatError(where + ": needs a length when the graph has no coordinates");
    }
    const Value* time = item.find("time");
    const Value* speed = item.find("speed");
    if (time != nullptr && speed != nullptr) throw FormatError(where + ": give time or speed, not both");
    if (time != nullptr) {
        edge.time = time->as_number();
    } else if (speed != nullptr) {
        if (speed->as_number() <= 0) throw FormatError(where + ": speed must be above zero");
        edge.time = edge.length / speed->as_number();
    } else {
        edge.time = edge.length;
    }
    if (const Value* name = item.find("name")) edge.name = name->as_string();
    edge.tags = read_tags(item);

    bool two_way = false;
    if (const Value* value = item.find("two_way")) two_way = value->as_bool();
    if (two_way && edge.key.empty()) throw FormatError(where + ": a two_way edge needs an id");

    Edge reverse = edge;
    std::swap(reverse.from, reverse.to);
    reverse.key = edge.key + "~rev";
    graph.add_edge(std::move(edge));
    if (two_way) graph.add_edge(std::move(reverse));
}

void read_restriction(Graph& graph, const Value& item, std::size_t index) {
    std::string where = "turn restriction " + std::to_string(index);
    check_keys(item, {"type", "from", "to"}, where);
    const std::string& type = required(item, "type", where).as_string();
    auto from = graph.find_edge(required(item, "from", where).as_string());
    auto to = graph.find_edge(required(item, "to", where).as_string());
    if (!from) throw FormatError(where + ": unknown edge " + item.find("from")->as_string());
    if (!to) throw FormatError(where + ": unknown edge " + item.find("to")->as_string());
    if (type == "no") graph.ban_turn(*from, *to);
    else if (type == "only") graph.require_turn(*from, *to);
    else throw FormatError(where + ": type must be \"no\" or \"only\"");
}

}  // namespace

GraphDocument load_graph(const Value& document) {
    try {
        const std::string where = "graph";
        check_keys(document, {"coordinates", "nodes", "edges", "turn_restrictions", "profiles"}, where);
        Graph graph(read_coordinates(document));
        const Value::Array& nodes = required(document, "nodes", where).as_array();
        for (std::size_t i = 0; i < nodes.size(); ++i) read_node(graph, nodes[i], i);
        if (const Value* edges = document.find("edges")) {
            const Value::Array& items = edges->as_array();
            for (std::size_t i = 0; i < items.size(); ++i) read_edge(graph, items[i], i);
        }
        if (const Value* restrictions = document.find("turn_restrictions")) {
            const Value::Array& items = restrictions->as_array();
            for (std::size_t i = 0; i < items.size(); ++i) read_restriction(graph, items[i], i);
        }
        GraphDocument result{std::move(graph), {}};
        result.profiles["distance"] = CostProfile::distance();
        result.profiles["time"] = CostProfile::travel_time();
        if (const Value* profiles = document.find("profiles")) {
            for (const Value::Member& member : profiles->as_object()) {
                result.profiles[member.first] = CostProfile::from_json(member.first, member.second);
            }
        }
        return result;
    } catch (const json::TypeError& e) {
        throw FormatError(std::string("wrong type in the graph file: ") + e.what());
    } catch (const GraphError& e) {
        throw FormatError(e.what());
    } catch (const ProfileError& e) {
        throw FormatError(e.what());
    }
}

GraphDocument load_graph_text(const std::string& text) {
    try {
        return load_graph(json::parse(text));
    } catch (const json::ParseError& e) {
        throw FormatError(std::string("not valid JSON: ") + e.what());
    }
}

GraphDocument load_graph_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw FormatError("cannot read " + path);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return load_graph_text(buffer.str());
}

json::Value graph_to_json(const Graph& graph) {
    Value::Object document;
    const char* coordinates = "none";
    if (graph.coordinates() == Coordinates::Plane) coordinates = "plane";
    if (graph.coordinates() == Coordinates::Geographic) coordinates = "geographic";
    document.emplace_back("coordinates", coordinates);

    Value::Array nodes;
    for (NodeId id = 0; id < graph.node_count(); ++id) {
        const Node& node = graph.node(id);
        Value::Object item;
        item.emplace_back("id", node.key);
        if (graph.coordinates() == Coordinates::Plane) {
            item.emplace_back("x", node.x);
            item.emplace_back("y", node.y);
        } else if (graph.coordinates() == Coordinates::Geographic) {
            item.emplace_back("lat", node.y);
            item.emplace_back("lon", node.x);
        }
        nodes.emplace_back(std::move(item));
    }
    document.emplace_back("nodes", std::move(nodes));

    Value::Array edges;
    for (EdgeId id = 0; id < graph.edge_count(); ++id) {
        const Edge& edge = graph.edge(id);
        Value::Object item;
        item.emplace_back("id", edge.key);
        item.emplace_back("from", graph.node(edge.from).key);
        item.emplace_back("to", graph.node(edge.to).key);
        item.emplace_back("length", edge.length);
        item.emplace_back("time", edge.time);
        if (!edge.name.empty()) item.emplace_back("name", edge.name);
        if (!edge.tags.empty()) {
            Value::Array tags;
            for (const std::string& tag : edge.tags) tags.emplace_back(tag);
            item.emplace_back("tags", std::move(tags));
        }
        edges.emplace_back(std::move(item));
    }
    document.emplace_back("edges", std::move(edges));

    Value::Array restrictions;
    auto add_restrictions = [&](const char* type, const std::unordered_map<EdgeId, std::unordered_set<EdgeId>>& table) {
        std::set<std::pair<EdgeId, EdgeId>> sorted;
        for (const auto& entry : table) {
            for (EdgeId to : entry.second) sorted.emplace(entry.first, to);
        }
        for (const auto& pair : sorted) {
            Value::Object item;
            item.emplace_back("type", type);
            item.emplace_back("from", graph.edge(pair.first).key);
            item.emplace_back("to", graph.edge(pair.second).key);
            restrictions.emplace_back(std::move(item));
        }
    };
    add_restrictions("no", graph.banned_turns());
    add_restrictions("only", graph.required_turns());
    if (!restrictions.empty()) document.emplace_back("turn_restrictions", std::move(restrictions));
    return Value(std::move(document));
}

}  // namespace routeplan
