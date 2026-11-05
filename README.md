# graph-route-planner

A small C++17 library and command line tool for finding routes through a road
network described in a JSON file. It handles the awkward bits that a textbook
Dijkstra leaves out: one-way streets, parallel roads, turn restrictions, U-turn
rules and different cost profiles for different kinds of traveller. On top of the
core search it can list alternative routes, work out what is reachable within a
budget, route through waypoints, choose a visiting order for a set of stops and
produce turn by turn directions.

It has no dependencies beyond the standard library. The JSON reader is hand-written.

## Building

You need a C++17 compiler and CMake 3.20 or newer.

    cmake -S . -B build
    cmake --build build -j 2

That gives you `build/route-plan` and a static library, `routeplan`.

## Trying it

`examples/town.json` is a made-up town of eight junctions.

    $ build/route-plan route examples/town.json station harbour
    station -> market -> bridge -> harbour
    cost 1200, length 1200, time 150, 3 edges

    $ build/route-plan directions examples/town.json station hill --profile cycling
    1. Start on School Lane and go 300
    2. Turn right onto Elm Avenue and go 400
    3. Turn left onto Hill Road and go 450
    Arrive at hill

The direct road from the market to the library is ruled out when coming from the
station, because the file bans that turn. `route-plan --help` lists the other
commands: `alternatives`, `reach`, `via`, `order`, `matrix`, `check`, `summary` and
`dot`. Exit status is 0 on success, 1 when no route exists or `check` finds a
problem, and 2 for bad usage or a bad graph file.

## The graph file

```json
{
  "coordinates": "plane",
  "nodes": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 100, "y": 0}],
  "edges": [
    {"id": "ab", "from": "a", "to": "b", "speed": 8, "name": "Mill Road",
     "tags": ["cobbled"], "two_way": true}
  ],
  "turn_restrictions": [{"type": "no", "from": "ab", "to": "ab~rev"}],
  "profiles": {
    "cycling": {"tag_multipliers": {"cobbled": 1.5}, "avoid": ["highway"],
                "u_turns": "dead_end_only", "u_turn_penalty": 30}
  }
}
```

`coordinates` is `none` (the default), `plane` (`x` and `y`) or `geographic` (`lon`
and `lat` in degrees, lengths in metres). An edge may leave out its `length` when
there are coordinates, and takes its `time` from `time`, from `speed`, or else from
its length. A `two_way` edge also creates a reverse edge called `<id>~rev`.

A profile prices each edge as
`(distance_weight * length + time_weight * time) * product of tag multipliers`. Tags
listed in `avoid` close an edge entirely. `distance` and `time` always exist, and a
file can replace them. Restrictions come as `no` (this turn is banned) and `only`
(from this edge, only the listed turns are allowed).

## Using the library

```cpp
#include "routeplan/graph_io.hpp"
#include "routeplan/router.hpp"

auto document = routeplan::load_graph_file("town.json");
routeplan::Router router(document.graph, document.profiles.at("cycling"));
auto route = router.route(document.graph.require_node("station"),
                          document.graph.require_node("hill"));
```

The router keeps a reference to the graph and prices every edge when it is built, so
build a new one after changing the graph.

## How the search works

The search state is the edge just travelled rather than the current node, which is
what allows a turn rule to depend on how a junction was reached, and a route to pass
through a node twice if a restriction forces that. A* is used when the graph has
coordinates. Its heuristic is the straight-line distance scaled by the lowest cost
per unit of distance of any open edge in the graph, so it stays a true lower bound
under any profile, at the price of guiding the search poorly if one edge is very
cheap.

## Tests

    cmake -S . -B build
    cmake --build build -j 2
    ctest --test-dir build --output-on-failure

The tests use a tiny helper in `tests/check.hpp`. `test_router.cpp` includes a check
of the search against a separate Bellman-Ford on seeded random graphs.

## Known limitations

- Turn rules cover one step, from one edge to the next. A restriction spanning three
  or more edges can't be expressed.
- Costs are fixed for a search. There is nothing for traffic that varies by the time
  of day, and no preprocessing such as contraction hierarchies, so every query
  searches from scratch.
- `via` plans each leg in turn, which can miss a dearer early leg that would have made
  the next one cheaper.
- `order` costs its table without knowing how each leg arrives, so with a turn rule at
  a stop the true route may cost more than the reported total. Above 12 stops it uses
  nearest neighbour with 2-opt, which can miss the best order. A test pins down one
  such case.
- `alternatives` often returns routes that differ only slightly, because nothing
  measures how much they overlap.
- `check` and the component count ignore turn rules, so a node that a rule cuts off
  won't show up as a separate component.
- U-turns are recognised only when one edge exactly reverses the other's two nodes, so
  doubling back along a parallel road counts, but going round a small loop doesn't.
- Costs printed as text keep six significant digits, so large values appear as
  `1.23457e+06`. Use `--json` for full precision.
- Graph files are read into memory whole, and `graph_to_json` doesn't write profiles.
