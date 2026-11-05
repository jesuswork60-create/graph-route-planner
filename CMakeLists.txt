cmake_minimum_required(VERSION 3.20)
project(graph_route_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(routeplan STATIC
  src/json.cpp
  src/graph.cpp
  src/graph_io.cpp
  src/profile.cpp
  src/router.cpp
  src/alternatives.cpp
  src/reach.cpp
  src/via.cpp
  src/stop_order.cpp
  src/analysis.cpp
  src/itinerary.cpp
  src/render.cpp
  src/cli.cpp)
target_include_directories(routeplan PUBLIC include)
target_compile_options(routeplan PUBLIC -Wall -Wextra)

add_executable(route-plan src/main.cpp)
target_link_libraries(route-plan PRIVATE routeplan)

enable_testing()
add_library(check_helper STATIC tests/check.cpp)
target_include_directories(check_helper PUBLIC tests)
target_link_libraries(check_helper PUBLIC routeplan)

foreach(name json graph profile router alternatives reach via stop_order analysis itinerary cli)
  add_executable(test_${name} tests/test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE check_helper)
  add_test(NAME ${name} COMMAND test_${name})
endforeach()
