#include <iostream>
#include "Environment.hpp"
#include "Geometry.hpp"
#include "PathPlanner.hpp"
#include "Visualizer.hpp"

using Planner::Point;
using Planner::Polygon;
using Planner::LineString;
using Planner::Environment;
using Planner::PathPlanner;
using Planner::Visualizer;

int main() {
    std::cout << "--- Initialisiere Geometrie-basiertes Environment ---" << std::endl;

    Polygon perimeter = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {0.0, 10.0}};

    auto myEnv = Environment{perimeter};

    Polygon obstacle1 = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.5}};
    Polygon obstacle2 = {{10.0, 4.0}, {12.0, 3.0}, {12.0, 6.0}, {10.0, 5.5}};

    myEnv.addObstacle(obstacle1);
    myEnv.addObstacle(obstacle2);

    LineString vWire = {{0.0, 9.5}, {20.0, 9.5}};
    myEnv.setVirtualWire(vWire);

    LineString dWire = {{2.0, 2.0}, {1.0, 1.0}, {0.0, 0.0}};
    myEnv.setVirtualWire(dWire);

    auto slices = PathPlanner::generateSlices(myEnv, 0.2);

    Visualizer::exportToSVG("test_map.svg", myEnv, {}, slices);

    std::cout << "Setup erfolgreich: Perimeter, Hindernis und Draehte geladen." << std::endl;
}