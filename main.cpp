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
    std::cout << "--- Starte Coverage Path Planner ---" << std::endl;

    double offset = 0.2;
    double angle = 0;

    Polygon perimeter = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 30.0}, {15.0, 30.0}, {15.0, 25.0}, {10.0, 25.0}, {10.0, 20.0},  {0.0, 10.0}};

    std::cout << "Initialisiere geometriebasiertes Environment..." << std::endl;
    auto myEnv = Environment{perimeter};

    Polygon obstacle1 = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.5}};
    Polygon obstacle2 = {{10.0, 4.0}, {12.0, 3.0}, {12.0, 6.0}, {10.0, 5.5}};

    myEnv.addObstacle(obstacle1);
    myEnv.addObstacle(obstacle2);

    LineString vWire = {{0.0, 9.5}, {20.0, 9.5}};
    myEnv.setVirtualWire(vWire);

    LineString dWire = {{2.0, 2.0}, {1.0, 1.0}, {0.0, 0.0}};
    myEnv.setVirtualWire(dWire);
    std::cout << "Drehe die Environment..." << std::endl;
    myEnv.rotate(angle);

    std::cout << "Generiere Slice..." << std::endl;
    auto slices = PathPlanner::generateSlices(myEnv, offset);
    std::cout << "Verbnde Slice..." << std::endl;
    auto fullPath = PathPlanner::connectSlices(myEnv, slices);
    std::cout << "Drehe die Environment zurück..." << std::endl;
    myEnv.rotate(-angle);
    fullPath.rotate(-angle);
    std::cout << "Schreibe das Ergbnis in ein SVG Format..." << std::endl;
    Visualizer::exportToSVG("test_map.svg", myEnv, fullPath, {});

    std::cout << "Setup erfolgreich!" << std::endl;
}