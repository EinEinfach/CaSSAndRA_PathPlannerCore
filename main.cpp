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

    Point startPos = {0.0, 0.0};
    double offset = 1.0;
    double angle = 0.3;

    std::cout << "Initialisiere geometriebasiertes Environment..." << std::endl;
    Polygon perimeter1 = {{-5.0, -3.0}, {20.0, 0.0}, {20.0, 30.0}, {2.0, 30.0}, {2.0, 20.0}, {15.0, 20.0}, {17.0, 15.0},  {0.0, 10.0}};
    auto myEnv1 = Environment{perimeter1};
    Polygon obstacle1 = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.5}};
    Polygon obstacle2 = {{10.0, 4.0}, {12.0, 3.0}, {12.0, 6.0}, {10.0, 5.5}};
    myEnv1.addObstacle(obstacle1);
    myEnv1.addObstacle(obstacle2);
    LineString virtualWire = {{2.0, 10.0}, {18.0, 14.0}, {18.0, 26.0}};
    myEnv1.setVirtualWire(virtualWire);

    Polygon perimeter2 = {{-5.0, -5.0}, {-5.0, -3.0}, {-3.0, -3.0}, {-3.0, 3.0}, {-5.0, 3.0}, {-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}};
    auto myEnv2 = Environment(perimeter2);
    Polygon obstacle3 = {{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
    myEnv2.addObstacle(obstacle3);

    auto myEnv = myEnv1;

    std::cout << "Drehe die Environment..." << std::endl;
    myEnv.rotate(-angle);

    std::cout << "Generiere Slice..." << std::endl;
    auto slices = PathPlanner::generateSlices(myEnv, offset);
    std::cout << "Verbnde Slice..." << std::endl;
    auto result = PathPlanner::connectSlices(myEnv, slices, startPos);
    std::cout << "Drehe die Environment zurück..." << std::endl;
    myEnv.rotate(angle);
    result.path.rotate(angle);
    // Für Visualization
    for (auto& l : result.debugLines) {
        l.rotate(angle);
    }
    for (auto& l : slices) {
        l.rotate(angle);
    }
    std::cout << "Schreibe das Ergbnis in ein SVG Format..." << std::endl;
    Visualizer::exportToSVG("test_map.svg", myEnv, result.path, result.debugLines, slices); 

    std::cout << "Setup erfolgreich!" << std::endl;
}