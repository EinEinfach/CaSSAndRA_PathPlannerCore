#include <iostream>
#include "Enviroment.hpp"
#include "Geometry.hpp"
#include "Visualizer.hpp"

using Planner::Point;
using Planner::Polygon;
using Planner::LineString;
using Planner::Enviroment;
using Planner::Visualizer;

int main() {
    std::cout << "--- Initialisiere Geometrie-basiertes Enviroment ---" << std::endl;

    Polygon perimeter = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {0.0, 10.0}};

    auto myEnv = Enviroment{perimeter};

    Polygon obstacle = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.0}};
    myEnv.addObstacle(obstacle);

    LineString vWire = {{0.0, 9.5}, {20.0, 9.5}};
    myEnv.setVirtualWire(vWire);

    LineString dWire = {{2.0, 2.0}, {1.0, 1.0}, {0.0, 0.0}};
    myEnv.setVirtualWire(dWire);

    Visualizer::exportToSVG("test_map.svg", myEnv);

    std::cout << "Setup erfolgreich: Perimeter, Hindernis und Draehte geladen." << std::endl;
}