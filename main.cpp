#include <iostream>
#include "PathService.hpp"
#include "Environment.hpp"
#include "Geometry.hpp"
#include "PathPlanner.hpp"
#include "Visualizer.hpp"
#include "GeoJSONReader.hpp"

using Planner::Environment;
using Planner::LineString;
using Planner::PathPlanner;
using Planner::Point;
using Planner::Polygon;
using Planner::Visualizer;

int main()
{
    std::cout << "--- Starte Coverage Path Planner ---" << std::endl;

    Point startPos = {-2.0, -2.0}; // Startposition des Robots
    Planner::PathSettings settings;
    settings.pattern = "lines";             // Mögliche Werte (lines, squares, rings)
    settings.offset = 0.2;                  // Abstand zwischen der Bahnen (m)
    settings.angle = 0.5;                   // Winkel der Bahnen (RAD)
    settings.distanceToBorder = 0.4;        // Abstand zu Perimetergrenze (m)
    settings.mowArea = true;                // Soll die Route innerhalb des Bereiches berechnet werden
    settings.mowBorder = true;              // Sollen die Grenzen des Perimeters abgefahren werden (in Kombination mit borderLaps)
    settings.mowBorderCcw = true;           // Soll die Perimetergrenze gegen Uhrzeigersinn abgefahren werden
    settings.borderLaps = 2;                // Wie veile Runden für Perimetergrenze, jede neue Runde versetzt um den Offset (in Kombination mit mowBorder)
    settings.mowExclusionsBoder = true;     // Soll die Exclusiongrenze abgefahren werden (in Kombination mit mowExclusionsBorderLaps)
    settings.mowExclusionsBorderCcw = true; // Soll die Exclusiongrenze gegen Uhrzeigersinn abgefahren werden
    settings.exclusionsBorderLaps = 2;      // Wie viele Runden für Exclusiongrnze, jede neue Runde versetzt um den Offset (in Kombination mit mowExclusionBoder)

    std::cout << "Initialisiere geometriebasiertes Environment..." << std::endl;
    // Polygon perimeter1 = {{-5.0, -3.0}, {20.0, 0.0}, {20.0, 30.0}, {2.0, 30.0}, {2.0, 20.0}, {15.0, 20.0}, {17.0, 15.0}, {0.0, 10.0}};
    // auto myEnv1 = Environment{perimeter1};
    // Polygon obstacle1 = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.5}};
    // Polygon obstacle2 = {{10.0, 4.0}, {12.0, 3.0}, {12.0, 6.0}, {10.0, 5.5}};
    // Polygon obstacle3 = {{3.0, 24.0}, {3.0, 22.0}, {12.0, 22.0}, {12.0, 24.0}};
    // Polygon mowArea1 = {{2.0, 25.0}, {2.0, 21.0}, {13.0, 21.0}, {13.0, 25.0}};
    // myEnv1.addObstacle(obstacle1);
    // myEnv1.addObstacle(obstacle2);
    // myEnv1.addObstacle(obstacle3);
    // myEnv1.addMowArea(mowArea1);
    // LineString virtualWire = {{9.0, 10.0}, {18.0, 14.0}, {18.0, 26.0}};
    // myEnv1.setVirtualWire(virtualWire);

    // Polygon perimeter2 = {{-5.0, -5.0}, {-5.0, -3.0}, {-3.0, -3.0}, {-3.0, 3.0}, {-5.0, 3.0}, {-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}};
    // auto myEnv2 = Environment{perimeter2};
    // Polygon obstacle4 = {{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
    // myEnv2.addObstacle(obstacle4);

    auto myEnv = Planner::GeoJSONReader::loadEnvironmentFromFile("example_map.json");

    Planner::PathService service;
    std::cout << "Starte PathService..." << std::endl;
    auto result = service.computeFullTask(myEnv, settings, startPos);

    std::cout << "Schreibe das Ergbnis in ein SVG Format..." << std::endl;
    Visualizer::exportToSVG("test_map.svg", myEnv, result.path, result.debugLines, result.debugLinesSec, result.slices);

    std::cout << "Setup erfolgreich! test_map.svg liegt in dem Root-Verzeichnis." << std::endl;
}
