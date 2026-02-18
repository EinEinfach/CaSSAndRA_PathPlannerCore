#include <iostream>
#include "Environment.hpp"
#include "Geometry.hpp"
#include "PathPlanner.hpp"
#include "Visualizer.hpp"

using Planner::Environment;
using Planner::LineString;
using Planner::PathPlanner;
using Planner::Point;
using Planner::Polygon;
using Planner::Visualizer;

int main()
{
    std::cout << "--- Starte Coverage Path Planner ---" << std::endl;

    Point startPos = {0.0, 0.0};
    std::string pattern = "lines";
    double offset = 1.0;
    double angle = 0.5;
    //size_t distanceToBorder = 0;
    bool mowArea = true;
    bool mowBorder = true;
    //bool mowBorderCcw = false;
    size_t borderLaps = 1;
    bool mowExclusionsBorder = true;
    //bool mowExclusionBorderCcw = false;
    size_t exclusionBorderLaps = 1;

    std::cout << "Initialisiere geometriebasiertes Environment..." << std::endl;
    Polygon perimeter1 = {{-5.0, -3.0}, {20.0, 0.0}, {20.0, 30.0}, {2.0, 30.0}, {2.0, 20.0}, {15.0, 20.0}, {17.0, 15.0}, {0.0, 10.0}};
    auto myEnv1 = Environment{perimeter1};
    Polygon obstacle1 = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.5}};
    Polygon obstacle2 = {{10.0, 4.0}, {12.0, 3.0}, {12.0, 6.0}, {10.0, 5.5}};
    myEnv1.addObstacle(obstacle1);
    myEnv1.addObstacle(obstacle2);
    LineString virtualWire = {{9.0, 10.0}, {18.0, 14.0}, {18.0, 26.0}};
    myEnv1.setVirtualWire(virtualWire);

    Polygon perimeter2 = {{-5.0, -5.0}, {-5.0, -3.0}, {-3.0, -3.0}, {-3.0, 3.0}, {-5.0, 3.0}, {-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}};
    auto myEnv2 = Environment(perimeter2);
    Polygon obstacle3 = {{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
    myEnv2.addObstacle(obstacle3);

    auto myEnv = myEnv1;

    std::cout << "Drehe die Environment. Winkel: " << angle << std::endl;
    myEnv.rotate(-angle);

    std::vector<LineString> slices;

    if (mowArea)
    {
        std::cout << "Generiere Slices. Muster: " << pattern << " Abstand: " << offset << std::endl;
        if (pattern.compare("lines") == 0) {
            slices = PathPlanner::generateSlices(myEnv, offset);
        } else {
            slices = PathPlanner::generateRingSlices(myEnv, offset);
        }
    }

    if (mowBorder && borderLaps > 0)
    {
        std::cout << "Generiere Border Slices. Anzahl Runden: " << borderLaps << std::endl;
        slices.push_back(myEnv.getPerimeter());
    }

    if (mowExclusionsBorder && exclusionBorderLaps > 0)
    {
        std::cout << "Generieren Exclusions Slices. Anzahl Runden: " << exclusionBorderLaps << std::endl;
        for (auto &obs : myEnv.getObstacles())
        {
            slices.push_back(obs);
        }
    }

    std::cout << "Verbnde Slices..." << std::endl;
    auto result = PathPlanner::connectSlices(myEnv, slices, startPos);
    std::cout << "Drehe die Environment zurück..." << std::endl;
    myEnv.rotate(angle);
    result.path.rotate(angle);
    // Für Visualization
    for (auto &l : result.debugLines)
    {
        l.rotate(angle);
    }
    for (auto &l : result.debugLinesSec)
    {
        l.rotate(angle);
    }
    for (auto &l : slices)
    {
        l.rotate(angle);
    }
    std::cout << "Schreibe das Ergbnis in ein SVG Format..." << std::endl;
    Visualizer::exportToSVG("test_map.svg", myEnv, result.path, result.debugLines, {}, slices);

    std::cout << "Setup erfolgreich!" << std::endl;
}