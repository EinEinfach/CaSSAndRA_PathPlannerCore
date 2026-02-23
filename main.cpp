#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "PathService.hpp"
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
using json = nlohmann::json;

// Funktion zum Einlesen der GeoJSON
Environment loadEnvironmentFromGeoJSON(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Konnte Datei nicht öffnen: " + filename);
    }

    json j;
    file >> j;

    Polygon perimeter;
    std::vector<Polygon> exclusions;

    for (auto &feature : j["features"])
    {
        std::string name = feature["properties"]["name"];
        auto &geometry = feature["geometry"];

        if (name == "perimeter" && geometry["type"] == "Polygon")
        {
            // GeoJSON Polygons haben ein Array von Arrays (Ring + Holes)
            // Wir nehmen den äußeren Ring [0]
            for (auto &coord : geometry["coordinates"][0])
            {
                perimeter.addPoint({coord[0].get<double>(), coord[1].get<double>()});
            }
        }
        else if (name == "exclusion" && geometry["type"] == "Polygon")
        {
            Polygon ex;
            for (auto &coord : geometry["coordinates"][0])
            {
                ex.addPoint({coord[0].get<double>(), coord[1].get<double>()});
            }
            exclusions.push_back(ex);
        }
    }

    // Environment erstellen
    Environment env(perimeter);
    for (const auto &ex : exclusions)
    {
        env.addObstacle(ex);
    }

    return env;
}

int main()
{
    std::cout << "--- Starte Coverage Path Planner ---" << std::endl;

    Point startPos = {1.0497111925, -1.8987146778000001};//{0.0, 0.0};
    Planner::PathSettings settings;
    settings.pattern = "squares";
    settings.offset = 0.5;
    settings.angle = 0.0;
    settings.distanceToBorder = 0.5;
    settings.mowArea = false;
    settings.mowBorder = false;
    settings.mowBorderCcw = false;
    settings.borderLaps = 1;
    settings.mowExclusionsBoder = true;
    settings.mowExclusionsBorderCcw = false;
    settings.exclusionsBorderLaps = 1;

    std::cout << "Initialisiere geometriebasiertes Environment..." << std::endl;
    Polygon perimeter1 = {{-5.0, -3.0}, {20.0, 0.0}, {20.0, 30.0}, {2.0, 30.0}, {2.0, 20.0}, {15.0, 20.0}, {17.0, 15.0}, {0.0, 10.0}};
    auto myEnv1 = Environment{perimeter1};
    Polygon obstacle1 = {{5.0, 2.0}, {7.0, 2.0}, {7.0, 4.0}, {5.0, 4.5}};
    Polygon obstacle2 = {{10.0, 4.0}, {12.0, 3.0}, {12.0, 6.0}, {10.0, 5.5}};
    Polygon obstacle3 = {{3.0, 24.0}, {3.0, 22.0}, {12.0, 22.0}, {12.0, 24.0}};
    Polygon mowArea1 = {{2.0, 25.0}, {2.0, 21.0}, {13.0, 21.0}, {13.0, 25.0}};
    myEnv1.addObstacle(obstacle1);
    myEnv1.addObstacle(obstacle2);
    myEnv1.addObstacle(obstacle3);
    myEnv1.addMowArea(mowArea1);
    LineString virtualWire = {{9.0, 10.0}, {18.0, 14.0}, {18.0, 26.0}};
    myEnv1.setVirtualWire(virtualWire);

    Polygon perimeter2 = {{-5.0, -5.0}, {-5.0, -3.0}, {-3.0, -3.0}, {-3.0, 3.0}, {-5.0, 3.0}, {-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}};
    auto myEnv2 = Environment{perimeter2};
    Polygon obstacle4 = {{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
    myEnv2.addObstacle(obstacle4);

    auto myEnv = loadEnvironmentFromGeoJSON("big map.json");

    Planner::PathService service;
    std::cout << "Starte PathService..." << std::endl;
    auto result = service.computeFullTask(myEnv, settings, startPos);

    std::cout << "Schreibe das Ergbnis in ein SVG Format..." << std::endl;
    Visualizer::exportToSVG("test_map.svg", myEnv, result.path, result.debugLines, {}, result.slices);

    std::cout << "Setup erfolgreich!" << std::endl;
}
