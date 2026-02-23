#include "GeoJSONReader.hpp"

using json = nlohmann::json;

namespace Planner
{
    Environment GeoJSONReader::loadEnvironmentFromFile(const std::string &filename)
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
}
