#include "Visualizer.hpp"
#include <fstream>
#include <iostream>

namespace Planner
{
    void Visualizer::exportToSVG(const std::string& filename, const Enviroment& env, const LineString& path)
    {
        std::ofstream file(filename);
        if (!file.is_open())
            return;

        // SVG Header (wir skalieren alles mal 20 für bessere Sichtbarkeit)
        file << "<svg viewBox=\"-10 -10 500 300\" xmlns=\"http://www.w3.org/2000/svg\">\n";

        //
        // 1. Perimeter zeichnen (Grau ausgefüllt)
        file << "<polygon points=\"";
        for (const auto &p : env.getPerimeter().getPoints())
        {
            file << p.x * 20 << "," << p.y * 20 << " ";
        }
        file << "\" fill=\"#f0f0f0\" stroke=\"black\" stroke-width=\"0.5\" />\n";

        // 2. Hindernisse zeichnen (Rot)
        for (const auto &obs : env.getObstacles())
        {
            file << "<polygon points=\"";
            for (const auto &p : obs.getPoints())
            {
                file << p.x * 20 << "," << p.y * 20 << " ";
            }
            file << "\" fill=\"#ffcccc\" stroke=\"red\" stroke-width=\"0.5\" />\n";
        }

        // 3. Der geplante Pfad (Blau)
        if (!path.getPoints().empty())
        {
            file << "<polyline points=\"";
            for (const auto &p : path.getPoints())
            {
                file << p.x * 20 << "," << p.y * 20 << " ";
            }
            file << "\" fill=\"none\" stroke=\"blue\" stroke-width=\"1\" stroke-dasharray=\"2,1\" />\n";
        }
        file << "</svg>";
        file.close();
        std::cout << "Visualisierung gespeichert in: " << filename << std::endl;
    }

}