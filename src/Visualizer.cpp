#include "Visualizer.hpp"
#include <fstream>
#include <iostream>

namespace Planner
{
    void Visualizer::exportToSVG(const std::string &filename, const Environment &env, const LineString &path, const std::vector<LineString> &slices)
    {
        std::ofstream file(filename);
        if (!file.is_open())
            return;

        // Alle Punkte sammeln, um die Grenzen (Bounding Box) zu finden
        double minX = 1e10, minY = 1e10, maxX = -1e10, maxY = -1e10;
        auto updateBounds = [&](double x, double y)
        {
            if (x < minX)
                minX = x;
            if (y < minY)
                minY = y;
            if (x > maxX)
                maxX = x;
            if (y > maxY)
                maxY = y;
        };

        // Perimeter prüfen
        for (const auto &p : env.getPerimeter().getPoints())
            updateBounds(p.x, p.y);

        // Skalierung und Puffer
        double scale = 20.0;
        double padding = 20.0;

        // Die viewBox berechnet sich nun aus den tatsächlichen Daten
        // Format: min_x min_y width height
        double vbX = minX * scale - padding;
        double vbY = minY * scale - padding;
        double vbW = (maxX - minX) * scale + 2 * padding;
        double vbH = (maxY - minY) * scale + 2 * padding;

        // SVG Header (wir skalieren alles mal 20 für bessere Sichtbarkeit)
        file << "<svg viewBox=\"" << vbX << " " << vbY << " " << vbW << " " << vbH
             << "\" xmlns=\"http://www.w3.org/2000/svg\" style=\"background: #fafafa\">\n";

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

        // 3. Der geplante Pfad (Blau) optional
        if (!path.getPoints().empty())
        {
            file << "<polyline points=\"";
            for (const auto &p : path.getPoints())
            {
                file << p.x * 20 << "," << p.y * 20 << " ";
            }
            file << "\" fill=\"none\" stroke=\"blue\" stroke-width=\"0.3\" stroke-dasharray=\"2,1\" />\n";
        }

        // 4. Die slices zeichnen (Grün) optional
        if (!slices.empty())
        {
            for (const auto &slice : slices)
            {
                const auto &pts = slice.getPoints();
                if (pts.size() >= 2)
                {
                    file << "<polyline points=\"";
                    for (const auto &p : pts)
                    {
                        file << p.x * 20 << "," << p.y * 20 << " ";
                    }
                    // Grün, etwas dünner, um das Muster gut zu erkennen
                    file << "\" fill=\"none\" stroke=\"#2ecc71\" stroke-width=\"0.3\" />\n";
                }
            }
        }

        file << "</svg>";
        file.close();
        std::cout << "Visualisierung gespeichert in: " << filename << std::endl;
    }

}