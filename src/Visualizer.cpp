#include "Visualizer.hpp"
#include <fstream>
#include <iostream>

namespace Planner
{
    void Visualizer::exportToSVG(const std::string &filename, const Environment &env, const LineString &path, const std::vector<LineString> &debugLines, const std::vector<LineString> &debugLinesSec, const std::vector<LineString> &originalSlices)
    {
        std::ofstream file(filename);
        if (!file.is_open())
            return;

        // 1. Bounds finden
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

        for (const auto &p : env.getPerimeter().getPoints())
            updateBounds(p.x, p.y);

        // Maßstab und Padding
        double scale = 40.0;   // Höherer Scale macht 1er Schritte besser sichtbar
        double padding = 60.0; // Mehr Platz für die Achsenbeschriftung

        double width = (maxX - minX) * scale + 2 * padding;
        double height = (maxY - minY) * scale + 2 * padding;

        auto transformX = [&](double x)
        { return (x - minX) * scale + padding; };
        auto transformY = [&](double y)
        { return height - ((y - minY) * scale + padding); };

        file << "<svg viewBox=\"0 0 " << width << " " << height
             << "\" xmlns=\"http://www.w3.org/2000/svg\" style=\"background: #fafafa\">\n";

        // --- NEU: Gitter und Achsbeschriftung ---

        // X-Achse Ticks und Gitter (vertikale Linien)
        for (int x = (int)std::floor(minX); x <= (int)std::ceil(maxX); ++x)
        {
            double xPos = transformX(x);
            // Gitterlinie
            file << "<line x1=\"" << xPos << "\" y1=\"0\" x2=\"" << xPos << "\" y2=\"" << height
                 << "\" stroke=\"#eee\" stroke-width=\"1\" />\n";
            // Zahl an der Unterseite
            file << "<text x=\"" << xPos << "\" y=\"" << height - 10
                 << "\" fill=\"#888\" font-size=\"12\" font-family=\"Arial\" text-anchor=\"middle\">" << x << "</text>\n";
        }

        // Y-Achse Ticks und Gitter (horizontale Linien)
        for (int y = (int)std::floor(minY); y <= (int)std::ceil(maxY); ++y)
        {
            double yPos = transformY(y);
            // Gitterlinie
            file << "<line x1=\"0\" y1=\"" << yPos << "\" x2=\"" << width << "\" y2=\"" << yPos
                 << "\" stroke=\"#eee\" stroke-width=\"1\" />\n";
            // Zahl an der linken Seite
            file << "<text x=\"25\" y=\"" << yPos + 4
                 << "\" fill=\"#888\" font-size=\"12\" font-family=\"Arial\" text-anchor=\"end\">" << y << "</text>\n";
        }

        // Hauptachsen hervorheben (0,0)
        file << "<line x1=\"0\" y1=\"" << transformY(0) << "\" x2=\"" << width << "\" y2=\"" << transformY(0)
             << "\" stroke=\"#bbb\" stroke-width=\"2\" />\n";
        file << "<line x1=\"" << transformX(0) << "\" y1=\"0\" x2=\"" << transformX(0) << "\" y2=\"" << height
             << "\" stroke=\"#bbb\" stroke-width=\"2\" />\n";

        // --- REST DER ZEICHNUNG ---

        // Perimeter
        file << "<polygon points=\"";
        for (const auto &p : env.getPerimeter().getPoints())
            file << transformX(p.x) << "," << transformY(p.y) << " ";
        file << "\" fill=\"#f0f0f0\" stroke=\"black\" stroke-width=\"1\" fill-opacity=\"0.5\" />\n";

        // Hindernisse
        for (const auto &obs : env.getObstacles())
        {
            file << "<polygon points=\"";
            for (const auto &p : obs.getPoints())
                file << transformX(p.x) << "," << transformY(p.y) << " ";
            file << "\" fill=\"#ffcccc\" stroke=\"red\" stroke-width=\"1\" />\n";
        }

        // Debug-Lines (Warum wurde nicht verbunden?)
        for (const auto &line : debugLines)
        {
            const auto &pts = line.getPoints();
            if (pts.size() >= 2)
            {
                file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
                     << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
                     << "\" stroke=\"red\" stroke-width=\"2.5\" stroke-dasharray=\"5,5\" />\n";
            }
        }

        // Debug-Lines (AStar Wege die betracttet worden)
        for (const auto &line : debugLinesSec)
        {
            const auto &pts = line.getPoints();
            if (pts.size() >= 2)
            {
                file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
                     << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
                     << "\" stroke=\"grey\" stroke-width=\"1.0\" stroke-dasharray=\"5,5\" />\n";
            }
        }

        // Slice-Beschriftung
        for (size_t i = 0; i < originalSlices.size(); ++i)
        {
            const auto &pts = originalSlices[i].getPoints();
            if (!pts.empty())
            {
                Point p = pts.front();
                file << "<text x=\"" << transformX(p.x) << "\" y=\"" << transformY(p.y) - 8
                     << "\" fill=\"darkgreen\" font-weight=\"bold\" font-size=\"8\" font-family=\"Arial\">" << i << "</text>\n";
                file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
                     << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
                     << "\" stroke=\"green\" stroke-width=\"0.5\" stroke-dasharray=\"5,5\" />\n";
            }
        }

        // Pfad
        if (!path.getPoints().empty())
        {
            file << "<polyline points=\"";
            for (const auto &p : path.getPoints())
                file << transformX(p.x) << "," << transformY(p.y) << " ";
            file << "\" fill=\"none\" stroke=\"#87cefa\" stroke-width=\"1.0\" />\n";
        }

        file << "</svg>";
        file.close();
    }
    // void Visualizer::exportToSVG(const std::string &filename, const Environment &env, const LineString &path, const std::vector<LineString> &debugLines, const std::vector<LineString> &originalSlices)
    // {
    //     std::ofstream file(filename);
    //     if (!file.is_open())
    //         return;

    //     // 1. Bounds finden
    //     double minX = 1e10, minY = 1e10, maxX = -1e10, maxY = -1e10;
    //     auto updateBounds = [&](double x, double y)
    //     {
    //         if (x < minX)
    //             minX = x;
    //         if (y < minY)
    //             minY = y;
    //         if (x > maxX)
    //             maxX = x;
    //         if (y > maxY)
    //             maxY = y;
    //     };

    //     for (const auto &p : env.getPerimeter().getPoints())
    //         updateBounds(p.x, p.y);

    //     // Konstanten für die Darstellung
    //     double scale = 20.0;
    //     double padding = 40.0; // Etwas mehr Platz für Achsenbeschriftung

    //     // ViewBox Maße
    //     double width = (maxX - minX) * scale + 2 * padding;
    //     double height = (maxY - minY) * scale + 2 * padding;

    //     // Hilfsfunktion zur Koordinatentransformation (Spiegelung an der Y-Achse)
    //     // Wir ziehen minY ab, damit die Zeichnung bei 0 beginnt,
    //     // und ziehen das Ergebnis von der Gesamthöhe ab, um Y umzukehren.
    //     auto transformX = [&](double x)
    //     { return (x - minX) * scale + padding; };
    //     auto transformY = [&](double y)
    //     { return height - ((y - minY) * scale + padding); };

    //     file << "<svg viewBox=\"0 0 " << width << " " << height
    //          << "\" xmlns=\"http://www.w3.org/2000/svg\" style=\"background: #fafafa\">\n";

    //     // 2. Achsen zeichnen (X und Y bei 0,0 falls im Sichtfeld, sonst am Rand)
    //     double xAxisY = transformY(0.0);
    //     double yAxisX = transformX(0.0);

    //     // X-Achse
    //     file << "<line x1=\"0\" y1=\"" << xAxisY << "\" x2=\"" << width << "\" y2=\"" << xAxisY
    //          << "\" stroke=\"#ccc\" stroke-width=\"1\" />\n";
    //     // Y-Achse
    //     file << "<line x1=\"" << yAxisX << "\" y1=\"0\" x2=\"" << yAxisX << "\" y2=\"" << height
    //          << "\" stroke=\"#ccc\" stroke-width=\"1\" />\n";

    //     // 3. Perimeter (Grau)
    //     file << "<polygon points=\"";
    //     for (const auto &p : env.getPerimeter().getPoints())
    //         file << transformX(p.x) << "," << transformY(p.y) << " ";
    //     file << "\" fill=\"#f0f0f0\" stroke=\"black\" stroke-width=\"0.5\" />\n";

    //     // 4. Hindernisse (Rot)
    //     for (const auto &obs : env.getObstacles())
    //     {
    //         file << "<polygon points=\"";
    //         for (const auto &p : obs.getPoints())
    //             file << transformX(p.x) << "," << transformY(p.y) << " ";
    //         file << "\" fill=\"#ffcccc\" stroke=\"red\" stroke-width=\"0.5\" />\n";
    //     }

    //     // 5. Slice-Nummern zur Fehleranalyse (Index-Beschriftung)
    //     // Falls du die originalSlices übergibst, beschriften wir sie hier
    //     for (size_t i = 0; i < originalSlices.size(); ++i)
    //     {
    //         const auto &pts = originalSlices[i].getPoints();
    //         if (!pts.empty())
    //         {
    //             Point p = pts.front();
    //             file << "<text x=\"" << transformX(p.x) << "\" y=\"" << transformY(p.y) - 5
    //                  << "\" fill=\"green\" font-size=\"10\" font-family=\"Arial\">#" << i << "</text>\n";
    //         }
    //     }

    //     // 6. Debug-Lines (Rote gestrichelte Linien für abgelehnte Verbindungen)
    //     for (const auto &line : debugLines)
    //     {
    //         const auto &pts = line.getPoints();
    //         if (pts.size() >= 2)
    //         {
    //             file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
    //                  << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
    //                  << "\" stroke=\"red\" stroke-width=\"1.5\" stroke-dasharray=\"4,2\" />\n";
    //         }
    //     }

    //     // 7. Geplanter Pfad (Blau)
    //     if (!path.getPoints().empty())
    //     {
    //         file << "<polyline points=\"";
    //         for (const auto &p : path.getPoints())
    //             file << transformX(p.x) << "," << transformY(p.y) << " ";
    //         file << "\" fill=\"none\" stroke=\"blue\" stroke-width=\"1\" stroke-dasharray=\"2,2\" />\n";
    //     }

    //     file << "</svg>";
    //     file.close();
    // }
}