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

        file << "<defs>\n"
             << "  \n"
             << "  <g id=\"tractor\">\n"
             << "    <rect x=\"-8\" y=\"-6\" width=\"16\" height=\"12\" rx=\"2\" fill=\"#228B22\" /> \n"
             << "    <rect x=\"2\" y=\"-7\" width=\"4\" height=\"14\" rx=\"1\" fill=\"#333\" />     \n"
             << "    <rect x=\"-6\" y=\"-5\" width=\"3\" height=\"10\" rx=\"1\" fill=\"#333\" />    \n"
             << "    <rect x=\"-2\" y=\"-3\" width=\"6\" height=\"6\" rx=\"1\" fill=\"#87CEEB\" />  \n"
             << "  </g>\n"
             << "  \n"
             << "  <marker id=\"arrowhead\" markerWidth=\"10\" markerHeight=\"7\" refX=\"5\" refY=\"3.5\" orient=\"auto\">\n"
             << "    <polygon points=\"0 0, 10 3.5, 0 7\" fill=\"#00008b\" fill-opacity=\"0.4\" />\n"
             << "  </marker>\n"
             << "</defs>\n";

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

        // --- 1. LAYER: Der geplante Pfad (Halbtransparent, damit man sieht was darunter liegt) ---
        if (!path.getPoints().empty())
        {
            // 1. Der Pfad selbst (Layer 1)
            const auto &pts = path.getPoints();

            // A. Der eigentliche Pfad (Hintergrund)
            file << "<polyline points=\"";
            for (const auto &p : pts)
                file << transformX(p.x) << "," << transformY(p.y) << " ";
            file << "\" fill=\"none\" stroke=\"#87cefa\" stroke-width=\"2.0\" stroke-opacity=\"0.6\" />\n";

            // B. Animation
            double totalLength = 0.0;
            const auto &p = path.getPoints();
            for (size_t i = 0; i < pts.size() - 1; ++i)
            {
                double dx = p[i + 1].x - p[i].x;
                double dy = p[i + 1].y - p[i].y;
                totalLength += std::sqrt(dx * dx + dy * dy);
            }

            // Geschwindigkeit definieren: z.B. 5.0 Meter pro Sekunde (im SVG-Maßstab)
            double metersPerSecond = 5.0;
            double durationInSeconds = totalLength / metersPerSecond;

            file << "  \n";
            file << "  <path id=\"animPath\" d=\"M ";
            for (size_t i = 0; i < pts.size(); ++i)
            {
                file << transformX(pts[i].x) << " " << transformY(pts[i].y) << (i == pts.size() - 1 ? "" : " L ");
            }
            file << "\" fill=\"none\" stroke=\"none\" />\n";

            file << "  \n"
                 << "  <g>\n"
                 << "    <use href=\"#tractor\" />\n"
                 << "    <animateMotion dur=\"" << durationInSeconds << "s\" repeatCount=\"indefinite\" rotate=\"auto\">\n"
                 << "      <mpath href=\"#animPath\" />\n"
                 << "    </animateMotion>\n"
                 << "  </g>\n";

            // 2. Startpunkt als Kreuz markieren (erster Punkt)
            const auto &start = path.getPoints().front();
            double sX = transformX(start.x);
            double sY = transformY(start.y);
            double crossSize = 6.0; // Größe des Kreuzes

            file << "  \n";
            file << "  <line x1=\"" << sX - crossSize << "\" y1=\"" << sY - crossSize << "\" x2=\"" << sX + crossSize << "\" y2=\"" << sY + crossSize << "\" stroke=\"#00008b\" stroke-width=\"2\" />\n";
            file << "  <line x1=\"" << sX - crossSize << "\" y1=\"" << sY + crossSize << "\" x2=\"" << sX + crossSize << "\" y2=\"" << sY - crossSize << "\" stroke=\"#00008b\" stroke-width=\"2\" />\n";
        }

        // --- 2. LAYER: A* Suchraum (Hintergrund) ---
        // for (const auto &line : debugLinesSec)
        // {
        //     const auto &pts = line.getPoints();
        //     if (pts.size() >= 2)
        //     {
        //         file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
        //              << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
        //              << "\" stroke=\"#bbb\" stroke-width=\"0.8\" stroke-dasharray=\"2,2\" />\n";
        //     }
        // }

        // --- 3. LAYER: Slices / Bahnen ---
        // for (size_t i = 0; i < originalSlices.size(); ++i)
        // {
        //     const auto &pts = originalSlices[i].getPoints();
        //     if (pts.size() >= 2)
        //     {
        //         file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
        //              << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
        //              << "\" stroke=\"green\" stroke-width=\"0.5\" stroke-dasharray=\"5,5\" />\n";
        //     }
        // }

        // --- 4. LAYER: Fehler / Verbindungslinien (Ganz oben, da kritisch) ---
        for (const auto &line : debugLines)
        {
            const auto &pts = line.getPoints();
            if (pts.size() >= 2)
            {
                file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
                     << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
                     << "\" stroke=\"red\" stroke-width=\"3\" stroke-dasharray=\"8,4\" />\n";
            }
        }

        // Pfeile
        const auto &pathPts = path.getPoints();
        if (pathPts.size() >= 2)
        {
            for (size_t i = 0; i < pathPts.size() - 1; ++i)
            {
                const auto &p1 = pathPts[i];
                const auto &p2 = pathPts[i + 1];

                // Berechne Distanz dieses Segments
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                // Nur Pfeile auf Segmenten zeichnen, die länger als z.B. 1.0 Meter sind
                // Das ignoriert meist die kurzen Wendemanöver zwischen den Bahnen
                if (dist > 1.0)
                {
                    double midX = (p1.x + p2.x) / 2.0;
                    double midY = (p1.y + p2.y) / 2.0;

                    // Zeichne einen kurzen Richtungsvektor für den Marker
                    // Wir nutzen hier eine etwas kräftigere Farbe als das Pfad-Blau
                    file << "<line x1=\"" << transformX(midX) << "\" y1=\"" << transformY(midY)
                         << "\" x2=\"" << transformX(midX + (dx / dist) * 0.1) << "\" y2=\"" << transformY(midY + (dy / dist) * 0.1)
                         << "\" fill=\"none\" stroke=\"none\" marker-start=\"url(#arrowhead)\" />\n";
                }
            }
        }

        // Slice-Beschriftung
        for (size_t i = 0; i < originalSlices.size(); ++i)
        {
            const auto &pts = originalSlices[i].getPoints();
            if (pts.size() >= 2)
            {
                // 1. Die Bahn zeichnen (gestrichelt)
                file << "<line x1=\"" << transformX(pts[0].x) << "\" y1=\"" << transformY(pts[0].y)
                     << "\" x2=\"" << transformX(pts[1].x) << "\" y2=\"" << transformY(pts[1].y)
                     << "\" stroke=\"green\" stroke-width=\"0.5\" stroke-dasharray=\"5,5\" />\n";

                // 2. Mittelpunkt berechnen
                double midX = (pts[0].x + pts[1].x) / 2.0;
                double midY = (pts[0].y + pts[1].y) / 2.0;

                // 3. Slice-Index (Nummer) etwas versetzt daneben schreiben
                file << "<text x=\"" << transformX(midX) + 5 << "\" y=\"" << transformY(midY) - 5
                     << "\" fill=\"darkgreen\" font-size=\"8\" font-family=\"Arial\" opacity=\"0.6\">" << i << "</text>\n";
            }
        }

        // --- LEGENDE ---
        double legendX = width - 210; // Position rechts oben
        double legendY = 20;

        file << "  \n";
        file << "  <rect x=\"" << legendX << "\" y=\"" << legendY << "\" width=\"190\" height=\"150\" rx=\"5\" fill=\"white\" fill-opacity=\"0.8\" stroke=\"#ccc\" />\n";
        file << "  <text x=\"" << legendX + 10 << "\" y=\"" << legendY + 20 << "\" font-family=\"Arial\" font-size=\"14\" font-weight=\"bold\">Legende</text>\n";

        auto addLegendItem = [&](int index, std::string color, std::string label, bool dashed = false, double strokeWidth = 2.0)
        {
            double itemY = legendY + 45 + (index * 20);
            file << "  <line x1=\"" << legendX + 10 << "\" y1=\"" << itemY - 5 << "\" x2=\"" << legendX + 40 << "\" y2=\"" << itemY - 5
                 << "\" stroke=\"" << color << "\" stroke-width=\"" << strokeWidth << "\" "
                 << (dashed ? "stroke-dasharray=\"5,5\"" : "") << " />\n";
            file << "  <text x=\"" << legendX + 50 << "\" y=\"" << itemY << "\" font-family=\"Arial\" font-size=\"12\" fill=\"#333\">" << label << "</text>\n";
        };

        addLegendItem(0, "#87cefa", "Geplanter Pfad", false, 2.0);
        addLegendItem(1, "red", "Weg löst A* Suche aus", true, 2.5);
        // addLegendItem(2, "grey", "A* Suchraum", true, 1.0);
        // addLegendItem(3, "green", "Slices / Bahnen", true, 0.5);

        // Startpunkt Kreuz in der Legende
        double startY = legendY + 45 + (4 * 20);
        file << "  <line x1=\"" << legendX + 15 << "\" y1=\"" << startY - 10 << "\" x2=\"" << legendX + 35 << "\" y2=\"" << startY << "\" stroke=\"#00008b\" stroke-width=\"2\" />\n";
        file << "  <line x1=\"" << legendX + 15 << "\" y1=\"" << startY << "\" x2=\"" << legendX + 35 << "\" y2=\"" << startY - 10 << "\" stroke=\"#00008b\" stroke-width=\"2\" />\n";
        file << "  <text x=\"" << legendX + 50 << "\" y=\"" << startY << "\" font-family=\"Arial\" font-size=\"12\" fill=\"#333\">Startpunkt</text>\n";

        // Marker für Flächen
        double areaY = legendY + 45 + (5 * 20);
        file << "  <rect x=\"" << legendX + 10 << "\" y=\"" << areaY - 10 << "\" width=\"30\" height=\"10\" fill=\"#ffcccc\" stroke=\"red\" />\n";
        file << "  <text x=\"" << legendX + 50 << "\" y=\"" << areaY << "\" font-family=\"Arial\" font-size=\"12\" fill=\"#333\">Hindernis</text>\n";

        double tractorLegendY = legendY + 45 + (3 * 20);
        file << "  <use href=\"#tractor\" transform=\"translate(" << legendX + 25 << "," << tractorLegendY - 5 << ") scale(0.8)\" />\n";
        file << "  <text x=\"" << legendX + 50 << "\" y=\"" << tractorLegendY << "\" font-family=\"Arial\" font-size=\"12\" fill=\"#333\">Mäher (animiert)</text>\n";

        file << "</svg>";
        file.close();
    }
}