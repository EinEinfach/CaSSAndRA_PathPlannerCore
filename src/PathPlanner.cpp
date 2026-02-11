#include "PathPlanner.hpp"
#include "Geometry.hpp"
#include <algorithm>

namespace Planner
{
    std::vector<LineString> PathPlanner::generateSlices(const Environment &env, double spacing)
    {
        std::vector<LineString> result;
        auto perimeterPoints = env.getPerimeter().getPoints();
        if (perimeterPoints.size() < 3)
            return result;

        // Bounding Box für Y finden
        double minY = perimeterPoints[0].y;
        double maxY = perimeterPoints[0].y;
        for (const auto &p : perimeterPoints)
        {
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }

        for (double y = minY + spacing; y < maxY; y += spacing)
        {
            std::vector<double> x_intersections;
            Point c = {-1000.0, y};
            Point d = {1000.0, y};

            // 1. Schnittpunkte mit Perimeter
            for (size_t i = 0; i < perimeterPoints.size(); ++i)
            {
                Point intersect;
                if (GeometryUtils::getIntersection(perimeterPoints[i],
                                                   perimeterPoints[(i + 1) % perimeterPoints.size()], c, d, intersect))
                {
                    x_intersections.push_back(intersect.x);
                }
            }

            // 2. NEU: Schnittpunkte mit ALLEN Hindernissen
            for (const auto &obs : env.getObstacles())
            {
                const auto &obsPts = obs.getPoints();
                for (size_t i = 0; i < obsPts.size(); ++i)
                {
                    Point intersect;
                    if (GeometryUtils::getIntersection(obsPts[i],
                                                       obsPts[(i + 1) % obsPts.size()], c, d, intersect))
                    {
                        x_intersections.push_back(intersect.x);
                    }
                }
            }

            // 3. Sortieren
            std::sort(x_intersections.begin(), x_intersections.end());

            // 4. Paare bilden (Die Logik: Perimeter_In -> Obstacle_In -> Obstacle_Out -> Perimeter_Out)
            // Das funktioniert nur, wenn die Hindernisse komplett im Perimeter liegen!
            for (size_t i = 0; i + 1 < x_intersections.size(); i += 2)
            {
                double xStart = x_intersections[i];
                double xEnd = x_intersections[i + 1];

                // Nur hinzufügen, wenn die Strecke nicht winzig ist (numerische Stabilität)
                if (std::abs(xEnd - xStart) > 1e-7)
                {
                    LineString slice;
                    slice.addPoint({xStart, y});
                    slice.addPoint({xEnd, y});
                    result.push_back(slice);
                }
            }
        }
        return result;
    }
    // std::vector<LineString> PathPlanner::generateSlices(const Environment &env, double spacing)
    // {
    //     std::vector<LineString> result;
    //     auto perimeter = env.getPerimeter().getPoints();
    //     if (perimeter.size() < 3)
    //         return result;

    //     // 1. Min/Max Y des Perimeters finden
    //     double minY = perimeter[0].y;
    //     double maxY = perimeter[0].y;
    //     for (const auto &p : perimeter)
    //     {
    //         minY = std::min(minY, p.y);
    //         maxY = std::max(maxY, p.y);
    //     }

    //     // 2. Von unten nach oben durchgehen
    //     for (double y = minY + spacing; y < maxY; y += spacing)
    //     {
    //         std::vector<double> intersections;

    //         for (size_t i = 0; perimeter.size(); ++i)
    //         {
    //             Point a = perimeter[i];
    //             Point b = perimeter[(i + 1) % perimeter.size()];

    //             // Horizontale Test-Linie (sehr weit links bis sehr weit rechts)
    //             Point c = {-1000.0, y};
    //             Point d = {1000.0, y};

    //             Point intersect;
    //             if (GeometryUtils::getIntersection(a, b, c, d, intersect))
    //             {
    //                 intersections.push_back(intersect.x);
    //             }
    //         }

    //         // 3. Schnittpunkte sortieren (von links nach rechts)
    //         std::sort(intersections.begin(), intersections.end());

    //         // 4. Paare bilden (in/out prinzip)
    //         // WICHTIG: Wenn wir eine ungerade Anzahl an Schnittpunkten haben,
    //         // ist die Linie wahrscheinlich exakt durch eine Ecke gegangen.
    //         // Wir ignorieren den letzten Punkt in dem Fall, um den Crash zu verhindern.
    //         for (size_t i = 0; i + 1 < intersections.size(); i += 2)
    //         {
    //             LineString slice;
    //             slice.addPoint({intersections[i], y});
    //             slice.addPoint({intersections[i + 1], y});
    //             result.push_back(slice);
    //         }
    //     }
    //     return result;
    // }

}