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
            // Jedes Segment zwischen zwei Schnittpunkten prüfen
            for (size_t i = 0; i + 1 < x_intersections.size(); ++i)
            {
                double xStart = x_intersections[i];
                double xEnd = x_intersections[i + 1];

                if (std::abs(xEnd - xStart) < 1e-7)
                    continue;

                // Testpunkt in der Mitte des Segments
                Point midPoint = {(xStart + xEnd) / 2.0, y};

                // Ist die Mitte im Perimeter?
                bool inPerimeter = GeometryUtils::isPointInPolygon(midPoint, env.getPerimeter());

                // Ist die Mitte in IRGENDEINEM Hindernis?
                bool inObstacle = false;
                for (const auto &obs : env.getObstacles())
                {
                    if (GeometryUtils::isPointInPolygon(midPoint, obs))
                    {
                        inObstacle = true;
                        break;
                    }
                }

                // Nur wenn im Perimeter UND NICHT im Hindernis
                if (inPerimeter && !inObstacle)
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

    LineString PathPlanner::connectSlices(std::vector<LineString>& slices) {
        LineString fullPath;
        if (slices.empty()) return fullPath;

        bool reverseDirection = false;

        for (auto& slice : slices) {
            auto pts = slice.getPoints();
            if (pts.size() < 2) continue;

            // Wenn wir in der "Rückwärts-Phase" des Zick-Zacks sind:
            if(reverseDirection) {
                // Punkte in umgekehrter Reihenfolge hinzufügen
                for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
                    fullPath.addPoint(*it);
                }
            } else {
                // Punkte in normaler Reihenfolge hinzufügen
                for (const auto& p : pts) {
                    fullPath.addPoint(p);
                }
            }
            // Richtung für den nächsten Slice umkehren
            reverseDirection = !reverseDirection;
        }
        return fullPath;
    }

    bool isPathClear(Point a, Point b, const Environment& env) {
        for (const auto& obs : env.getObstacles()) {
            if (GeometryUtils::isLineIntersectingPolygon(a, b, obs)) {
                return false;
            }
        }
        if (GeometryUtils::isLineIntersectingPolygon(a, b, env.getPerimeter())) {
            return false;
        }
        return true;
    }
}