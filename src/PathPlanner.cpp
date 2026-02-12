#include "PathPlanner.hpp"
#include "Geometry.hpp"
#include <algorithm>
#include <cmath>

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
    LineString PathPlanner::connectSlices(const Environment &env, std::vector<LineString> &slices)
    {
        LineString fullPath;
        if (slices.empty())
            return fullPath;

        std::vector<bool> visited(slices.size(), false);

        // Startpunkt: Wir nehmen den ersten Punkt vom ersten Slice
        visited[0] = true;
        for (const auto &p : slices[0].getPoints())
            fullPath.addPoint(p);
        Point currentPos = slices[0].getPoints().back();

        for (size_t count = 1; count < slices.size(); ++count)
        {
            int bestNext = -1;
            double bestDist = 1e10;
            bool bestReverse = false;

            for (size_t j = 0; j < slices.size(); ++j)
            {
                if (visited[j])
                    continue;

                const auto &pts = slices[j].getPoints();
                Point start = pts.front();
                Point end = pts.back();

                // Wir testen: currentPos -> start (normal abfahren)
                double dStart = std::sqrt(std::pow(start.x - currentPos.x, 2) + std::pow(start.y - currentPos.y, 2));
                // Wir testen: currentPos -> end (rückwärts abfahren)
                double dEnd = std::sqrt(std::pow(end.x - currentPos.x, 2) + std::pow(end.y - currentPos.y, 2));

                // Favorisiere den Punkt, der frei erreichbar ist
                if (isPathClear(currentPos, start, env))
                {
                    if (dStart < bestDist)
                    {
                        bestDist = dStart;
                        bestNext = j;
                        bestReverse = false;
                    }
                }
                if (isPathClear(currentPos, end, env))
                {
                    if (dEnd < bestDist)
                    {
                        bestDist = dEnd;
                        bestNext = j;
                        bestReverse = true;
                    }
                }
            }

            // Wenn wir einen freien Weg gefunden haben:
            if (bestNext != -1)
            {
                visited[bestNext] = true;
                const auto &nextPts = slices[bestNext].getPoints();
                if (bestReverse)
                {
                    for (auto it = nextPts.rbegin(); it != nextPts.rend(); ++it)
                        fullPath.addPoint(*it);
                    currentPos = nextPts.front();
                }
                else
                {
                    for (const auto &p : nextPts)
                        fullPath.addPoint(p);
                    currentPos = nextPts.back();
                }
            }
            else
            {
                // NOTFALL: Wenn kein Weg frei ist, nimm den absolut nächsten unbesuchten
                // (Hier wird die Linie später durch das Obstacle gehen, bis wir A* haben)
                for (size_t j = 0; j < slices.size(); ++j)
                {
                    if (!visited[j])
                    {
                        visited[j] = true;
                        const auto &pts = slices[j].getPoints();
                        // Einfachste Distanz entscheiden (Zick-Zack Erhaltung)
                        double dS = std::sqrt(std::pow(pts.front().x - currentPos.x, 2) + std::pow(pts.front().y - currentPos.y, 2));
                        double dE = std::sqrt(std::pow(pts.back().x - currentPos.x, 2) + std::pow(pts.back().y - currentPos.y, 2));

                        if (dE < dS)
                        {
                            for (auto it = pts.rbegin(); it != pts.rend(); ++it)
                                fullPath.addPoint(*it);
                            currentPos = pts.front();
                        }
                        else
                        {
                            for (const auto &p : pts)
                                fullPath.addPoint(p);
                            currentPos = pts.back();
                        }
                        break;
                    }
                }
            }
        }
        return fullPath;
    }

    bool PathPlanner::isPathClear(Point a, Point b, const Environment &env)
    {
        for (const auto &obs : env.getObstacles())
        {
            if (GeometryUtils::isLineIntersectingPolygon(a, b, obs))
            {
                return false;
            }
        }
        if (GeometryUtils::isLineIntersectingPolygon(a, b, env.getPerimeter()))
        {
            return false;
        }
        return true;
    }
}