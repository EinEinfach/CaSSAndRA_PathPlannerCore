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
    LineString PathPlanner::connectSlices(const Environment &env, std::vector<LineString> &slices, Point startPos)
    {
        LineString fullPath;
        if (slices.empty())
            return fullPath;

        std::vector<bool> visited(slices.size(), false);
        Point currentPos = startPos;
        fullPath.addPoint(currentPos);

        // Wir fangen bei startPos.
        for (size_t count = 0; count < slices.size(); ++count)
        {

            // 1. Finde das nächste erreichbare Segment ausgehend von currentPos (anfangs startPos)
            BestNextSegment next = findBestNext(currentPos, slices, visited, env);

            // 2. Fallback, falls der Weg blockiert ist
            if (next.index == -1)
            {
                next = findBestNextFallback(currentPos, slices, visited);
            }

            // 3. Segment hinzufügen
            visited[next.index] = true;
            addSliceToPath(fullPath, slices[next.index], next.reverse);

            // 4. Update der Position für den nächsten Durchlauf
            currentPos = fullPath.getPoints().back();
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

    void PathPlanner::addSliceToPath(LineString &path, const LineString &slice, bool reverse)
    {
        const auto &pts = slice.getPoints();
        if (reverse)
        {
            for (auto it = pts.rbegin(); it != pts.rend(); ++it)
                path.addPoint(*it);
        }
        else
        {
            for (const auto &p : pts)
                path.addPoint(p);
        }
    }

    PathPlanner::BestNextSegment PathPlanner::findBestNext(Point currentPos,
                                                           const std::vector<LineString> &slices,
                                                           const std::vector<bool> &visited,
                                                           const Environment &env)
    {
        BestNextSegment best;
        for (size_t j = 0; j < slices.size(); ++j)
        {
            if (visited[j])
                continue;

            const auto &pts = slices[j].getPoints();
            double dStart = GeometryUtils::calculateDistance(currentPos, pts.front());
            double dEnd = GeometryUtils::calculateDistance(currentPos, pts.back());

            if (dStart < best.distance && isPathClear(currentPos, pts.front(), env))
            {
                best = {(int)j, false, dStart};
            }
            if (dEnd < best.distance && isPathClear(currentPos, pts.back(), env))
            {
                best = {(int)j, true, dEnd};
            }
        }
        return best;
    }

    PathPlanner::BestNextSegment PathPlanner::findBestNextFallback(Point currentPos, const std::vector<LineString> &slices, const std::vector<bool> &visited)
    {
        BestNextSegment best;
        for (size_t j = 0; j < slices.size(); ++j)
        {
            if (visited[j])
                continue;

            const auto &pts = slices[j].getPoints();
            double dStart = GeometryUtils::calculateDistance(currentPos, pts.front());
            double dEnd = GeometryUtils::calculateDistance(currentPos, pts.back());

            // Hier prüfen wir NICHT isPathClear, wir nehmen einfach das Naheliegendste
            if (dStart < best.distance)
            {
                best = {(int)j, false, dStart};
            }
            if (dEnd < best.distance)
            {
                best = {(int)j, true, dEnd};
            }
        }
        return best;
    }
}