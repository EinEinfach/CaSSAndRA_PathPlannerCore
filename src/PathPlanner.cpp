#include "PathPlanner.hpp"
#include "Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

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
                if (GeometryUtils::getIntersectionPoint(perimeterPoints[i],
                                                        perimeterPoints[(i + 1) % perimeterPoints.size()], c, d, intersect))
                {
                    x_intersections.push_back(intersect.x);
                }
                // prüfe ob es bereich gibt wo die Linie auf dem Perimeter liegt, wenn ja, füge diese Linie hnzu
                LineString intersectLine;
                if (GeometryUtils::getIntersectionLine(perimeterPoints[i], perimeterPoints[(i + 1) % perimeterPoints.size()], c, d, intersectLine))
                {
                    const auto &pts = intersectLine.getPoints();
                    for (const auto &p : pts)
                    {
                        x_intersections.push_back(p.x);
                    }
                }
            }

            // 2. NEU: Schnittpunkte mit ALLEN Hindernissen
            for (const auto &obs : env.getObstacles())
            {
                const auto &obsPts = obs.getPoints();
                for (size_t i = 0; i < obsPts.size(); ++i)
                {
                    Point intersect;
                    if (GeometryUtils::getIntersectionPoint(obsPts[i],
                                                            obsPts[(i + 1) % obsPts.size()], c, d, intersect))
                    {
                        x_intersections.push_back(intersect.x);
                    }
                    // prüfe ob es bereich gibt wo die Linie auf dem Obstacle liegt, wenn ja, füge diese Linie hnzu
                    LineString intersectLine;
                    if (GeometryUtils::getIntersectionLine(obsPts[i], obsPts[(i + 1) % obsPts.size()], c, d, intersectLine))
                    {
                        const auto &pts = intersectLine.getPoints();
                        for (const auto &p : pts)
                        {
                            x_intersections.push_back(p.x);
                        }
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

    PathPlanner::PlanningResult PathPlanner::connectSlices(const Environment &env, std::vector<LineString> &slices, Point startPos)
    {
        PlanningResult result;
        if (slices.empty())
            return result;

        std::vector<bool> visited(slices.size(), false);
        Point currentPos = startPos;
        result.path.addPoint(currentPos);

        for (size_t count = 0; count < slices.size(); ++count)
        {
            // 1. Finde das nächste erreichbare Segment ausgehend von currentPos
            std::cout << "--- Bin bei Pos (" << currentPos.x << "|" << currentPos.y << ") ---" << std::endl;
            BestNextSegment next = findBestNext(currentPos, slices, visited, env);

            // 2.1 Weg ohne Umwege möglich
            if (next.index != -1)
            {
                visited[next.index] = true;
                addSliceToPath(result.path, slices[next.index], next.reverse);
            }
            // 2.2 Weg braucht A*
            else
            {
                next = findBestNextFallback(currentPos, slices, visited);
                if (next.index != -1)
                {
                    Point goal = next.reverse ? slices[next.index].getPoints().back() : slices[next.index].getPoints().front();

                    // --- DEBUG LINE START ---
                    // Hier loggen wir die Linie, die findBestNext als "nicht frei" abgelehnt hat
                    LineString failLine;
                    failLine.addPoint(currentPos);
                    failLine.addPoint(goal);
                    result.debugLines.push_back(failLine);
                    // --- DEBUG LINE END ---

                    // Berechne Umweg um Hindernisse
                    std::vector<Point> bypass = findAStarPath(currentPos, goal, env);

                    for (size_t i = 1; i < bypass.size(); ++i)
                    {
                        result.path.addPoint(bypass[i]);
                    }

                    visited[next.index] = true;
                    addSliceToPath(result.path, slices[next.index], next.reverse);
                }
            }
            currentPos = result.path.getPoints().back();
        }

        return result;
    }

    bool PathPlanner::isPathClear(Point a, Point b, const Environment &env)
    {
        // Ein winziges Stückchen von den Endpunkten weggehen,
        // um numerische Probleme an Ecken zu vermeiden
        double epsilon = 0.01;
        Point dir = {b.x - a.x, b.y - a.y};
        double len = std::sqrt(dir.x * dir.x + dir.y * dir.y);

        if (len < epsilon)
            return true;

        // Point testA = {a.x + (dir.x / len) * epsilon, a.y + (dir.y / len) * epsilon};
        // Point testB = {b.x - (dir.x / len) * epsilon, b.y - (dir.y / len) * epsilon};

        // Prüfe ob innerhalb des Perimeters
        auto &perimeter = env.getPerimeter();
        if (!GeometryUtils::isLineCoverdByPolygon(a, b, perimeter))
        {
            return false;
        }

        // Prüfe Hindernisse
        for (const auto &obs : env.getObstacles())
        {
            // Wird Hindernis durchquert
            if (GeometryUtils::isLineIntersectingPolygon(a, b, obs))
            {
                return false;
            }
            // Liegt der Weg innerhalb des Hindernisses
            if (GeometryUtils::isLineCoverdByPolygon(a, b, obs))
            {
                return false;
            }
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
        best.distance = 1e10; // WICHTIG: Mit sehr hohem Wert starten!

        for (size_t j = 0; j < slices.size(); ++j)
        {
            if (visited[j])
                continue;

            const auto &pts = slices[j].getPoints();
            if (pts.empty())
                continue;

            // Wir prüfen Start- und Endpunkt des Slices
            Point start = pts.front();
            Point end = pts.back();

            double dStart = GeometryUtils::calculateDistance(currentPos, start);
            double dEnd = GeometryUtils::calculateDistance(currentPos, end);

            // Wir nehmen das absolut nächste Ende, sofern der Weg frei ist
            if (dStart < best.distance && isPathClear(currentPos, start, env))
            {
                best = {(int)j, false, dStart};
            }
            if (dEnd < best.distance && isPathClear(currentPos, end, env))
            {
                best = {(int)j, true, dEnd};
            }
        }
        if (best.index != -1)
        {
            std::cout << "Entscheidung: Gehe zu Slice #" << best.index
                      << " (Dist: " << best.distance << ", Reverse: " << (best.reverse ? "Ja" : "Nein") << ")" << std::endl;
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

    std::vector<Point> PathPlanner::getNavigationNodes(const Environment &env)
    {
        std::vector<Point> nodes;
        // Ecken der Hindernisse hinzufügen
        for (const auto &obs : env.getObstacles())
        {
            for (const auto &p : obs.getPoints())
            {
                nodes.push_back(p);
            }
        }
        // Hier kommen Perimeter Ecken
        for (const auto &p : env.getPerimeter().getPoints())
        {
            nodes.push_back(p);
        }
        return nodes;
    }

    struct Node
    {
        Point pos;
        double gCost; // Weg vom Start bis hierher
        double hCost; // Schätzung bis zum Ziel (Luftlinie)
        int parentIdx = -1;

        double fCost() const { return gCost + hCost; }
    };

    std::vector<Point> PathPlanner::findAStarPath(Point start, Point goal, const Environment &env)
    {
        std::vector<Point> navPoints = getNavigationNodes(env);
        navPoints.push_back(goal);

        std::vector<Node> openList;
        std::vector<Node> closedList;

        openList.push_back({start, 0.0, GeometryUtils::calculateDistance(start, goal), -1});

        while (!openList.empty())
        {
            // 1. Besten Knoten aus Open List wählen
            size_t bestIdx = 0;
            for (size_t i = 1; i < openList.size(); ++i)
            {
                if (openList[i].fCost() < openList[bestIdx].fCost())
                    bestIdx = i;
            }

            Node current = openList[bestIdx];

            // Ziel erreicht?
            if (GeometryUtils::calculateDistance(current.pos, goal) < 0.001)
            {
                std::vector<Point> path;
                path.push_back(current.pos);
                while (current.parentIdx != -1)
                {
                    current = closedList[current.parentIdx];
                    path.push_back(current.pos);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Von Open nach Closed verschieben
            openList.erase(openList.begin() + bestIdx);
            int currentInClosedIdx = static_cast<int>(closedList.size());
            closedList.push_back(current);

            // 2. Nachbarn prüfen
            for (const auto &nextPos : navPoints)
            {
                // Punkt-Identität prüfen (nicht zu sich selbst springen)
                if (GeometryUtils::calculateDistance(current.pos, nextPos) < 0.001)
                    continue;

                if (isPathClear(current.pos, nextPos, env))
                {
                    double newGCost = current.gCost + GeometryUtils::calculateDistance(current.pos, nextPos);

                    // Check Closed List: Haben wir diesen Ort schon final besucht?
                    bool inClosed = false;
                    for (const auto &cNode : closedList)
                    {
                        if (GeometryUtils::calculateDistance(cNode.pos, nextPos) < 0.001)
                        {
                            inClosed = true;
                            break;
                        }
                    }
                    if (inClosed)
                        continue;

                    // Check Open List: Kennen wir diesen Ort schon und ist der neue Weg besser?
                    bool inOpen = false;
                    for (auto &oNode : openList)
                    {
                        if (GeometryUtils::calculateDistance(oNode.pos, nextPos) < 0.001)
                        {
                            if (newGCost < oNode.gCost)
                            {
                                oNode.gCost = newGCost;
                                oNode.parentIdx = currentInClosedIdx;
                            }
                            inOpen = true;
                            break;
                        }
                    }

                    if (!inOpen)
                    {
                        openList.push_back({nextPos, newGCost, GeometryUtils::calculateDistance(nextPos, goal), currentInClosedIdx});
                    }
                }
            }
        }
        return {};
    }

}