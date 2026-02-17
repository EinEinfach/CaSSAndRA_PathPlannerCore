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
                // Prüfe echte Durchschüsse
                if (GeometryUtils::getIntersectionPoint(perimeterPoints[i],
                                                        perimeterPoints[(i + 1) % perimeterPoints.size()], c, d, intersect))
                {
                    x_intersections.push_back(intersect.x);
                }
                // Prüfe Berührungen
                if (GeometryUtils::getTouchPoint(perimeterPoints[i],
                                                 perimeterPoints[(i + 1) % perimeterPoints.size()], c, d, intersect))
                {
                    x_intersections.push_back(intersect.x);
                }
                // Prüfe ob es bereich gibt wo die Linie auf dem Perimeter liegt, wenn ja, füge diese Linie hnzu
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
                    // Prüfe echte Durchschüsse
                    if (GeometryUtils::getIntersectionPoint(obsPts[i],
                                                            obsPts[(i + 1) % obsPts.size()], c, d, intersect))
                    {
                        x_intersections.push_back(intersect.x);
                    }
                    // Prüfe Berührungen
                    if (GeometryUtils::getTouchPoint(obsPts[i],
                                                     obsPts[(i + 1) % obsPts.size()], c, d, intersect))
                    {
                        x_intersections.push_back(intersect.x);
                    }

                    // Prüfe ob es bereich gibt wo die Linie auf dem Obstacle liegt, wenn ja, füge diese Linie hnzu
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
                bool inPerimeter = GeometryUtils::isPointCoveredByPolygon(midPoint, env.getPerimeter());

                // Ist die Mitte in IRGENDEINEM Hindernis?
                bool inObstacle = false;
                for (const auto &obs : env.getObstacles())
                {
                    if (GeometryUtils::isPointCoveredByPolygon(midPoint, obs))
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
        if (!GeometryUtils::isLineCoveredByPolygon(a, b, perimeter))
        {
            return false;
        }

        // Prüfe Hindernisse
        for (const auto &obs : env.getObstacles())
        {
            // 1. Wird die Kante echt geschnitten? (Der "Durchschuss")
            if (GeometryUtils::isLineIntersectingPolygon(a, b, obs))
            {
                return false;
            }
            // 2. Liegt die Linie im Inneren?
            // Wir prüfen den Mittelpunkt mit der "strikten" Inside-Methode.
            // Wenn der Mittelpunkt STRIKT drin ist, ist die Linie im Hindernis.
            // Wenn der Mittelpunkt auf der Kante liegt, gibt isPointInsidePolygon FALSE,
            // und der Weg wird (korrekterweise) als frei betrachtet.
            Point mid = {(a.x + b.x) / 2.0, (a.y + b.y) / 2.0};
            if (GeometryUtils::isPointInsidePolygon(mid, obs))
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

    std::vector<PathPlanner::NavNode> PathPlanner::getExtendedNavNodes(const Environment &env)
    {
        std::vector<NavNode> nodes;
        const double eps = 0.001; // Toleranz für Punkt-Gleichheit (1mm)

        // Hilfsfunktion zum Prüfen auf Duplikate
        auto isDuplicate = [&](Point p)
        {
            for (const auto &node : nodes)
            {
                double dx = node.pos.x - p.x;
                double dy = node.pos.y - p.y;
                if (std::sqrt(dx * dx + dy * dy) < eps)
                    return true;
            }
            return false;
        };

        // 1. Virtual Wire Punkte zuerst (haben Vorrang wegen wireIdx)
        const auto &wirePoints = env.getVirtualWire().getPoints();
        for (size_t i = 0; i < wirePoints.size(); ++i)
        {
            nodes.push_back({wirePoints[i], true, i});
        }

        // 2. Standard Navigationspunkte (Ecken)
        // Wir rufen deine bestehende getNavigationNodes() auf
        std::vector<Point> basicNodes = getNavigationNodes(env);
        for (const auto &p : basicNodes)
        {
            if (!isDuplicate(p))
            {
                nodes.push_back({p, false, 0});
            }
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

    // Interne Struktur für den A*-Algorithmus
    struct AStarNode
    {
        Point pos;
        double gCost;
        double hCost;
        int parentIdx;
        bool isWire;
        size_t wireIdx;

        double fCost() const { return gCost + hCost; }
    };

    std::vector<Point> PathPlanner::findAStarPath(Point start, Point goal, const Environment &env)
    {
        /*
        A* Regel für Virtual Wire
        1. Man darf an jedem beliebiger virtual Wire Koordinate virtual Wire betreten (unter der Berücksichtigung isPathClear)
        2. Man darf an jeder beliebigen virtual Wire Koordinate den virtual Wire verlassen (unter der Berücksichtigung isPathClear)
        3. Innerhalb des virtual Wire darf man die Punkte nur nacheinander Abfahren (rückwärts oder vorwaärts) 1 -> 2 -> 3 ist Ok. Nicht ok ist 1 -> 3 -> 2
        4. Die Bewegung innerhalb des virtual Wires ignoriert den isPathClear Bedingung
        */

        // 1. Alle Navigationsknoten vorbereiten (Regeln 1-4 Infrastruktur)
        std::vector<NavNode> navNodes = getExtendedNavNodes(env);

        // Ziel als NavNode hinzufügen (kein Wire)
        navNodes.push_back({goal, false, 0});

        std::vector<AStarNode> openList;
        std::vector<AStarNode> closedList;

        // Startknoten initialisieren
        // Wir prüfen nicht, ob Start auf Wire liegt, er wird als normaler Punkt behandelt
        openList.push_back({start, 0.0, GeometryUtils::calculateDistance(start, goal), -1, false, 0});

        while (!openList.empty())
        {
            // Knoten mit niedrigstem fCost finden
            size_t bestIdx = 0;
            for (size_t i = 1; i < openList.size(); ++i)
            {
                if (openList[i].fCost() < openList[bestIdx].fCost())
                    bestIdx = i;
            }

            AStarNode current = openList[bestIdx];

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

            openList.erase(openList.begin() + bestIdx);
            int currentInClosedIdx = static_cast<int>(closedList.size());
            closedList.push_back(current);

            // 2. Nachbarn prüfen
            for (const auto &nextNav : navNodes)
            {
                if (GeometryUtils::calculateDistance(current.pos, nextNav.pos) < 0.001)
                    continue;

                bool canMove = false;
                double weightMultiplier = 1.0;

                // LOGIK-CHECK FÜR VIRTUAL WIRE
                if (current.isWire && nextNav.isWire)
                {
                    // REGEL 3 & 4: Innerhalb des Drahtes nur Nachbarn (+/- 1 Index)
                    int idxDiff = std::abs(static_cast<int>(current.wireIdx) - static_cast<int>(nextNav.wireIdx));
                    if (idxDiff == 1)
                    {
                        canMove = true;
                        weightMultiplier = 0.1; // Hohe Priorität (Autobahn-Bonus)
                        // isPathClear wird hier ignoriert (Regel 4)
                    }
                }

                // REGEL 1 & 2: Normaler Weg oder Draht betreten/verlassen
                if (!canMove)
                {
                    if (isPathClear(current.pos, nextNav.pos, env))
                    {
                        canMove = true;
                        weightMultiplier = 1.0;
                    }
                }

                if (canMove)
                {
                    double dist = GeometryUtils::calculateDistance(current.pos, nextNav.pos);
                    double newGCost = current.gCost + (dist * weightMultiplier);

                    // Check Closed List
                    bool inClosed = false;
                    for (const auto &cNode : closedList)
                    {
                        if (GeometryUtils::calculateDistance(cNode.pos, nextNav.pos) < 0.001)
                        {
                            inClosed = true;
                            break;
                        }
                    }
                    if (inClosed)
                        continue;

                    // Check Open List
                    bool inOpen = false;
                    for (auto &oNode : openList)
                    {
                        if (GeometryUtils::calculateDistance(oNode.pos, nextNav.pos) < 0.001)
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
                        openList.push_back({nextNav.pos,
                                            newGCost,
                                            GeometryUtils::calculateDistance(nextNav.pos, goal),
                                            currentInClosedIdx,
                                            nextNav.isWire,
                                            nextNav.wireIdx});
                    }
                }
            }
        }
        return {}; // Kein Pfad gefunden
    }

}