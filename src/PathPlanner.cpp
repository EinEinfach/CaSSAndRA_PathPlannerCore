#include "PathPlanner.hpp"
#include "Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace Planner
{
    bool PathPlanner::enableDebugLogs = true;

    void logDebug(const std::string &message)
    {
        if (PathPlanner::enableDebugLogs)
        {
            std::cout << "PathPlanner: " << message << std::endl;
        }
    }

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
            std::stringstream ss;
            ss << "--- Bin bei Pos (" << currentPos.x << "|" << currentPos.y << ") ---";
            logDebug(ss.str());

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

                    // DEBUG
                    // Hier loggen wir die Linie, die findBestNext als "nicht frei" abgelehnt hat
                    LineString failLine;
                    failLine.addPoint(currentPos);
                    failLine.addPoint(goal);
                    result.debugLines.push_back(failLine);
                    // END DEBUG

                    // Berechne Umweg um Hindernisse
                    std::vector<Point> bypass = findAStarPath(currentPos, goal, env, result.debugLinesSec);

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
            std::stringstream ss;
            ss << "Entscheidung: Gehe zu Slice #" << best.index
               << " (Dist: " << best.distance << ", Reverse: " << (best.reverse ? "Ja" : "Nein") << ")";
            logDebug(ss.str());
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

    //******************* A Star ****************************
    /*
        A* Regel für Virtual Wire
        1. Man darf an jedem beliebiger virtual Wire Koordinate virtual Wire betreten (unter der Berücksichtigung isPathClear)
        2. Man darf an jeder beliebigen virtual Wire Koordinate den virtual Wire verlassen (unter der Berücksichtigung isPathClear)
        3. Innerhalb des virtual Wire darf man die Punkte nur nacheinander Abfahren (rückwärts oder vorwaärts) 1 -> 2 -> 3 ist Ok. Nicht ok ist 1 -> 3 -> 2
        4. Die Bewegung innerhalb des virtual Wires ignoriert den isPathClear Bedingung
    */

    std::vector<Point> PathPlanner::findAStarPath(Point start, Point goal, const Environment &env, std::vector<LineString> &debugLines)
    {
        auto navNodes = prepareNavigationGraph(goal, env);

        std::vector<AStarNode> openList = {{start, 0.0, GeometryUtils::calculateDistance(start, goal), -1, false, 0}};
        std::vector<AStarNode> closedList;

        while (!openList.empty())
        {
            // 1. Nächsten besten Punkt wählen
            size_t bestIdx = findBestNodeIdx(openList);
            AStarNode current = openList[bestIdx];

            // 2. Ziel erreicht?
            if (GeometryUtils::calculateDistance(current.pos, goal) < 0.001)
            {
                std::vector<Point> rawPath = reconstructPath(current, closedList);
                return smoothPath(rawPath, env);
            }

            // 3. In das Logbuch (Closed List) übernehmen
            openList.erase(openList.begin() + bestIdx);
            int currentInClosedIdx = static_cast<int>(closedList.size());
            closedList.push_back(current);

            // 4. Umgebung erkunden
            for (const auto &nextNav : navNodes)
            {
                if (GeometryUtils::calculateDistance(current.pos, nextNav.pos) < 0.001)
                    continue;

                Movement move = checkMovementRules(current, nextNav, env);
                // DEBUG
                if (nextNav.isWire)
                {
                    std::stringstream ss;
                    ss << "[Debug] Draht-Check: Von (" << current.pos.x << "," << current.pos.y
                       << ") nach Draht-Index " << nextNav.wireIdx
                       << " | Erlaubt: " << (move.allowed ? "JA" : "NEIN")
                       << " | Multiplier: " << move.costMultiplier;
                    logDebug(ss.str());
                }
                // END DEBUG

                if (move.allowed)
                {
                    // DEBUG
                    LineString edge;
                    edge.addPoint(current.pos);
                    edge.addPoint(nextNav.pos);
                    debugLines.push_back(edge);
                    // END DEBUG
                    updateOpenList(current, nextNav, move.costMultiplier, goal, currentInClosedIdx, openList, closedList);
                }
            }
        }
        return {}; // Sackgasse
    }

    std::vector<PathPlanner::NavNode> PathPlanner::prepareNavigationGraph(Point goal, const Environment &env)
    {
        std::vector<PathPlanner::NavNode> nodes = getExtendedNavNodes(env);
        // Das Ziel wird als "normaler" Knoten ohne Wire-Funktion ergänzt
        nodes.push_back({goal, false, 0});
        return nodes;
    }

    size_t PathPlanner::findBestNodeIdx(const std::vector<AStarNode> &openList)
    {
        size_t bestIdx = 0;
        for (size_t i = 1; i < openList.size(); ++i)
        {
            if (openList[i].fCost() < openList[bestIdx].fCost())
                bestIdx = i;
        }
        return bestIdx;
    }

    PathPlanner::Movement PathPlanner::checkMovementRules(const AStarNode &current, const PathPlanner::NavNode &next, const Environment &env)
    {
        // REGEL 3 & 4: Auf der Autobahn (Virtual Wire)
        if (current.isWire && next.isWire)
        {
            int idxDiff = std::abs(static_cast<int>(current.wireIdx) - static_cast<int>(next.wireIdx));
            if (idxDiff == 1)
            {
                return {true, 0.1}; // Erlaubt ohne isPathClear, hoher Bonus
            }
        }

        // REGEL 1 & 2: Landstraße (Normales Gelände / Auffahrt / Abfahrt)
        if (isPathClear(current.pos, next.pos, env))
        {
            return {true, 1.0}; // Erlaubt, wenn Weg frei
        }

        return {false, 1.0}; // Blockiert
    }

    std::vector<Point> PathPlanner::reconstructPath(const PathPlanner::AStarNode &goalNode, const std::vector<PathPlanner::AStarNode> &closedList)
    {
        std::vector<Point> path;
        PathPlanner::AStarNode current = goalNode;
        path.push_back(current.pos);

        while (current.parentIdx != -1)
        {
            current = closedList[current.parentIdx];
            path.push_back(current.pos);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void PathPlanner::updateOpenList(const AStarNode &current, const NavNode &nextNav,
                                     double costMultiplier, Point goal, int currentInClosedIdx,
                                     std::vector<AStarNode> &openList, const std::vector<AStarNode> &closedList)
    {
        double dist = GeometryUtils::calculateDistance(current.pos, nextNav.pos);
        double newGCost = current.gCost + (dist * costMultiplier);
        double hCost = GeometryUtils::calculateDistance(nextNav.pos, goal);

        // Wenn es ein Drahtpunkt ist, geben wir einen "Motivations-Bonus" 
        // auf die Heuristik, damit der f-Wert sinkt.
        if (nextNav.isWire)
        {
            hCost *= 0.65;
            double fCost = newGCost + hCost;
            std::stringstream ss;
            ss << "   [A* Update] Draht-Punkt " << nextNav.wireIdx
               << " | g:" << newGCost << " (+" << (dist * costMultiplier) << ")"
               << " | h:" << hCost
               << " | f_TOTAL:" << fCost;
            logDebug(ss.str());
        }

        // Schon final besucht?
        for (const auto &cNode : closedList)
        {
            if (GeometryUtils::calculateDistance(cNode.pos, nextNav.pos) < 0.001)
                return;
        }

        // Schon in der Warteschlange?
        for (auto &oNode : openList)
        {
            if (GeometryUtils::calculateDistance(oNode.pos, nextNav.pos) < 0.001)
            {
                if (newGCost < oNode.gCost)
                {
                    oNode.gCost = newGCost;
                    oNode.hCost = hCost;
                    oNode.parentIdx = currentInClosedIdx;
                }
                return;
            }
        }

        // Ganz neuer Punkt!
        openList.push_back({nextNav.pos,
                            newGCost,
                            hCost,
                            currentInClosedIdx,
                            nextNav.isWire,
                            nextNav.wireIdx});
    }

    std::vector<Point> PathPlanner::smoothPath(const std::vector<Point> &path, const Environment &env)
    {
        if (path.size() < 3)
            return path; // Nichts zu glätten

        std::vector<Point> smoothed;
        smoothed.push_back(path.front()); // Startpunkt immer behalten

        size_t current = 0;
        while (current < path.size() - 1)
        {
            size_t furthestVisible = current + 1;

            // Wir schauen vom aktuellen Punkt so weit wie möglich nach vorne
            for (size_t next = current + 2; next < path.size(); ++next)
            {
                if (isPathClear(path[current], path[next], env))
                {
                    furthestVisible = next;
                }
                else
                {
                    // Sobald ein Hindernis die Sicht blockiert,
                    // nehmen wir den letzten freien Punkt
                    break;
                }
            }

            smoothed.push_back(path[furthestVisible]);
            current = furthestVisible;
        }

        return smoothed;
    }
}