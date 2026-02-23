#include "PathPlanner.hpp"
#include "Geometry.hpp"
#include "clipper2/clipper.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace Planner
{
    bool PathPlanner::enableDebugLogs = false;

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

        // 1. Bounding Box
        auto [minY, maxY] = std::minmax_element(perimeterPoints.begin(), perimeterPoints.end(),
                                                [](const Point &a, const Point &b)
                                                { return a.y < b.y; });

        // Hilfsfunktion zum Sammeln aller X-Schnittpunkte für ein Polygon
        auto collectIntersections = [&](const Polygon &poly, double y, std::vector<double> &x_list)
        {
            const auto &pts = poly.getPoints();
            Point c = {-1000.0, y}, d = {1000.0, y};
            for (size_t i = 0; i < pts.size(); ++i)
            {
                Point p1 = pts[i], p2 = pts[(i + 1) % pts.size()];
                Point intersect;

                if (GeometryUtils::getIntersectionPoint(p1, p2, c, d, intersect))
                    x_list.push_back(intersect.x);

                if (GeometryUtils::getTouchPoint(p1, p2, c, d, intersect))
                    x_list.push_back(intersect.x);

                LineString iLine;
                if (GeometryUtils::getIntersectionLine(p1, p2, c, d, iLine))
                {
                    for (const auto &p : iLine.getPoints())
                        x_list.push_back(p.x);
                }
            }
        };

        // 2. Scanline Loop
        for (double y = minY->y + spacing; y < maxY->y; y += spacing)
        {
            std::vector<double> x_intersections;

            // Schnittpunkte von ALLEN Geometrien sammeln
            collectIntersections(env.getPerimeter(), y, x_intersections);
            for (const auto &obs : env.getObstacles())
                collectIntersections(obs, y, x_intersections);
            for (const auto &area : env.getMowAreas())
                collectIntersections(area, y, x_intersections);

            std::sort(x_intersections.begin(), x_intersections.end());
            x_intersections.erase(std::unique(x_intersections.begin(), x_intersections.end(),
                                              [](double a, double b)
                                              { return std::abs(a - b) < 1e-7; }),
                                  x_intersections.end());

            // 3. Segmente validieren
            for (size_t i = 0; i + 1 < x_intersections.size(); ++i)
            {
                double xStart = x_intersections[i];
                double xEnd = x_intersections[i + 1];
                if (std::abs(xEnd - xStart) < 1e-7)
                    continue;

                Point midP = {(xStart + xEnd) / 2.0, y};

                // Validierungskette
                bool ok = GeometryUtils::isPointCoveredByPolygon(midP, env.getPerimeter());

                if (ok)
                { // Check Obstacles
                    for (const auto &obs : env.getObstacles())
                    {
                        if (GeometryUtils::isPointCoveredByPolygon(midP, obs))
                        {
                            ok = false;
                            break;
                        }
                    }
                }

                if (ok && !env.getMowAreas().empty())
                { // Check MowAreas
                    bool inMow = false;
                    for (const auto &area : env.getMowAreas())
                    {
                        if (GeometryUtils::isPointCoveredByPolygon(midP, area))
                        {
                            inMow = true;
                            break;
                        }
                    }
                    ok = inMow;
                }

                if (ok)
                {
                    result.push_back(LineString({{xStart, y}, {xEnd, y}}));
                }
            }
        }
        return result;
    }

    std::vector<LineString> PathPlanner::generateRingSlices(const Environment &env, double spacing, int maxRings, double initialOffset)
    {
        // Fall A: Keine mowAreas -> Direkt die bewährte Logik auf der Original-Env
        if (env.getMowAreas().empty())
        {
            return generateRingSlicesInternal(env, env, spacing, maxRings, initialOffset);
        }

        // Fall B: MowAreas vorhanden
        std::vector<LineString> allSlices;

        for (const auto &area : env.getMowAreas())
        {
            // Wir bauen eine temporäre Environment für diese eine MowArea
            // Der neue Perimeter ist die MowArea selbst
            Environment tempEnv(area);

            // Die globalen Hindernisse übernehmen
            for (const auto &obs : env.getObstacles())
            {
                if (GeometryUtils::isPolygonCoveredByPolygon(obs, tempEnv.getPerimeter()) ||
                    GeometryUtils::isPolygonIntersectingPolygon(obs, tempEnv.getPerimeter()))
                {
                    tempEnv.addObstacle(obs);
                }
            }

            // Die bewährte Methode für diese Teil-Umgebung aufrufen
            std::vector<LineString> areaSlices = generateRingSlicesInternal(env, tempEnv, spacing, maxRings, initialOffset);

            // Ergebnisse sammeln
            allSlices.insert(allSlices.end(), areaSlices.begin(), areaSlices.end());
        }

        return allSlices;
    }

    std::vector<LineString> PathPlanner::generateRingSlicesInternal(const Environment &absBorder, const Environment &env, double spacing, int maxRings, double initialOffset)
    {
        using namespace Clipper2Lib;
        std::vector<LineString> result;
        const double scale = 1000.0;
        Paths64 currentLevel;

        // 1. Setup (Perimeter & Obstacles) - Wie bisher
        Path64 perimeterPath;
        for (const auto &p : env.getPerimeter().getPoints())
            perimeterPath.push_back(Point64(p.x * scale, p.y * scale));

        // perimeterPath = SimplifyPath(perimeterPath, 0.01 * scale);
        // if (!IsPositive(perimeterPath))
        //     std::reverse(perimeterPath.begin(), perimeterPath.end());
        currentLevel.push_back(perimeterPath);

        for (const auto &obs : env.getObstacles())
        {
            Path64 obstaclePath;
            for (const auto &p : obs.getPoints())
                obstaclePath.push_back(Point64(p.x * scale, p.y * scale));

            // obstaclePath = SimplifyPath(obstaclePath, 0.01 * scale);
            // if (IsPositive(obstaclePath))
            //     std::reverse(obstaclePath.begin(), obstaclePath.end());
            currentLevel.push_back(obstaclePath);
        }

        int currentRingCount = 0;

        // 2. Iteratives Schrumpfen
        while (!currentLevel.empty())
        {
            if (maxRings > 0 && currentRingCount >= maxRings)
                break;

            // LOGIK: Beim ersten Ring nutzen wir den initialOffset.
            // Wenn dieser 0 ist (Standard für Border-Laps), erhalten wir die Kontur.
            // Wenn dieser > 0 ist (für distanceToBorder), schrumpfen wir sofort.
            double currentDelta;
            if (currentRingCount == 0)
            {
                currentDelta = -initialOffset * scale;
            }
            else
            {
                currentDelta = -spacing * scale;
            }

            ClipperOffset co;
            co.MiterLimit(2.0);
            co.AddPaths(currentLevel, JoinType::Miter, EndType::Polygon);

            Paths64 nextLevel;
            co.Execute(currentDelta, nextLevel);

            // Sonderfall für Spacing 0: Clipper liefert oft identische Pfade zurück.
            // Um Endlosschleifen zu vermeiden, müssen wir sicherstellen, dass wir weitermachen.
            if (currentRingCount == 0 && nextLevel.empty() && !currentLevel.empty())
            {
                // Falls Execute(0) fehlschlägt (selten), nimm die bereinigte currentLevel
                nextLevel = currentLevel;
            }

            // --- FILTER & SPEICHERN ---
            for (const auto &path : nextLevel)
            {
                if (path.empty())
                    continue;

                LineString ring;
                for (const auto &pt : path)
                    ring.addPoint({(double)pt.x / scale, (double)pt.y / scale});

                // Ring schließen
                ring.addPoint({(double)path[0].x / scale, (double)path[0].y / scale});

                // Validierung gegen Hindernisse
                bool isInsideObstacle = false;
                Point firstPt = ring.getPoints()[0];
                for (const auto &obs : absBorder.getObstacles())
                {
                    if (GeometryUtils::isPointInsidePolygon(firstPt, obs))
                    {
                        isInsideObstacle = true;
                        break;
                    }
                }

                if (!isInsideObstacle)
                    result.push_back(ring);
            }

            // --- RETTUNG NUR WENN WIR WIRKLICH SCHRUMPFEN ---
            if (currentRingCount > 0)
            {
                Paths64 areasToRescue;
                if (nextLevel.empty())
                {
                    areasToRescue = currentLevel;
                }
                else
                {
                    areasToRescue = Difference(currentLevel, InflatePaths(nextLevel, spacing * scale, JoinType::Miter, EndType::Polygon), FillRule::EvenOdd);
                }

                if (!areasToRescue.empty())
                {
                    ClipperOffset rescueOffset;
                    rescueOffset.MiterLimit(2.0);
                    rescueOffset.AddPaths(areasToRescue, JoinType::Miter, EndType::Polygon);

                    Paths64 bonusLevel;
                    rescueOffset.Execute(-(spacing * Weights::FINAL_RING_SPACING_FACTOR) * scale, bonusLevel);
                    bool isInsideObstacle = false;
                    for (const auto &bonusPath : bonusLevel)
                    {
                        LineString bonusRing;
                        for (const auto &pt : bonusPath)
                            bonusRing.addPoint({(double)pt.x / scale, (double)pt.y / scale});
                        for (const auto &obs : absBorder.getObstacles())
                        {
                            if (GeometryUtils::isPointInsidePolygon(bonusRing.getPoints()[0], obs))
                            {
                                isInsideObstacle = true;
                            }
                        }
                        if (!isInsideObstacle)
                        {
                            result.push_back(bonusRing);
                        }
                    }

                    if (nextLevel.empty())
                        break;
                }
            }

            currentLevel = nextLevel;
            currentRingCount++;

            // Sicherheits-Check: Wenn wir bei Spacing 0 feststecken, erzwinge Fortschritt
            if (currentDelta == 0 && currentRingCount == 1)
            {
                // Nach dem 0-Durchgang müssen wir beim nächsten Mal schrumpfen
            }

            if (result.size() > 5000)
                break;
        }

        return result;
    }

    // std::vector<LineString> PathPlanner::generateRingSlicesInternal(const Environment &absBorder, const Environment &env, double spacing, int maxRings)
    // {
    //     using namespace Clipper2Lib;
    //     std::vector<LineString> result;
    //     const double scale = 1000.0;
    //     Paths64 currentLevel;

    //     // 1. Initiales Setup (Perimeter CCW, Obstacles CW)
    //     Path64 perimeterPath;
    //     for (const auto &p : env.getPerimeter().getPoints())
    //         perimeterPath.push_back(Point64(p.x * scale, p.y * scale));

    //     perimeterPath = SimplifyPath(perimeterPath, 0.01 * scale);
    //     if (!IsPositive(perimeterPath))
    //         std::reverse(perimeterPath.begin(), perimeterPath.end());
    //     currentLevel.push_back(perimeterPath);

    //     for (const auto &obs : env.getObstacles())
    //     {
    //         Path64 obstaclePath;
    //         for (const auto &p : obs.getPoints())
    //             obstaclePath.push_back(Point64(p.x * scale, p.y * scale));

    //         obstaclePath = SimplifyPath(obstaclePath, 0.01 * scale);
    //         if (IsPositive(obstaclePath))
    //             std::reverse(obstaclePath.begin(), obstaclePath.end());
    //         currentLevel.push_back(obstaclePath);
    //     }

    //     int currentRingCount = 0;

    //     // 2. Iteratives Schrumpfen
    //     while (!currentLevel.empty())
    //     {
    //         // Abbruch-Bedingung für die Anzahl der Ringe
    //         if (maxRings > 0 && currentRingCount >= maxRings)
    //             break;

    //         ClipperOffset co;
    //         co.MiterLimit(2.0);
    //         co.AddPaths(currentLevel, JoinType::Miter, EndType::Polygon);

    //         Paths64 nextLevel;
    //         co.Execute(-spacing * scale, nextLevel);

    //         // --- INTELLIGENTE RETTUNG ---
    //         Paths64 areasToRescue;
    //         if (nextLevel.empty())
    //         {
    //             areasToRescue = currentLevel;
    //         }
    //         else
    //         {
    //             areasToRescue = Difference(currentLevel, InflatePaths(nextLevel, spacing * scale, JoinType::Miter, EndType::Polygon), FillRule::EvenOdd);
    //         }

    //         if (!areasToRescue.empty())
    //         {
    //             ClipperOffset rescueOffset;
    //             rescueOffset.MiterLimit(2.0);
    //             rescueOffset.AddPaths(areasToRescue, JoinType::Miter, EndType::Polygon);

    //             Paths64 bonusLevel;
    //             rescueOffset.Execute(-(spacing * Weights::FINAL_RING_SPACING_FACTOR) * scale, bonusLevel);
    //             bool isInsideObstacle = false;
    //             for (const auto &bonusPath : bonusLevel)
    //             {
    //                 LineString bonusRing;
    //                 for (const auto &pt : bonusPath)
    //                     bonusRing.addPoint({(double)pt.x / scale, (double)pt.y / scale});
    //                 for (const auto &obs : absBorder.getObstacles())
    //                 {
    //                     if (GeometryUtils::isPointInsidePolygon(bonusRing.getPoints()[0], obs))
    //                     {
    //                         isInsideObstacle = true;
    //                     }
    //                 }
    //                 if (!isInsideObstacle)
    //                 {
    //                     result.push_back(bonusRing);
    //                 }
    //             }

    //             if (nextLevel.empty())
    //                 break;
    //         }

    //         // Normal gefundene Ringe zum Ergebnis
    //         for (const auto &path : nextLevel)
    //         {
    //             LineString ring;
    //             bool isInsideObstacle = false;
    //             for (const auto &pt : path)
    //             {
    //                 ring.addPoint({(double)pt.x / scale, (double)pt.y / scale});
    //             }
    //             for (const auto &obs : absBorder.getObstacles())
    //             {
    //                 if (GeometryUtils::isPointInsidePolygon(ring.getPoints()[0], obs))
    //                 {
    //                     isInsideObstacle = true;
    //                 }
    //             }
    //             if (!isInsideObstacle)
    //                 result.push_back(ring);
    //         }

    //         currentLevel = nextLevel;
    //         currentRingCount++; // Zähler erhöhen

    //         if (result.size() > 5000)
    //             break;
    //     }

    //     return result;
    // }

    std::vector<LineString> PathPlanner::filterRings(const std::vector<LineString> &rings, bool filterForObstacle)
    {
        using namespace Clipper2Lib;
        double scale = 1000.0;
        std::vector<LineString> filteredRings;

        for (const auto &r : rings)
        {
            if (r.getPoints().empty())
                continue;

            Path64 tempR;
            for (const auto &p : r.getPoints())
            {
                tempR.push_back(Point64(p.x * scale, p.y * scale));
            }

            // IsPositive gibt true zurück, wenn CCW (Perimeter)
            // IsPositive gibt false zurück, wenn CW (Obstacle/Loch)
            bool isPositive = IsPositive(tempR);

            if (filterForObstacle)
            {
                if (!isPositive)
                    filteredRings.push_back(r);
            }
            else
            {
                if (isPositive)
                    filteredRings.push_back(r);
            }
        }
        return filteredRings;
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
                addSliceToPath(result.path, slices[next.index], next);
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
                    if (bypass.empty())
                    {
                        // A* hat keinen Pfad gefunden!
                        // Wir überspringen dieses Slice komplett, um nicht durch Hindernisse zu fahren.
                        visited[next.index] = true;
                        std::stringstream ss;
                        ss << "WARN: No path found to slice " << next.index << ". Skipping.";
                        logDebug(ss.str()); 

                        // Wir setzen currentPos NICHT neu, wir bleiben wo wir sind und suchen das übernächste
                        continue;
                        // // Sicherheitsnetz: Wenn A* versagt, nimm die Luftlinie zum Ziel
                        // result.path.addPoint(goal);
                    }
                    else
                    {
                        for (size_t i = 1; i < bypass.size(); ++i)
                        {
                            result.path.addPoint(bypass[i]);
                        }

                        // Wenn es ein Polygon ist, schauen wir, wo A* uns wirklich abgesetzt hat.
                        // Das A*-Ende ist bypass.back().
                        if (next.isPolygon)
                        {
                            Point actualArrival = bypass.back();
                            double minD = 1e18;
                            size_t bestIdx = 0;
                            const auto &pts = slices[next.index].getPoints();

                            for (size_t i = 0; i < pts.size(); ++i)
                            {
                                double d = GeometryUtils::calculateDistance(actualArrival, pts[i]);
                                if (d < minD)
                                {
                                    minD = d;
                                    bestIdx = i;
                                }
                            }
                            // Wir überschreiben den Einstiegspunkt mit dem Punkt, an dem wir real angekommen sind
                            next.entryPointIdx = bestIdx;
                        }
                    }
                    visited[next.index] = true;
                    addSliceToPath(result.path, slices[next.index], next);
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
        double epsilon = Weights::EPSILON_DISTANCE;
        Point dir = {b.x - a.x, b.y - a.y};
        double len = std::sqrt(dir.x * dir.x + dir.y * dir.y);

        if (len < epsilon)
            return true;

        Point testA = {a.x + (dir.x / len) * epsilon, a.y + (dir.y / len) * epsilon};
        Point testB = {b.x - (dir.x / len) * epsilon, b.y - (dir.y / len) * epsilon};

        // Prüfe ob innerhalb des Perimeters
        auto &perimeter = env.getPerimeter();
        if (!GeometryUtils::isLineCoveredByPolygon(testA, testB, perimeter))
        {
            return false;
        }

        // Prüfe Hindernisse
        for (const auto &obs : env.getObstacles())
        {
            // 1. Wird die Kante echt geschnitten? (Der "Durchschuss")
            if (GeometryUtils::isLineIntersectingPolygon(testA, testB, obs))
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

    void PathPlanner::addSliceToPath(LineString &path, const LineString &slice, BestNextSegment best)
    {
        const auto &pts = slice.getPoints();
        if (pts.empty())
            return;
        if (best.isPolygon)
        {
            size_t n = pts.size();
            size_t startIdx = best.entryPointIdx;

            Point lastInPath = path.getPoints().back();
            Point entryPt = pts[best.entryPointIdx];

            // Wenn wir schon exakt auf dem Punkt stehen, fangen wir beim nächsten an
            size_t i = (GeometryUtils::calculateDistance(lastInPath, entryPt) < 0.001) ? 1 : 0;

            // Einmal komplett im Kreis laufen
            for (; i <= n; ++i)
            {
                // Modulo sorgt dafür, dass wir nach dem letzten Punkt wieder bei 0 landen
                size_t currentIdx = (startIdx + i) % n;
                path.addPoint(pts[currentIdx]);
            }

            // Den Kreis explizit schließen, indem wir den allerersten
            // angefahrenen Punkt (startIdx) nochmal hinzufügen
            // (Das passiert durch die Schleife bis <= n bereits automatisch)
        }
        else
        {
            if (best.reverse)
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

            // Polygon?
            if (pts.size() > 2)
            {
                for (size_t pIdx = 0; pIdx < pts.size(); ++pIdx)
                {
                    double d = GeometryUtils::calculateDistance(currentPos, pts[pIdx]);
                    if (d < best.distance && isPathClear(currentPos, pts[pIdx], env))
                    {
                        best = {(int)j, false, d, pIdx, true}; // true = isPolygon
                    }
                }
            }
            else
            {
                // Linie!
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
            // Polygon?
            if (pts.size() > 2)
            {
                for (size_t pIdx = 0; pIdx < pts.size(); ++pIdx)
                {
                    double d = GeometryUtils::calculateDistance(currentPos, pts[pIdx]);
                    if (d < best.distance)
                    {
                        best = {(int)j, false, d, pIdx, true}; // true = isPolygon
                    }
                }
            }
            else
            {
                // Linie!
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
                return {true, Weights::WIRE_COST_MULTIPLIER}; // Erlaubt ohne isPathClear, hoher Bonus
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
            hCost *= Weights::WIRE_HEURISTIC_BIAS;
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