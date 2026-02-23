#include "PathService.hpp"

namespace Planner
{
    std::string PathService::getVersion() {
        return version;
    }

    PathPlanner::PlanningResult PathService::computeFullTask(const Environment &rawEnv, const PathSettings &settings, const Point &startPos)
    {
        // Kopie der Env für Bearbeitung (Rotation)
        Environment workEnv = rawEnv;

        workEnv.rotate(-settings.angle);
        std::vector<LineString> allSlices; // Hier packen wir alles was abgefahren sein muss

        // Teil 1 Bearbeite die mow area
        if (settings.mowArea)
        {
            // Zerlegung in Teil-Inseln (Shrinking-Logik)
            std::vector<Environment> islands;
            if (settings.distanceToBorder > 0.0)
            {
                auto rings = PathPlanner::generateRingSlices(workEnv, settings.distanceToBorder, 1, settings.distanceToBorder);
                auto perimeters = PathPlanner::filterRings(rings, false);
                auto obstacles = PathPlanner::filterRings(rings, true);

                for (auto &p : perimeters)
                {
                    Environment island(Polygon(p.getPoints()));
                    for (auto &o : obstacles)
                    {
                        island.addObstacle(Polygon(o.getPoints()));
                    }
                    islands.push_back(island);
                }
            }
            else
            {
                islands.push_back(workEnv);
            }

            // 3. Slices generieren (Insel für Insel)
            for (auto &island : islands)
            {
                std::vector<LineString> islandSlices;
                if (settings.pattern == "lines")
                {
                    islandSlices = PathPlanner::generateSlices(island, settings.offset);
                }
                else if (settings.pattern == "squares")
                {
                    islandSlices = PathPlanner::generateSlices(island, settings.offset);
                    island.rotate(M_PI / 2);
                    auto islandSlices90 = PathPlanner::generateSlices(island, settings.offset);
                    island.rotate(-M_PI / 2);
                    for (auto &l : islandSlices90)
                    {
                        l.rotate(-M_PI / 2);
                    }
                    islandSlices.insert(islandSlices.end(), islandSlices90.begin(), islandSlices90.end());
                }
                else
                {
                    islandSlices = PathPlanner::generateRingSlices(island, settings.offset);
                }
                allSlices.insert(allSlices.end(), islandSlices.begin(), islandSlices.end());
            }
        }

        // Teil 2 Bearbeite perimeter border
        if (settings.mowBorder && settings.borderLaps > 0)
        {
            // Wir fordern borderLaps an.
            // Intern wird der erste Ring mit Spacing 0 erzeugt (= Perimeter/Obstacle-Kontur)
            auto borderSlices = PathPlanner::filterRings(
                PathPlanner::generateRingSlices(workEnv, settings.offset, settings.borderLaps),
                false);

            for (auto &s : borderSlices)
            {
                allSlices.push_back(s);
            }
        }

        // Teil 3 Bearbeite exclusions
        if (settings.mowExclusionsBoder && settings.exclusionsBorderLaps > 0)
        {
            // for (auto &obs : workEnv.getObstacles())
            // {
            //     allSlices.push_back(obs);
            // }
            if (settings.mowExclusionsBoder && settings.exclusionsBorderLaps > 0)
            {
                auto exclusionsSlices = PathPlanner::filterRings(PathPlanner::generateRingSlices(workEnv, settings.offset, settings.exclusionsBorderLaps), true);
                for (auto &s : exclusionsSlices)
                {
                    allSlices.push_back(s);
                }
            }
        }

        // Teil 4 Verbinde alle Wege
        PathPlanner::PlanningResult planningResult = PathPlanner::connectSlices(workEnv, allSlices, startPos);

        // Teil 5 Drehe alles zurück
        planningResult.slices = allSlices;
        rotateResult(planningResult, settings);
        return planningResult;
    }

    void PathService::rotateResult(PathPlanner::PlanningResult &planningResult, const PathSettings &settings)
    {
        planningResult.path.rotate(settings.angle);
        for (auto &l : planningResult.debugLines)
        {
            l.rotate(settings.angle);
        }
        for (auto &l : planningResult.debugLinesSec)
        {
            l.rotate(settings.angle);
        }
        for (auto &l : planningResult.slices)
        {
            l.rotate(settings.angle);
        }
    }

}