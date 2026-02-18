#pragma once
#include <vector>
#include <string>
#include "Environment.hpp"
#include "PathPlanner.hpp"

namespace Planner
{
    struct PathSettings
    {
        std::string pattern = "lines";
        double offset = 0.5;
        double angle = 0.0;
        double distanceToBorder = 1.5;
        bool mowArea = true;
        bool mowBorder = true;
        bool mowBorderCcw = false;
        size_t borderLaps = 3;
        bool mowExclusionsBoder = true;
        bool mowExclusionsBorderCcw = false;
        size_t exclusionsBorderLaps = 2;
    };

    class PathService
    {
    public:
        PathService() = default;
        static bool enableDebugLogs;

        // Die Hauptmethode: Verarbeitet die Env zu einem fertigen Pfad
        PathPlanner::PlanningResult computeFullTask(const Environment &rawEnv, const PathSettings &settings, const Point &startPos);

    private:
        // Interne Hilfsmethoden zur Strukturierung
        void rotateResult(PathPlanner::PlanningResult &planningResult, const PathSettings &settings);
    };
}