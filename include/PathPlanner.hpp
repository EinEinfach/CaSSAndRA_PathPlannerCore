#pragma once
#include "Environment.hpp"
#include <vector>

namespace Planner
{
    class PathPlanner
    {
    public:
        struct PlanningResult {
            LineString path;
            std::vector<LineString> debugLines;
        };
        
        static std::vector<LineString> generateSlices(const Environment &env, double spacing);
        static PlanningResult connectSlices(const Environment &env, std::vector<LineString> &slices, Point startPos);
        static bool isPathClear(Point a, Point b, const Environment &env);

    private:
        struct BestNextSegment
        {
            int index = -1;
            bool reverse = false;
            double distance = 1e10;
        };

        static BestNextSegment findBestNext(Point currentPos, const std::vector<LineString> &slices, const std::vector<bool> &visited, const Environment &env);
        static BestNextSegment findBestNextFallback(Point currentPos, const std::vector<LineString> &slices, const std::vector<bool> &visited);
        static void addSliceToPath(LineString &path, const LineString &slice, bool reverse);
        static std::vector<Point> getNavigationNodes(const Environment& env);
        static std::vector<Point> findAStarPath(Point start, Point goal, const Environment& env);
    };
}