#pragma once
#include "Environment.hpp"
#include <vector>

namespace Planner
{
    class PathPlanner
    {
    public:
        static std::vector<LineString> generateSlices(const Environment &env, double spacing);
        static LineString connectSlices(const Environment& env, std::vector<LineString>& slices);
        static bool isPathClear(Point a, Point b, const Environment& env);
    };
}