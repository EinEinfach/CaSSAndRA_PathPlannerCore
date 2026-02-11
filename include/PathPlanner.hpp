#pragma once
#include "Environment.hpp"
#include <vector>

namespace Planner
{
    class PathPlanner
    {
    public:
        static std::vector<LineString> generateSlices(const Environment &env, double spacing);
    };
}