#pragma once
#include "Environment.hpp"
#include <string>

namespace Planner
{
    class Visualizer
    {
    public:
        static void exportToSVG(const std::string &filename, const Environment &env, const LineString &path = {}, const std::vector<LineString> &debugLines = {}, const std::vector<LineString> &originalSlices = {});
    };
}