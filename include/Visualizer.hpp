#pragma once
#include "Enviroment.hpp"
#include <string>

namespace Planner
{
    class Visualizer
    {
        public:
        static void exportToSVG(const std::string &filename, const Enviroment &env, const LineString &path = {});
    };
}