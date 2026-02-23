#pragma once
#include <fstream>
#include <nlohmann/json.hpp>
#include "Environment.hpp"

namespace Planner
{
    class GeoJSONReader
    {
        public:
            static Environment loadEnvironmentFromFile(const std::string &filename);
    };
}