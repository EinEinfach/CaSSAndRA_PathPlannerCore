#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PathService.hpp"
#include "Environment.hpp"
#include "Geometry.hpp"

namespace py = pybind11;
using namespace Planner;

PYBIND11_MODULE(planner_module, m)
{
    m.doc() = "C++ Path Planner Module für Python UI";

    // 1. Point
    py::class_<Point>(m, "Point")
        .def(py::init<double, double>())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y)
        .def("__repr__", [](const Point &p)
             { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; });

    // 2. LineString (Basis für Path und Polygon)
    py::class_<LineString>(m, "LineString")
        .def(py::init<>())
        .def("getPoints", &LineString::getPoints)
        .def("addPoint", &LineString::addPoint);

    // 3. Polygon
    py::class_<Polygon, LineString>(m, "Polygon")
        .def(py::init<>())
        .def(py::init<std::vector<Point>>());

    // 4. Environment
    py::class_<Environment>(m, "Environment")
        .def(py::init<Polygon>())
        .def("addObstacle", &Environment::addObstacle)
        .def("getPerimeter", &Environment::getPerimeter)
        .def("getObstacles", &Environment::getObstacles)
        .def("setVirtualWire", &Environment::setVirtualWire)
        .def("setDockingWire", &Environment::setDockingWire);

    // 5. PathSettings
    py::class_<PathSettings>(m, "PathSettings")
        .def(py::init<>())
        .def_readwrite("offset", &PathSettings::offset)
        .def_readwrite("angle", &PathSettings::angle)
        .def_readwrite("distanceToBorder", &PathSettings::distanceToBorder)
        .def_readwrite("mowArea", &PathSettings::mowArea)
        .def_readwrite("mowBorder", &PathSettings::mowBorder)
        .def_readwrite("borderLaps", &PathSettings::borderLaps)
        .def_readwrite("mowExclusionsBoder", &PathSettings::mowExclusionsBoder)
        .def_readwrite("exclusionsBorderLaps", &PathSettings::exclusionsBorderLaps)
        .def_readwrite("pattern", &PathSettings::pattern);

    // 6. PlanningResult
    py::class_<PathPlanner::PlanningResult>(m, "PlanningResult")
        .def_readwrite("path", &PathPlanner::PlanningResult::path) // Gibt LineString zurück
        .def_readwrite("slices", &PathPlanner::PlanningResult::slices);

    // 7. PathService
    py::class_<PathService>(m, "PathService")
        .def(py::init<>())
        .def("computeFullTask", &PathService::computeFullTask);
}