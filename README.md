# CaSSAndRA Path Planner Core

---

## 🇬🇧 English

## Overview
![Path Planner Preview](docs/preview.png)

This project provides a high-performance path planning engine written in C++, specifically designed for autonomous robotic lawnmowers (inspired by the CaSSAndRA project).

It solves the **Complete Coverage Path Planning (CCPP)** problem, allowing a robot to efficiently cover a defined area while avoiding obstacles (exclusions).

### Key Features

- **Multiple Patterns**  
  Supports:
  - "Lines" (parallel zig-zag mowing)
  - "Rings" (concentric mowing)
  - "Squares"

- **Clipper2 Integration**  
  Uses the state-of-the-art Clipper2 library for robust polygon offsetting and clipping.

- **Python Binding**  
  Seamless integration into Python environments via pybind11.

- **GeoJSON Support**  
  Easily load map data from standard GeoJSON formats.

---

## Usage

### Standalone (C++)

To test the planner without Python, use the provided `main.cpp` debug script.  
It reads a `map.geojson` and calculates the path directly.

> **Note:**  
> When running in standalone mode, the planner automatically generates an SVG file named `test_map.svg` in the root directory for visual verification of the calculated path and geometry.

#### Compilation Example

```bash
g++ -O3 -std=c++20 -Iinclude -I/opt/homebrew/include main.cpp PathPlanner.cpp -o planner_test
./planner_test
```

---

### Python Integration

The core is designed to be compiled as a Python module (`planner_module`).

```python
import planner_module as pm

# Setup environment
peri = pm.Polygon([(0,0), (10,0), (10,10), (0,10)])
env = pm.Environment(peri)

# Configure settings
settings = pm.PathSettings()
settings.pattern = "lines"
settings.offset = 0.18

# Compute path
service = pm.PathService()
result = service.computeFullTask(env, settings, pm.Point(5, 5))
```

---

## 🇩🇪 Deutsch

## Überblick

Dieses Projekt bietet eine Hochleistungs-Pfadplanungs-Engine in C++, die speziell für autonome Rasenmäher-Roboter entwickelt wurde.

Es löst das Problem der vollständigen Flächenabdeckung  
(**Complete Coverage Path Planning – CCPP**) und ermöglicht es einem Roboter, definierte Flächen effizient zu mähen, während Hindernisse (Exclusions) umfahren werden.

### Hauptmerkmale

- **Mäh-Muster**
  - "Lines" (parallele Bahnen)
  - "Rings" (konzentrische Ringe)
  - "Squares"

- **Clipper2 Integration**  
  Nutzt die Clipper2-Bibliothek für robustes Polygon-Offsetting.

- **Python-Anbindung**  
  Nahtlose Integration in Python-Anwendungen mittels pybind11.

- **GeoJSON Support**  
  Lädt Kartendaten direkt aus dem standardisierten GeoJSON-Format.

---

## Benutzung

### Eigenständig (C++)

Um den Planner ohne Python zu testen, kann das `main.cpp` Debug-Script genutzt werden.  
Es liest eine `map.geojson` ein und berechnet den Pfad direkt.

> **Hinweis:**  
> Im Stand-alone-Modus erzeugt der Planner automatisch eine SVG-Datei namens `test_map.svg` im Stammverzeichnis zur visuellen Kontrolle des berechneten Pfads.

#### Beispiel-Kompilierung

```bash
g++ -O3 -std=c++20 -Iinclude -I/opt/homebrew/include main.cpp PathPlanner.cpp -o planner_test
./planner_test
```

---

### Python-Anbindung

Der Kern wird als Python-Modul (`planner_module`) kompiliert und eingebunden.

```python
import math
import planner_module as pm

# Umgebung erstellen
perimeter = pm.Polygon([(0,0), (20,0), (20,20), (0,20)])
env = pm.Environment(perimeter)

# Einstellungen festlegen
settings = pm.PathSettings()
settings.angle = math.radians(45)  # Rotation unterstützen
settings.offset = 0.18             # Messerbreite

# Pfad berechnen
service = pm.PathService()
start_pos = pm.Point(1, 1)
result = service.computeFullTask(env, settings, start_pos)
```

---

## Technical Requirements / Anforderungen

- C++20 Compiler (Clang, GCC)
- nlohmann-json (for GeoJSON support)
- pybind11 (for Python bindings)
- Clipper2 Library