# Steiner Traversal Initialization

Steiner Traversal Initialization (STI) is an initial curve generation algorithm designed to accelerate geometric flow-based surface filling. STI can be seamlessly integrated into the [latest surface-filling curve generation algorithm](https://github.com/yutanoma/surface-filling-curve-flows).

First, add this repository as a `git submodule` and update your `CMakeLists.txt` accordingly. You can then use STI as shown in the **Simple Usage** section below. (See also [sample.cpp](https://github.com/mti-lab/SteinerTraversalInitialization/blob/main/src/sample.cpp))

## Simple usage

```cpp
#include "sti/generate_initial_curve.h"

int main(void) {
    std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> geometry;

    std::tie(mesh, geometry) = readManifoldSurfaceMesh("../models/surfaces/bunny/bunny_hr.obj");

    double radius = 0.02;

    std::vector<geometrycentral::surface::SurfacePoint> nodes;
    std::vector<bool> isFixedNode;
    std::vector<std::array<int, 2>> segments;

    std::tie(nodes, isFixedNode, segments) = generate_initial_curve(mesh, geometry, radius);
}
```

# How to run our sample code

```bash
git clone git@github.com:mti-lab/SteinerTraversalInitialization.git
cd SteinerTraversalInitialization
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j
./sti_example
```