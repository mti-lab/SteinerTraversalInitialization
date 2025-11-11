#pragma once

#include "geometrycentral/surface/meshio.h"

geometrycentral::Vector3 faceCenter(const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const geometrycentral::surface::Face& f) {
    geometrycentral::Vector3 center = geometrycentral::Vector3::zero();
    int cnt = 0;
    for(const geometrycentral::surface::Vertex& v : f.adjacentVertices()) {
        center += geometry->inputVertexPositions[v];
        cnt++;
    }
    return center / cnt;
}