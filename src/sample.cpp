#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/surface_point.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include "sti/generate_initial_curve.h"

geometrycentral::Vector3 surface_point_to_cartesian(const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const geometrycentral::surface::SurfacePoint& sp) {
    auto faceCoords = sp.faceCoords;
    auto v0 = geometry->vertexPositions[sp.face.halfedge().vertex()];
    auto v1 = geometry->vertexPositions[sp.face.halfedge().next().vertex()];
    auto v2 = geometry->vertexPositions[sp.face.halfedge().next().next().vertex()];
    return (faceCoords.x * v0) + (faceCoords.y * v1) + (faceCoords.z * v2);
}

int main() {
    using namespace geometrycentral;
    using namespace geometrycentral::surface;

    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;

    std::tie(mesh, geometry) = readManifoldSurfaceMesh("../models/surfaces/bunny/bunny_hr.obj");

    polyscope::init();
    polyscope::registerSurfaceMesh("mesh", geometry->inputVertexPositions, mesh->getFaceVertexList());

    std::vector<SurfacePoint> nodes;
    std::vector<bool> isFixedNode;
    std::vector<std::array<int, 2>> segments;

    std::tie(nodes, isFixedNode, segments) = generate_initial_curve(mesh, geometry, 0.02);

    std::vector<Vector3> nodes_coords;
    for(const auto& node : nodes) {
        nodes_coords.emplace_back(surface_point_to_cartesian(geometry, node));
    }

    polyscope::registerPointCloud("nodes", nodes_coords);
    polyscope::registerCurveNetwork("edges", nodes_coords, segments);

    polyscope::show();
}