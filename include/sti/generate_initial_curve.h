#pragma once

#include "sti/place_terminals.h"
#include "sti/approximate_minimum_steiner_tree.h"
#include "sti/right_hand_traversal.h"

std::tuple<std::vector<geometrycentral::surface::SurfacePoint>, std::vector<bool>, std::vector<std::array<int, 2>>> generate_initial_curve(const std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& mesh, const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const double radius, const double space = 4.0, const std::string& sampling_method = "Voxel") {
    std::vector<geometrycentral::surface::SurfacePoint> nodes;
    std::vector<bool> isFixedNode;
    std::vector<std::array<int, 2>> segments;

    const std::vector<int> terminals = place_terminals(mesh, geometry, radius, space, sampling_method);

    const std::vector<std::vector<int>> mst = approximate_minimum_steiner_tree(mesh, geometry, terminals);

    std::tie(nodes, isFixedNode, segments) = right_hand_traversal(mesh, mst);

    return {nodes, isFixedNode, segments};
}