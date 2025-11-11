#pragma once

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/surface_point.h"
#include "geometrycentral/surface/poisson_disk_sampler.h"

#include "sti/face_center.h"

#include <set>

std::vector<int> place_terminals_poisson(const std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& mesh, const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const double radius, const double space) {
    geometrycentral::surface::PoissonDiskSampler poissonSampler(*mesh, *geometry);
    geometrycentral::surface::PoissonDiskOptions option;
    option.minDist = radius * space;
    const std::vector<geometrycentral::surface::SurfacePoint> samples = poissonSampler.sample(option);

    std::set<int> terminals;
    for(const auto& p : samples) {
        if(0 <= p.face.getIndex() and p.face.getIndex() < mesh->nFaces()) {
            terminals.insert(p.face.getIndex());
        }
    }

    const std::vector<int> res(terminals.begin(), terminals.end());
    return res;
}

std::vector<int> place_terminals_voxel(const std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& mesh, const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const double radius, const double space) {
    const double interval = space * radius;
    struct VoxelIndexHash {
        std::size_t operator()(const std::tuple<int, int, int>& k) const {
            std::size_t h1 = std::hash<int>{}(std::get<0>(k));
            std::size_t h2 = std::hash<int>{}(std::get<1>(k));
            std::size_t h3 = std::hash<int>{}(std::get<2>(k));
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
    std::unordered_map<std::tuple<int, int, int>, std::tuple<geometrycentral::Vector3, int, double>, VoxelIndexHash> mp;
    for(const geometrycentral::surface::Face& f : mesh->faces()) {
        const geometrycentral::Vector3 centroid = faceCenter(geometry, f);
        const std::tuple<int, int, int> index = {floor(centroid[0] / interval), floor(centroid[1] / interval), floor(centroid[2] / interval)};
        const geometrycentral::Vector3 center = {(std::get<0>(index) + 0.5) * interval, (std::get<1>(index) + 0.5) * interval, (std::get<2>(index) + 0.5) * interval};
        const double distance = (center - centroid).norm();
        if(mp.find(index) == mp.end()) {
            mp[index] = {centroid, f.getIndex(), distance};
        } else {
            if(distance < std::get<2>(mp[index])) {
                mp[index] = {centroid, f.getIndex(), distance};
            }
        }
    }
    std::vector<int> terminals;
    for(const auto& it : mp) {
        terminals.push_back(std::get<1>(it.second));
    }
    return terminals;
}

std::vector<int> place_terminals(const std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& mesh, const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const double radius, const double space = 4.0, const std::string& sampling_method = "Voxel") {
    assert(radius > 0);
    assert(space > 0);
    if(sampling_method == "Voxel") {
        return place_terminals_voxel(mesh, geometry, radius, space);
    } else if(sampling_method == "Poisson") {
        return place_terminals_poisson(mesh, geometry, radius, space);
    } else {
        assert(0);
    }
}