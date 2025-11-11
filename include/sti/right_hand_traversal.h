#pragma once

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_point.h"

std::tuple<std::vector<geometrycentral::surface::SurfacePoint>, std::vector<bool>, std::vector<std::array<int, 2>>> right_hand_traversal(const std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& mesh, const std::vector<std::vector<int>>& mst) {
    std::vector<geometrycentral::surface::SurfacePoint> nodes;
    std::vector<bool> isFixedNode;
    std::vector<std::array<int, 2>> segments;

    static std::vector<bool> visited(mesh->nFaces(), false);

    auto dfs = [&](auto& dfs, int cur, int p) -> void {
        static constexpr geometrycentral::Vector3 barycentricCenter = geometrycentral::Vector3{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
        static constexpr geometrycentral::Vector3 barycentric[3] = {geometrycentral::Vector3{0.6, 0.2, 0.2}, geometrycentral::Vector3{0.2, 0.6, 0.2}, geometrycentral::Vector3{0.2, 0.2, 0.6}};
        const geometrycentral::surface::Face face_cur = mesh->face(cur);
        visited[cur] = true;
        if(p == -1) {
            const auto sp = geometrycentral::surface::SurfacePoint(mesh->face(cur), barycentricCenter);
            nodes.emplace_back(sp);
            dfs(dfs, mst[cur][0], cur);
            return;
        }
        geometrycentral::surface::Halfedge he = face_cur.halfedge();
        int idx = 0;
        while((int)he.twin().face().getIndex() != p) {
            he = he.next();
            idx = (idx + 1) % 3;
        }
        he = he.next();
        idx = (idx + 1) % 3;
        auto sp = geometrycentral::surface::SurfacePoint(mesh->face(cur), barycentric[idx]);
        nodes.emplace_back(sp);
        segments.emplace_back(std::array<int, 2>{(int)nodes.size() - 2, (int)nodes.size() - 1});
        while((int)he.twin().face().getIndex() != p) {
            for(int i = 0; i < (int)mst[cur].size(); ++i) {
                if(mst[cur][i] == (int)he.twin().face().getIndex()) {
                    dfs(dfs, mst[cur][i], cur);
                }
            }
            he = he.next();
            idx = (idx + 1) % 3;
            sp = geometrycentral::surface::SurfacePoint(mesh->face(cur), barycentric[idx]);
            nodes.emplace_back(sp);
            segments.emplace_back(std::array<int, 2>{(int)nodes.size() - 2, (int)nodes.size() - 1});
        }
    };

    for(int i = 0; i < (int)mesh->nFaces(); ++i) {
        if((int)mst[i].size() == 1 and !visited[i]) {
            const int sz = nodes.size();
            dfs(dfs, i, -1);
            segments.emplace_back(std::array<int, 2>{(int)nodes.size() - 1, sz});
        }
    }

    isFixedNode.resize(nodes.size(), false);

    return {nodes, isFixedNode, segments};
}