#pragma once

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_point.h"

#include "sti/face_center.h"
#include "sti/union_find.h"

#include <queue>
#include <set>

std::vector<std::vector<int>> approximate_minimum_steiner_tree(const std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& mesh, const std::unique_ptr<geometrycentral::surface::VertexPositionGeometry>& geometry, const std::vector<int>& terminals) {
    const int num_face = mesh->nFaces();

    std::vector<double> length(num_face, std::numeric_limits<double>::max());
    std::vector<int> parent(num_face, -1);
    std::vector<int> source(num_face, -1);

    std::priority_queue<std::tuple<double, int, int, int>, std::vector<std::tuple<double, int, int, int>>, std::greater<std::tuple<double, int, int, int>>> pq;

    for(const int v : terminals) {
        length[v] = 0;
        parent[v] = v;
        source[v] = v;
        pq.emplace(length[v], parent[v], source[v], v);
    }

    std::vector<int> finished(num_face);
    UnionFind uf(num_face);
    std::set<std::pair<int, int>> edges;
    while(!pq.empty()) {
        const auto [d, p, s, cur] = pq.top();
        pq.pop();
        if(finished[cur]) continue;
        finished[cur] = 1;
        const geometrycentral::surface::Face face = mesh->face(cur);
        for(const geometrycentral::surface::Face& adjF : face.adjacentFaces()) {
            const int nxt = adjF.getIndex();
            const double distance = (faceCenter(geometry, face) - faceCenter(geometry, adjF)).norm();
            if(source[nxt] == -1) {
                length[nxt] = d + distance;
                parent[nxt] = cur;
                source[nxt] = s;
                pq.emplace(length[nxt], parent[nxt], source[nxt], nxt);
            } else {
                if(!uf.same(source[cur], source[nxt])) {
                    uf.merge(source[cur], source[nxt]);
                    {
                        int current = cur, pre = p;
                        while(current != pre and edges.find(std::minmax(current, pre)) == edges.end()) {
                            edges.insert(std::minmax(current, pre));
                            current = pre;
                            pre = parent[pre];
                        }
                    }
                    {
                        int current = nxt, pre = parent[nxt];
                        while(current != pre and edges.find(std::minmax(current, pre)) == edges.end()) {
                            edges.insert(std::minmax(current, pre));
                            current = pre;
                            pre = parent[pre];
                        }
                    }
                    edges.insert(std::minmax(cur, nxt));
                }
            }
        }
    }

    std::vector<std::vector<int>> mst(num_face);
    for(const auto& edge : edges) {
        mst[edge.first].push_back(edge.second);
        mst[edge.second].push_back(edge.first);
    }
    return mst;
}