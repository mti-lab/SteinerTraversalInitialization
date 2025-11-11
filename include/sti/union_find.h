#pragma once

#include <cassert>
#include <vector>

struct UnionFind {
    UnionFind(const int N)
        : n(N), data(N, -1) {}
    int merge(const int a, const int b) {
        assert(0 <= a and a < n);
        assert(0 <= b and b < n);
        int x = leader(a), y = leader(b);
        if(x == y) return x;
        if(-data[x] < -data[y]) std::swap(x, y);
        data[x] += data[y];
        data[y] = x;
        return x;
    }
    bool same(const int a, const int b) {
        assert(0 <= a and a < n);
        assert(0 <= b and b < n);
        return leader(a) == leader(b);
    }
    int leader(const int a) {
        assert(0 <= a and a < n);
        if(data[a] < 0) return a;
        return data[a] = leader(data[a]);
    }

   private:
    int n;
    std::vector<int> data;
};