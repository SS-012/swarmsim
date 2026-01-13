#pragma once
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include "Vec3.hpp"

class SpatialHash {
public:
    explicit SpatialHash(float cs = 1.0f) : cellSize_(cs) {}

    void setCellSize(float cs) { cellSize_ = cs; }

    void build(const std::vector<Vec3>& positions) {
        for (auto& [k, v] : buckets_) v.clear();
        
        for (int i = 0; i < (int)positions.size(); ++i) {
            int cx, cy;
            cellOf(positions[i], cx, cy);
            buckets_[key(cx, cy)].push_back(i);
        }
    }

    void queryNeighbors(const Vec3& p, std::vector<int>& out) const {
        out.clear();
        int cx, cy;
        cellOf(p, cx, cy);

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                auto it = buckets_.find(key(cx + dx, cy + dy));
                if (it == buckets_.end()) continue;
                const auto& v = it->second;
                out.insert(out.end(), v.begin(), v.end());
            }
        }
    }

    void clear() { buckets_.clear(); }

private:
    float cellSize_ = 1.0f;
    std::unordered_map<int64_t, std::vector<int>> buckets_;

    static int64_t key(int cx, int cy) {
        return ((int64_t)cx << 32) ^ (uint32_t)cy;
    }

    inline void cellOf(const Vec3& p, int& cx, int& cy) const {
        cx = (int)std::floor(p.x / cellSize_);
        cy = (int)std::floor(p.y / cellSize_);
    }
};
