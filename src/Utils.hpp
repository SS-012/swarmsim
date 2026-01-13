#pragma once
#include "Vec3.hpp"
#include <algorithm>
#include <random>
#include "Drone.hpp"
#include "Role.hpp"

inline float squaredDist(const Vec3& a, const Vec3& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
}

inline Vec3 clampNorm(const Vec3& v, float maxNorm) {
    float norm = v.norm();
    if (norm < 1e-6f || norm <= maxNorm) return v;
    return v * (maxNorm / norm);
}

inline Vec3 clampVel(const Vec3& v, float vMax) {
    float n = v.norm();
    if (n < 1e-6f || n <= vMax) return v;
    return v * (vMax / n);
}

inline Vec3 centroidOf(const std::vector<Drone>& drones) {
    Vec3 c{0,0,0};
    for (const auto& d : drones) c += d.pos;
    return c / (float)drones.size();
}

inline std::vector<Drone> spawnRing(int N, float radius, float jitter, unsigned seed=123) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> unif(-jitter, jitter);

    std::vector<Drone> drones;
    drones.reserve(N);

    for (int i = 0; i < N; ++i) {
        float a = 2.0f * (float)M_PI * (float)i / (float)N;
        float x = radius * std::cos(a) + unif(rng);
        float y = radius * std::sin(a) + unif(rng);
        drones.push_back(Drone{ i, Vec3{x,y,0}, Vec3{0,0,0}, -1 });
    }
    return drones;
}

inline void assignRolesByN(std::vector<Drone>& drones) {
    int N = (int)drones.size();
    for (auto& d : drones) d.role = Role::Anchor;

    int numRelays = std::max(2, N / 10);
    int numScouts = std::max(2, N / 10);

    std::vector<int> idx(N);
    for (int i = 0; i < N; ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](int a, int b){
        return drones[a].id < drones[b].id;
    });

    for (int k = 0; k < numRelays && k < N; ++k) drones[idx[k]].role = Role::Relay;
    for (int k = numRelays; k < numRelays + numScouts && k < N; ++k) drones[idx[k]].role = Role::Scout;
}
