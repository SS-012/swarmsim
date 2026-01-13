#include "Controller.hpp"
#include "SpatialHash.hpp"
#include "Utils.hpp"
#include <vector>


Vec3 PDFormation(
    const Drone& d,
    const Vec3& target,
    const PDParams& params
) {
    Vec3 ePos = target - d.pos;
    Vec3 eVel = Vec3{} - d.vel;

    Vec3 a = ePos * params.kp + eVel * params.kd;
    return clampNorm(a, params.aMax);
}

Vec3 separationAccel(
    int i,
    const std::vector<Drone>& drones,
    const SepParams& params
) {
    const Vec3& pi = drones[i].pos;
    Vec3 acc{0,0,0};

    for (int j = 0; j < (int)drones.size(); ++j) {
        if (j == i) continue;
        Vec3 d = pi - drones[j].pos;
        float r = d.norm();
        if (r < 1e-6f) continue;

        if (r > params.neighborR) continue;

        float strength = 0.0f;
        if (r < params.avoidR) {
            strength = (params.avoidR - r) / params.avoidR;
            strength = strength * strength * 5.0f;
        } else {
            float t = (params.neighborR - r) / (params.neighborR - params.avoidR);
            strength = t * t;
        }

        acc += d.normalized() * strength;
    }

    acc *= params.wSep;
    return clampNorm(acc, params.aMaxSep);
}

Vec3 separationAccelHashed(
    int i,
    const std::vector<Vec3>& positions,
    const SepParams& params,
    const SpatialHash& grid,
    std::vector<int>& scratch
) {
    const Vec3& pi = positions[i];
    Vec3 acc{0,0,0};

    grid.queryNeighbors(pi, scratch);

    for (int j : scratch) {
        if (j == i) continue;

        Vec3 d = pi - positions[j];
        float r = d.norm();
        if (r < 1e-6f) continue;

        if (r > params.neighborR) continue;

        float strength = 0.0f;
        if (r < params.avoidR) {
            strength = (params.avoidR - r) / params.avoidR;
            strength = strength * strength * 5.0f;
        } else {
            float t = (params.neighborR - r) / (params.neighborR - params.avoidR);
            strength = t * t;
        }

        acc += d.normalized() * strength;
    }

    acc *= params.wSep;
    return clampNorm(acc, params.aMaxSep);
}
