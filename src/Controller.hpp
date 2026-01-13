#pragma once
#include "Drone.hpp"
#include "Vec3.hpp"
#include <vector>

class SpatialHash;

struct PDParams {
    float kp = 2.0f;
    float kd = 1.5f;
    float aMax = 10.0f;
};

struct SepParams {
    float neighborR = 2.5f; 
    float avoidR = 1.0f;    
    float wSep = 6.0f;      
    float aMaxSep = 20.0f;  
};

struct ControlParams {
    PDParams pd;
    SepParams sep;
};

Vec3 PDFormation(
    const Drone& d,
    const Vec3& target,
    const PDParams& params
);

Vec3 separationAccel(
    int i,
    const std::vector<Drone>& drones,
    const SepParams& params
);

Vec3 separationAccelHashed(
    int i,
    const std::vector<Vec3>& positions,  
    const SepParams& params,
    const SpatialHash& grid,
    std::vector<int>& scratch
);