#pragma once 
#include <vector>
#include "Drone.hpp"
#include "Vec3.hpp"
#include "Controller.hpp"
#include "SpatialHash.hpp"

struct SimParams {
    float dt = 1.0f / 60.0f; 
    float vMax = 6.0f;
    PDParams pdParams;
    SepParams sepParams;
};

class Sim { 
public:
    explicit Sim(const SimParams& params = {}) : params_(params) {
        scratch_.reserve(64);
    } 

    std::vector<Drone> drones;
    std::vector<Vec3> slotsWorld;

    PDParams pdParams; 
    SepParams sepParams;

    void step(); 

    void setSlots(std::vector<Vec3> slots) { 
        slotsWorld = std::move(slots);
    }



private:
    SimParams params_;
    std::vector<Vec3> posCache_;   
    SpatialHash grid_;             
    std::vector<int> scratch_;

    void ensureBuffers();
};