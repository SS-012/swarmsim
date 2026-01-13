#pragma once
#include "Vec3.hpp"
#include "Role.hpp"

struct Drone {
    int id = -1;
    Vec3 pos{0,0,0};
    Vec3 vel{0,0,0}; 
    int slotId = -1;

    Vec3 tempTarget{0,0,0};
    bool useTempTarget = false;
    Role role = Role::Anchor;
};

