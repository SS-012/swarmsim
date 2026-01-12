struct Capability {
    float speed;
    float sensing;
    float commRange;
    float endurance;
}

enum class Role {
    Anchor,
    Surveyor,
    Relay,
};

struct Drone {
    int id;
    Vec3 pos, vel;
    Role role;
    Capability cap;
    int slotIdl;
    int groupId;
}