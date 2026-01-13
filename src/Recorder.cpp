#include "Recorder.hpp" 

Recorder::Recorder(const std::string& path) : out_(path) {
    out_ << "frame, drone_id, role, x, y, z\n";
}

void Recorder::writeFrame(int frame, const Sim& sim) {
    for (const auto& d : sim.drones) {
        out_ << frame << ", " << d.id << ", " << (int)d.role << ", " 
            << d.pos.x << ", " << d.pos.y << ", " << d.pos.z << "\n";
    }
}

void Recorder::close() {
    if (out_.is_open()) out_.close();
}
