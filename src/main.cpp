#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <random>

#include "Sim.hpp"
#include "Formation.hpp"
#include "assignment/Assignment.hpp"
#include "Recorder.hpp"
#include "Mode.hpp"
#include "Metrics.hpp"
#include "Utils.hpp"



// Assign slots using Hungarian
static void assignSlots(Sim& sim, const std::vector<Vec3>& slotsWorld) {
    sim.setSlots(slotsWorld);

    std::vector<Vec3> pos;
    pos.reserve(sim.drones.size());
    for (auto& d : sim.drones) pos.push_back(d.pos);

    auto assignment = assignHungarianSlots(pos, sim.slotsWorld);
    for (int i = 0; i < (int)sim.drones.size(); ++i) {
        sim.drones[i].slotId = assignment[i];
        sim.drones[i].useTempTarget = false;
    }
}

static void logFrame(int frame, int mode, Sim& sim, Recorder& rec, std::ofstream& mout) {
    sim.step();
    rec.writeFrame(frame, sim);
    mout << frame << "," << mode << ","
         << avgFormationError(sim) << ","
         << minSeparation(sim) << "\n";
}

static void runAssignedPhase(
    int mode,
    int durationFrames,
    Sim& sim,
    const std::vector<Vec3>& slotsLocal,
    Recorder& rec,
    std::ofstream& mout,
    int& frame
) {
    Vec3 c = centroidOf(sim.drones);

    auto slots = slotsLocal;       
    for (auto& s : slots) s += c;     

    assignSlots(sim, slots);

    for (int i = 0; i < durationFrames; ++i, ++frame) {
        logFrame(frame, mode, sim, rec, mout);
    }
}

template <typename PerDroneFn>
static void runTempTargetPhase(
    int mode,
    int durationFrames,
    Sim& sim,
    PerDroneFn setTargets,
    Recorder& rec,
    std::ofstream& mout,
    int& frame
) {
    setTargets(sim);

    for (int i = 0; i < durationFrames; ++i, ++frame) {
        logFrame(frame, mode, sim, rec, mout);
    }
}


int main() {
    Sim sim;
    sim.pdParams.kp = 2.0f;
    sim.pdParams.kd = 1.5f;
    sim.pdParams.aMax = 10.0f;

    float N0 = 15.0f;
    int N = 30;

    float wSepBase = 8.0f; 
    sim.sepParams.wSep = wSepBase * std::sqrt((float)N / N0);

    float spacing = 1.8f;
    float circleR  = (N * spacing) / (2.0f * M_PI);
    float spawnR = 1.2f * circleR;  
    float regroupR = 1.2f * circleR;
    float disperseDist = 1.6f * circleR;
    
    auto v_local      = makeVShape(N, spacing, 35.0f);
    auto circle_local = makeCircle(N, circleR);
    auto ring_local   = makeCircle(N, regroupR);
    auto grid_local   = makeGrid(N, spacing);
    
    // Initialize drones 
    sim.drones = spawnRing(N, spawnR, 0.15f * spacing, 123);
    assignRolesByN(sim.drones);

    Recorder rec("out.csv");
    std::ofstream mout("metrics.csv");
    mout << "frame,mode,avg_err,min_sep\n";

    const int fps = 60;
    int frame = 0;

    // ---- Mode schedule (tweak durations later) ----
     int tFormV        = 2 * fps;
    int tDisperse     = 3 * fps;
    int tRegroup      = 2 * fps;
    int tFormCircle = 3 * fps;
    int tFormGrid     = 3 * fps;

    runAssignedPhase(0, tFormV, sim, v_local, rec, mout, frame);

    runTempTargetPhase(1, tDisperse, sim, [&](Sim& s){
        Vec3 c = centroidOf(s.drones);
        for (auto& d : s.drones) {
            Vec3 dir = (d.pos - c).normalized();
            if (dir.norm() < 1e-6f) dir = Vec3{1,0,0};

            float mult = 1.0f;
            if (d.role == Role::Scout) mult = 1.6f;
            if (d.role == Role::Relay) mult = 0.7f;

            d.tempTarget = c + dir * disperseDist * mult;
            d.useTempTarget = true;
        }
    }, rec, mout, frame);

    runTempTargetPhase(2, tRegroup, sim, [&](Sim& s){
        Vec3 c = centroidOf(s.drones);
        for (int i = 0; i < (int)s.drones.size(); ++i) {
            float rMult = 1.0f;
            if (s.drones[i].role == Role::Relay) rMult = 0.75f;
            if (s.drones[i].role == Role::Scout) rMult = 1.15f;

            s.drones[i].tempTarget = c + ring_local[i] * rMult;
            s.drones[i].useTempTarget = true;
        }
    }, rec, mout, frame);

    runAssignedPhase(3, tFormCircle, sim, circle_local, rec, mout, frame);
    runAssignedPhase(4, tFormGrid, sim, grid_local,   rec, mout, frame);
    runAssignedPhase(5, tFormV, sim, v_local,      rec, mout, frame);



    rec.close();
    mout.close();

    std::cout << "Wrote out.csv with " << frame << " frames\n";
    return 0;
}
