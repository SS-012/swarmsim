#pragma once
#include <fstream>
#include <string> 
#include "Sim.hpp"

class Recorder { 
    public:
        explicit Recorder(const std::string& path);
        void writeFrame(int frame, const Sim& sim);
        void close(); 

    private:
        std::ofstream out_; 
};
