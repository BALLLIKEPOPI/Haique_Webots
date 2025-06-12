#pragma once
#include <vector>
#include "casadi/casadi.hpp"

using namespace std;
using namespace casadi;

class MPCController {
public:
    virtual ~MPCController() {}

    virtual SX Dynamics(const SX state, const SX con, SX drag) = 0;
    
    virtual void setupProblem() = 0;
    virtual vector<double> solve() = 0;

    virtual void updatePara(vector<double> torque, vector<double> force) = 0;

    virtual vector<double> getFirstCon() = 0;

    std::vector<double> step(const std::vector<double>& current_state,
                             const std::vector<double>& reference) {
        setupProblem();
        return solve();
    }

    float h = 0.15; // step[s]
    int N = 25; // prediction horizon
    // float I1 = 0.103; float I2 = 0.104; float I3 = 0.161; 
    float I1 = 1; float I2 = 1; float I3 = 1; 
    float m = 4.8; // kg
    float V = 0.00285; // m3
    float rou = 1000; // kg/m3
    float G = 9.8; // m/s2
    float l = 0.6; // m
    float c1 = 0.01; float c2 = 0.01; float c3 = 0.01;
    float k = 0.3; // 推力系数
    float c = 0.03; // 反扭系数
};
