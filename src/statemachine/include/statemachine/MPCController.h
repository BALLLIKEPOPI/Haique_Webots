#pragma once
#include <iostream>
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
    
    virtual void updatex0(double x_, double y_, double z_,
                            double vx_, double vy_, double vz_,
                            double phi_, double theta_, double psi_, 
                            double p_, double q_, double r_) = 0;

    // virtual void updatePara(vector<double> torque, vector<double> force) = 0;

    virtual vector<double> getFirstCon() = 0;

    vector<double> step(const std::vector<double>& current_state,
                             const std::vector<double>& reference) {
        setupProblem();
        return solve();
    }

    vector<double> step(bool isModeChanged){
        if(isModeChanged) {
            cout << "Mode changed, resetting problem..." << endl;
            // Reset the problem if the mode has changed
            setupProblem();
        }
        return solve();
    }

    void updatePara(vector<double> drag_t, vector<double> drag_f) {
        // Update the parameters vector with the current state
        para.clear();
        para.insert(para.end(), x0.begin(), x0.end());
        para.insert(para.end(), xs.begin(), xs.end());
        para.insert(para.end(), drag_t.begin(), drag_t.end());
        para.insert(para.end(), drag_f.begin(), drag_f.end());
    }

protected:
    float h = 0.03; // step[s]
    int N = 15; // prediction horizon
    // float I1 = 0.103; float I2 = 0.104; float I3 = 0.161; 
    float I1 = 0.102761; float I2 = 0.104523; float I3 = 0.161378; 
    float m = 4.8; // kg
    float V = 0.00285; // m3
    float rou = 1000; // kg/m3
    float G = 9.8; // m/s2
    float l = 0.6; // m
    float c1 = 0.01; float c2 = 0.01; float c3 = 0.01;
    float k = 0.3; // 推力系数
    float c = 0.03; // 反扭系数

    vector<double> x0;
    vector<double> xs;
    vector<double> para;
    vector<double> lbx;
    vector<double> ubx;
    // Nonlinear bounds
    vector<double> lbg;
    vector<double> ubg;
    map<std::string, DM> arg, res;
    Function solver;
};
