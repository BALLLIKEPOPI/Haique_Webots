#pragma once
#include "MPCController.h"
#include <iostream>

class HoverMPC : public MPCController {
public:
    HoverMPC();

    SX Dynamics(const SX state, const SX con, SX drag) override;

    void setupProblem() override;

    void updatex0(double x_, double y_, double z_,
                    double vx_, double vy_, double vz_,
                    double phi_, double theta_, double psi_, 
                    double p_, double q_, double r_) override {
        x0 = {x_, y_, z_, vx_, vy_, vz_,
            phi_, theta_, psi_, p_, q_, r_};
    }
    
    // void updatePara(vector<double> drag_t, vector<double> drag_f) override;

    vector<double> getFirstCon() override;

    std::vector<double> solve() override;

private:
    Slice all;
    int horizon_;
    double dt_;

    int n_state = 12; // [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    int n_control = 10; // [f1, f2, f3, f4, f5, f6, f7, f8, alpha, beta]
    int n_drag = 6; // [drag_tx, drag_ty, drag_tz, drag_fx, drag_fy, drag_fz]
    
    SX Q_hover = SX::zeros(n_state, n_state); // weighing matrices (states)
    SX R_hover = SX::zeros(n_control, n_control); // weighing matrices (controls)
    SX st = SX::sym("st", n_state); // initial state
    SX st_next = SX::sym("st_next", n_state);
    SX con = SX::sym("con", n_control);
    SX drag = SX::sym("drag", n_drag);
    SX OPT_variables = SX::sym("OPT_variables", n_state*(N+1)+n_control*N);
    
    SX U = SX::sym("U", n_control, N); // Decision variables (controls)
    // parameters (which include the initial state and the reference state)
    SX P = SX::sym("P", 2*n_state+n_drag); 
    // A vector that represents the states over the optimization problem.
    SX X = SX::sym("X", n_state, N+1);
    // initialization of the states decision variables
    SX X0 = SX::sym("X0", n_state, N+1);
    // 3 control inputs for each robot
    SX u0 = SX::zeros(N, n_control);

    SX obj = 0; // objective function
    SX g = SX::sym("g", n_state*(N+1)); // constraints vector

    // RK4
    SX k1 = SX::sym("k1");
    SX k2 = SX::sym("k2");
    SX k3 = SX::sym("k3");
    SX k4 = SX::sym("k4");
    SX st_next_RK4 = SX::sym("st_next_RK4", n_state);

    // Initial guess and bounds for the optimization variables
    // vector<double> x0;
    // vector<double> xs;
    // vector<double> para;
    vector<double> state_upper_bound;
    vector<double> state_lower_bound;
    vector<double> con_upper_bound;
    vector<double> con_lower_bound;


    // output
    vector<double> u_f;
};