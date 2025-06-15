#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

#include "casadi/casadi.hpp"
#include "attitudectl/controlPub.h"
#include "eigen3/Eigen/Eigen"
#include <vector>
#include <ros/ros.h>
#include "ros/publisher.h"
#include "MPCController.h"
#include "HoverMPC.h"

using namespace std;
using namespace casadi;

class StateMachine
{
    public:
        StateMachine();
        ~StateMachine();
        void setMode(const string& mode);
        string getMode() const;
        bool isModeChanged() const;
        vector<double> controlStep(const vector<double>& state,
                                    const vector<double>& ref);
        vector<double> controlStep();
        void updatex0(double x_, double y_, double z_,
                        double vx_, double vy_, double vz_,
                        double phi_, double theta_, double psi_, 
                        double p_, double q_, double r_);
        void updatePara(vector<double> torque, vector<double> force);
    private:
        map<string, unique_ptr<MPCController>> controllers_;
        string mode_;
        string prev_mode_;
};

#endif