#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

#include "casadi/casadi.hpp"
#include "attitudectl/controlPub.h"
#include "eigen3/Eigen/Eigen"
#include <vector>
#include <ros/ros.h>
#include "ros/publisher.h"

using namespace std;
using namespace casadi;

class StateMachine
{
    public:
        StateMachine();
        ~StateMachine();

        enum State
        {
            HOVER = 0,
            RUN = 1,
            HOVER2RUN = 2,
            RUN2HOVER = 3,
        };
        void init();
        void run();
        void setState(State newState);
        State getState();
        void setControlInput(const Eigen::VectorXd &input);
        Eigen::VectorXd getControlInput(); 
        void setStateMachineState(State state);
        int getStateMachineState();
        void hovering();
        void running();
        SX hoverDynamics(const SX state, const SX con, SX drag);
        SX runDynamics(const SX state, const SX con, SX drag);
        

    private:
        State currentState;
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

#endif