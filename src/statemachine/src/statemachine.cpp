#include "statemachine/statemachine.hpp"
#include <casadi/core/sx_fwd.hpp>
#include <cmath>
#include <iostream>
#include <vector>

StateMachine::StateMachine(){
    controllers_["hover"] = std::make_unique<HoverMPC>();
    // controllers_["run"] = std::make_unique<RunMPC>(10, 0.1);
    mode_ = "hover";
}

StateMachine::~StateMachine(){

}

void StateMachine::setMode(const std::string& mode) {
    if (controllers_.find(mode) != controllers_.end()) {
        if (mode_ != mode) {
            prev_mode_ = mode_;
            mode_ = mode;
        }
    } else {
        throw std::invalid_argument("Invalid mode: " + mode);
    }
}

string StateMachine::getMode() const {
    return mode_;
}

bool StateMachine::isModeChanged() const {
    return mode_ != prev_mode_;
}

vector<double> StateMachine::controlStep(const vector<double>& state,
                                                  const vector<double>& ref) {
    return controllers_[mode_]->step(state, ref);
}

vector<double> StateMachine::controlStep() {
    bool modeChanged = isModeChanged();
    vector<double> result = controllers_[mode_]->step(modeChanged);
    prev_mode_ = mode_;  // sync the previous mode after step
    return result;
}

void StateMachine::updatex0(double x_, double y_, double z_,
                                double vx_, double vy_, double vz_,
                                double phi_, double theta_, double psi_, 
                                double p_, double q_, double r_){
    controllers_[mode_]->updatex0(x_, y_, z_, vx_, vy_, vz_,
                                    phi_, theta_, psi_, p_, q_, r_);
}

void StateMachine::updateControl(vector<double> last_control){
    controllers_[mode_]->updateControl(last_control);
}

void StateMachine::updateEso(vector<double> eso_force, vector<double> eso_torque){
    controllers_[mode_]->updateEso(eso_force, eso_torque);
}

void StateMachine::updatePara(){
    controllers_[mode_]->updatePara();
}