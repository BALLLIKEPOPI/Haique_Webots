#include "statemachine/statemachine.hpp"
#include <casadi/core/sx_fwd.hpp>
#include <cmath>

StateMachine::StateMachine(){
    init();
}

StateMachine::~StateMachine(){

}

void StateMachine::init(){
    // Initialize the state machine
    setState(State::HOVER);
    
}

void StateMachine::run(){
    // Check the current state and execute the corresponding action
    switch (currentState) {
        case State::HOVER:
            hovering();
            break;
        case State::RUN:
            running();
            break;
        default:
            break;
    }
}

void StateMachine::setState(State newState){
    // Set the new state
    currentState = newState;
}

StateMachine::State StateMachine::getState(){
    // Get the current state
    return currentState;
}

void StateMachine::hovering(){
    // Perform actions for the hovering state
    cout << "Hovering..." << endl;

}

void StateMachine::running(){
    // Perform actions for the running state
    cout << "Running..." << endl;

}

/*
    * Dynamics functions
    * state(12 dim): [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
    * con(10 dim): [f1, f2, f3, f4, f5, f6, f7, f8, alpha, beta]
    * drag(3 dim): [drag_x, drag_y, drag_z]
*/
SX StateMachine::hoverDynamics(const SX state, const SX con, SX drag){
    // State
    SX x = state(0); SX y = state(1); SX z = state(2);
    SX vx = state(3); SX vy = state(4); SX vz = state(5);
    SX phi = state(6); SX theta = state(7); SX psi = state(8);
    SX wx = state(9); SX wy = state(10); SX wz = state(11);
    // Control inputs
    SX f1 = con(0); SX f2 = con(1); SX f3 = con(2); SX f4 = con(3);
    SX f5 = con(4); SX f6 = con(5); SX f7 = con(6); SX f8 = con(7);
    SX alpha = con(8); SX beta = con(9);
    // Drag torques
    SX drag_tx = drag(0); SX drag_ty = drag(1); SX drag_tz = drag(2);
    // Drag forces
    SX drag_fx = drag(3); SX drag_fy = drag(4); SX drag_fz = drag(5);

    SX F1 = f1 + f5;
    SX F2 = f2 + f6;
    SX F3 = f3 + f7;
    SX F4 = f4 + f8;
    SX totalForce = F1 + F2 + F3 + F4;
    
    SX Fx = (sin(psi)*sin(phi) + cos(phi)*cos(psi)*sin(theta))*totalForce;
    SX Fy = (cos(phi)*sin(phi)*sin(theta) - cos(psi)*sin(phi))*totalForce;
    SX Fz = cos(phi)*cos(theta)*totalForce;

    SX wx_ = wx + tan(theta)*(wy*sin(phi) + wz*cos(phi));
    SX wy_ = wy*cos(phi) - wz*sin(phi);
    SX wz_ = (wy*sin(phi) + wz*cos(phi))/cos(theta);

    SX Mx = (F2 - F4)*l;
    SX My = (F3 - F1)*l;
    SX Mz = (f1 - f2 + f3 - f4 - f5 + f6 - f7 + f8)*c/k;

    return SX::vertcat({vx,
                        vy,
                        vz,
                        1/m*Fx - drag_fx,
                        1/m*Fy - drag_fy,
                        1/m*(Fz + (rou*V - m)*G) - drag_fz,
                        wx_,
                        wy_,
                        wz_,
                        (1/I1)*Mx - drag_tx,
                        (1/I2)*My - drag_ty,
                        (1/I3)*Mz - drag_tz});
}

SX StateMachine::runDynamics(const SX state, const SX con, SX drag){
    SX M1 = c*con(0)/k;     SX M2 = c*con(1)/k;     
    SX M3 = c*con(2)/k;     SX M4 = c*con(3)/k; 
    SX M5 = c*con(4)/k;     SX M6 = c*con(5)/k; 
    SX M7 = c*con(6)/k;     SX M8 = c*con(7)/k; 

    SX Mx = l*((con(1)+con(5))*sin(con(9))-(con(3)+con(7))*sin(con(8)));
    SX My = l*(-con(0)-con(4)+con(2)+con(6));
    SX Mz = M1+M3-M5-M7-
                l*((con(1)+con(5))*cos(con(9))-(con(3)+con(7))*cos(con(8)));

    return SX::vertcat({state(3),
                        state(4),
                        state(5),
                        (Mx)/I1+drag(0),
                        (My)/I2+drag(1),
                        (Mz)/I3+drag(2)});
}