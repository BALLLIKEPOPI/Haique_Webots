#include "statemachine/statemachine.hpp"

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

SX StateMachine::hoverDynamics(const SX state, const SX con, SX drag){

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