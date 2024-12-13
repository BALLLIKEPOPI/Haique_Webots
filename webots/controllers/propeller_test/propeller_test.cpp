// File:          propeller_test.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Motor *motor1 = robot->getMotor("motor1");
  Motor *motor2 = robot->getMotor("motor2");
  Motor *motor3 = robot->getMotor("motor3");
  Motor *motor4 = robot->getMotor("motor4");
  Motor *motor5 = robot->getMotor("motor5");
  Motor *motor6 = robot->getMotor("motor6");
  Motor *motor7 = robot->getMotor("motor7");
  Motor *motor8 = robot->getMotor("motor8");
  
  Motor *servoL = robot->getMotor("left_arm_servo");
  Motor *servoR = robot->getMotor("right_arm_servo");
  
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  motor3->setPosition(INFINITY);
  motor4->setPosition(INFINITY);
  motor5->setPosition(INFINITY);
  motor6->setPosition(INFINITY);
  motor7->setPosition(INFINITY);
  motor8->setPosition(INFINITY);
  
  motor1->setVelocity(5.0);
  motor2->setVelocity(5.0);
  motor3->setVelocity(5.0);
  motor4->setVelocity(5.0);
  motor5->setVelocity(5.0);
  motor6->setVelocity(5.0);
  motor7->setVelocity(5.0);
  motor8->setVelocity(5.0);
  
  // servoL->setPosition(1.5708);
  // servoL->setVelocity(10);
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
