#include <casadi/core/sx_fwd.hpp>
#include<iostream>
#include<vector>
#include <casadi/casadi.hpp>
#include "statemachine/statemachine.hpp"
#include "eso/eso.h"
#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "ros/time.h"
// for realfly
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include "statePublish/statePub.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#define TIME_STEP 8  // [ms]

using namespace std;
using namespace casadi;

int main(int argc, char **argv){
    
    return 0;
}