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

StateMachine stateMachine;

void local_pose_cb(const statePublish::statePub::ConstPtr& msg){
    double roll, pitch, yaw;
    double w_x, w_y, w_z;
    double x_, y_, z_, v_x, v_y, v_z;
    roll = msg->roll;
    pitch = msg->pitch;
    yaw = msg->yaw;
    w_x = msg->angular_velocity_x;
    w_y = msg->angular_velocity_y;
    w_z = msg->angular_velocity_z;
    x_ = msg->global_position_x;
    y_ = msg->global_position_y;
    z_ = msg->global_position_z;
    v_x = msg->linear_velocity_x;
    v_y = msg->linear_velocity_y;
    v_z = msg->linear_velocity_z;
    // for sim
    stateMachine.updatex0(x_, y_, z_, v_x, v_y, v_z, 
                            roll, pitch, yaw, w_x, w_y, w_z);
    // for realfly
    // MPC_Ctl.updatex0(roll, pitch, yaw, w_x, w_y, w_z, 
    //                     x_, y_, z_, v_x, v_y, v_z);
}

void eso_cb(const eso::eso::ConstPtr &msg){
    eso::eso eso_msg = *msg;
    stateMachine.updatePara({eso_msg.torque1, eso_msg.torque2, eso_msg.torque3},
                            {eso_msg.force1, eso_msg.force2, eso_msg.force3});
}

attitudectl::controlPub getControlCommand(vector<double> control){
    attitudectl::controlPub control_msg;
    control_msg.thrust1 = control[0];
    control_msg.thrust2 = control[1];
    control_msg.thrust3 = control[2];
    control_msg.thrust4 = control[3];
    control_msg.thrust5 = control[4];
    control_msg.thrust6 = control[5];
    control_msg.thrust7 = control[6];
    control_msg.thrust8 = control[7];
    return control_msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "statemachine_node");
    ros::NodeHandle nh;

    // Wait for the `ros` controller.
    ros::service::waitForService("/robot/time_step");
    ros::spinOnce();
    ros::Duration(1).sleep();

    ros::Publisher conPub = nh.advertise<attitudectl::controlPub>("/mpc_ctl", 10);
    ros::Subscriber state_sub = nh.subscribe("/statePub", 10, local_pose_cb);
    ros::Subscriber eso_Sub = nh.subscribe("/eso_pub", 10, eso_cb);

    ros::Rate rate(50.0);

    while (ros::ok()) {
        // TODO: check the state of the system and update the state machine
        if(stateMachine.isModeChanged()){
            stateMachine.setMode("hover");
        }
        ros::Time begin = ros::Time::now();
        vector<double> control = stateMachine.controlStep();
        ros::Time end = ros::Time::now();
        ros::Duration duration = end - begin;
        ROS_INFO("Control step took: %f seconds", duration.toSec());
        
        // Publish the control command
        conPub.publish(getControlCommand(control));
    }

    return 0;
}