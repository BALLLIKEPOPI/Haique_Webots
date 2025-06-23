// #include <casadi/core/calculus.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <iostream>
#include <vector>
#include <casadi/casadi.hpp>
#include "attitudectl/attctl.h"
#include "haique_msgs/eso_msg.h"
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
#include "haique_msgs/statepub_msg.h"
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

// for realfly
// mavros_msgs::State current_state;
ros::Publisher odometry_pub;
MPC_CTL MPC_Ctl;

// for realfly
// void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
// for sim
void local_pose_cb(const haique_msgs::statepub_msg::ConstPtr& msg){
    double roll, pitch, yaw;
    double w_x, w_y, w_z;
    roll = msg->roll;
    pitch = msg->pitch;
    yaw = msg->yaw;
    w_x = msg->angular_velocity_x;
    w_y = msg->angular_velocity_y;
    w_z = msg->angular_velocity_z;
    // for sim
    MPC_Ctl.updatex0(roll, pitch, yaw, w_x, w_y, w_z);
    // for realfly
    // MPC_Ctl.updatex0(roll, pitch, yaw, w_x, w_y, w_z, 
    //                     x_, y_, z_, v_x, v_y, v_z);
}

void eso_cb(const haique_msgs::eso_msg::ConstPtr &msg){
    haique_msgs::eso_msg eso_msg = *msg;
    MPC_Ctl.updatePara(eso_msg.torque1, eso_msg.torque2, eso_msg.torque3);
}

//for realfly
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    
    // Wait for the `ros` controller.
    ros::service::waitForService("/robot/time_step");
    ros::spinOnce();
    ros::Duration(1).sleep();

    MPC_Ctl.init(nh);
    ROS_INFO("attitude control initialized.");
    // for realfly
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //                                 "mavros/local_position/pose", 10, local_pose_cb);
    // for sim
    ros::Subscriber state_sub = nh.subscribe("/statePub", 10, local_pose_cb);
    ros::Subscriber eso_Sub = nh.subscribe("/eso_pub", 10, eso_cb);

    ros::Rate rate(50.0);
    // for realfly
    // while(ros::ok() && !current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // MPC_Ctl.solve();
    while(ros::ok()) {
        ros::Time begin = ros::Time::now();
        MPC_Ctl.solve();
        ros::Time end = ros::Time::now();
        cout << "Time: " << end - begin << endl;
        ros::spinOnce();
        rate.sleep();
    }

}