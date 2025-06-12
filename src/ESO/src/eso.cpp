#include "../include/eso.h"
#include "eso/eso.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "statePublish/statePub.h"

using namespace std;

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

float Mx = 0.0;
float My = 0.0;
float Mz = 0.0;
float Tx = 0.0;
float Ty = 0.0;
float Tz = 0.0;

float input[6] = {0};
float state[6] = {0};
float drag[6] = {0};
ros::Publisher eso_Pub;
static eso::eso eso_msg;
vector<ESO> ESO_vec(6);

// for sim
void local_pose_cb(const statePublish::statePub::ConstPtr& msg){
    double w_x, w_y, w_z;
    double v_x, v_y, v_z;
    // for realfly
    // geometry_msgs::PoseStamped current_odom = *msg;


    // for realfly
    // double w = current_odom.pose.orientation.w;
    // double x = current_odom.pose.orientation.x;
    // double y = current_odom.pose.orientation.y;
    // double z = current_odom.pose.orientation.z;
    // for sim
    w_x = msg->angular_velocity_x;
    w_y = msg->angular_velocity_y;
    w_z = msg->angular_velocity_z;
    v_x = msg->linear_acceleration_x;
    v_y = msg->linear_acceleration_y;
    v_z = msg->linear_acceleration_z;

    state[0] = w_x; state[1] = w_y; state[2] = w_z; 
    state[3] = v_x; state[4] = v_y; state[5] = v_z; 

    for(int i = 0; i < 6; i++){
        ESO_vec[i].SetInput(input[i], state[i]);
        ESO_vec[i].ESO_ADRC();
        drag[i] = ESO_vec[i].GetEState();
        if(i == 5){
            drag[i] = drag[i] - (rou*V-m)*G/m;
        }
        // cout << "drag" << i << ": " << drag[i] << endl;
        // cout << "state" << i << ": " << state[i] << endl;
    }
    eso_msg.torque1 = drag[0];
    eso_msg.torque2 = drag[1];
    eso_msg.torque3 = drag[2];
    eso_msg.force1 = drag[3];
    eso_msg.force2 = drag[4];
    eso_msg.force3 = drag[5];
}

void paraInit(double para[]){
    para[0] = 1/I1;
    para[1] = 1/I2;
    para[2] = 1/I3;
    para[3] = 1/m;
    para[4] = 1/m;
    para[5] = 1/m;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "eso_node");
    ros::NodeHandle nh;

    // for sim
    ros::Subscriber sub = nh.subscribe("/statePub", 10, local_pose_cb);
    // ros::Subscriber conSub = nh.subscribe<mpc_control::controlPub>("/mpc_ctl", 10, conSub_cb);
    eso_Pub = nh.advertise<eso::eso>("/eso_pub", 10);
    double para[6];
    paraInit(para);
    for(int i = 0; i < 6; i++){
        ESO_vec[i].ESOInit(para[i]);
    }
    ros::Rate rate(50.0); 
    

    while(ros::ok()) {
        eso_Pub.publish(eso_msg);
        ros::spinOnce();
        rate.sleep();
    }
}