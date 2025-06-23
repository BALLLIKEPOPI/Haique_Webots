#include "../include/eso.h"
#include "haique_msgs/eso_msg.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <cmath>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "haique_msgs/statepub_msg.h"
#include "haique_msgs/controlpub_msg.h"

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

float input[6] = {0, 0, 0, 0, 0, 0};
float state[6] = {0, 0, 0, 0, 0, 0};
float drag[6] = {0, 0, 0, 0, 0, 0}; // drag[0-2] for torque, drag[3-5] for force
ros::Publisher eso_Pub;
static haique_msgs::eso_msg eso_msg;
vector<ESO> ESO_vec(6);

// for sim
void local_pose_cb(const haique_msgs::statepub_msg::ConstPtr& msg){
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
    // TODO: check if the angular velocity is correct
    // v_x = msg->linear_acceleration_x;
    // v_y = msg->linear_acceleration_y;
    // v_z = msg->linear_acceleration_z;
    v_x = msg->linear_velocity_x;
    v_y = msg->linear_velocity_y;
    v_z = msg->linear_velocity_z;

    state[0] = w_x; state[1] = w_y; state[2] = w_z; 
    state[3] = v_x; state[4] = v_y; state[5] = v_z; 
}

void conSub_cb(const haique_msgs::controlpub_msg::ConstPtr& msg){
    float thrusts[8] = {msg->thrust1, msg->thrust2, msg->thrust3, msg->thrust4,
                        msg->thrust5, msg->thrust6, msg->thrust7, msg->thrust8};
    float alpha = msg->alpha;
    float beta = msg->beta;

    float M1 = c*thrusts[0]/k;
    float M2 = c*thrusts[1]/k;
    float M3 = c*thrusts[2]/k;
    float M4 = c*thrusts[3]/k;
    float M5 = c*thrusts[4]/k;
    float M6 = c*thrusts[5]/k;
    float M7 = c*thrusts[6]/k;
    float M8 = c*thrusts[7]/k;

    float Tx_b = (thrusts[1] + thrusts[5]) * cos(beta) + (thrusts[3] + thrusts[7]) * cos(alpha); 
    float Ty_b = 0.0;
    float Tz_b = thrusts[0] + thrusts[2] + thrusts[4] + thrusts[6] + 
                    (thrusts[1] + thrusts[5]) * sin(beta) + (thrusts[3] + thrusts[7]) * sin(alpha);
    
    // phi = state[0]; theta = state[1]; psi = state[2];
    float Tx_w = cos(state[1])*cos(state[2])*Tx_b+
                 (cos(state[2])*sin(state[1])*sin(state[0])-sin(state[2])*cos(state[0]))*Ty_b+
                 (cos(state[2])*sin(state[1])*cos(state[0])+sin(state[2])*sin(state[0]))*Tz_b;
    float Ty_w = cos(state[1])*sin(state[2])*Tx_b+
                 (sin(state[2])*sin(state[1])*sin(state[0])+cos(state[2])*cos(state[0]))*Ty_b+
                 (sin(state[2])*sin(state[1])*cos(state[0])-cos(state[2])*sin(state[0]))*Tz_b;
    float Tz_w = -sin(state[1])*Tx_b+
                 cos(state[1])*sin(state[0])*Ty_b+
                 cos(state[1])*cos(state[0])*Tz_b;

    float Mx = l*((thrusts[1]+thrusts[5])*sin(beta)-(thrusts[3]+thrusts[7])*sin(alpha))-
                M2*cos(beta)+M6*cos(beta)-M4*cos(alpha)+M8*cos(alpha);
    float My = l*(-thrusts[0]-thrusts[4]+thrusts[2]+thrusts[6]);
    float Mz = M1+M3-M5-M7-M2*sin(beta)+M6*sin(beta)-M4*sin(alpha)+M8*sin(alpha)-
                l*((thrusts[1]+thrusts[5])*cos(beta)-(thrusts[3]+thrusts[7])*cos(alpha));

    input[0] = Mx;
    input[1] = My;
    input[2] = Mz;
    input[3] = Tx_w;
    input[4] = Ty_w;
    input[5] = Tz_w;
}

void esoTimerCallback(const ros::TimerEvent& event){
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
    ros::Subscriber conSub = nh.subscribe<haique_msgs::controlpub_msg>("/mpc_ctl", 10, conSub_cb);
    ros::Timer esoTimer = nh.createTimer(ros::Duration(0.02), esoTimerCallback);
    eso_Pub = nh.advertise<haique_msgs::eso_msg>("/eso_pub", 10);
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