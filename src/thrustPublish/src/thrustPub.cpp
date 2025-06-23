#include "attitudectl/controlPub.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <iostream>
#include <ostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <vector>

#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <sensor_msgs/Range.h>

#include <webots_ros/get_bool.h>
#include <webots_ros/get_int.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/set_int.h>
#include <webots_ros/field_get_name.h>
#include <webots_ros/field_get_node.h>
#include <webots_ros/field_get_rotation.h>
#include <webots_ros/field_get_vec3f.h>
#include <webots_ros/field_set_rotation.h>
#include <webots_ros/field_set_vec3f.h>
#include <webots_ros/node_get_field.h>
#include <webots_ros/supervisor_movie_start_recording.h>
#include <webots_ros/supervisor_set_label.h>
#include <sstream>

#define TIME_STEP 8  // [ms]
#define thrust_gain 50
#define Pi 3.1415926

using namespace std;

webots_ros::set_float motor1Srv;
webots_ros::set_float motor2Srv;
webots_ros::set_float motor3Srv;
webots_ros::set_float motor4Srv;
webots_ros::set_float motor5Srv;
webots_ros::set_float motor6Srv;
webots_ros::set_float motor7Srv;
webots_ros::set_float motor8Srv;
webots_ros::set_float joint1Srv;
webots_ros::set_float joint2Srv;

ros::ServiceClient right_arm_joint_PositionClient;
ros::ServiceClient left_arm_joint_PositionClient;

ros::ServiceClient motor1_VelocityClient;
ros::ServiceClient motor2_VelocityClient;
ros::ServiceClient motor3_VelocityClient;
ros::ServiceClient motor4_VelocityClient;
ros::ServiceClient motor5_VelocityClient;
ros::ServiceClient motor6_VelocityClient;
ros::ServiceClient motor7_VelocityClient;
ros::ServiceClient motor8_VelocityClient;

ros::ServiceClient setTimeStepClient;
webots_ros::set_int setTimeStepSrv;
static int step = TIME_STEP;

// vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> thrust(8);
// vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> last_thrust(8);
// vector<std_msgs::Float64> joint_angle(2);
// vector<std_msgs::Float64> last_joint_angle(2);

void conSub_cb(const attitudectl::controlPub::ConstPtr &msg){
    attitudectl::controlPub conSub_msg = *msg;
    motor1Srv.request.value = (double)conSub_msg.thrust1*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust1*thrust_gain;
    motor2Srv.request.value = (double)conSub_msg.thrust2*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust2*thrust_gain;
    motor3Srv.request.value = (double)conSub_msg.thrust3*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust3*thrust_gain;
    motor4Srv.request.value = (double)conSub_msg.thrust4*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust4*thrust_gain;
    motor5Srv.request.value = (double)conSub_msg.thrust5*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust5*thrust_gain;
    motor6Srv.request.value = (double)conSub_msg.thrust6*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust6*thrust_gain;
    motor7Srv.request.value = (double)conSub_msg.thrust7*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust7*thrust_gain;
    motor8Srv.request.value = (double)conSub_msg.thrust8*thrust_gain > 50 ? 50 : (double)conSub_msg.thrust8*thrust_gain;
    joint1Srv.request.value = Pi/2 - (double)conSub_msg.alpha;
    joint2Srv.request.value = Pi/2 - (double)conSub_msg.beta;
    // thrust[0].data = copysign(1.0, conSub_msg.thrust1)*sqrt(abs(conSub_msg.thrust1)/2)*thrust_gain;
    // thrust[1].data = copysign(1.0, conSub_msg.thrust2)*sqrt(abs(conSub_msg.thrust2)/2)*thrust_gain;
    // thrust[2].data = copysign(1.0, conSub_msg.thrust3)*sqrt(abs(conSub_msg.thrust3)/2)*thrust_gain;
    // thrust[3].data = -copysign(1.0, conSub_msg.thrust4)*sqrt(abs(conSub_msg.thrust4)/2)*thrust_gain;
    // thrust[4].data = copysign(1.0, conSub_msg.thrust5)*sqrt(abs(conSub_msg.thrust5)/2)*thrust_gain;
    // thrust[5].data = copysign(1.0, conSub_msg.thrust6)*sqrt(abs(conSub_msg.thrust6)/2)*thrust_gain;
    // thrust[6].data = copysign(1.0, conSub_msg.thrust7)*sqrt(abs(conSub_msg.thrust7)/2)*thrust_gain;
    // thrust[7].data = -copysign(1.0, conSub_msg.thrust8)*sqrt(abs(conSub_msg.thrust8)/2)*thrust_gain;
    // joint_angle[1].data = -conSub_msg.alpha;
    // joint_angle[0].data = -conSub_msg.beta;

    // for(int i = 0; i < 8; i++){
    //     if(std::isnan(thrust[i].data)){
    //         thrust[i].data = last_thrust[i].data;
    //     }
    // }
    // for(int i = 0; i < 2; i++){
    //     if(std::isnan(joint_angle[i].data)){
    //         joint_angle[i].data = last_joint_angle[i].data;
    //     }
    // // }    
    // last_thrust.insert(last_thrust.begin(), thrust.begin(), thrust.end());
    // last_joint_angle.insert(last_joint_angle.begin(), joint_angle.begin(), joint_angle.end());
}

int main(int argc, char **argv){
    int nStep = 0;

    ros::init(argc, argv, "thrustPub");
    ros::NodeHandle nh;

    // Wait for the `ros` controller.
    ros::service::waitForService("/robot/time_step");
    ros::spinOnce();

     // send robot time step to webots
    setTimeStepClient = nh.serviceClient<webots_ros::set_int>("/robot/time_step");
    setTimeStepSrv.request.value = step;

    // nh.param("thrust_gain/value", thrust_gain, 1000);
    
    ros::Duration(1).sleep();

    // set motor position to infinity
    webots_ros::set_float motorSrv;
    motorSrv.request.value = INFINITY;

    std::array<ros::ServiceClient, 8> motorPositionClients;
    for (size_t i = 0; i < motorPositionClients.size(); ++i) {
        std::ostringstream service_name;
        service_name << "/motor" << (i + 1) << "/set_position";
        motorPositionClients[i] = nh.serviceClient<webots_ros::set_float>(service_name.str());
        if (!motorPositionClients[i].call(motorSrv)) {
            ROS_WARN_STREAM("Failed to call service " << service_name.str());
        }
    }
    right_arm_joint_PositionClient = nh.serviceClient<webots_ros::set_float>("/right_arm_servo/set_position");
    left_arm_joint_PositionClient = nh.serviceClient<webots_ros::set_float>("/left_arm_servo/set_position");

    // set motor velocity to 0
    motor1Srv.request.value = 0.0;
    motor2Srv.request.value = 0.0;
    motor3Srv.request.value = 0.0;
    motor4Srv.request.value = 0.0;
    motor5Srv.request.value = 0.0;
    motor6Srv.request.value = 0.0;
    motor7Srv.request.value = 0.0;
    motor8Srv.request.value = 0.0;
    motor1_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor1/set_velocity");
    motor2_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor2/set_velocity");
    motor3_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor3/set_velocity");
    motor4_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor4/set_velocity");
    motor5_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor5/set_velocity");
    motor6_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor6/set_velocity");
    motor7_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor7/set_velocity");
    motor8_VelocityClient = nh.serviceClient<webots_ros::set_float>("/motor8/set_velocity");

    ros::Subscriber conSub = nh.subscribe<attitudectl::controlPub>("/mpc_ctl", 10, conSub_cb);
    ros::Rate rate(50.0); 
    while(ros::ok()) {      
        motor1_VelocityClient.call(motor1Srv);
        motor2_VelocityClient.call(motor2Srv);
        motor3_VelocityClient.call(motor3Srv);
        motor4_VelocityClient.call(motor4Srv);
        motor5_VelocityClient.call(motor5Srv);
        motor6_VelocityClient.call(motor6Srv);
        motor7_VelocityClient.call(motor7Srv);
        motor8_VelocityClient.call(motor8Srv);
        right_arm_joint_PositionClient.call(joint1Srv);
        left_arm_joint_PositionClient.call(joint2Srv);
        ros::spinOnce();
        rate.sleep();
    }
}