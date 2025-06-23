#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
// for realfly
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "statePublish/statePub.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <webots_ros/set_int.h>

#define TIME_STEP 8  // [ms]

using namespace std;

static statePublish::statePub statePub;

// for realfly
// mavros_msgs::State current_state;

// for realfly
// void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
// for sim
void inertial_unitCallback(const sensor_msgs::Imu::ConstPtr& msg){
    double roll, pitch, yaw;

    // for realfly
    // geometry_msgs::PoseStamped current_odom = *msg;
    // for sim
    geometry_msgs::Quaternion model_orientation = msg->orientation;
    geometry_msgs::Twist model_twist;
    model_twist.angular = msg->angular_velocity;

    // for realfly
    // double w = current_odom.pose.orientation.w;
    // double x = current_odom.pose.orientation.x;
    // double y = current_odom.pose.orientation.y;
    // double z = current_odom.pose.orientation.z;
    // for sim
    double w = model_orientation.w;
    double x = model_orientation.x;
    double y = model_orientation.y;
    double z = model_orientation.z;

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // 使用 90 度或 -90 度
    else
        pitch = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    statePub.roll = roll;
    statePub.pitch = pitch;
    statePub.yaw = yaw;
    // cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << endl;
}

void GyroCallback(const sensor_msgs::Imu::ConstPtr& msg){
    geometry_msgs::Twist model_twist;
    model_twist.angular = msg->angular_velocity;
    double w_x = model_twist.angular.x;
    double w_y = model_twist.angular.y;
    double w_z = model_twist.angular.z;
    statePub.angular_velocity_x = w_x;
    statePub.angular_velocity_y = w_y;
    statePub.angular_velocity_z = w_z;
}

void AccCallback(const sensor_msgs::Imu::ConstPtr& msg){
    geometry_msgs::Twist model_twist;
    model_twist.linear = msg->linear_acceleration;
    double v_x = model_twist.linear.x;
    double v_y = model_twist.linear.y;
    double v_z = model_twist.linear.z;
    statePub.linear_acceleration_x = v_x;
    statePub.linear_acceleration_y = v_y;
    statePub.linear_acceleration_z = v_z;
}

void GpsCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    double x = msg->point.x;
    double y = msg->point.y;
    double z = msg->point.z;
    statePub.global_position_x = x;
    statePub.global_position_y = y;
    statePub.global_position_z = z;
    // cout << "global_position_x: " << x << " global_position_y: " << y << " global_position_z: " << z << endl;
}

void GpsSpeedCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    double v_x = msg->point.x;
    double v_y = msg->point.y;
    double v_z = msg->point.z;
    statePub.linear_velocity_x = v_x;
    statePub.linear_velocity_y = v_y;
    statePub.linear_velocity_z = v_z;
    // cout << "linear_velocity_x: " << v_x << " linear_velocity_y: " << v_y << " linear_velocity_z: " << v_z << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    
    // Wait for the `ros` controller.
    ros::service::waitForService("/robot/time_step");
    ros::spinOnce();
    ros::Duration(1).sleep();

    ROS_INFO("state publish initialized.");
    // for realfly
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //                                 "mavros/local_position/pose", 10, local_pose_cb);
    // for sim
    ros::ServiceClient inertial_unit_Client = nh.serviceClient<webots_ros::set_int>("/inertial_unit/enable");
    ros::Subscriber inertial_unit_sub = nh.subscribe("/inertial_unit/quaternion", 10, inertial_unitCallback);
    ros::ServiceClient Gyro_Client = nh.serviceClient<webots_ros::set_int>("/gyro/enable");
    ros::Subscriber Gyro_sub = nh.subscribe("/gyro/values", 10, GyroCallback);
    ros::ServiceClient accelerometer_Client = nh.serviceClient<webots_ros::set_int>("/accelerometer/enable");
    ros::Subscriber accelerometer_sub = nh.subscribe("/accelerometer/values", 10, AccCallback);
    ros::Publisher statePublisher = nh.advertise<statePublish::statePub>("/statePub", 10);
    ros::ServiceClient Gps_Client = nh.serviceClient<webots_ros::set_int>("/gps/enable");
    ros::Subscriber Gps_sub = nh.subscribe("/gps/values", 10, GpsCallback);
    ros::Subscriber Gps_speed_sub = nh.subscribe("/gps/speed_vector", 10, GpsSpeedCallback);

    // initialize the statePub
    statePub.roll = 0;
    statePub.pitch = 0;
    statePub.yaw = 0;
    statePub.angular_velocity_x = 0;
    statePub.angular_velocity_y = 0;
    statePub.angular_velocity_z = 0;

    webots_ros::set_int inertial_unit_Srv;     
    inertial_unit_Srv.request.value = TIME_STEP;
    if (inertial_unit_Client.call(inertial_unit_Srv) && inertial_unit_Srv.response.success) {
        ROS_INFO("imu enabled.");
    } else {
        if (!inertial_unit_Srv.response.success)
        ROS_ERROR("Failed to enable imu.");
        return 1;
    }

    webots_ros::set_int gyro_Srv;
    gyro_Srv.request.value = TIME_STEP;
    if (Gyro_Client.call(gyro_Srv) && gyro_Srv.response.success) {
        ROS_INFO("gyro enabled.");
    } else {
        if (!gyro_Srv.response.success)
        ROS_ERROR("Failed to enable gyro.");
        return 1;
    }

    webots_ros::set_int acc_Srv;
    acc_Srv.request.value = TIME_STEP;
    if (accelerometer_Client.call(acc_Srv) && acc_Srv.response.success) {
        ROS_INFO("accelerometer enabled.");
    } else {
        if (!acc_Srv.response.success)
        ROS_ERROR("Failed to enable accelerometer.");
        return 1;
    }

    webots_ros::set_int gps_Srv;
    gps_Srv.request.value = TIME_STEP;
    if (Gps_Client.call(gps_Srv) && gps_Srv.response.success) {
        ROS_INFO("gps enabled.");
    } else {
        if (!gps_Srv.response.success)
        ROS_ERROR("Failed to enable gps.");
        return 1;
    }

    ros::Rate rate(50.0);

    while(ros::ok()) {
        statePub.header.stamp = ros::Time::now();
        statePub.header.frame_id = "statePub";
        statePublisher.publish(statePub);
        ros::spinOnce();
        rate.sleep();
    }
}