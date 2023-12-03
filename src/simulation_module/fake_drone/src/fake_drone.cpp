#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "fake_drone/fake_drone.h"

/*Fake Drone is described as a odom-to-state transform process*/
FakeDrone::FakeDrone(ros::NodeHandle* node): nh_(*node){
	nh_.param("/fake_drone/place_x", current_pos(0), 0.0);
	nh_.param("/fake_drone/place_y", current_pos(1), 0.0);
	nh_.param("/fake_drone/place_z", current_pos(2), 0.05);
	current_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
	current_acc = Eigen::Vector3d(0.0, 0.0, 0.0);
	tf2::Quaternion quat;
	double place_yaw = 0.0;
	nh_.param("/fake_drone/place_yaw", place_yaw, 0.0);
	quat.setRPY(0, 0, place_yaw);
	current_att = Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
	current_uav_mode = 0;
	nh_.param("/fake_drone/frequency", sim_freq, 30.0);

	uav_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/uav/pos", 10);
	uav_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/uav/vel", 10);
	flight_mode_pub = nh_.advertise<std_msgs::Int16>("/uav/flight_mode", 10);

	target_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/uav/target_pos", 10, &FakeDrone::subPosCb, this);
	target_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav/target_vel", 10, &FakeDrone::subVelCb, this);
	target_acc_sub = nh_.subscribe<geometry_msgs::AccelStamped>("/uav/target_acc", 10, &FakeDrone::subAccCb, this);
	target_mode_sub = nh_.subscribe<std_msgs::Int16>("/uav/target_mode", 1, &FakeDrone::subModeCb, this);

	timer1 = nh_.createTimer(ros::Rate(sim_freq), &FakeDrone::timer1Cb, this);
}

void FakeDrone::subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(current_uav_mode == 1 || current_uav_mode == 2){
		current_pos(0) = msg->pose.position.x;
		current_pos(1) = msg->pose.position.y;
		current_pos(2) = msg->pose.position.z;
		Eigen::Vector4d temp_quat;
		temp_quat(0) = msg->pose.orientation.w;
		temp_quat(1) = msg->pose.orientation.x;
		temp_quat(2) = msg->pose.orientation.y;
		temp_quat(3) = msg->pose.orientation.z;
		if(temp_quat.norm() > 0){
			current_att = temp_quat;
		}
	}
}

void FakeDrone::subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
	if(current_uav_mode == 2){
		current_vel(0) = msg->twist.linear.x;
		current_vel(1) = msg->twist.linear.y;
		current_vel(2) = msg->twist.linear.z;
		current_angrate(0) = msg->twist.angular.x;
		current_angrate(1) = msg->twist.angular.y;
		current_angrate(2) = msg->twist.angular.z;
	}
	else{
		current_vel(0) = 0;
		current_vel(1) = 0;
		current_vel(2) = 0;
		current_angrate(0) = 0;
		current_angrate(1) = 0;
		current_angrate(2) = 0;
	}
}

void FakeDrone::subAccCb(const geometry_msgs::AccelStamped::ConstPtr& msg){
	if(current_uav_mode == 2){
		current_acc(0) = msg->accel.linear.x;
		current_acc(1) = msg->accel.linear.y;
		current_acc(2) = msg->accel.linear.z;
	}
	else{
		current_acc(0) = 0;
		current_acc(1) = 0;
		current_acc(2) = 0;
	}
}

void FakeDrone::subModeCb(const std_msgs::Int16::ConstPtr& msg){
	current_uav_mode = msg->data;
	std_msgs::Int16 _mode;
	_mode.data = current_uav_mode;
	flight_mode_pub.publish(_mode);
}

void FakeDrone::timer1Cb(const ros::TimerEvent&){
	geometry_msgs::PoseStamped _pos;
	_pos.header.stamp = ros::Time::now();
	_pos.pose.position.x = current_pos(0);
	_pos.pose.position.y = current_pos(1);
	_pos.pose.position.z = current_pos(2);
	_pos.pose.orientation.w = current_att(0);
	_pos.pose.orientation.x = current_att(1);
	_pos.pose.orientation.y = current_att(2);
	_pos.pose.orientation.z = current_att(3);
	uav_pos_pub.publish(_pos);
	geometry_msgs::TwistStamped _vel;
	_vel.header.stamp = ros::Time::now();
	_vel.twist.linear.x = current_vel(0);
	_vel.twist.linear.y = current_vel(1);
	_vel.twist.linear.z = current_vel(2);
	_vel.twist.angular.x = current_angrate(0);
	_vel.twist.angular.y = current_angrate(1);
	_vel.twist.angular.z = current_angrate(2);
	uav_vel_pub.publish(_vel);

	geometry_msgs::TransformStamped tfStamped;
	tfStamped.header.stamp = ros::Time::now();
	tfStamped.header.frame_id = "world";
	tfStamped.child_frame_id = "drone";
	tfStamped.transform.translation.x = current_pos(0);
	tfStamped.transform.translation.y = current_pos(1);
	tfStamped.transform.translation.z = current_pos(2);
	tfStamped.transform.rotation.w = current_att(0);
	tfStamped.transform.rotation.x = current_att(1);
	tfStamped.transform.rotation.y = current_att(2);
	tfStamped.transform.rotation.z = current_att(3);
	tf_br.sendTransform(tfStamped);
}

int main (int argc, char** argv) 
{        
    ros::init(argc, argv, "fake_drone");
    ros::NodeHandle nh;
    FakeDrone drone(&nh);
	ros::spin();

    return 0;
}