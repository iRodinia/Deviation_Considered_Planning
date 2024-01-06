#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "fake_drone_pro/fake_drone_pro.h"

/* Fake Drone Pro is described as a 1st-order system with T_c being its time constant */
FakeDronePro::FakeDronePro(ros::NodeHandle* node): nh_(*node){
	nh_.param("fake_drone/place_x", current_pos(0), 0.0);
	nh_.param("fake_drone/place_y", current_pos(1), 0.0);
	nh_.param("fake_drone/place_z", current_pos(2), 0.05);
    ref_pos = current_pos;
	current_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
	current_acc = Eigen::Vector3d(0.0, 0.0, 0.0);
	double place_yaw = 0.0;
	nh_.param("fake_drone/place_yaw", place_yaw, 0.0);
    current_att.setIdentity();
    current_att.w() = cos(place_yaw / 2.0);
    current_acc.z() = sin(place_yaw / 2.0);
    current_att.normalize();
    ref_att = current_att;
	current_uav_mode = 0;
	nh_.param("fake_drone/frequency", sim_freq, 30.0);
    sim_freq = max(sim_freq, 5.0);
    sim_dt = 1.0 / sim_freq;
    nh_.param("fake_drone/time_constant", T_c, 1.0);
    T_c = max(T_c, 0.01);
    damp_factor = exp(-sim_dt / T_c);

	uav_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/uav/pos", 10);
	uav_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/uav/vel", 10);
	flight_mode_pub = nh_.advertise<std_msgs::Int16>("/uav/flight_mode", 10);

	target_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/uav/target_pos", 10, &FakeDronePro::subPosCb, this);
	target_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav/target_vel", 10, &FakeDronePro::subVelCb, this);
	target_acc_sub = nh_.subscribe<geometry_msgs::AccelStamped>("/uav/target_acc", 10, &FakeDronePro::subAccCb, this);
	target_mode_sub = nh_.subscribe<std_msgs::Int16>("/uav/target_mode", 1, &FakeDronePro::subModeCb, this);

	timer1 = nh_.createTimer(ros::Rate(sim_freq), &FakeDronePro::timer1Cb, this);

	nh_.param("Log/enable_log", enable_log, false);
    if(enable_log){
        string log_folder_name;
	    nh_.param("Log/log_folder_name", log_folder_name, string("default_folder"));
        logger_ptr = std::shared_ptr<FlightLogger>(new FlightLogger(log_folder_name, "quadrotor"));
        vector<string> tags = {
			"ref_pos_x", "ref_pos_y", "ref_pos_z",
			"ref_vel_x", "ref_vel_y", "ref_vel_z",
			"ref_acc_x", "ref_acc_y", "ref_acc_z",
			"ref_att_w", "ref_att_x", "ref_att_y", "ref_att_z",
			"ref_angrate_x", "ref_angrate_y", "ref_angrate_z",
			"ref_flight_mode",
			"pos_x", "pos_y", "pos_z",
			"vel_x", "vel_y", "vel_z",
			"acc_x", "acc_y", "acc_z",
			"att_w", "att_x", "att_y", "att_z",
			"angrate_x", "angrate_y", "angrate_z",
			"flight_mode",
		};
		logger_ptr->setDataTags(tags);
        dumpParams();
    }
}

FakeDronePro::~FakeDronePro(){
	if(enable_log){
        logger_ptr->saveFile();
    }
}

void FakeDronePro::dumpParams(){
	if(!enable_log){
        return;
    }
	logger_ptr->logParameter("sim_freq", sim_freq);
	start_log_ts = ros::Time::now();
}

void FakeDronePro::subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(current_uav_mode == 1 || current_uav_mode == 2){
		ref_pos(0) = msg->pose.position.x;
		ref_pos(1) = msg->pose.position.y;
		ref_pos(2) = msg->pose.position.z;
		Eigen::Vector4d temp_quat;
		temp_quat(0) = msg->pose.orientation.w;
		temp_quat(1) = msg->pose.orientation.x;
		temp_quat(2) = msg->pose.orientation.y;
		temp_quat(3) = msg->pose.orientation.z;
		if(temp_quat.norm() > 0){
			ref_att.w() = temp_quat(0);
            ref_att.x() = temp_quat(1);
            ref_att.y() = temp_quat(2);
            ref_att.z() = temp_quat(3);
            ref_att.normalize();
		}

		if(enable_log){
			double _log_ts = (ros::Time::now() - start_log_ts).toSec();
			vector<string> _log_tags = {
				"ref_pos_x", "ref_pos_y", "ref_pos_z",
				"ref_att_w", "ref_att_x", "ref_att_y", "ref_att_z",
			};
			vector<double> _log_data = {
				ref_pos(0), ref_pos(1), ref_pos(2),
				ref_att.w(), ref_att.x(), ref_att.y(), ref_att.z(),
			};
			logger_ptr->logData(_log_ts, _log_tags, _log_data);
		}
	}
}

void FakeDronePro::subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
	if(current_uav_mode == 2){
		current_vel(0) = msg->twist.linear.x;
		current_vel(1) = msg->twist.linear.y;
		current_vel(2) = msg->twist.linear.z;
		current_angrate(0) = msg->twist.angular.x;
		current_angrate(1) = msg->twist.angular.y;
		current_angrate(2) = msg->twist.angular.z;

		if(enable_log){
			double _log_ts = (ros::Time::now() - start_log_ts).toSec();
			vector<string> _log_tags = {
				"ref_vel_x", "ref_vel_y", "ref_vel_z",
				"ref_angrate_x", "ref_angrate_y", "ref_angrate_z",
			};
			vector<double> _log_data = {
				current_vel(0), current_vel(1), current_vel(2),
				current_angrate(0), current_angrate(1), current_angrate(2),
			};
			logger_ptr->logData(_log_ts, _log_tags, _log_data);
		}
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

void FakeDronePro::subAccCb(const geometry_msgs::AccelStamped::ConstPtr& msg){
	if(current_uav_mode == 2){
		current_acc(0) = msg->accel.linear.x;
		current_acc(1) = msg->accel.linear.y;
		current_acc(2) = msg->accel.linear.z;

		if(enable_log){
			double _log_ts = (ros::Time::now() - start_log_ts).toSec();
			vector<string> _log_tags = {
				"ref_acc_x", "ref_acc_y", "ref_acc_z",
			};
			vector<double> _log_data = {
				current_acc(0), current_acc(1), current_acc(2),
			};
			logger_ptr->logData(_log_ts, _log_tags, _log_data);
		}
	}
	else{
		current_acc(0) = 0;
		current_acc(1) = 0;
		current_acc(2) = 0;
	}
}

void FakeDronePro::subModeCb(const std_msgs::Int16::ConstPtr& msg){
	current_uav_mode = msg->data;

	if(enable_log){
		double _log_ts = (ros::Time::now() - start_log_ts).toSec();
		logger_ptr->logData(_log_ts, "ref_flight_mode", current_uav_mode);
	}
}

void FakeDronePro::timer1Cb(const ros::TimerEvent&){
    current_pos = ref_pos + damp_factor*(current_pos - ref_pos);
    current_att = current_att.slerp(1-damp_factor, ref_att);

	geometry_msgs::PoseStamped _pos;
	_pos.header.stamp = ros::Time::now();
	_pos.pose.position.x = current_pos(0);
	_pos.pose.position.y = current_pos(1);
	_pos.pose.position.z = current_pos(2);
	_pos.pose.orientation.w = current_att.w();
	_pos.pose.orientation.x = current_att.x();
	_pos.pose.orientation.y = current_att.y();
	_pos.pose.orientation.z = current_att.z();
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

	std_msgs::Int16 _mode;
	_mode.data = current_uav_mode;
	flight_mode_pub.publish(_mode);

	geometry_msgs::TransformStamped tfStamped;
	tfStamped.header.stamp = ros::Time::now();
	tfStamped.header.frame_id = "world";
	tfStamped.child_frame_id = "drone";
	tfStamped.transform.translation.x = current_pos(0);
	tfStamped.transform.translation.y = current_pos(1);
	tfStamped.transform.translation.z = current_pos(2);
	tfStamped.transform.rotation.w = current_att.w();
	tfStamped.transform.rotation.x = current_att.x();
	tfStamped.transform.rotation.y = current_att.y();
	tfStamped.transform.rotation.z = current_att.z();
	tf_br.sendTransform(tfStamped);

	if(enable_log){
		double _log_ts = (ros::Time::now() - start_log_ts).toSec();
		vector<string> _log_tags = {
			"pos_x", "pos_y", "pos_z",
			"vel_x", "vel_y", "vel_z",
			"acc_x", "acc_y", "acc_z",
			"att_w", "att_x", "att_y", "att_z",
			"angrate_x", "angrate_y", "angrate_z",
			"flight_mode",
		};
		vector<double> _log_data = {
			current_pos(0), current_pos(1), current_pos(2),
			current_vel(0), current_vel(1), current_vel(2),
			current_acc(0), current_acc(1), current_acc(2),
			current_att.w(), current_att.x(), current_att.y(), current_att.z(),
			current_angrate(0), current_angrate(1), current_angrate(2),
			double(current_uav_mode),
		};
		logger_ptr->logData(_log_ts, _log_tags, _log_data);
	}
}

int main (int argc, char** argv) 
{        
    ros::init(argc, argv, "fake_drone");
    ros::NodeHandle nh;
	FakeDronePro drone(&nh);
    
	ros::spin();

    return 0;
}