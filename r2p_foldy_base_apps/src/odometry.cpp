/*
 * Copyright (c) 2018, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file odometry.cpp
 * @author Cyril Jourdan
 * @date Nov 26, 2018
 * @version 0.1.0
 * @brief File for the odometry of the holonomic base with 3 wheels
 * With help of website https://bharat-robotics.github.io/blog/kinematic-analysis-of-holonomic-robot
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Nov 26, 2018
 */

/*** Includes ***/
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <osa_msgs/MotorDataMultiArray.h>
//#include <Eigen/Dense>

using namespace std;

/*** Variables ***/
osa_msgs::MotorDataMultiArray motor_data_array;

//to wait for the msg from both posture and anglesArmDescription topics
bool motor_data_array_arrived = false;

/*** Callback functions ***/
void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array = *data;
	motor_data_array_arrived = true;
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_r2p1_odometry_node");
	ros::NodeHandle nh("~");

	//Subscribers
	ros::Subscriber sub_motor_data_array = nh.subscribe("/foldy_base/motor_data_array", 10, motorDataArrayCallback);

	//Publishers
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

	// initial position
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	// velocity
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;

	// motor position
	int curr_mot_enc_pos[3] = {0};
	int prev_mot_enc_pos[3] = {0};
	int diff_mot_enc_pos[3] = {0};

	double wheel_lin_vel[3] = {0}; //V1, V2, V3

	const double gear_ratio = 28/1;
	const double enc_tic_per_motor_turn = 1000;
	const double enc_tic_per_shaft_turn = enc_tic_per_motor_turn*gear_ratio;
	const double robot_base_radius = 0.27; //in meters
	const double swedish_wheel_radius = 0.06;

	ros::Time curr_time;
	ros::Time prev_time;
	curr_time = ros::Time::now();
	prev_time = ros::Time::now();
	double dt = 0.0;

	double delta_x = 0.0;
	double delta_y = 0.0;
	double delta_th = 0.0;

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	//const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	//grasp first position before starting the loop
	if(ros::ok())
	{
		ros::spinOnce();

		if(motor_data_array_arrived)
		{
			prev_time = ros::Time::now();

			for(int i=0; i<3; i++) prev_mot_enc_pos[i] = motor_data_array.motor_data.at(i).position; //fill the prev values

			motor_data_array_arrived = false;
		}
	}

	while(ros::ok())
	{
		ros::spinOnce();

		if(motor_data_array_arrived)
		{
			// time calculation
			curr_time = ros::Time::now();
			dt = (curr_time - prev_time).toSec();

			//ROS_INFO("dt=%f", dt);
			
			// encoder position
			for(int i=0; i<3; i++)
			{
				curr_mot_enc_pos[i] = motor_data_array.motor_data.at(i).position;
				diff_mot_enc_pos[i] = curr_mot_enc_pos[i] - prev_mot_enc_pos[i];

				//Explaination : enc_tic_per_shaft_turn is done in 2*Pi. So an angle theta is done in 2*Pi*diff_enc/enc_tic_per_shaft_turn
				// v = d/t so d = theta*r = 2*Pi*r*diff_enc/enc_tic_per_shaft_turn
				// then just divide by dt to get the linear velocity of the wheel
				wheel_lin_vel[i] = (2*M_PI*swedish_wheel_radius*diff_mot_enc_pos[i])/(enc_tic_per_shaft_turn*dt);
				
				//ROS_INFO("Motor %d: enc=%d, diff_enc=%d, lin_vel=%f", i, curr_mot_enc_pos[i], diff_mot_enc_pos[i], curr_mot_enc_pos[2]);
			}

			//ROS_INFO("enc1=%d, enc2=%d, enc3=%d", curr_mot_enc_pos[0], curr_mot_enc_pos[1], curr_mot_enc_pos[2]);

			//calculation of theta_dot which is the variable vth here
			vth = -(wheel_lin_vel[0]+wheel_lin_vel[1]+wheel_lin_vel[2])/(3*robot_base_radius);
			delta_th = vth*dt/4; //don't understand why a factor 4 is needed to achieve correct angle measurement, maybe because of quadrature encoder ?

			//ROS_INFO("delta_th=%f", delta_th);

			vx = -2*(-cos(delta_th)*wheel_lin_vel[0] + cos(M_PI/3-delta_th)*wheel_lin_vel[1]  + cos(M_PI/3+delta_th)*wheel_lin_vel[2])/3;
			//ROS_INFO("vx1=%f", vx);
			vy = 2*(-sin(delta_th)*wheel_lin_vel[0] - sin(M_PI/3-delta_th)*wheel_lin_vel[1]  + sin(M_PI/3+delta_th)*wheel_lin_vel[2])/3;
			ROS_INFO("vy=%f", vy);

			delta_x = vx*dt;
			delta_y = vy*dt;

			//accumulated position rotated by delta_th and final angle, this will drift over time with the accumulated errors
			x += cos(delta_th)*delta_x - sin(delta_th)*delta_y; //delta_x;
			y += sin(delta_th)*delta_x + cos(delta_th)*delta_y; //delta_y;
			th += delta_th;

			ROS_INFO("vth=%f, delta_th=%f, vx=%f, vy=%f, delta_x=%f, delta_y=%f, x=%f, y=%f, th=%f", vth, delta_th, vx, vy, delta_x, delta_y, x, y, th);
			//ROS_INFO("vx=%f, delta_x=%f, x=%f", vx, delta_x, x);
			//ROS_INFO("th=%f", th);
			
			geometry_msgs::Quaternion odom_quat;
			odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

			// update transform
			odom_trans.header.stamp = curr_time;
			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

			// filling the odometry
			nav_msgs::Odometry odom;
			odom.header.stamp = curr_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			// position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			// velocity
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			odom.twist.twist.angular.z = vth;

			prev_time = curr_time;
			prev_mot_enc_pos[0] = curr_mot_enc_pos[0];
			prev_mot_enc_pos[1] = curr_mot_enc_pos[1];
			prev_mot_enc_pos[2] = curr_mot_enc_pos[2];

			// publishing the odometry and the new tf
			broadcaster.sendTransform(odom_trans);
			odom_pub.publish(odom);

			motor_data_array_arrived = false;

			loop_rate.sleep();
		}
	}

	return 0;
}
