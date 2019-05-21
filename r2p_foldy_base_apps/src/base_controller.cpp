/*
 * Copyright (c) 2019, The Robot Studio
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
 * @file base_controller.cpp
 * @author Cyril Jourdan
 * @date Jan 4, 2019
 * @version 0.1.0
 * @brief File for the controller of the holonomic base with 3 wheels, it takes the output of the SLAM cmd_vel and turn it into motor commands
 * With help of website https://bharat-robotics.github.io/blog/kinematic-analysis-of-holonomic-robot
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Nov 29, 2018
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <iostream>
#include "osa_common/enums.h"

#define NUMBER_MOTORS_BASE 3

using namespace std;

geometry_msgs::Twist cmd_vel;
osa_msgs::MotorCmdMultiArray mobileBaseMotorCmd_ma;

bool cmd_vel_arrived = false;

const double gear_ratio = 28/1;
const double enc_tic_per_motor_turn = 1000*4; //multiply by 4 for the quadrate encoder
const double enc_tic_per_shaft_turn = enc_tic_per_motor_turn*gear_ratio;
const double robot_base_radius = 0.29; //in meters
const double swedish_wheel_radius = 0.0625;

//double width_robot = 0.1;
double vl = 0.0; //V1
double vr = 0.0; //V2
double vb = 0.0; //V3
ros::Time curr_time;
ros::Time prev_time;
double dt = 0.0;
double right_rpm = 0.0;
double left_rpm = 0.0;
double back_rpm = 0.0;
double ticks_per_meter = 100;
double x = 0.0;
double y = 0.0;
double th = 0.0; 

void cmdVelCallback(const geometry_msgs::TwistConstPtr &twist)
{
	cmd_vel = *twist;
	cmd_vel_arrived = true;
}

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_r2p_base_controller");
	ros::NodeHandle nh("~");

	//Subscribers
	ros::Subscriber cmd_vel_sub = nh.subscribe("/foldy_base/cmd_vel", 10, cmdVelCallback);
	//Publishers
	ros::Publisher pub_setMobileBaseCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/foldy_base/motor_cmd_to_filter", 1); //set_mobile_base_cmd

	ros::Rate loop_rate(10); //test up to 50Hz

	//create the commands multi array
	mobileBaseMotorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mobileBaseMotorCmd_ma.layout.dim[0].size = NUMBER_MOTORS_BASE;
	mobileBaseMotorCmd_ma.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	mobileBaseMotorCmd_ma.layout.dim[0].label = "motors";
	mobileBaseMotorCmd_ma.layout.data_offset = 0;
	mobileBaseMotorCmd_ma.motor_cmd.clear();
	mobileBaseMotorCmd_ma.motor_cmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		mobileBaseMotorCmd_ma.motor_cmd[i].node_id = i+1;
		mobileBaseMotorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		mobileBaseMotorCmd_ma.motor_cmd[i].value = 0;
	}

	float wheel_radius = 0;
	float base_radius = 0;
	int maximum_velocity_rpm = 0;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	double delta_th = 0.0;

	//Grab parameters
	try
	{
		nh.param("/holonomic_base/wheel_radius", wheel_radius, (float)0.0625);
		nh.param("/holonomic_base/base_radius", base_radius, (float)0.29);
		nh.param("/holonomic_base/maximum_velocity_rpm", maximum_velocity_rpm, (int)50);

		ROS_INFO("Grab the Foldy Base parameters: [%f,%f,%d]", wheel_radius, base_radius, maximum_velocity_rpm);
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		ROS_ERROR("Parameter of Foldy Base didn't load correctly!");
		ROS_ERROR("Please check the name and try again.");

		throw e;
	}

	ROS_INFO("start node");

	curr_time = ros::Time::now();
	prev_time = ros::Time::now();

	while(ros::ok())
	{
		//read the SLAM output
		ros::spinOnce();

		if(cmd_vel_arrived)
		{
			// time calculation
			curr_time = ros::Time::now();
			dt = (curr_time - prev_time).toSec();

			//reset base value to default
			for(int i=0; i<NUMBER_MOTORS_BASE; i++)
			{
				mobileBaseMotorCmd_ma.motor_cmd[i].node_id = i+1;
				mobileBaseMotorCmd_ma.motor_cmd[i].command = SET_TARGET_VELOCITY;
				mobileBaseMotorCmd_ma.motor_cmd[i].value = 0;
			}

			//get the demanded velocity in x, y and theta
			vx = cmd_vel.linear.x; //positive x is forward
			vy = cmd_vel.linear.y; //positive y is left sideway
			vth = cmd_vel.angular.z; //positive theta around z is turning to the left

			delta_th = vth*dt;

			ROS_INFO("vx=%f, vy=%f, vth=%f, dth=%f", vx, vy, vth, delta_th);

			// Inverse kinemtics formula 
			// This gives linear velocity of each wheel, left right and back
			vl = -sin(delta_th+M_PI/3)*vx + cos(delta_th+M_PI/3)*vy + robot_base_radius*vth; 
			vr = sin(M_PI/3-delta_th)*vx + cos(M_PI/3-delta_th)*vy + robot_base_radius*vth;
			vb = sin(delta_th)*vx - cos(delta_th)*vy + robot_base_radius*vth;

			// We need to convert this values to rpm (revolution per minute), relatively to the motor without gearbox
			left_rpm = ((vl*60)/(2*M_PI*swedish_wheel_radius))*gear_ratio;
			right_rpm = ((vr*60)/(2*M_PI*swedish_wheel_radius))*gear_ratio;
			back_rpm = ((vb*60)/(2*M_PI*swedish_wheel_radius))*gear_ratio;

			ROS_INFO("rpm= %f, %f, %f", left_rpm, right_rpm, back_rpm);
			// Asign result to the msg
			mobileBaseMotorCmd_ma.motor_cmd[0].value = -(int)(left_rpm/2); // divided by 2 to decrease the speed, TODO use the max velocity param instead
			mobileBaseMotorCmd_ma.motor_cmd[1].value = -(int)(right_rpm/2);// add a minus sign because the maxon non inverted convention is counter-clockwise ?
			mobileBaseMotorCmd_ma.motor_cmd[2].value = -(int)(back_rpm/2);

			//publish to the command filter node
			pub_setMobileBaseCommand.publish(mobileBaseMotorCmd_ma);

			cmd_vel_arrived = false;
			prev_time = curr_time;

			loop_rate.sleep();
		}
	}
}
