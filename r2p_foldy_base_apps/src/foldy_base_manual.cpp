/*
 * Copyright (c) 2016, The Robot Studio
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
 *
 *  Created on: Sep 30, 2016
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
//ROS services
#include "r2p_foldy_base_apps/switchNode.h"
#include "r2p_foldy_base_apps/getSlaveCmdArray.h"
//other
#include <stdio.h>
//ROS packages include 
#include "r2p_foldy_base_apps/robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT
#define DOF_DIM 				2//NUMBER_MOTORS_ARM

/*** Variables ***/
bool switch_node = false; //disable by default
osa_msgs::MotorCmdMultiArray motorCmd_ma;
sensor_msgs::Joy xboxJoy;
bool joy_arrived = false;

//ROS publisher
//ros::Publisher pub_motorBaseCmdMultiArray;

/*** Callback functions ***/
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(r2p_foldy_base_apps::switchNode::Request  &req, r2p_foldy_base_apps::switchNode::Response &res)
{
	//ROS_INFO("switch node");
	switch_node = req.state;
	return true;
}

bool getMotorCmd_ma(r2p_foldy_base_apps::getSlaveCmdArray::Request  &req, r2p_foldy_base_apps::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motorCmdMultiArray = motorCmd_ma;
		return true;
	}
	else
	{
		return false;
	}
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "osa_mobileBaseManual_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);

	//Publishers	
	//pub_motorBaseCmdMultiArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_base_cmd", 100, true);

	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_mobile_base_manual_srv", switchNode);
	ros::ServiceServer srv_getMotorCmd_ma = nh.advertiseService("get_mobile_base_manual_cmd_srv", getMotorCmd_ma);

	//create the commands multi array
	motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorCmd_ma.layout.dim[0].size = NUMBER_MOTORS_BASE;
	motorCmd_ma.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	motorCmd_ma.layout.dim[0].label = "motors";
	motorCmd_ma.layout.data_offset = 0;
	motorCmd_ma.motor_cmd.clear();
	motorCmd_ma.motor_cmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		//motorCmd_ma.motor_cmd[i].slaveBoardID = BIBOT_BASE_SLAVEBOARD_ID;
		motorCmd_ma.motor_cmd[i].node_id = i+1;
		motorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motorCmd_ma.motor_cmd[i].value = 0;
	}

	float baseLR_f = 0; //left right
	float baseUD_f = 0; //up down
	float leftWheel_f = 0;
	float rightWheel_f = 0;
	int leftWheel_i = 0;
	int rightWheel_i = 0;

	while(ros::ok())
	{	/*	
		for(int i=0; i<NUMBER_MOTORS_BASE; i++)
		{
			motorCmd_ma.motor_cmd[i].node_id = i+1;
			motorCmd_ma.motor_cmd[i].mode = NO_MODE; //VELOCITY_MODE;
			motorCmd_ma.motor_cmd[i].value = 0;
		}*/

		ros::spinOnce();

		if(switch_node)
		{
			//ROS_INFO("srv ON");

			if(joy_arrived)
			{
				if(!((xboxJoy.axes[3]<0.1)&&(xboxJoy.axes[3]>-0.1)&&(xboxJoy.axes[4]<0.1)&&(xboxJoy.axes[4]>-0.1)))
				{
					baseLR_f = xboxJoy.axes[3]/2; //left right
					baseUD_f = xboxJoy.axes[4]; //up down
								
					leftWheel_f = -(baseLR_f - baseUD_f)*4000;
					rightWheel_f = (baseLR_f + baseUD_f)*4000;

					leftWheel_i = (int)leftWheel_f;
					rightWheel_i = (int)rightWheel_f;
				}
				else //add a deadband of +/- 0.1 on both axis
				{
					leftWheel_i = 0;
					rightWheel_i = 0;
				}

				//Apply values
				motorCmd_ma.motor_cmd[0].command = SET_TARGET_VELOCITY;
				motorCmd_ma.motor_cmd[1].command = SET_TARGET_VELOCITY;
				motorCmd_ma.motor_cmd[0].value =  leftWheel_i;
				motorCmd_ma.motor_cmd[1].value = rightWheel_i;
			}
			else
			{
				//ROS_INFO("no joy");
				//STOP base motors	
				//baseCmd_ma.motor_cmd[0].mode = VELOCITY_MODE;
				//baseCmd_ma.motor_cmd[1].mode = VELOCITY_MODE;
				//baseCmd_ma.motor_cmd[0].value = 0;
				//baseCmd_ma.motor_cmd[1].value = 0;
			}

			//joy_arrived = false;
		}//if(switch_node)
		//else ROS_INFO("srv OFF");
 
		//r.sleep();
	}

	return 0;
}
