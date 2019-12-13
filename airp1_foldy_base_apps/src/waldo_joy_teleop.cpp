/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 *  Created on: Mar 14, 2019
 *      Author: Cyril Jourdan (cyril.jourdan.86@gmail.com)
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
#include "airp1_foldy_base_apps/switchNode.h"
#include "airp1_foldy_base_apps/getSlaveCmdArray.h"
//other
#include <stdio.h>

#include "airp1_foldy_base_apps/kiwi_drive.h"
#include "osa_common/enums.h"

/*** Defines ***/
//#define LOOP_RATE				50 //HEART_BEAT
#define DEADBAND	1000

using namespace airp1_foldy_base_apps;

/*** Variables ***/
bool switch_node = false; //disable by default
osa_msgs::MotorCmdMultiArray motor_cmd_ma;
sensor_msgs::Joy xboxJoy;
bool joy_arrived = false;

/*** Callback functions ***/
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNodeService(airp1_foldy_base_apps::switchNode::Request  &req, airp1_foldy_base_apps::switchNode::Response &res)
{
	ROS_DEBUG("switch node");
	switch_node = req.state;
	return true;
}

bool getMotorCmdArray(airp1_foldy_base_apps::getSlaveCmdArray::Request  &req, airp1_foldy_base_apps::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motorCmdMultiArray = motor_cmd_ma;
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
	//ros::Rate r(LOOP_RATE);

	float wheel_radius = 0;
	float base_radius = 0;
	int maximum_velocity_rpm = 0;

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

	//Create the Kiwi Drive with the parameters provided from the Parameter Server
	KiwiDrive *kiwi_drive = new KiwiDrive(wheel_radius, base_radius);

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);

	//Services
	ros::ServiceServer srv_switch_node = nh.advertiseService("switch_foldy_base_manual_srv", switchNodeService);
	ros::ServiceServer srv_get_motor_cmd_ma = nh.advertiseService("get_foldy_base_manual_cmd_srv", getMotorCmdArray);

	//create the commands multi array
	motor_cmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_ma.layout.dim[0].size = KiwiDrive::NUMBER_OF_WHEELS;
	motor_cmd_ma.layout.dim[0].stride = KiwiDrive::NUMBER_OF_WHEELS;
	motor_cmd_ma.layout.dim[0].label = "motors";
	motor_cmd_ma.layout.data_offset = 0;
	motor_cmd_ma.motor_cmd.clear();
	motor_cmd_ma.motor_cmd.resize(KiwiDrive::NUMBER_OF_WHEELS);

	for(int i=0; i<KiwiDrive::NUMBER_OF_WHEELS; i++)
	{
		motor_cmd_ma.motor_cmd[i].node_id = i+1;
		motor_cmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_ma.motor_cmd[i].value = 0;
	}
/*
	float baseLR_f = 0; //left right
	float baseUD_f = 0; //up down

	float leftWheel_f = 0;
	float rightWheel_f = 0;
	int leftWheel_i = 0;
	int rightWheel_i = 0;
*/
	switch_node = true; //switch on by default

	float joy_h = 0; //x horizontal v(x,y)
	float joy_v = 0; //y vertical v(x,y)
	float joy_r = 0; //w rotation

	while(ros::ok())
	{
		ros::spinOnce();

		if(switch_node)
		{
			for(int i=0; i<KiwiDrive::NUMBER_OF_WHEELS; i++)
			{
				motor_cmd_ma.motor_cmd[i].node_id = i+1;
				motor_cmd_ma.motor_cmd[i].command = SET_TARGET_VELOCITY;
//				motor_cmd_ma.motor_cmd[i].value = 0; 
			}

			if(joy_arrived)
			{
			    if(xboxJoy.buttons[4] == 1)
			    {
				if(!((xboxJoy.axes[7]<13500+DEADBAND)&&(xboxJoy.axes[7]>13500-DEADBAND)&&(xboxJoy.axes[6]<14300+DEADBAND)&&(xboxJoy.axes[6]>14300-DEADBAND)))
				{

					joy_h = -1*xboxJoy.axes[7]; //left right
					joy_v = 1*xboxJoy.axes[6]; //up down
				}
				else //add a deadband of +/- 1000 on both axis
				{
					joy_h = -13500;
					joy_v = 14300;
				}

				if(!((xboxJoy.axes[5]<16300+DEADBAND)&&(xboxJoy.axes[5]>16300-DEADBAND)))
				{
					joy_r = 1*xboxJoy.axes[5]; //turn left right
				}
				else //add a deadband of +/- 1000 on rotation axis
				{
					joy_r = 16300;
				}


				// center value
				joy_h += 13500;
				joy_v -= 14300;
				joy_r -= 16300;


				// scale value
				joy_h /= 3500;
				joy_v /= 9300;
				joy_r /= 13800;


				// clip value				
				if(joy_h<-1) joy_h = -1;	
				if(joy_h>1) joy_h = 1;	
				if(joy_v<-1) joy_v = -1;	
				if(joy_v>1) joy_v = 1;	
				if(joy_r<-1) joy_r = -1;	
				if(joy_r>1) joy_r = 1;	

				ROS_INFO("w(%f, %f, %f)", joy_h, joy_v, joy_r); 

				//compute motor velocities
				kiwi_drive->computeWheelAngularVelocity(joy_h, joy_v, joy_r, maximum_velocity_rpm);


				//factor to increase speed
				//calculate distance of joystick to the center
				float power_factor = (joy_h*joy_h + joy_v*joy_v)*5 + joy_r*joy_r*20;

				motor_cmd_ma.motor_cmd[0].value = power_factor*kiwi_drive->getWheelAngularVelocity()(1);
				motor_cmd_ma.motor_cmd[1].value = power_factor*kiwi_drive->getWheelAngularVelocity()(2);
				motor_cmd_ma.motor_cmd[2].value = power_factor*kiwi_drive->getWheelAngularVelocity()(0);

				//print
				ROS_INFO("w(%d, %d, %d)", motor_cmd_ma.motor_cmd[0].value, motor_cmd_ma.motor_cmd[1].value, motor_cmd_ma.motor_cmd[2].value);

				joy_arrived = false;
			    }
			    else
			    {	
				motor_cmd_ma.motor_cmd[0].value = 0; 
				motor_cmd_ma.motor_cmd[1].value = 0; 
				motor_cmd_ma.motor_cmd[2].value = 0; 
			    }
			}
		}//if(switch_node)
	}

	return 0;
}