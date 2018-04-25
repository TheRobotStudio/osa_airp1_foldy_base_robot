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
 * @file kiwi_drive.cpp
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on Apr 24, 2018
 * @date Created on Apr 24, 2018
 * @version 0.1.1
 * @brief Implementation file for the KiwiDrive class, to compute motor velocities from a given pair of vector direction and angular velocity (v, w)
 * 		  Adapted from: http://www.cs.cmu.edu/~pprk/physics.html
 */

#include "r2p_foldy_base_apps/kiwi_drive.h"

#include <exception>
#include <stdexcept>
#include <math.h>

using namespace std;
using namespace r2p_foldy_base_apps;

KiwiDrive::KiwiDrive(float wheel_radius, float mobile_base_radius) :
wheel_radius_(wheel_radius),
mobile_base_radius_(mobile_base_radius),
unit_direction_mat_(), //(3, 2)
velocity_command_vec_(),
angular_velocity_command_vec_(),
wheel_linear_velocity_vec_(),
wheel_angular_velocity_vec_()
{
	//Set unit direction matrix
	//F0
	unit_direction_mat_(0, 0) = -1;
	unit_direction_mat_(0, 1) = 0;
	//F1
	unit_direction_mat_(1, 0) = 0.5;
	unit_direction_mat_(1, 1) = -sqrt(3)/2;
	//F2
	unit_direction_mat_(2, 0) = 0.5;
	unit_direction_mat_(2, 1) = sqrt(3)/2;
}

KiwiDrive::~KiwiDrive()
{
}

void KiwiDrive::computeWheelAngularVelocity(const float joy_horizontal, const float joy_vertical, const float joy_rotation, const int maximal_velocity)
{
	//expecting values from analog joystick in the range [-1; 1]
	//remapping this range to [-maximal_velocity; maximal_velocity]
	velocity_command_vec_(0) = joy_horizontal;
	velocity_command_vec_(1) = joy_vertical;
	velocity_command_vec_ *= maximal_velocity;
	//same for the angular velocity
	angular_velocity_command_vec_ = joy_rotation*maximal_velocity;

	Eigen::Vector3f identity(1,1,1);

	//W = (F*v + b*w*I)/r
	wheel_angular_velocity_vec_ = (unit_direction_mat_*velocity_command_vec_ + mobile_base_radius_*angular_velocity_command_vec_*identity)/wheel_radius_;
}
