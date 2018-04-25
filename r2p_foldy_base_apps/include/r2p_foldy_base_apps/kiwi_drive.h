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
 * @file kiwi_drive.h
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on Apr 24, 2018
 * @date Created on Apr 24, 2018
 * @version 0.1.1
 * @brief Header file for the KiwiDrive class, to compute motor velocities from a given pair of vector direction and angular velocity (v, w)
 * 		  Adapted from: http://www.cs.cmu.edu/~pprk/physics.html
 */

#ifndef R2P_FOLDY_BASE_APPS_KIWI_DRIVE_H
#define R2P_FOLDY_BASE_APPS_KIWI_DRIVE_H

#include <ros/ros.h>
#include <Eigen/Dense>

namespace r2p_foldy_base_apps
{

/**
 * @brief This is the class for KiwiDrive.
 */
class KiwiDrive
{
public:
	/**
	 * @brief Constructor.
	 */
	KiwiDrive(float wheel_radius, float mobile_base_radius);

	/**
	 * @brief Destructor.
	 */
	~KiwiDrive();

	void computeWheelAngularVelocity(const float joy_horizontal, const float joy_vertical, const float joy_rotation, const int maximal_velocity);
	Eigen::Vector3f getWheelAngularVelocity() const { return wheel_angular_velocity_vec_; };

	const static int NUMBER_OF_WHEELS = 3;

private:
	float wheel_radius_; /**< in meter */
	float mobile_base_radius_; /**< in meter */
	Eigen::Matrix<float, 3, 2> unit_direction_mat_; /**< made of 3 vectors: F0, F1, F2, one for each wheel. */
	Eigen::Vector2f velocity_command_vec_; /**< v(x,y) expressed in the coordinates of the mobile base center in m/s. */
	float angular_velocity_command_vec_; /**< w in rad/S, from the mobile base center */
	Eigen::Vector3f wheel_linear_velocity_vec_; /**< v0 v1 v2 in m/s */
	Eigen::Vector3f wheel_angular_velocity_vec_; /**< w0 w1 w2 in rad/s */

};

} // namespace r2p_foldy_base_apps

#endif // R2P_FOLDY_BASE_APPS_KIWI_DRIVE_H
