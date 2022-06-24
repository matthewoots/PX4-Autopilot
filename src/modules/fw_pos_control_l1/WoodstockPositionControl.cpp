/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file WoodstockPositionControl.c
 * WoodstockPositionControl for PX4 autopilot
 *
 * @author Matthew Woo <matthewoots@gmail.com>
 */

#include "FixedwingPositionControl.hpp"

#include <px4_platform_common/events.h>

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

void
FixedwingPositionControl::control_ws_mission(const hrt_abstime &now, const Vector2d &curr_pos,
					      const Vector2f &ground_speed)
{
	// matrix::Vector3f immediate_goal_waypoint = use_ws_waypoints[0];

	// float euclidean_distance = get_distance_to_next_waypoint(
	// 	(double)curr_pos(0), (double)curr_pos(1),
	// 	(double)immediate_goal_waypoint(0), (double)immediate_goal_waypoint(1));

	// Vector2f pos_sp_prev, pos_sp_curr;

	// const float dt = math::constrain((now - _last_time_position_control_called) * 1e-6f, MIN_AUTO_TIMESTEP,
	// 				 MAX_AUTO_TIMESTEP);
	// _last_time_position_control_called = now;

	// // if we assume that user is taking off then help by demanding altitude setpoint well above ground
	// // and set limit to pitch angle to prevent steering into ground
	// float pitch_limit_min = _param_fw_p_lim_min.get();
	// float height_rate_sp = NAN;
	// float altitude_sp_amsl = _current_altitude;

	// if (in_takeoff_situation()) {
	// 	// if we assume that user is taking off then help by demanding altitude setpoint well above ground
	// 	// and set limit to pitch angle to prevent steering into ground
	// 	// this will only affect planes and not VTOL
	// 	altitude_sp_amsl = _takeoff_ground_alt + _param_fw_clmbout_diff.get();
	// 	pitch_limit_min = radians(10.0f);

	// } else {
	// 	height_rate_sp = getManualHeightRateSetpoint();
	// }

	// /* throttle limiting */
	// float throttle_max = _param_fw_thr_max.get();
}

void
FixedwingPositionControl::update_ws_mission_navigator(const Vector2d &curr_pos)
{
	// Augmenting the position_setpoint_triplet
	matrix::Vector3f immediate_goal_waypoint = use_ws_waypoints[0];

	float loiter_radius_abs = fabsf(_param_nav_loiter_rad.get());
	double desired_lat = (double)immediate_goal_waypoint(0);
	double desired_long = (double)immediate_goal_waypoint(1);

	float euclidean_distance = get_distance_to_next_waypoint(
		(double)curr_pos(0), (double)curr_pos(1),
		desired_lat, desired_long);

	if (euclidean_distance > loiter_radius_abs)
		return;

	mavlink_log_critical(&_mavlink_log_pub, "%d \t", (int)use_ws_waypoints.size());

	// if it is smaller then we minus off the previous waypoint and add in the new waypoint
	use_ws_waypoints.erase(use_ws_waypoints.begin());

	mavlink_log_critical(&_mavlink_log_pub, "%d \t", (int)use_ws_waypoints.size());
	if (use_ws_waypoints.empty())
	{
		mavlink_log_critical(&_mavlink_log_pub, "use_ws_waypoints empty\t");
		ws_tracking_status = false;
		return;
	}

	_pos_sp_triplet.previous = _pos_sp_triplet.current;
	_pos_sp_triplet.previous.yaw = _yaw;
	_pos_sp_triplet.previous.lat = curr_pos(0);
	_pos_sp_triplet.previous.lon = curr_pos(1);
	_pos_sp_triplet.previous.alt = _current_altitude;
	_pos_sp_triplet.previous.valid = false;


	_pos_sp_triplet.current.timestamp = hrt_absolute_time();
	_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	_pos_sp_triplet.current.lat = desired_lat;
	_pos_sp_triplet.current.lon = desired_long;
	_pos_sp_triplet.current.alt = immediate_goal_waypoint(2);
	_pos_sp_triplet.current.loiter_radius = loiter_radius_abs;
	_pos_sp_triplet.current.valid = true;

	_pos_sp_triplet.timestamp = hrt_absolute_time();
	orb_advert_t _pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);

	orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
	PX4_INFO("[Woodstock] _pos_sp_triplet command sent");

}
