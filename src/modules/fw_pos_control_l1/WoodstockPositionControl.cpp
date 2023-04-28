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
FixedwingPositionControl::control_ws_mission(const hrt_abstime now, const Vector2d curr_pos,
					      const Vector2f ground_speed)
{
	// printf("[control_ws_mission] current_pos %f %f\n",
	// 	curr_pos(0), curr_pos(1));
	const float dt = math::constrain((now - _last_time_position_control_called) * 1e-6f, MIN_AUTO_TIMESTEP,
					 MAX_AUTO_TIMESTEP);
	_last_time_position_control_called = now;

	float total_speed = sqrt(pow(ground_speed(0),2 + pow(ground_speed(1),2)));

	matrix::Vector3f immediate_goal_waypoint = use_ws_waypoints[use_ws_waypoints.size()-1];
	matrix::Vector3f immediate_current_pose =
		matrix::Vector3f((float)curr_pos(0), (float)curr_pos(1),
		_current_altitude);

	float loiter_radius_abs = fabsf(_param_nav_loiter_rad.get());
	double desired_lat = (double)immediate_goal_waypoint(0);
	double desired_long = (double)immediate_goal_waypoint(1);

	float euclidean_distance = get_distance_to_next_waypoint(
		(double)curr_pos(0), (double)curr_pos(1),
		desired_lat, desired_long);

	// If the distance is near the loiter radius / 2 (near to the waypoint)
	if (euclidean_distance < loiter_radius_abs / 2)
	{
		if (path_check)
		{
			_ws_mission_leg = 1;
			// If it is smaller then minus off the previous waypoint and add in the new waypoint
			use_ws_waypoints.remove(use_ws_waypoints.size()-1);

		}
		// New use_ws_waypoint size after removing
		// mavlink_log_critical(&_mavlink_log_pub, "%d \t", (int)use_ws_waypoints.size());

		if (use_ws_waypoints.empty())
		{
			mavlink_log_critical(&_mavlink_log_pub, "waypoints empty, Woodstock navigation completed\t");

			if (_ws_mission.mission_state == vehicle_ws_state_s::WS_VEHICLE_LAND)
				ws_mode = landing_phase;
			else
				ws_mode = not_selected;

			publish_navigation_message(immediate_current_pose, immediate_goal_waypoint);
			return;
		}

		// Setup final bearing for path calculation
		float bearing_next_waypoint;
		if (use_ws_waypoints.size() > 1)
		{
			bearing_next_waypoint = get_bearing_to_next_waypoint(
				(double)use_ws_waypoints[use_ws_waypoints.size()-1](0),
				(double)use_ws_waypoints[use_ws_waypoints.size()-1](1),
				(double)use_ws_waypoints[use_ws_waypoints.size()-2](0),
				(double)use_ws_waypoints[use_ws_waypoints.size()-2](1));
		}
		else
			bearing_next_waypoint = 0.0;


		// Add target tracking
		immediate_goal_waypoint = use_ws_waypoints[use_ws_waypoints.size()-1];
		Vector2d immediate_goal_2d = Vector2d(immediate_goal_waypoint(0), immediate_goal_waypoint(1));
		path_check = _precision_flight.PathCalculationGeometrical(_yaw, bearing_next_waypoint,
			curr_pos, immediate_goal_2d);
		mavlink_log_info(&_mavlink_log_pub,
			"[Woodstock] waypoint (%d) left",
			(int)use_ws_waypoints.size()-1);

	}

	// if path check fails then we continue moving forward
	if (!path_check)
	{
		_att_sp.roll_body = 0;
		// _att_sp.yaw_body = _l1_control.nav_bearing();
		_att_sp.yaw_body = _yaw;
		_att_sp.fw_control_yaw = false;

	}
	else
	{
		matrix::Vector3d P = matrix::Vector3d((double)immediate_current_pose(0),
				(double)immediate_current_pose(1), (double)immediate_current_pose(2));

		double tolerance = 10.0;
		// Tracker function
		// When legs are circles
		if (_ws_mission_leg == 1 || _ws_mission_leg == 3)
		{
			// printf("[circle tracker] %d \n", (int)round(ceil((double)_ws_mission_leg/2.0))-1);
			Vector2d limit_pair =
				_precision_flight.get_angle_limits_pair(
				(int)round(ceil((double)_ws_mission_leg/2.0))-1);
			// Vector2d turn_angles =
			// 	_precision_flight.get_turn_angles();
			int rotation =
				_precision_flight.get_turn_direction(
				(int)round(ceil((double)_ws_mission_leg/2.0)-1));

			// When flag is deactivated, that means that the leg is over

			Vector2d C_xy = _precision_flight.get_circle_center((int)round(ceil((double)_ws_mission_leg/2.0))-1);
			matrix::Vector3d C = matrix::Vector3d(C_xy(0), C_xy(1), (double)immediate_goal_waypoint(2));

			if (!_precision_flight.planar_circle_tracker(
				C, P, (double)immediate_goal_waypoint(2), _precision_flight.get_min_turn_radius(),
				tolerance, rotation, limit_pair))
				_ws_mission_leg++;
		}

		// When legs are lines
		else
		{
			Vector2d S_xy = _precision_flight.get_line_points(0);
			Vector2d E_xy = _precision_flight.get_line_points(1);

			matrix::Vector3d S = matrix::Vector3d(S_xy(0), S_xy(1), _current_altitude);
			matrix::Vector3d E = matrix::Vector3d(E_xy(0), E_xy(1), _current_altitude);

			if (!_precision_flight.line_tracker(P, tolerance, S, E))
				_ws_mission_leg++;
		}

		matrix::Vector3d target = _precision_flight.get_target();
		matrix::Vector3d nearest_point_target = _precision_flight.get_nearest_point();

		Vector2f curr_wp_local = _global_local_proj_ref.project(target(0), target(1));
		Vector2f prev_wp_local = _global_local_proj_ref.project(
			nearest_point_target(0), nearest_point_target(1));
		Vector2f curr_pos_local = _global_local_proj_ref.project(
			P(0), P(1));

		// Using L1 to navigate to waypoint
		_l1_control.navigate_waypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed);

		_att_sp.roll_body = _l1_control.get_roll_setpoint();
		// _att_sp.yaw_body = _target_bearing;
		_att_sp.fw_control_yaw = false;


	}

	/*
	* Update tecs
	* Change the altitude setpoint = Current setpoint according to Dive Control
	*/
	float desired_alt = immediate_goal_waypoint(2);
	float target_airspeed = get_auto_airspeed_setpoint(
		now, total_speed, ground_speed, dt);
	tecs_update_pitch_throttle(now,
		desired_alt,
		target_airspeed,
		radians(_param_fw_p_lim_min.get()),
		radians(_param_fw_p_lim_max.get()),
		_param_fw_thr_min.get(),
		_param_fw_thr_max.get(),
		_param_fw_thr_cruise.get(),
		false,
		_param_fw_p_lim_min.get());

	// assign values from TECS
	_att_sp.pitch_body = get_tecs_pitch();
	_att_sp.thrust_body[0] = min(get_tecs_thrust(), _param_fw_thr_max.get());

}

void
FixedwingPositionControl::control_ws_landing(const hrt_abstime now, const Vector2d curr_pos,
					      const Vector2f ground_speed)
{

}

void
FixedwingPositionControl::update_ws_mission_navigator(const Vector2d curr_pos)
{
	// Augmenting the position_setpoint_triplet
	matrix::Vector3f immediate_goal_waypoint = use_ws_waypoints[use_ws_waypoints.size()-1];

	float loiter_radius_abs = fabsf(_param_nav_loiter_rad.get());
	double desired_lat = (double)immediate_goal_waypoint(0);
	double desired_long = (double)immediate_goal_waypoint(1);

	float euclidean_distance = get_distance_to_next_waypoint(
		(double)curr_pos(0), (double)curr_pos(1),
		desired_lat, desired_long);

	// mavlink_log_critical(&_mavlink_log_pub, "%d %f/%f\t",
	// 	(int)use_ws_waypoints.size(), (double)euclidean_distance,
	// 	(double)loiter_radius_abs);

	if (euclidean_distance > loiter_radius_abs)
		return;

	// If it is smaller then minus off the previous waypoint and add in the new waypoint
	use_ws_waypoints.remove(use_ws_waypoints.size()-1);

	// New use_ws_waypoint size after removing
	// mavlink_log_critical(&_mavlink_log_pub, "%d \t", (int)use_ws_waypoints.size());

	if (use_ws_waypoints.empty())
	{
		mavlink_log_critical(&_mavlink_log_pub, "waypoints empty, Woodstock navigation completed\t");
		ws_tracking_status = false;
		return;
	}

	// If used_waypoints are not empty, update the new gps coordinates
	immediate_goal_waypoint = use_ws_waypoints[use_ws_waypoints.size()-1];
	desired_lat = (double)immediate_goal_waypoint(0);
	desired_long = (double)immediate_goal_waypoint(1);

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
	PX4_INFO("[Woodstock] waypoint (%d) _pos_sp_triplet command sent", (int)use_ws_waypoints.size()-1);

}

void
FixedwingPositionControl::publish_navigation_message(
	const matrix::Vector3f curr_pos, const matrix::Vector3f waypoint_pos)
{
	float loiter_radius_abs = fabsf(_param_nav_loiter_rad.get());

	_pos_sp_triplet.previous = _pos_sp_triplet.current;
	_pos_sp_triplet.previous.yaw = _yaw;
	_pos_sp_triplet.previous.lat = curr_pos(0);
	_pos_sp_triplet.previous.lon = curr_pos(1);
	_pos_sp_triplet.previous.alt = curr_pos(2);
	_pos_sp_triplet.previous.valid = false;


	_pos_sp_triplet.current.timestamp = hrt_absolute_time();
	_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	_pos_sp_triplet.current.lat = waypoint_pos(0);
	_pos_sp_triplet.current.lon = waypoint_pos(1);
	_pos_sp_triplet.current.alt = waypoint_pos(2);
	_pos_sp_triplet.current.loiter_radius = loiter_radius_abs;
	_pos_sp_triplet.current.valid = true;

	_pos_sp_triplet.timestamp = hrt_absolute_time();
	orb_advert_t _pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);

	orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
	PX4_INFO("[Woodstock] waypoint (%d) _pos_sp_triplet command sent", (int)use_ws_waypoints.size()-1);
}
