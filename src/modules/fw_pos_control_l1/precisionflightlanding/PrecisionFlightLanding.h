/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file PrecisionFlightLanding.h
 * PrecisionFlightLanding for PX4 autopilot
 *
 * @author Matthew Woo <matthewoots@gmail.com>
 */

#ifndef PRECISIONFLIGHTLANDING_H
#define PRECISIONFLIGHTLANDING_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/geo/geo.h>

using matrix::Vector2d;
using matrix::Vector2f;
using matrix::wrap_pi;
using matrix::wrap_2pi;

#define g 9.81
#define pi 3.141593

namespace precisionflightlanding
{

enum DubinsPathDecision {
	RightStraightRight = 0,
	RightStraightLeft = 1,
	LeftStraightRight = 2,
	LeftStraightLeft = 3,
	NONE = 4
};

class PrecisionFlightLanding
{
public:
	PrecisionFlightLanding(){};
	~PrecisionFlightLanding(){};



	bool PathCalculationGeometrical(float init_bearing, float final_bearing,
		Vector2d start_coord, Vector2d final_coord)
	{
		// printf("[PathCalculationGeometrical] Bearing %f %f, start %f %f, end %f %f\n",
		// 	(double)init_bearing, (double)final_bearing,
		// 	start_coord(0),start_coord(1),
		// 	final_coord(0),final_coord(1));
		// dubin_geometrical(ip, fp, ib, fb, minTurnRad, flightHeight)

		float line_segment_distance = UpdateCircleChoices(init_bearing, final_bearing,
			start_coord, final_coord);

		if (pow(line_segment_distance,2) <= 4 * pow(MinTurnRadius(),2))
		{
			printf("[PathCalculationGeometrical] dist must be more than %f, currently %f\n",
				(double)sqrt(4 * pow(MinTurnRadius(),2)), (double)line_segment_distance);
			return false;
		}

		double line_seg_bearing = LineSegmentBearing(_Ci, _Cf);
		double ac = line_seg_bearing;

		double gamma = acos(MinTurnRadius() / (0.5f * line_segment_distance));

		if (sel == DubinsPathDecision::LeftStraightRight)
		{
			ac = wrap_pi(ac + gamma - pi/2.0);
		}
		else if (sel == DubinsPathDecision::RightStraightLeft)
		{
			ac = wrap_pi(ac + (pi - pi/2 - gamma));
		}
		else if (sel == DubinsPathDecision::LeftStraightLeft ||
			sel == DubinsPathDecision::RightStraightRight)
		{
			// Dont do anything
		}

		double offset1 = 0.0, offset2 = 0.0;
		double tf1 = 0.0, tf2 = 0.0;
		// Check with [pathDecision](0)
		// strcmp(pathDecision(0),'L')
		if (decision[0] == 0)
		{
			// Anticlockwise
			// Rotation offset to center pointing to point in line
			offset1 = pi/2.0;
			// Final must be smaller than initial value
			if (ac > (double)init_bearing)
				tf1 = ac - 2.0 * pi;
			else
				tf1 = ac;

			turn_rotation[0] = -1;
			printf("Turn 1 Anticlockwise\n");
		}
		// strcmp(pathDecision(0),'R')
		else if (decision[0] == 1)
		{
			// Clockwise
			// Rotation offset to center pointing to point in line
			offset1 = -pi/2;
			// Final must be larger than initial value
			if (ac < (double)init_bearing)
				tf1 = ac + 2.0 * pi;
			else
				tf1 = ac;

			turn_rotation[0] = 1;
			printf("Turn 1 Clockwise\n");
		}

		// Check with [pathDecision](1)
		// strcmp(pathDecision(1),'L')
		if (decision[1] == 0)
		{
			// Anticlockwise
			// Rotation offset to center pointing to point in line
			offset2 = pi/2;
			// Final must be smaller than initial value
			if ((double)final_bearing > ac)
				tf2 = (double)final_bearing - 2.0 * pi;
			else
				tf2 = (double)final_bearing;

			turn_rotation[1] = -1;
			printf("Turn 2 Anticlockwise\n");
		}
		// strcmp(pathDecision(1),'R')
		else if (decision[1] == 1)
		{
			// Clockwise
			// Rotation offset to center pointing to point in line
			offset2 = -pi/2;
			// Final must be larger than initial value
			if ((double)final_bearing < ac)
				tf2 = (double)final_bearing + 2.0 * pi;
			else
				tf2 = (double)final_bearing;

			turn_rotation[1] = 1;
			printf("Turn 2 Clockwise\n");
		}

		// Get the difference in bearing
		double ab1 = tf1 - (double)init_bearing;
		double ang_1turn = ab1 / pi * 180;

		double ab2 = tf2 - ac;
		double ang_2turn = ab2 / pi * 180;
		printf("1st Turn Segment Angle = %f deg\n", ang_1turn);
		printf("2nd Turn Segment Angle = %f deg\n", ang_2turn);

		// What is the angle of turns during the turn portion
		double turn1b = wrap_pi((double)init_bearing + ab1);
		// double turn2b = wrap_pi(turn1b + ab2);

		// y is long, x is lat

		// float x, y, h, vect;
		// Find points of straight line
		// Bearing for 1st turn
		float a1 = (float)wrap_pi(turn1b + offset1);

		// x = MinTurnRadius() * sin(a1);
		// y = MinTurnRadius() * cos(a1);
		// h = sqrt(pow(x,2) + pow(y,2));
		// vect = getBearingRad(tan(x/y));


		// Position of Straight Line Point of 1st turn
		waypoint_from_heading_and_distance(_Ci(0), _Ci(1), a1, MinTurnRadius(), &_line_points[0](0), &_line_points[0](1));

		// Bearing for 2nd turn
		float a2 = (float)wrap_pi(turn1b + offset2);

		// x = MinTurnRadius() * sin(a2);
		// y = MinTurnRadius() * cos(a2);
		// h = sqrt(pow(x,2) + pow(y,2));
		// vect = getBearingRad(tan(x/y));


		// Position of Straight Line Point of 2nd turn
		waypoint_from_heading_and_distance(_Cf(0), _Cf(1), a2, MinTurnRadius(), &_line_points[1](0), &_line_points[1](1));

		float distance_point_to_point = get_distance_to_next_waypoint(_line_points[0](0), _line_points[0](0), _line_points[1](0), _line_points[1](1));

		// keypoints = (start_coord,
		// 	_line_points[0], _line_points[1],
		// 	final_coord);
		// turn_angles = [ang_1turn, ang_2turn];
		// length_of_legs = [arclength1, distance_point_to_point, arclength2];

		// Find the arc length
		float arclength1 = MinTurnRadius() * abs((float)ab1);
		float arclength2 = MinTurnRadius() * abs((float)ab2);
		length_of_legs[0] = arclength1;
		length_of_legs[1] = distance_point_to_point;
		length_of_legs[2] = arclength2;
		turn_angles(0) = ab1;
		turn_angles(1) = ab2;

		// start_coord - _Ci center to point
		update_angle_limits(_Ci, start_coord, 0);

		// _line_points[0] - _Ci center to point
		update_angle_limits(_Ci, _line_points[0], 1);

		// _line_points[1] - _Cf center to point
		update_angle_limits(_Cf, _line_points[1], 2);

		// final_coord - _Cf center to point
		update_angle_limits(_Cf, final_coord, 3);

		printf("Angle limits [%f %f %f %f], turn1 %d (%f) turn2 %d (%f)\n",
			angle_limits[0], angle_limits[1], angle_limits[2], angle_limits[3],
			turn_rotation[0], turn_angles(0), turn_rotation[1], turn_angles(1));

		return true;
	}

	bool planar_circle_tracker(
		matrix::Vector3d C, matrix::Vector3d P, double end_height, double radius, double tolerance, int rot, Vector2d limit_pair)
	{
		// rot (-1) anticlockwise
		// rot (1) clockwise

		// Make sure this aligns to clockwise or anticlockwise
		// The pair consist of [start end] angle
		// This is the output from dubin flight path

		// printf("[circle tracker] limit_pair (%.3f %.3f) rot (%d)\n",
		// 	limit_pair(0), limit_pair(1), rot);

		// Find nearest point on the circle perimeter closest to the point
		// https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point

		// P - C; // center to point
		float bearing_c_to_p = get_bearing_to_next_waypoint(C(0), C(1), P(0), P(1));

		Vector2d np;
		// Nearest point on the circle
		waypoint_from_heading_and_distance(C(0), C(1), bearing_c_to_p, (float)radius, &np(0), &np(1));

		Vector2d C_xy = Vector2d(C(0), C(1));
		Vector2d P_xy = Vector2d(P(0), P(1));
		Vector2d cp = coordinates_to_local_xy(C_xy, P_xy);

		double cir_ang_change = tolerance / radius;

		// Find current point's angle
		// vect1 is facing y (cf) and vect2 is facing current angle (cp)
		Vector2d cf = Vector2d(0, radius);
		// https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
		double dotp = cf(0) * cp(0) + cf(1) * cp(1); // dot product between [x1, y1] and [x2, y2] cf dot cp
		double detp = cf(1)*cp(0) - cp(1)*cf(0); // determinant
		double curr_ang_rot = atan2(detp, dotp);

		double next_ang = wrap_pi(curr_ang_rot + rot * cir_ang_change);

		waypoint_from_heading_and_distance(C(0), C(1), next_ang, (float)radius, &target(0), &target(1));

		// Track height here
		double height_gain = 0.1;
		target(2) = P(2) + height_gain * (end_height - P(2));

		matrix::Vector3d nearest_point =
			matrix::Vector3d(np(0), np(1), C(2));
		nearest_point_target = nearest_point;

		// Check that [next_point] is within the limits from dubin path
		// For clockwise (cw) final - initial will yield a positive value
		// For counter-clockwise (ccw) -(final - initial) will yield a positive value

		double angle_length, angle_length_path;

		// strcmp(rot,'cw')
		if (rot == 1)
		{
			angle_length = limit_pair(1) - limit_pair(0);
			bool wrap_check = angle_length > 0.0;
			if (!wrap_check)
				angle_length = angle_length + 2.0 * pi;

			angle_length_path = next_ang - limit_pair(0);
			bool wrap_check_path = angle_length_path > 0.0;
			if (!wrap_check_path)
				angle_length_path = angle_length_path + 2.0 * pi;
		}
		else if (rot == -1)
		{
			angle_length = -(limit_pair(1) - limit_pair(0));
			bool wrap_check = angle_length > 0.0;
			if (!wrap_check)
				angle_length = angle_length + 2.0 * pi;

			angle_length_path = -(next_ang - limit_pair(0));
			bool wrap_check_path = angle_length_path > 0.0;
			if (!wrap_check_path)
				angle_length_path = angle_length_path + 2.0 * pi;
		}
		else
			return false;

		// printf("[circle tracker] feasible angle_length %.3fdeg angle_length_path %.3fdeg current %.3fdeg\n",
		// 	angle_length/3.14 * 180, angle_length_path/3.14 * 180, (double)bearing_c_to_p/3.14 * 180);

		// Check with the next angle [next_ang]
		bool flag = (angle_length - angle_length_path) > 0;

		printf("[circle tracker] flag %d (%f - %f = %f)\n",
			flag, angle_length, angle_length_path,
			(double)(angle_length - angle_length_path));

		return flag;
	}

	bool line_tracker(matrix::Vector3d P, double tolerance, matrix::Vector3d S, matrix::Vector3d E)
	{
		// Find nearest point on the line closest to the point
		// https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
		float bearing_es = get_bearing_to_next_waypoint(S(0), S(1), E(0), E(1));
		// printf("[line tracker] bearing_es (%.3f)\n", (double)bearing_es);

		Vector2d S_xy = Vector2d(S(0), S(1));
		Vector2d E_xy = Vector2d(E(0), E(1));
		Vector2d P_xy = Vector2d(P(0), P(1));

		Vector2d es = coordinates_to_local_xy(S_xy, E_xy);
		double height_diff_es = E(2) - S(2);
		matrix::Vector3d v =
			matrix::Vector3d(es(0), es(1), height_diff_es);
		v = v / v.norm();

		Vector2d ps = coordinates_to_local_xy(S_xy, P_xy);
		double height_diff_ps = P(2) - S(2);

		double d_xy = (v(0)*ps(0) + v(1)*ps(1));
		matrix::Vector3d nearest_point;
		waypoint_from_heading_and_distance(S(0), S(1), bearing_es, (float)d_xy, &nearest_point(0), &nearest_point(1));

		nearest_point(2) = S(2) + height_diff_es*height_diff_ps * v(2);

		// Find the next point
		matrix::Vector3d next_point;
		waypoint_from_heading_and_distance(
			nearest_point(0), nearest_point(1), bearing_es,
			(float)tolerance, &next_point(0), &next_point(1));

		// Need to fix the height for tolerance
		next_point(2) = nearest_point(2) + v(2) * tolerance;

		float next_point_d = get_distance_to_next_waypoint(S(0), S(1), next_point(0), next_point(1));

		float end_start_d = get_distance_to_next_waypoint(S(0), S(1), E(0), E(1));

		// Check with the maximum allowed distance [next_point_d]
		bool flag = (end_start_d - next_point_d) > 0.0f;
		printf("[line tracker] flag %d (%.4f - %.4f = %.4f)\n",
			flag, (double)(end_start_d), (double)(next_point_d),
			(double)(end_start_d - next_point_d));

		target = next_point;
		nearest_point_target = nearest_point;

		return flag;
	}

	// get_bvp_coefficients using PX4 matrix (without eigen)
	void get_bvp_coefficients(matrix::SquareMatrix<double, 3> initial,
		matrix::SquareMatrix<double, 3> final, double total_time,
		matrix::Vector3d *alpha, matrix::Vector3d *beta,
		matrix::Vector3d *gamma)
	{
		double T = total_time;
		// Update the initial values
		matrix::Vector3d p0 = matrix::Vector3d(
			initial(0,0), initial(1,0), initial(2,0));
		matrix::Vector3d v0 = matrix::Vector3d(
			initial(0,1), initial(1,1), initial(2,1));
		matrix::Vector3d a0 = matrix::Vector3d(
			initial(0,2), initial(1,2), initial(2,2));

		// Update the destination values
		matrix::Vector3d pf = matrix::Vector3d(
			final(0,0), final(1,0), final(2,0));
		matrix::Vector3d vf = matrix::Vector3d(
			final(0,1), final(1,1), final(2,1));
		matrix::Vector3d af = matrix::Vector3d(
			final(0,2), final(1,2), final(2,2));

		matrix::Vector<double, 9> delta;
		delta(0) = pf(0) - p0(0) - v0(0) * T - 0.5 * a0(0) * pow(T,2);
		delta(1) = pf(1) - p0(1) - v0(1) * T - 0.5 * a0(1) * pow(T,2);
		delta(2) = pf(2) - p0(2) - v0(2) * T - 0.5 * a0(2) * pow(T,2);
		delta(3) = (vf(0) - v0(0) - a0(0) * T);
		delta(4) = (vf(1) - v0(1) - a0(1) * T);
		delta(5) = (vf(2) - v0(2) - a0(2) * T);
		delta(6) = (af(0) - a0(0));
		delta(7) = (af(1) - a0(1));
		delta(8) = (af(2) - a0(2));

		double fifth_order[9] = {720, -360*T, 60*pow(T,2),
					-360*T, 168*pow(T,2), -24*pow(T,3),
					60*pow(T,2), -24*pow(T,3), 3*pow(T,4)};

		matrix::SquareMatrix<double, 3> m(fifth_order);

		matrix::SquareMatrix<double, 9> M;
		M.setZero();

		// Make the 3x3 matrix into a 9x9 matrix so that xyz is included
		for (int i = 0; i < 9; i++)
		{
			int l_m = (i+1) % 3 + 3*(int)(((i+1) % 3) == 0) - 1;
			int h_m = (int)ceil((double)(i+1) / 3.0) - 1;

			int l_M = (l_m)*3;
			int h_M = (h_m)*3;

			M(l_M,h_M) = m(l_m,h_m);
			M(l_M+1,h_M+1) = m(l_m,h_m);
			M(l_M+2,h_M+2) = m(l_m,h_m);
		}

		matrix::Vector<double, 9> abg;

		abg = (1/pow(T,5) * M * delta);
		*alpha = matrix::Vector3d(abg(0), abg(1), abg(2));
		*beta = matrix::Vector3d(abg(3), abg(4), abg(5));
		*gamma = matrix::Vector3d(abg(6), abg(7), abg(8));

	}

	int check_z_vel(matrix::SquareMatrix<double, 3> initial,
		matrix::SquareMatrix<double, 3> final,
        	double total_time, double command_time, matrix::Vector3d alpha, matrix::Vector3d beta,
		matrix::Vector3d gamma)
	{
		matrix::Vector3d v0 = matrix::Vector3d(
			initial(0,1), initial(1,1), initial(2,1));
		matrix::Vector3d a0 = matrix::Vector3d(
			initial(0,2), initial(1,2), initial(2,2));

		int waypoint_size = (int)ceil(total_time / command_time);
		double corrected_interval = total_time / (double)waypoint_size;
			int bad_counts = 0;
		for (int i = 0; i < waypoint_size; i++)
		{
			matrix::Vector3d vel = (alpha/24 * pow((corrected_interval*i),4) +
				beta/6 * pow((corrected_interval*i),3) +
				gamma/2 * pow((corrected_interval*i),2) + a0 * corrected_interval + v0);

			if (vel(2) > 0.001)
				bad_counts += 1;
		}

		return bad_counts;
	}

	void set_rollmax(float value) {rollmax = value;}

	void set_constant_velocity(float value) {Vconst = value;}

	Vector2d get_angle_limits_pair(int index)
	{
		Vector2d pair =
			Vector2d(angle_limits[index*2], angle_limits[index*2+1]);
		return pair;
	}

	Vector2d get_turn_angles(){return turn_angles;}

	int get_turn_direction(int index)
	{
		return turn_rotation[index];
	}

	double get_min_turn_radius(){return (double)MinTurnRadius();}

	Vector2d get_circle_center(int index)
	{
		if (index == 0)
			return _Ci;
		else if (index == 1)
			return _Cf;
		else
			return Vector2d();
	}

	matrix::Vector3d get_target(){return target;}

	matrix::Vector3d get_nearest_point(){return nearest_point_target;}

	Vector2d get_line_points(int index){return _line_points[index];}

private:
	float rollmax, Vconst;
	DubinsPathDecision sel;
	Vector2d _Ci, _Cf;
	Vector2d _line_points[2];
	Vector2d turn_angles;
	// 0 left 1 right
	int decision[2];
	double angle_limits[4];
	float length_of_legs[3];

	matrix::Vector3d target;
	matrix::Vector3d nearest_point_target;
	int turn_rotation[2];


	// Choosing whether to use 1. RSR, 2. RSL, 3. LSR, 4. LSL,
	float UpdateCircleChoices(float init_bearing, float final_bearing,
		Vector2d start_coord, Vector2d final_coord)
	{
		// y is long, x is lat
		Vector2d Ci [2];
		Vector2d Cf [2];
		float dist = pow(10,6);

		printf("Angle = %f \n", wrap_pi((double)init_bearing + (pi/2)));
		printf("Angle = %f \n", wrap_pi((double)init_bearing - (pi/2)));
		printf("Angle = %f \n", wrap_pi((double)final_bearing + (pi/2)));
		printf("Angle = %f \n", wrap_pi((double)final_bearing - (pi/2)));

		// initial centers of the right and left circles about the initial position and the initial heading using [rad] and [lat,lon]
		waypoint_from_heading_and_distance(start_coord(0), start_coord(1), wrap_pi((double)init_bearing + (pi/2)), MinTurnRadius(), &Ci[0](0), &Ci[0](1));
		waypoint_from_heading_and_distance(start_coord(0), start_coord(1), wrap_pi((double)init_bearing - (pi/2)), MinTurnRadius(), &Ci[1](0), &Ci[1](1));

		// final centers of the right and left circles about the initial position and the initial heading using [rad] and [lat,lon]
		waypoint_from_heading_and_distance(final_coord(0), final_coord(1), wrap_pi((double)final_bearing + (pi/2)), MinTurnRadius(), &Cf[0](0), &Cf[0](1));
		waypoint_from_heading_and_distance(final_coord(0), final_coord(1), wrap_pi((double)final_bearing - (pi/2)), MinTurnRadius(), &Cf[1](0), &Cf[1](1));

		printf("[CircleChoices] Start %f %f, %f %f\n",Ci[0](0),Ci[0](1),Ci[1](0),Ci[1](1));
		printf("[CircleChoices] Final %f %f, %f %f\n",Cf[0](0),Cf[0](1),Cf[1](0),Cf[1](1));
		// 1. RSR, 2. RSL, 3. LSR, 4. LSL
		int tmp_sel = 10;
		for (int i = 0; i < 2; i++){
			for (int f = 0; f < 2; f++){
				float tmp_dist = get_distance_to_next_waypoint(Ci[i](0), Ci[i](1), Cf[f](0), Cf[f](1));
				printf("[CircleChoices] %d : %f\n",(i*2)+f,(double)tmp_dist);
				if (tmp_dist < dist){
					tmp_sel = (i*2)+f;
					dist = tmp_dist;
					_Ci = Ci[i];
					_Cf = Cf[f];
				}
			}
		}
		if (tmp_sel == static_cast<int>(DubinsPathDecision::RightStraightRight))
		{
			sel = DubinsPathDecision::RightStraightRight;
			decision[0] = 1;
			decision[1] = 1;
		}
		if (tmp_sel == static_cast<int>(DubinsPathDecision::RightStraightLeft))
		{
			sel = DubinsPathDecision::RightStraightLeft;
			decision[0] = 1;
			decision[1] = 0;
		}
		if (tmp_sel == static_cast<int>(DubinsPathDecision::LeftStraightRight))
		{
			sel = DubinsPathDecision::LeftStraightRight;
			decision[0] = 0;
			decision[1] = 1;
		}
		if (tmp_sel == static_cast<int>(DubinsPathDecision::LeftStraightLeft))
		{
			sel = DubinsPathDecision::LeftStraightLeft;
			decision[0] = 0;
			decision[1] = 0;
		}

		return dist;
	}

	float MinTurnRadius()
	{
		// minumum turn distance for the UAV [Radius]
		return (powf(Vconst,2) / ((float)g * tanf(rollmax)));
	}

	float LineSegmentDistance(Vector2d start, Vector2d final)
	{
		return get_distance_to_next_waypoint(start(0), start(1), final(0), final(1));
	}

	float LineSegmentBearing(Vector2d start, Vector2d final)
	{
		return get_bearing_to_next_waypoint(start(0), start(1), final(0), final(1));
	}

	double getBearingDifferenceRad(double b1, double b2) {
		double r = fmod(b2 - b1, 2*pi);
		if (r < -pi)
			r += 2*pi;
		if (r >= pi)
			r -= 2*pi;
		return r;
	}

	double getBearingRad(double b1) {
		double r = fmod(b1, 2*pi);
		if (r < -pi)
			r += 2*pi;
		if (r >= pi)
			r -= 2*pi;
		return r;
	}

	Vector2d coordinates_to_local_xy(Vector2d start, Vector2d end)
	{
		float bearing = get_bearing_to_next_waypoint(start(0), start(1), end(0), end(1));
		float distance = get_distance_to_next_waypoint(start(0), start(1), end(0), end(1));
		double x_p = (double)distance * sin((double)bearing);
		double y_p = (double)distance * cos((double)bearing);

		return Vector2d(x_p, y_p);
	}

	void update_angle_limits(Vector2d start, Vector2d end, int index)
	{
		Vector2d cf = Vector2d(0, 1);
		Vector2d cp;
		double dotp, detp;

		cp = coordinates_to_local_xy(start, end);

		dotp = cf(0) * cp(0) + cf(1) * cp(1); // dot product between [x1, y1] and [x2, y2] cf dot cp
		detp = cf(1)*cp(0) - cp(1)*cf(0); // determinant
		double al = atan2(detp, dotp);
		angle_limits[index] = al;

		return;
	}

};

}

#endif // RUNWAYTAKEOFF_H
