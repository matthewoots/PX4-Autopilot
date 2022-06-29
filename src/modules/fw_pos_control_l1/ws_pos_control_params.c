/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file ws_pos_control_params.c
 *
 * Parameters defined by the ws position control task
 *
 * @author Matthew Woo <matthewoots@gmail.com>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

 /**
 * Acceleration / Load limit
 *
 * Acceleration limit for platform
 *
 * @unit deg
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ACC_LIM, 1.000f);

/**
 * Latitude 1
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LAT_1, 0.000000000000000f);

/**
 * Longitude 1
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LONG_1, 0.000000000000000f);

/**
 * Altitude 1
 *
 * Waypoint Altitude
 *
 * @unit m
 * @min 10.0
 * @max 60.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ALT_1, 10.0f);

/**
 * Latitude 2
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LAT_2, 0.000000000000000f);

/**
 * Longitude 2
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LONG_2, 0.000000000000000f);

/**
 * Altitude 2
 *
 * Waypoint Altitude
 *
 * @unit m
 * @min 10.0
 * @max 60.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ALT_2, 10.0f);

/**
 * Latitude 3
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LAT_3, 0.000000000000000f);

/**
 * Longitude 3
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LONG_3, 0.000000000000000f);

/**
 * Altitude 3
 *
 * Waypoint Altitude
 *
 * @unit m
 * @min 10.0
 * @max 60.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ALT_3, 10.0f);

/**
 * Latitude 4
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LAT_4, 0.000000000000000f);

/**
 * Longitude 4
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LONG_4, 0.000000000000000f);

/**
 * Altitude 4
 *
 * Waypoint Altitude
 *
 * @unit m
 * @min 10.0
 * @max 60.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ALT_4, 10.0f);

/**
 * Latitude 5
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LAT_5, 0.000000000000000f);

/**
 * Longitude 5
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LONG_5, 0.000000000000000f);

/**
 * Altitude 5
 *
 * Waypoint Altitude
 *
 * @unit m
 * @min 10.0
 * @max 60.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ALT_5, 10.0f);

/**
 * Latitude Precision Landing
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LAT_P_LAND, 0.000000000000000f);

/**
 * Longitude Precision Landing
 *
 * Waypoint latitude
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 15
 * @increment 0.000000000000001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_LONG_P_LAND, 0.000000000000000f);

/**
 * Elevation Precision Landing
 *
 * Waypoint Elevation
 *
 * @unit m
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(WS_ALT_P_LAND, 0.001f);
