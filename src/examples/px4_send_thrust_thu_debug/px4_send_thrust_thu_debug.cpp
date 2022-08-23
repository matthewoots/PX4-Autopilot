/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_mavlink_debug.cpp
 * Debug application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>

#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/battery_status.h>


extern "C" __EXPORT int px4_send_thrust_thu_debug_main(int argc, char *argv[]);

int px4_send_thrust_thu_debug_main(int argc, char *argv[])
{
	printf("Hello thrust!\n");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	int batt_sub_fd = orb_subscribe(ORB_ID(battery_status));

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);
	orb_set_interval(batt_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = batt_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	/* advertise debug vect */
	struct debug_vect_s dbg_vect;
	strncpy(dbg_vect.name, "vel3D", 10);
	dbg_vect.x = 1.0f;
	dbg_vect.y = 2.0f;
	dbg_vect.z = 3.0f;
	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

	/* advertise named debug value */
	struct debug_key_value_s dbg_key;
	strncpy(dbg_key.key, "velx", 10);
	dbg_key.value = 0.0f;
	orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

	/* advertise indexed debug value */
	struct debug_value_s dbg_ind;
	dbg_ind.ind = 42;
	dbg_ind.value = 0.5f;
	orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);


	int value_counter = 0;
	int index_counter = 0;

	int error_counter = 0;

	struct vehicle_rates_setpoint_s v_rate_sp;
	struct battery_status_s battery;

	while (value_counter < 10) {

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_rates_setpoint), sensor_sub_fd, &v_rate_sp);
				// PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				// 	 (double)v_rate_sp.thrust_body[0],
				// 	 (double)v_rate_sp.thrust_body[1],
				// 	 (double)v_rate_sp.thrust_body[2]);
			}

			if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(battery_status), batt_sub_fd, &battery);
				// PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				// 	 (double)v_rate_sp.thrust_body[0],
				// 	 (double)v_rate_sp.thrust_body[1],
				// 	 (double)v_rate_sp.thrust_body[2]);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}

		uint64_t timestamp_us = hrt_absolute_time();

		/* send one named value */
		dbg_key.value = battery.voltage_v;
		dbg_key.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

		/* send one indexed value */
		// can be hacked to send command
		dbg_ind.ind = 42+index_counter;
		// dbg_ind.value = 0.5f * value_counter;
		dbg_ind.value = v_rate_sp.thrust_body[2];
		dbg_ind.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);

		/* send one vector */
		dbg_vect.x = 1.0f * value_counter;
		dbg_vect.y = battery.voltage_v;
		dbg_vect.z = v_rate_sp.thrust_body[2];
		dbg_vect.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);

		// warnx("sending thrust to mavlink..");
				PX4_INFO("throttle is:\t%8.4f\n voltage is: \t%8.4f",
				// 	 (double)v_rate_sp.thrust_body[0],
				// 	 (double)v_rate_sp.thrust_body[1],
					 (double)v_rate_sp.thrust_body[2],
					 (double)dbg_key.value);
		value_counter++;
		index_counter++;
		px4_usleep(500000);
	}

	return 0;
}
