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
 * @file swept.c
 * Swept wing application test functions for PX4 autopilot
 *
 * @author Matthew Woo <matthewoots@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/swept_mode.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <px4_platform_common/module_params.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

static void	usage(const char *reason);
__BEGIN_DECLS
__EXPORT void 	swept_main(int argc, char *argv[]);
__END_DECLS

extern "C" __EXPORT void swept_main(int argc, char *argv[]);

struct swept_mode_s _swept_mode;
// actuator_controls_s::INDEX_SWEPT
// actuator_controls_s::INDEX_RETRACT

static void
usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		This command is used to test the swept wing capability of the UAV (using control group 7).
		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("swept", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("forward", "Forward motion (for flaring phase)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("backward", "Backward motion (for diving phase)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("step", "Given value to sweep");
}

void swept_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage(nullptr);
		PX4_INFO("Incorrect Input, swept [COMMAND] [OPTIONS]");
		return ;
	}
	PX4_INFO("#Swept Start Time: %.2lf", (double)hrt_absolute_time());
	PX4_INFO("#Swept Wing Test");
	/* initialize parameters */

	const char *command = argv[1];
	if (strcmp(command, "forward") && strcmp(command, "backward") && strcmp(command, "reset")) return;

	/* subscribe to actuator_control topic */
	int _swept_mode_sub_fd = orb_subscribe(ORB_ID(swept_mode));
	/* limit the update rate to 5 Hz */
	orb_set_interval(_swept_mode_sub_fd, 200);
	/* obtained data for the first file descriptor */
	/* copy control raw data into local buffer */
	orb_copy(ORB_ID(swept_mode), _swept_mode_sub_fd, &_swept_mode);
	orb_advert_t _swept_mode_pub = orb_advertise(ORB_ID(swept_mode), &_swept_mode);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = _swept_mode_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0, threshold = 1;
	for (int i = 0; i < threshold; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms */
		int poll_ret = px4_poll(fds, 1, 1000);

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
				uORB::Subscription _swept_mode_sub{ORB_ID(swept_mode)};
				_swept_mode_sub.update(&_swept_mode);

				if (!strcmp(command, "forward")) {
					/* We take the current time from control output */
					_swept_mode.timestamp = hrt_absolute_time();

					_swept_mode.control[swept_mode_s::INDEX_SWEPT] = -1.0f;
					_swept_mode.control[swept_mode_s::INDEX_RETRACT] = 1.0f;

					/* Publication of trim condition */
					orb_publish(ORB_ID(swept_mode), _swept_mode_pub, &_swept_mode);
					PX4_INFO("#Swept Forward ... ");
				}

				if (!strcmp(command, "backward")) {
					/* We take the current time from control output */
					_swept_mode.timestamp = hrt_absolute_time();

					_swept_mode.control[swept_mode_s::INDEX_SWEPT] = 1.0f;
					_swept_mode.control[swept_mode_s::INDEX_RETRACT] = 1.0f;

					/* Publication of trim condition */
					orb_publish(ORB_ID(swept_mode), _swept_mode_pub, &_swept_mode);
					PX4_INFO("#Swept Backward ... ");
				}

				if (!strcmp(command, "reset")) {
					/* We take the current time from control output */
					_swept_mode.timestamp = hrt_absolute_time();

					_swept_mode.control[swept_mode_s::INDEX_SWEPT] = -1.0f;
					_swept_mode.control[swept_mode_s::INDEX_RETRACT] = -1.0f;

					/* Publication of trim condition */
					orb_publish(ORB_ID(swept_mode), _swept_mode_pub, &_swept_mode);
					PX4_INFO("#Swept Reset ... ");
				}

				// if (!strcmp(command, "step")) {
				// 	/* Return if not started */
				// 	if (!_sys_id.start_now)	{
				// 		PX4_INFO("#sysID Nothing to stop");
				// 		reset();
				// 		return;
				// 	}
				// 	/* Reset sys_id message */
				// 	reset();
				// 	_sys_id.initialised = false;

				// 	/* Publication of reset condition */
				// 	orb_publish(ORB_ID(sys_id), _sys_id_pub, &_sys_id);
				// 	PX4_INFO("#sysID Stop");
				// }
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}
	PX4_INFO("#Exiting");
	return;
}
