/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "mc_control_test.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <math.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/step_input.h>
#include <uORB/topics/vehicle_command.h>

int MulticopterControlTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to test multicopter controllers.
Commands step responses for various controllers.

### Implementation
Publishes setpoints for various controllers.

### Examples
CLI usage example:
$ module start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_control_test", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int MulticopterControlTest::print_status()
{
	PX4_INFO("Running");

	return 0;
}

int MulticopterControlTest::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}


int MulticopterControlTest::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mc_control_test",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MulticopterControlTest *MulticopterControlTest::instantiate(int argc, char *argv[])
{
	MulticopterControlTest *instance = new MulticopterControlTest();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

MulticopterControlTest::MulticopterControlTest() : ModuleParams(nullptr)
{
    // subscribe to vehicle command - used to check for takeoff
    _vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));

    // subscribe to global position - used for takeoff
    _vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

    // subscribe to local position - used to check when vehicle is in stable hover
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    // advertise vehicle position setpoint triplet
    memset(&_rep, 0, sizeof(_rep));
    _pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_rep);
}

void MulticopterControlTest::run()
{
	bool ready = false; // indicates whether testing can start
	bool commanded = false; // indicates whether a step input is commanded - only need to command once

	float takeoff_height; // the height at which the vehicle will hover when a takeoff is commanded

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _vehicle_local_position_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	// advertise to step_input
	struct step_input_s step;
    memset(&step, 0, sizeof(step));
    orb_advert_t step_input_pub = orb_advertise(ORB_ID(step_input), &step);

    // set everything to 0
    step.valid = false;
    step.x = NAN;
    step.y = NAN;
    step.z = NAN;
    step.vx = NAN;
    step.vy = NAN;
    step.vz = NAN;
    step.roll = NAN;
    step.pitch = NAN;
    step.yaw = NAN;
    step.rollspeed = NAN;
    step.pitchspeed = NAN;
    step.yawspeed = NAN;
    step.control_roll = false;
    step.control_pitch = false;
    step.control_yaw = false;

	while (!should_exit()) {

	    // Check if testing can start. Will only start when vehicle is in stable hover
        if(!ready) {
            // wait for up to 1000ms for data
            int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

            if (pret == 0) {
                // Timeout: let the loop run anyway, don't do `continue` here

            } else if (pret < 0) {
                // this is undesirable but not much we can do
                PX4_ERR("poll error %d, %d", pret, errno);
                usleep(50000);
                continue;

            } else if (fds[0].revents & POLLIN) {
                // obtain current position
                orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_local_pos);

                // poll vehicle global position.
                vehicle_global_position_poll();

                // poll vehicle command.
                vehicle_command_poll();

                // obtain takeoff height
                param_get(param_find("MIS_TAKEOFF_ALT"), &takeoff_height);
                takeoff_height *= -1; // height in DOWN position (negative)

                // check whether in stable hover at takeoff height
                if(fabsf(_local_pos.z - takeoff_height) <= (float)(0.1) || ready)
                {
                    ready = true;
                }
            }
        }

        if(ready && !commanded) {
            // command step
            step.vx = 1;//M_PI/180.0;
            step.control_pitch = true;

            step.valid = true;
            orb_publish(ORB_ID(step_input), step_input_pub, &step);

            // don't publish another step input
            commanded = true;
        } else if(!ready) {
            step.valid = true;
            orb_publish(ORB_ID(step_input), step_input_pub, &step);
        }

		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(_vehicle_command_sub);
	orb_unsubscribe(_vehicle_global_position_sub);
	orb_unsubscribe(_vehicle_local_position_sub);
	orb_unsubscribe(parameter_update_sub);
}

void MulticopterControlTest::vehicle_global_position_poll()
{
    /* vehicle_command updated */
    bool updated;
    orb_check(_vehicle_global_position_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &_global_pos);
    }
}

void MulticopterControlTest::vehicle_command_poll()
{
    /* vehicle_command updated */
    bool updated;
    orb_check(_vehicle_command_sub, &updated);

    if (updated) {
        struct vehicle_command_s cmd;
        orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

        if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
            // store current position as previous position and goal as next
            _rep.previous.yaw = _global_pos.yaw;
            _rep.previous.lat = _global_pos.lat;
            _rep.previous.lon = _global_pos.lon;
            _rep.previous.alt = _global_pos.alt;

            _rep.current.loiter_radius = 0;
            _rep.current.loiter_direction = 1;
            _rep.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

            _rep.current.yaw = _local_pos.yaw;
            _rep.previous.valid = false;

            if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
                _rep.current.lat = (cmd.param5 < 1000) ? cmd.param5 : cmd.param5 / (double)1e7;
                _rep.current.lon = (cmd.param6 < 1000) ? cmd.param6 : cmd.param6 / (double)1e7;

            } else {
                // If one of them is non-finite, reset both
                _rep.current.lat = NAN;
                _rep.current.lon = NAN;
            }

            _rep.current.alt = cmd.param7;

            _rep.current.valid = true;
            _rep.next.valid = false;

            // CMD_NAV_TAKEOFF is acknowledged by commander
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_rep);
        }
    }
}

void MulticopterControlTest::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int mc_control_test_main(int argc, char *argv[])
{
	return MulticopterControlTest::main(argc, argv);
}
