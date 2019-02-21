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

#include <drivers/drv_hrt.h>

#include <string.h>
#include <math.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_command.h>

#define STEP_TYPE ROLL_RATE
#define STEP_VAL 1.0f
#define STEP_DURATION NAN // duration of the step (in seconds). set to NAN if step should not change value again

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
$ module start -t vx -v 1 -d 10 -h
    - Give a step input of 1 for VX with a duration of 10 seconds after hover is reached

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_control_test", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_PARAM_FLOAT('d', STEP_DURATION, 0.0, 1e6f, "Step duration (in seconds)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('v', STEP_VAL, 0.0, 1e6f, "Step value", false);
	PRINT_MODULE_USAGE_PARAM_STRING('t', "roll_rate", "<roll_rate:pitch_rate:yaw_rate:roll:pitch:yaw:vx:vy:vz:n:e:d>", "Select the step type", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Wait for hover at takeoff height to start step input", true);

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
		return -1;
	}

	return 0;
}

MulticopterControlTest *MulticopterControlTest::instantiate(int argc, char *argv[])
{
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    enum StepType step_type = STEP_TYPE;
    float val = STEP_VAL;
    float duration = STEP_DURATION;
    bool hover = false;

    while ((ch = px4_getopt(argc, argv, "t:v:d:h", &myoptind, &myoptarg)) != EOF) {
        switch(ch) {
        case 'd':
            duration = strtof(myoptarg, nullptr);
            break;
        case 'v':
            val = strtof(myoptarg, nullptr);
            break;
        case 't':
            if(strcmp(myoptarg, "roll_rate") == 0)
                step_type = ROLL_RATE;
            else if(strcmp(myoptarg, "pitch_rate") == 0)
                step_type = PITCH_RATE;
            else if(strcmp(myoptarg, "yaw_rate") == 0)
                step_type = YAW_RATE;
            else if(strcmp(myoptarg, "roll") == 0)
                step_type = ROLL;
            else if(strcmp(myoptarg, "pitch") == 0)
                step_type = PITCH;
            else if(strcmp(myoptarg, "yaw") == 0)
                step_type = YAW;
            else if(strcmp(myoptarg, "vx") == 0)
                step_type = VX;
            else if(strcmp(myoptarg, "vy") == 0)
                step_type = VY;
            else if(strcmp(myoptarg, "vz") == 0)
                step_type = VZ;
            else if(strcmp(myoptarg, "n") == 0)
                step_type = N;
            else if(strcmp(myoptarg, "e") == 0)
                step_type = E;
            else if(strcmp(myoptarg, "d") == 0)
                step_type = D;
            break;
        case 'h':
            hover = true;
            break;
        }
    }

	MulticopterControlTest *instance = new MulticopterControlTest(step_type, val, duration, hover);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

MulticopterControlTest::MulticopterControlTest(enum StepType step_type, float val, float duration, bool hover) : ModuleParams(nullptr),  _step_type(step_type), _val(val), _duration(duration), _hover(hover)
{
    // subscribe to global position - used for takeoff
    _vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

    // subscribe to local position - used to check when vehicle is in stable hover
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
}

void MulticopterControlTest::run()
{
	bool ready = false; // indicates whether testing can start
	bool commanded = false; // indicates whether a step input is commanded - only need to command once
	bool step_down = false; // used if the step has a finite duration

	float takeoff_height; // the height at which the vehicle will hover when a takeoff is commanded

	hrt_abstime step_time = 0; // used to store the time that the step was given

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _vehicle_local_position_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	// advertise to attitude_setpoint
	struct vehicle_rates_setpoint_s att_sp;
    memset(&att_sp, 0, sizeof(att_sp));
    orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &att_sp);

    // // advertise to control_mode
	// struct vehicle_control_mode_s mode;
    // memset(&mode, 0, sizeof(mode));
    // orb_advert_t mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &mode);

    // // publish to control_mode
    // mode.flag_control_offboard_enabled = true;
    // orb_publish(ORB_ID(vehicle_control_mode), mode_pub, &mode);

    // set everything to 0
    att_sp.roll = 0;
    att_sp.pitch = 0;
    att_sp.yaw = 0;

	while (!should_exit()) {

	    // Check if testing can start. Will only start when vehicle is in stable hover
        if(!ready) {
            // wait for up to 1000ms for data
            int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

            if (pret == 0) {
                // Timeout: let the loop run anyway, don't do `continue` here

            } else if (pret < 0) {
                // this is undesirable but not much we can do
                PX4_ERR("poll error %d, %d", pret, 0);
                px4_usleep(50000);
                continue;

            } else if (fds[0].revents & POLLIN) {
                // obtain current position
                orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_local_pos);

                // poll vehicle global position.
                vehicle_global_position_poll();

                // obtain takeoff height
                param_get(param_find("MIS_TAKEOFF_ALT"), &takeoff_height);
                takeoff_height *= -1; // height in DOWN position (negative)

                // check whether in stable hover at takeoff height
                if(fabsf(_local_pos.z - takeoff_height) <= (float)(0.01) || ready)
                {
                    ready = true;

                    // advertise to control_mode
                    struct vehicle_control_mode_s mode;
                    memset(&mode, 0, sizeof(mode));
                    orb_advert_t mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &mode);

                    // publish to control_mode
                    mode.flag_control_offboard_enabled = true;
                    orb_publish(ORB_ID(vehicle_control_mode), mode_pub, &mode);
                }
            }

            parameters_update(parameter_update_sub);
        } else if(!_hover) {
            ready = true;
        }

        if(ready && !commanded) {
            // command step
            command_step(_step_type, _val, &att_sp, att_sp_pub);

            // don't publish another step input
            // commanded = true;
            // step_time = hrt_absolute_time();
        } else if(commanded && PX4_ISFINITE(_duration) && !step_down) {
            if(hrt_absolute_time() - step_time >= (float)_duration * 1e6f) {
                // command down step
                command_step(_step_type, 1e-6f, &att_sp, att_sp_pub); // Set back to 0. Well... Really close to 0

                // don't publish another step input
                step_down = true;
            }
        }
	}

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

void MulticopterControlTest::command_step(enum StepType step_type, float val, struct vehicle_rates_setpoint_s *att_sp, orb_advert_t att_sp_pub)
{
    switch(step_type)
    {
    case ROLL_RATE:
        att_sp->roll = val*(float)M_PI/180.0f;
        break;
    case PITCH_RATE:
        att_sp->pitch = val*(float)M_PI/180.0f;
        break;
    case YAW_RATE:
        att_sp->yaw = val*(float)M_PI/180.0f;
        break;
    case ROLL:
        break;
    case PITCH:
        break;
    case YAW:
        break;
    case VX:
        break;
    case VY:
        break;
    case VZ:
        break;
    case N:
        break;
    case E:
        break;
    case D:
        break;
    }
    
    PX4_INFO("Step commanded!");
    orb_publish(ORB_ID(vehicle_rates_setpoint), att_sp_pub, &att_sp);
}

int mc_control_test_main(int argc, char *argv[])
{
	return MulticopterControlTest::main(argc, argv);
}
