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

#include "mc_openloop_test.h"

#include <parameters/param.h>

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>

#include <math.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/openloop_input.h>

#define INITIAL_DELAY 5 // initial delay (in seconds)
#define IMPULSE_DURATION 0.25 // duration of the impulse (in seconds)
#define IMPULSE_VAL 0.1 // amplitude of the impulse
#define IMPULSE_WAIT 1.0 // duration between the 2 impulses (in seconds)
#define IMPULSE_GAP 1.0 // delay between subsequent openloop tests (in seconds)

int MulticopterOpenloopTest::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Module to run open-loop multicopter commands.
### Implementation
Publishes setpoints for thrust, aileron, elevator and rudder.
### Examples
CLI usage example:
$ module start
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("mc_openloop_test", "modules");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    PRINT_MODULE_USAGE_PARAM_FLOAT('d', IMPULSE_DURATION, 0.1, 2.0, "Impulse duration (in seconds)", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('v', IMPULSE_VAL, 0.01, 0.5, "Impulse value (normalised)", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('w', IMPULSE_WAIT, 0.1, 3.0, "Waiting time for the negative impulse (in seconds)", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('g', IMPULSE_GAP, 0.1, 10.0, "Gap between an impulse set (in seconds)", true);

    return 0;
}

int MulticopterOpenloopTest::print_status()
{
    PX4_INFO("Running");

    return 0;
}

int MulticopterOpenloopTest::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    return print_usage("unknown command");
}


int MulticopterOpenloopTest::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("mc_openloop_test",
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

MulticopterOpenloopTest *MulticopterOpenloopTest::instantiate(int argc, char *argv[])
{
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    float duration = IMPULSE_DURATION;
    float val = IMPULSE_VAL;
    float wait = IMPULSE_WAIT;
    float gap = IMPULSE_GAP;

    while ((ch = px4_getopt(argc, argv, "d:v:w:g", &myoptind, &myoptarg)) != EOF) {
        char *ep;
        float param_val = strtof(myoptarg, &ep);

        switch(ch) {
        case 'd':
            duration = param_val;
            break;
        case 'v':
            val = param_val;
            break;
        case 'w':
            wait = param_val;
            break;
        case 'g':
            gap = param_val;
            break;
        }
    }

    MulticopterOpenloopTest *instance = new MulticopterOpenloopTest(duration, val, wait, gap);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

MulticopterOpenloopTest::MulticopterOpenloopTest(float duration, float value, float wait, float gap) : ModuleParams(nullptr)
{
    _duration = duration;
    _val = value;
    _wait = wait;
    _gap = gap;

    // subscribe to local position - used to check the yaw angle of the vehicle
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    // subscribe to home position - used to check the yaw angle of the vehicle
    _home_position_sub = orb_subscribe(ORB_ID(home_position));
}

void MulticopterOpenloopTest::run()
{
    bool start = false;
    bool step_pos_up = false;
    bool step_pos_down = false;
    bool step_neg_up = false;
    bool step_neg_down = false;

    hrt_abstime step_time = 0;

    px4_pollfd_struct_t fds[1];
    fds[0].fd = _vehicle_local_position_sub;
    fds[0].events = POLLIN;

    // initialize parameters
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    parameters_update(parameter_update_sub, true);

    // get home position
    home_position_poll();

    // advertise to step_input
    struct openloop_input_s input;
    memset(&input, 0, sizeof(input));
    orb_advert_t openloop_input_pub = orb_advertise(ORB_ID(openloop_input), &input);

    // set everything to 0
    input.timestamp = hrt_absolute_time();

    input.thrust = 0;
    input.roll = 0;
    input.pitch = 0;
    input.yaw = 0;

    input.openloop_thrust = false;
    input.openloop_roll = false;
    input.openloop_pitch = false;
    input.openloop_yaw = false;

    float amplitude = NAN;
    float sign = 1;
    float yaw_diff;
    step_time = hrt_absolute_time();

    while (!should_exit()) {
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

            if(!start && hrt_absolute_time() - step_time >= INITIAL_DELAY * 1e6) { // initial delay
                start = true;
                step_time = hrt_absolute_time();
            } else if(start && !step_pos_up) { // first impulse rise
                // Get difference between yaw and home-yaw
                yaw_diff = _local_pos.yaw - _home_pos.yaw;
                // Handle edge-case where e.g. home-yaw might be -179 and yaw might be 179. Should then be -2 difference
                if(yaw_diff > (float)M_PI)
                    yaw_diff -= 2.0f*(float)M_PI;
                else if(yaw_diff < -(float)M_PI)
                    yaw_diff += 2.0f*(float)M_PI;

                if(yaw_diff >= 0)
                    sign = -(1 + fabsf(yaw_diff)/((float)M_PI));
                else
                    sign = 1 + fabsf(yaw_diff)/((float)M_PI);
                amplitude = sign * (float)_val;
                sign *= -1;
                step_pos_up = true;
                step_time = hrt_absolute_time();
            } else if(step_pos_up && !step_pos_down && hrt_absolute_time() - step_time >= _duration * 1e6f) { // first impulse fall
                amplitude = 0;
                step_pos_down = true;
                step_time = hrt_absolute_time();
            } else if(step_pos_up && step_pos_down && !step_neg_up && hrt_absolute_time() - step_time >= _wait * 1e6f) { // second impulse rise
                amplitude = sign * (float)_val;
                step_neg_up = true;
                step_time = hrt_absolute_time();
            } else if(step_pos_up && step_pos_down && step_neg_up && !step_neg_down && hrt_absolute_time() - step_time >= _duration * 1e6f) { // second impulse fall
                amplitude = 0;
                step_neg_down = true;
                step_time = hrt_absolute_time();
            } else if(step_pos_up && step_pos_down && step_neg_up && step_neg_down && hrt_absolute_time() - step_time >= _gap * 1e6f) {
                step_pos_up = false;
                step_pos_down = false;
                step_neg_up = false;
                step_neg_down = false;

                start = true;
                step_time = hrt_absolute_time();
            }

            if(PX4_ISFINITE(amplitude)) {
                // Set openloop input
                input.yaw = amplitude;
                input.openloop_yaw = true;
            } else {
                // Disable openloop input
                input.thrust = 0;
                input.roll = 0;
                input.pitch = 0;
                input.yaw = 0;

                input.openloop_thrust = false;
                input.openloop_roll = false;
                input.openloop_pitch = false;
                input.openloop_yaw = false;
            }

            input.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(openloop_input), openloop_input_pub, &input);
            parameters_update(parameter_update_sub);
        }
    }

    // Disable openloop before unsubscribe
    input.thrust = 0;
    input.roll = 0;
    input.pitch = 0;
    input.yaw = 0;

    input.openloop_thrust = false;
    input.openloop_roll = false;
    input.openloop_pitch = false;
    input.openloop_yaw = false;

    input.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(openloop_input), openloop_input_pub, &input);
    orb_unsubscribe(parameter_update_sub);
}

void MulticopterOpenloopTest::home_position_poll()
{
    orb_copy(ORB_ID(home_position), _home_position_sub, &_home_pos);
}

void MulticopterOpenloopTest::parameters_update(int parameter_update_sub, bool force)
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


int mc_openloop_test_main(int argc, char *argv[])
{
    return MulticopterOpenloopTest::main(argc, argv);
}
