/**
 * @file pid_control_module.c
 * Implementation of module for simple manual control using PID controllers
 *
 * @author Hrvoje Brezak <hrvoje.brezak@gmail.com>
 */

#include <px4_config.h> // Configuration flags used in code; either set NUTTX headers or configure PX4_POSIX for run
#include <px4_tasks.h>  // Handle task starting, scheduling and operation
#include <px4_posix.h>  // Include POSIX-like (UNIX-like) functions for virtual character devices
#include <unistd.h>     // POSIX UNIversal STanDard, define UNIX standard key words
#include <stdio.h>      // C/C++ standard input-output library
#include <poll.h>       // linux-based function for reading output readiness
#include <string.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h> // API for lightweight micro Object Request Broker (uORB) - handles communication between modules

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/actuator_controls.h>


extern "C" __EXPORT int pid_control_module_main(int argc, char *argv[]);

int pid_control_module_main(int argc, char *argv[])
{
    PX4_INFO("Hello sky!");

    static uint64_t last_run = 0;
    float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
    last_run = hrt_absolute_time();

    /* guard against too small (< 2ms) and too large (> 20ms) dt's */
    if (dt < 0.002f) {
        dt = 0.002f;

    } else if (dt > 0.02f) {
        dt = 0.02f;
    }

    /* subscribe to sensor_combined topic */
    int manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
    int state_sub_fd = orb_subscribe(ORB_ID(control_state));
    int motor_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));

    /* limit the update rate to 5 Hz */
    orb_set_interval(state_sub_fd, 200);
    orb_set_interval(motor_sub_fd, 200);

    /* advertise attitude topic */

    struct actuator_controls_s motor_output;


    memset(&motor_output, 0, sizeof(motor_output));


    orb_advert_t motor_output_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &motor_output);

    math::Vector<3> rates_previous;
    math::Vector<3> rates_integral;
    rates_previous.zero();
    rates_integral.zero();


    // file-descriptor-struct[]
    px4_pollfd_struct_t fds[] = {
        {   .fd = manual_sub_fd,    .events = POLLIN    },
        {   .fd = state_sub_fd,     .events = POLLIN    },
        {   .fd = motor_sub_fd,     .events = POLLIN    },
      /*{   .fd = other_sub_fd,     .events = POLLIN    }, */
    };

    int error_counter = 0;

    while (true) {
        /* wait for sensor update of 1 file descriptor for 1000 ms */
        int poll_ret = px4_poll(fds, 3, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("No new data!");
        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0){
                /* use a counter to prevent flooding (and slowing us down) */
                // warn about error 10 times at first occurence and then every 50th time
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }
            error_counter++;
        } else {

            if ((fds[0].revents & POLLIN) || (fds[1].revents & POLLIN)) {
                struct manual_control_setpoint_s manual;
                struct control_state_s state;

                orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &manual);
                /*PX4_INFO("RC controls: \t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                         (double)manual.x,
                         (double)manual.y,
                         (double)manual.z,
                         (double)manual.r);*/                

                orb_copy(ORB_ID(control_state), state_sub_fd, &state);
               /* PX4_INFO("Rates: \t%8.4f\t%8.4f\t%8.4f",
                         (double)state.roll_rate,
                         (double)state.pitch_rate,
                         (double)state.yaw_rate); */



                math::Vector<3> k_P;
                k_P(0) = 0.15f;
                k_P(1) = 0.15f;
                k_P(2) = 0.2f;

                math::Vector<3> k_D;
                k_D(0) = 0.003f;
                k_D(1) = 0.003f;
                k_D(2) = 0.0f;

                math::Vector<3> k_I;
                k_I(0) = 0.05f;
                k_I(1) = 0.05f;
                k_I(2) = 0.1f;


                math::Vector<3> rates_setpoint;
                rates_setpoint(0) = manual.y;
                rates_setpoint(1) = -manual.x;
                rates_setpoint(2) = manual.r;

                float thrust_setpoint = manual.z;

                math::Vector<3> current_rates;
                current_rates(0) = state.roll_rate;
                current_rates(1) = state.pitch_rate;
                current_rates(2) = state.yaw_rate;

                math::Vector<3> rates_error = rates_setpoint - current_rates;
                math::Vector<3> attitude_control = k_P.emult(rates_error) + k_D.emult(rates_previous - current_rates) / dt + k_I.emult(rates_integral);


                rates_previous = current_rates;

                for (int i = 0; i < 3; i++) {
                    float integral = rates_integral(i) + k_I(i) * rates_error(i) * dt;
                    rates_integral(i) = integral;
                }

                PX4_INFO("Output: \t%8.4f\t%8.4f\t%8.4f",
                         (double)attitude_control(0),
                         (double)attitude_control(1),
                         (double)attitude_control(2));

                motor_output.control[0] = attitude_control(0);
                motor_output.control[1] = attitude_control(1);
                motor_output.control[2] = attitude_control(2);
                motor_output.control[3] = thrust_setpoint;

               orb_publish(ORB_ID(actuator_controls_0), motor_output_pub_fd, &motor_output);

            }
            if (fds[2].revents & POLLIN) {
                struct actuator_controls_s motors;

                orb_copy(ORB_ID(actuator_controls_0), motor_sub_fd, &motors);
                PX4_INFO("Motors: \t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                         (double)motors.control[0],
                        (double)motors.control[1],
                        (double)motors.control[2],
                        (double)motors.control[3]);
            }

            /* more could be here:
             * if (fds[1].revents & POLLIN) {}
             */



        }
    }

    PX4_INFO("Exiting...");

    return 0;

}
