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

#include <uORB/uORB.h> // API for lightweight micro Object Request Broker (uORB) - handles communication between modules

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int pid_control_module_main(int argc, char *argv[]);

int pid_control_module_main(int argc, char *argv[])
{
    PX4_INFO("Hello sky!");

    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined)); // sensor_subscription_filedescriptor
                                                                // this is a topic handle
    int manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
    int state_sub_fd = orb_subscribe(ORB_ID(control_state));
    int motor_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));

    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 200);
    orb_set_interval(state_sub_fd, 200);
    orb_set_interval(motor_sub_fd, 200);

    /* advertise attitude topic */
    struct vehicle_attitude_s att;
    struct actuator_controls_s motor_output;

    memset(&att, 0, sizeof(att));
    memset(&motor_output, 0, sizeof(motor_output));

    orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
    orb_advert_t motor_output_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &motor_output);

    // file-descriptor-struct[]
    px4_pollfd_struct_t fds[] = {
        {   .fd = sensor_sub_fd,    .events = POLLIN    },
        {   .fd = manual_sub_fd,    .events = POLLIN    },
        {   .fd = state_sub_fd,     .events = POLLIN    },
        {   .fd = motor_sub_fd,     .events = POLLIN    },
      /*{   .fd = other_sub_fd,     .events = POLLIN    }, */
    };

    int error_counter = 0;

    while (true) {
        /* wait for sensor update of 1 file descriptor for 1000 ms */
        int poll_ret = px4_poll(fds, 4, 1000);

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
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file desctriptor */
                struct sensor_combined_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                /* PX4_INFO("Accelerometer: \t%8.4f\t%8.4f\t%8.4f",
                         (double)raw.accelerometer_m_s2[0],
                         (double)raw.accelerometer_m_s2[1],
                         (double)raw.accelerometer_m_s2[2]); */
            }
            if (fds[1].revents & POLLIN) {
                struct manual_control_setpoint_s manual;

                orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &manual);
                /*PX4_INFO("RC controls: \t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                         (double)manual.x,
                         (double)manual.y,
                         (double)manual.z,
                         (double)manual.r);*/

            }
            if (fds[2].revents & POLLIN) {
                struct control_state_s state;

                orb_copy(ORB_ID(control_state), state_sub_fd, &state);
               /* PX4_INFO("Rates: \t%8.4f\t%8.4f\t%8.4f",
                         (double)state.roll_rate,
                         (double)state.pitch_rate,
                         (double)state.yaw_rate); */
            }
            if (fds[3].revents & POLLIN) {
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

        /* set att and publish this information for other apps */
        att.rollspeed = 100;
        att.pitchspeed = 1000;
        att.yawspeed = 1800;

        orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);

        motor_output.control[3] = 0.9;

       orb_publish(ORB_ID(actuator_controls_0), motor_output_pub_fd, &motor_output);
    }

    PX4_INFO("Exiting...");

    return 0;

}
