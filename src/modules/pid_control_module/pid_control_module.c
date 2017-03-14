/**
 * @file pid_control_module.c
 * Implementation of module for simple manual control using PID controllers
 *
 * @author Hrvoje Brezak <hrvoje.brezak@gmail.com>
 */

#include <px4_config.h>     // Configuration flags used in code; either set NUTTX headers or configure PX4_POSIX for run
#include <px4_tasks.h>      // Handle task starting, scheduling and operation
#include <px4_posix.h>      // Include POSIX-like (UNIX-like) functions for virtual character devices
#include <unistd.h>         // POSIX UNIversal STanDard, define UNIX standard key words
#include <stdio.h>          // C/C++ standard input-output library
#include <poll.h>           // linux-based function for reading output readiness
#include <string.h>

#include <uORB/uORB.h>      // API for lightweight micro Object Request Broker (uORB) - handles communication between modules
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_direct.h>

#include <uORB/topics/actuator_controls.h> // Actuator control topics - mixer inputs.
                                           // Values published to these topics are the outputs of the vehicle control system, and are expected to be mixed and used to drive the actuators
                                           // (servos, speed controls, etc.) that operate the vehicle. Each topic can be published by a single controller.

#include <uORB/topics/control_state.h> // Contains info about vehicle position, velocity, acceleration, attitude quaternion, angular rates

__EXPORT int pid_control_module_main(int argc, char *argv[]);


int pid_control_module_main(int argc, char *argv[])
{
    PX4_INFO("Hello sky!");

    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    int rc_channels_fd = orb_subscribe(ORB_ID(rc_channels));

    int _ctrl_state_fd = orb_subscribe(ORB_ID(control_state));


    // limit the update rate to 5Hz
    orb_set_interval(sensor_sub_fd, 200);
    orb_set_interval(rc_channels_fd, 200);

    orb_set_interval(_ctrl_state_fd, 200);

    struct vehicle_attitude_s att;
    struct actuator_direct_s act_dir;
    struct actuator_controls_s _actuators;

    struct control_state_s _ctrl_state;

    // Declare things in which I plan to publish
    memset(&att, 0, sizeof(att));
    memset(&act_dir, 0, sizeof(act_dir));
//    orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
    orb_advert_t act_dir_pub = orb_advertise(ORB_ID(actuator_direct), &act_dir);

    memset(&_actuators, 0, sizeof(_actuators));
    orb_advert_t _actuators_pub = orb_advertise(ORB_ID(actuator_controls), &_actuators);

    // Define file descriptor structure
    px4_pollfd_struct_t fds[] = {
        {   .fd = sensor_sub_fd,    .events = POLLIN},
        {   .fd = rc_channels_fd,   .events = POLLIN},
        {   .fd = _ctrl_state_fd,   .events = POLLIN},
    };


    while (true) {

        // wait for sensor update of 1 file descriptor for 1000 ms ( 1 second)
        int poll_ret = px4_poll(fds, 3, 1000);

        if (poll_ret == 0) {
            // none of our providers is giving us data
            PX4_ERR ("Got no data within a second");

        } else if (poll_ret < 0) {
            // this is seriously bad
            PX4_ERR("ERROR return value from poll(): %d", poll_ret);

        } else {
            if (fds[0].revents & POLLIN) {
                // obtain data for the first file descriptor
                struct sensor_combined_s raw;
                // copy sensor raw data into local buffer
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

//                PX4_INFO("Accelerometer: \t%8.4f \t%8.4f \t%8.4f",
//                         (double)raw.accelerometer_m_s2[0],
//                         (double)raw.accelerometer_m_s2[1],
//                         (double)raw.accelerometer_m_s2[2]);


//            att.rollspeed = (double)raw.accelerometer_m_s2[0];
//            att.pitchspeed = (double)raw.accelerometer_m_s2[1];
//            att.yawspeed = (double)raw.accelerometer_m_s2[2];
//            orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);


                act_dir.values[0] = 5;
                act_dir.values[2] = 10;
                orb_publish(ORB_ID(actuator_direct), &act_dir_pub, &act_dir);
            }

            if (fds[1].revents & POLLIN) {
                struct rc_channels_s rc_raw;
                orb_copy(ORB_ID(rc_channels), rc_channels_fd, &rc_raw);

                PX4_INFO("Sticks: throttle \t%8.4f, roll \t%8.4f, pitch \t%8.4f, yaw \t%8.4f",
                        (double)rc_raw.channels[2],
                        (double)rc_raw.channels[0],
                        (double)rc_raw.channels[1],
                        (double)rc_raw.channels[3]);
            }

            if (fds[2].revents & POLLIN) {
                orb_copy(ORB_ID(control_state), _ctrl_state_fd, &_ctrl_state);

                PX4_INFO("Roll rate: %8.4f, pitch rate: %8.4f, yaw rate: %8.4f",
                         (double)_ctrl_state.roll_rate,
                         (double)_ctrl_state.pitch_rate,
                         (double)_ctrl_state.yaw_rate);
            }
        }

        // Publish to actuators
        _actuators.control[0] = 0; // INDEX_ROLL
        _actuators.control[1] = 0; // INDEX_PITCH
        _actuators.control[2] = 0; // INDEX_YAW
        _actuators.control[3] = 0; // INDEX_THROTTLE

        orb_publish(ORB_ID(actuator_controls), &_actuators_pub, &_actuators);





    }

    return OK;
}
