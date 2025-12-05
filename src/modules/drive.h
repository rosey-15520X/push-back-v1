#pragma once

#include "lemlib/api.hpp" 
#include "main.h"

using namespace pros;

// Define left motors
#define LEFT_FRONT_MOTOR_PORT 13
#define LEFT_MIDDLE_MOTOR_PORT 12
#define LEFT_BACK_MOTOR_PORT 11

// Define right motors
#define RIGHT_FRONT_MOTOR_PORT 10
#define RIGHT_MIDDLE_MOTOR_PORT 9
#define RIGHT_BACK_MOTOR_PORT 8

#define IMU_PORT 14

#define MOTOR_GEARSET v5::MotorGears::blue
#define TRACK_WIDTH 12.5
#define WHEEL_TYPE lemlib::Omniwheel::NEW_325
#define DRIVETRAIN_RPM 360
#define HORIZONTAL_DRIFT 2

MotorGroup left_motor_group({
    -LEFT_FRONT_MOTOR_PORT,
    -LEFT_MIDDLE_MOTOR_PORT,
    -LEFT_BACK_MOTOR_PORT
}, MOTOR_GEARSET, v5::MotorUnits::degrees);

MotorGroup right_motor_group({
    RIGHT_FRONT_MOTOR_PORT,
    RIGHT_MIDDLE_MOTOR_PORT,
    RIGHT_BACK_MOTOR_PORT
}, MOTOR_GEARSET, v5::MotorUnits::degrees);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              TRACK_WIDTH, // 14.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              DRIVETRAIN_RPM, // drivetrain rpm is 360
                              HORIZONTAL_DRIFT // horizontal drift is 2 (for now)
);

Imu imu(IMU_PORT);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(30, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);