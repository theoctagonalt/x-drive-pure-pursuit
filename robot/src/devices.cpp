#include "main.h"
#include "devices.h"
#include "lemlib/api.hpp"

pros::Motor front_right(1, pros::MotorGear::green);
pros::Motor back_left(3, pros::MotorGear::green);
pros::Motor back_right(4, pros::MotorGear::green);
pros::Motor front_left(10, pros::MotorGear::green);

pros::Rotation horizontal_tracking_sensor(2);
pros::Rotation vertical_tracking_sensor(5);

pros::IMU inertial_sensor(6);

pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::TrackingWheel vertical_tracking(&vertical_tracking_sensor, 2.75, 7.125);
lemlib::TrackingWheel horizontal_tracking(&horizontal_tracking_sensor, 2.75, -7.125);
lemlib::OdomSensors odom_sensors(&vertical_tracking, nullptr, &horizontal_tracking, nullptr, &inertial_sensor);

pros::MotorGroup left_motors({0, 1}, pros::MotorGears::green, pros::MotorEncoderUnits::degrees);
pros::MotorGroup right_motors({0, 1}, pros::MotorGears::green, pros::MotorEncoderUnits::degrees);

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 0, 0, 0, 0);
lemlib::ControllerSettings lateral_controller_settings(0, 0, 0, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller_settings(0, 0, 0, 0, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller_settings, angular_controller_settings, odom_sensors);
