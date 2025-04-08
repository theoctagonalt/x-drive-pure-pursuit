#include "main.h"
#include "lemlib/api.hpp"

#ifndef DEVICES_H_
#define DEVICES_H_

extern pros::Controller master;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor back_right;
extern pros::Motor front_left;

extern pros::Rotation horizontal_tracking_sensor;
extern pros::Rotation vertical_tracking_sensor;

extern pros::IMU inertial_sensor;

extern lemlib::Chassis chassis;

#endif