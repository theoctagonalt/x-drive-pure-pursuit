#include "main.h"
#include "devices.h"
#include "lemlib/api.hpp"
#include <math.h>
#include <cmath>

float distance_btwn_points(std::pair<float, float>point1, std::pair<float, float>point2){
	float dist = sqrt(pow(point2.first-point1.first,2) + pow(point2.second+point1.second,2));
	return dist; 
}

int sgn (float num){
	if (num>=0){
		return 1;
	} else {
		return -1;
	}
}

std::pair<float, float> findPoint(std::pair<float, float>cur_pos, std::pair<float, float>p1, std::pair<float, float>p2, float lookAheadDist){
	std::cout << "point 1 " << p2.first << ", " << p2.second << std::endl;
	p1 = std::make_pair(p1.first-cur_pos.first, p1.second-cur_pos.second);
  p2 = std::make_pair(p2.first-cur_pos.first, p2.second-cur_pos.second);
	bool intersectFound = false;
	bool sol1V = false;
	bool sol2V = false;
	float dx = p2.first - p1.first;
	float dy = p2.second - p1.second;
	float dr = std::sqrt(dx*dx + dy*dy);
	float D = (p1.first * p2.second) - (p2.first * p1.second);
	float discriminant = (lookAheadDist * lookAheadDist) * (dr*dr) - D*D;
	float goalx;
	float goaly;

	std::pair<float, float> sol1 = std::make_pair(0,0);
	std::pair<float, float> sol2 = std::make_pair(0,0);

	if (discriminant >=0){
		intersectFound = true;
		float sol_x1 = (D * dy + sgn(dy) * dx * std::sqrt(discriminant)) / (dr*dr);
		float sol_x2 = (D * dy - sgn(dy) * dx * std::sqrt(discriminant)) / (dr*dr);
		float sol_y1 = (- D * dx + std::abs(dy) * std::sqrt(discriminant)) / (dr*dr);
		float sol_y2 = (- D * dx - std::abs(dy) * std::sqrt(discriminant)) / (dr*dr);
		float max_x = std::max(p1.first,p2.first);
		float max_y = std::max(p1.second,p2.second);
		float min_x = std::min(p1.first,p2.first);
		float min_y = std::min(p1.second,p2.second);
		if(min_x <= sol_x1 && sol_x1 <= max_x && min_y <= sol_y1 && sol_y1 <= max_y){
			sol1V = true;
		}
		if(min_x <= sol_x2 && sol_x2 <= max_x && min_y <= sol_y2 && sol_y2 <= max_y){
			sol2V = true;
		}
		// if(sol_x1 >= p1.first && sol_x1 <= p2.first && sol_y1 >=p1.second && sol_y1 <= p2.second){
		// 	sol1V = true;
		// }
		// if(min_x <= sol_x2 && sol_x2 <= max_x && min_y <= sol_y2 && sol_y2 <= max_y){
		// 	sol2V = true;
		// }

		if(!sol1V && !sol2V){
			intersectFound=false;
		}

		sol1 = std::make_pair(sol_x1 + cur_pos.first, sol_y1 + cur_pos.second);
		sol2 = std::make_pair(sol_x2 + cur_pos.first, sol_y2 + cur_pos.second);

		// std::cout << "sol1 at " << sol1.first << ", " << sol1.second << std::endl;
		// std::cout << "sol2 at " << sol2.first << ", " << sol2.second << std::endl;		
	}

	if (!intersectFound){
		std::cout << "No intersect found" << std::endl;

		return std::make_pair(p1.first+cur_pos.first, p1.second+cur_pos.second);
	} else{
		if(sol1V && !sol2V){
			goalx = sol1.first;
			goaly = sol1.second;
		} else if(sol2V && !sol1V){
			goalx = sol2.first;
			goaly = sol2.second;
		} else if(sol1V && sol2V){
			float sol1_to_p2 = sqrt(pow(p2.first-sol1.first,2) + pow(p2.second-sol1.second,2));
			float sol2_to_p2 = sqrt(pow(p2.first-sol2.first,2) + pow(p1.second-sol2.second,2));
			if(sol1_to_p2 < sol2_to_p2){
				goalx = sol1.first;
				goaly = sol1.second;
			} else {
				goalx = sol2.first;
				goaly = sol2.second;
			}
		}
		std::cout << "goal x at" << goalx << " goal y at " << goaly << std::endl;
	std::cout << "current x at" << cur_pos.first << " current y at " << cur_pos.second << std::endl;

		return std::make_pair(goalx, goaly);
	}
}

void followPath(std::vector<std::pair<float, float>> path){
	bool isMoving = true;
	int lastFoundIndex = 0;
	while(isMoving){
		std::cout << "=================== " << lastFoundIndex << std::endl;
		std::pair<float, float> solution = path[lastFoundIndex];
		for(int i = lastFoundIndex; i<path.size()-1; i++){
			std::pair<float, float> current_pos = std::make_pair(chassis.getPose().x, chassis.getPose().y);
			std::pair<float, float> tempSolution = findPoint(current_pos, path[i], path[i+1], 15);
			pros::lcd::print(5, "temp_x: %f, temp_y: %f", tempSolution.first, tempSolution.second);
			if(solution.first != path[i].first || solution.second != path[i].second){
				solution = tempSolution;
				if(distance_btwn_points(solution, path[i+1]) < distance_btwn_points(std::make_pair(chassis.getPose().x, chassis.getPose().y), path[i+1])){
					break;	
				}else{
					continue;
					lastFoundIndex++;
					master.rumble(".");
				}
			}
		}
		int x = solution.first;
		int y = solution.second;

		int back_left_speed = y - x;
		int back_right_speed = -y - x;
		int front_left_speed = y + x;
		int front_right_speed = -y + x;
		
		back_right.move_velocity(back_right_speed);
		front_left.move_velocity(front_left_speed);
		front_right.move_velocity(front_right_speed);
		back_left.move_velocity(back_left_speed);
		pros::lcd::print(3, "target_x: %f, target_y: %f", x, y);
		pros::delay(20);
	}
}

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	pros::Task screen_task([&]() {
		while (true) {
				// print robot location to the brain screen
				pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
				pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
				pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
				// delay to save resources
				pros::delay(20);
		}
	});
}	

void autonomous() {
	
}

void opcontrol() {
	std::vector<std::pair<float, float>> coordinates = {
		std::make_pair(0.00, 0.00),
		std::make_pair(00, 40.00),
		std::make_pair(60, 60),
		std::make_pair(0, 60)
		// std::make_pair(30.4193, 24.7544),
		// std::make_pair(40.2745, 28.9638),
		// std::make_pair(49.4400, 35.4720)
	};
	// followPath(coordinates);
	while (true) {
		int input_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int input_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

		int rotation = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		int current_heading = inertial_sensor.get_rotation();

		if(abs(input_y) < 10){
			input_y = 0;
		}
		if(abs(input_x) < 10){
			input_x = 0;
		}
		if(abs(rotation) < 10){
			rotation = 0;
		}
		if(input_y == 0 && input_x == 0 && abs(rotation) > 120){
			rotation = sgn(rotation) * 200;
		}

		int back_left_speed = input_y - input_x + rotation;
		int back_right_speed = -input_y - input_x + rotation;
		int front_left_speed = input_y + input_x	+ rotation;
		int front_right_speed = -input_y + input_x + rotation;
		
		back_right.move_velocity(back_right_speed);
		front_left.move_velocity(front_left_speed);
		front_right.move_velocity(front_right_speed);
		back_left.move_velocity(back_left_speed);

		pros::delay(20);	
	}
}