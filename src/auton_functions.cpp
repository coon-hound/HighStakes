#include "auton_functions.h"
#include "definitions.h"
#include <cmath>

static int sign(double x) {
    if (x > 0) {
        return 1;
    } 
    if (x < 0) {
        return -1;
    }
    return 0;
}

void ForwardWithCorrection(double forward_power, double relative_heading, double kp, double P_max) {
    relative_heading = -relative_heading;
    double current_heading = Odom.GetHeadingDegrees(); // 0-360 clockwise
    double target_heading = current_heading + relative_heading;
    
    // Normalize target_heading to 0-360
    while (target_heading >= 360) {
        target_heading -= 360;
    }
    while (target_heading < 0) {
        target_heading += 360;
    }
    
    double error = target_heading - current_heading;
    
    // Normalize error to -180 to 180 for shortest path
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    double correction_output = error * kp;

    if (fabs(correction_output) > P_max) {
        correction_output = sign(correction_output) * P_max;
    }

    Right_Power = forward_power + correction_output;
    Left_Power = forward_power - correction_output;
}

// void ForwardWithCorrection(double forward_power, double target_heading, double kp, double P_max) {
//     double heading = Odom.GetHeadingDegrees(); // Assuming this returns 0-360 clockwise
    
//     double error = target_heading - heading;
    
//     // Normalize error to -180 to 180 for shortest path
//     if (error > 180) {
//         error -= 360;
//     } else if (error < -180) {
//         error += 360;
//     }

//     double correction_output = error * kp;

//     if (fabs(correction_output) > P_max) {
//         correction_output = sign(correction_output) * P_max;
//         printf("true");
//     }

//     Right_Power = forward_power + correction_output;
//     Left_Power = forward_power - correction_output;
// }

void ForwardTillDestination(double forward_power, double target_distance, double target_heading, double kp, double P_max) {
	prev_left_position = Odom.encoder_to_inches(left_motor_pos);
	prev_right_position = Odom.encoder_to_inches(right_motor_pos);

	double curr_left_position = 0;
	double curr_right_position = 0;

	double current_distance = 0;

	while (fabs(current_distance) < fabs(target_distance)) {
		curr_left_position = Odom.encoder_to_inches(left_motor_pos);
		curr_right_position = Odom.encoder_to_inches(right_motor_pos);

        current_distance = ((curr_left_position - prev_left_position) + (curr_right_position - prev_right_position)) / 2.0;

		double distance_to_target = target_distance - current_distance;

		ForwardWithCorrection(sign(distance_to_target) * forward_power, target_heading);


		this_thread::sleep_for(10);
	}
}

void ForwardTimer(double forward_power, int time_ms, double target_heading, double kp, double P_max) {
	prev_left_position = Odom.encoder_to_inches(left_motor_pos);
	prev_right_position = Odom.encoder_to_inches(right_motor_pos);

	double curr_left_position = 0;
	double curr_right_position = 0;

	int current_time = Timer.time();

	double current_distance = 0;

	while (Timer.time() - current_time < time_ms) {
		curr_left_position = Odom.encoder_to_inches(left_motor_pos);
		curr_right_position = Odom.encoder_to_inches(right_motor_pos);

		ForwardWithCorrection(forward_power, target_heading);

		this_thread::sleep_for(10);
	}
}

void PIDForward(double target_distance_inches) {
    Forward_PID.reset();
    Forward_PID.setSetpoint(target_distance_inches);

	prev_left_position = Odom.encoder_to_inches(left_motor_pos);
	prev_right_position = Odom.encoder_to_inches(right_motor_pos);

	double curr_left_position = 0;
	double curr_right_position = 0;
    
    while (!Forward_PID.hasArrived()) {
		curr_left_position = Odom.encoder_to_inches(left_motor_pos);
		curr_right_position = Odom.encoder_to_inches(right_motor_pos);
        // average left and right encoders to get current distance
        // double current_distance = (fabs(prev_left_position - curr_left_position) + fabs(prev_right_position - curr_right_position)) / 2.0;
        double current_distance = ((curr_left_position - prev_left_position) + (curr_right_position - prev_right_position)) / 2.0;
        
        double forward_power = Forward_PID.calculate_with_limits(current_distance);
		Right_Power = forward_power;
		Left_Power = forward_power;

		this_thread::sleep_for(10);
    }
}

void Forward(double forward_power, double correction_distance, double pid_distance, double turn_kp, double turn_P_max) {
	ForwardTillDestination(forward_power, correction_distance, turn_kp, turn_P_max);
	PIDForward(pid_distance);
} 

void PIDTurn(double target_heading_degrees) {
    double initial_heading = Odom.GetHeadingDegrees();
    
    // Calculate relative angle to turn
    double angle_difference = target_heading_degrees;
    
    // Normalize to -180 to +180
    while (angle_difference > 180) angle_difference -= 360;
    while (angle_difference < -180) angle_difference += 360;

    Turn_PID.reset();
    Turn_PID.setSetpoint(angle_difference);

    while (!Turn_PID.hasArrived()) {
        double current_heading = Odom.GetHeadingDegrees();
        double relative_angle = current_heading - initial_heading;
        
        // Normalize relative angle to -180 to +180
        while (relative_angle > 180) relative_angle -= 360;
        while (relative_angle < -180) relative_angle += 360;
        
        double correction_output = Turn_PID.calculate_with_limits(relative_angle);
        
        Right_Power = -correction_output;
        Left_Power = correction_output;
        
        this_thread::sleep_for(10);
    }
    Right_Power = 0;
    Left_Power = 0;
}

// void PIDTurn(double target_heading_degrees) {
//     double initial_heading = Odom.GetHeadingDegrees();
//     double current_relative_angle = 0;
    
//     // Calculate the shortest path to target
//     double angle_difference = target_heading_degrees;
//     if (angle_difference > 180) {
//         angle_difference -= 360;
//     } else if (angle_difference < -180) {
//         angle_difference += 360;
//     }

//     Turn_PID.reset();
    
//     // Set the PID setpoint relative to 0
//     Turn_PID.setSetpoint(angle_difference);

//     while (!Turn_PID.hasArrived()) {
//         // Calculate the relative angle traveled
//         double current_absolute = Odom.GetHeadingDegrees();
//         current_relative_angle = current_absolute - initial_heading;
        
//         // Normalize the relative angle to handle wraparound
//         if (current_relative_angle > 180) {
//             current_relative_angle -= 360;
//         } else if (current_relative_angle < -180) {
//             current_relative_angle += 360;
//         }
        
//         double correction_output = Turn_PID.calculate_with_limits(current_relative_angle);

//         printf("%f\n", correction_output);
        
//         Right_Power = -correction_output;
//         Left_Power = correction_output;
        
//         this_thread::sleep_for(10);
//     }
//     Right_Power = 0;
//     Left_Power = 0;
// }


void PIDTurnAbsolute(double target_heading_degrees) {
    double current_heading = Odom.GetHeadingDegrees();
    
    // Normalize target heading to 0-360 range
    target_heading_degrees = fmod(target_heading_degrees, 360);
    if (target_heading_degrees < 0) target_heading_degrees += 360;
    
    // Calculate the shortest path to target
    double angle_difference = target_heading_degrees - current_heading;
    if (angle_difference > 180) {
        angle_difference -= 360;
    } else if (angle_difference < -180) {
        angle_difference += 360;
    }

    Turn_PID.reset();
    
    // Set the PID setpoint to the target heading
    Turn_PID.setSetpoint(target_heading_degrees);

    while (!Turn_PID.hasArrived()) {
        double current_heading = Odom.GetHeadingDegrees();
        
        double correction_output = Turn_PID.calculate_with_limits(current_heading);
        
        Right_Power = -correction_output;
        Left_Power = correction_output;
        
        this_thread::sleep_for(10);
    }
}

// void PIDTurn(double target_heading_degrees) {
//     double initial_heading = Odom.GetHeadingDegrees();
//     double current_relative_angle = 0;
//     double target_relative_angle = target_heading_degrees;  // Positive = CW, Negative = CCW

//     Turn_PID.reset();
    
//     // Set the PID setpoint
//     Turn_PID.setSetpoint(target_relative_angle);

//     while (!Turn_PID.hasArrived()) {
//         // Calculate the relative angle traveled
//         double current_absolute = Odom.GetHeadingDegrees();
//         current_relative_angle = current_absolute - initial_heading;
        
//         // Normalize the relative angle to handle wraparound
//         if (current_relative_angle > 180) {
//             current_relative_angle -= 360;
//         } else if (current_relative_angle < -180) {
//             current_relative_angle += 360;
//         }
        
//         double correction_output = Turn_PID.calculate_with_limits(current_relative_angle);
        
//         // The sign of correction_output will automatically handle the direction
//         Right_Power = -correction_output;
//         Left_Power = correction_output;
        
//         this_thread::sleep_for(10);
//     }
// }

// void PIDTurn(double target_heading_degrees) {
//     double initial_heading = Odom.GetHeadingDegrees();
//     double relative_target = target_heading_degrees;
    
//     double error = relative_target;
//     if (error > 180) {
//         error -= 360;
//     } else if (error < -180) {
//         error += 360;
//     }
    
//     Turn_PID.reset();
//     double setpoint = initial_heading + error;
    
//     while (setpoint >= 360) setpoint -= 360;
//     while (setpoint < 0) setpoint += 360;
    
//     Turn_PID.setSetpoint(setpoint);

// 	int direction = 1;

// 	if (target_heading_degrees < 0) {
// 		direction = -1;
// 	}

//     while (!Turn_PID.hasArrived()) {
//         double current_heading = direction * Odom.GetHeadingDegrees();
        
//         double current_error = setpoint - current_heading;
//         if (current_error > 180) {
//             current_error -= 360;
//         } else if (current_error < -180) {
//             current_error += 360;
//         }
        
//         double correction_output = Turn_PID.calculate_with_limits(current_heading);
        
//         Right_Power = direction * -correction_output;
//         Left_Power = direction * correction_output;
        
//         this_thread::sleep_for(10);
//     }
// }


void PIDDrift(double target_heading_degrees, double bias) {
    Turn_PID.reset();
    Turn_PID.setSetpoint(target_heading_degrees);
    
    while (!Turn_PID.hasArrived()) {
        double current_heading = Odom.GetHeadingDegrees();
        
        double correction_output = Turn_PID.calculate_with_limits(current_heading);
        
        Right_Power = bias + correction_output;
        Left_Power = bias - correction_output;
        
        this_thread::sleep_for(10);
    }
}

void MoveToPoint(double x, double y, double power, double buffer, bool backwards, double target_heading, double target_distance) {

    if (backwards == true){
        target_heading = Odom.GetAngleToPointBack(x, y);

    } else{
        target_heading = Odom.GetAngleToPoint(x, y);
    }

    target_distance = Odom.GetDistanceToPoint(x, y);

    PIDTurn(target_heading);
    ForwardTillDestination(power, target_distance - (buffer + 5), 0);
    PIDForward(buffer);
}
// drift
/*
	double heading;

	PID TO rotational output (correection__output)

	if (fabs(correction_output) > P_max) {
		correction_output = sign(correction_output) * P_max;
	} 

	Right_Power = bias - correction_output; 
	Left_Power = bias + correction_output; 
*/#