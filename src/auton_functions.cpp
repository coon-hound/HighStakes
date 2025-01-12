#include "autonfunctions.h"
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

void ForwardWithCorrection(double forward_power, double target_heading, double kp, double P_max) {
	double heading = Odom.GetHeadingDegrees();

	double error = target_heading - heading;

	double correction_output = error * kp;

	// correction_output = 0;

	if (fabs(correction_output) > P_max) {
		correction_output = sign(correction_output) * P_max;
		printf("true");
	} 

	Right_Power = forward_power + correction_output; 
	Left_Power = forward_power - correction_output; 
}

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

		ForwardWithCorrection(sign(distance_to_target) * 100.0, target_heading);

		printf("currdistnace = %f\n", current_distance);
		printf("power = %f, heading = %f\n", sign(distance_to_target) * 100.0, target_distance);
		printf("rightpower = %f, left_power = %f\n", Right_Power, Left_Power);

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

		printf("curr_distance = %f, forward_power = %f\n", current_distance, forward_power);
		printf("Rightpower = %f, Left_power = %f\n", Right_Power, Left_Power);
        
		this_thread::sleep_for(10);
    }
}

void Forward(double forward_power, double correction_distance, double pid_distance, double turn_kp, double turn_P_max) {
	ForwardTillDestination(forward_power, correction_distance, turn_kp, turn_P_max);
	PIDForward(pid_distance);
} 

void PIDTurn(double target_heading_degrees) {
    Turn_PID.reset();
    Turn_PID.setSetpoint(target_heading_degrees);
    
    while (!Turn_PID.hasArrived()) {
        double current_heading = Odom.GetHeadingDegrees();
        
        double correction_output = Turn_PID.calculate_with_limits(current_heading);
        
        Right_Power = correction_output;
        Left_Power = -correction_output;
        
        printf("curr_heading = %f, correction_output = %f\n", current_heading, correction_output);
        printf("Right_power = %f, Left_power = %f\n", Right_Power, Left_Power);
        
        this_thread::sleep_for(10);
    }
}

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
// drift
/*
	double heading;

	PID TO rotational output (correection__output)

	if (fabs(correction_output) > P_max) {
		correction_output = sign(correction_output) * P_max;
	} 

	Right_Power = bias - correction_output; 
	Left_Power = bias + correction_output; 
*/