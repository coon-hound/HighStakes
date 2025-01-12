#ifndef AUTON_FUNCTIONS_H
#define AUTON_FUNCTIONS_H

void ForwardWithCorrection(double forward_power, double target_heading, double kp = 10.0, double P_max = 30.0);
void ForwardTillDestination(double forward_power, double target_distance, double target_heading, double kp = 10.0, double P_max = 30.0);
void PIDForward(double target_distance_inches);
void Forward(double forward_power, double correction_distance, double pid_distance, double turn_kp = 10.0, double turn_P_max = 30.0); 

void PIDTurn(double target_heading_degrees);
void PIDDrift(double target_heading_degrees, double bias);

#endif