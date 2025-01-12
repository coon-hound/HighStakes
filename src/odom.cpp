#include "odom.h"
#include <cmath>

odom::odom() : 
    x(0.0),
    y(0.0),
    heading(0.0),
    last_left_pos(0.0),
    last_right_pos(0.0) {}

double odom::encoder_to_inches(double encoder_counts) {
    return (encoder_counts / ENCODER_COUNTS_PER_REVOLUTION) * GEAR_RATIO * WHEEL_CIRCUMFERENCE;
}

void odom::UpdatePosition() {
    double left_distance = encoder_to_inches(left_motor_pos - last_left_pos);
    double right_distance = encoder_to_inches(right_motor_pos - last_right_pos);
    
    double distance = (left_distance + right_distance) / 2.0;
    double delta_heading = (right_distance - left_distance) / TRACK_WIDTH;
    
    heading += delta_heading;
    
    heading = atan2(sin(heading), cos(heading));
    
    double delta_x = distance * cos(heading);
    double delta_y = distance * sin(heading);
    
    x += delta_x;
    y += delta_y;
    
    last_left_pos = left_motor_pos;
    last_right_pos = right_motor_pos;
}

double odom::GetX() {
    return x;
}

double odom::GetY() {
    return y;
}

double odom::GetHeadingRadians() {
    return heading;
}

double odom::GetHeadingDegrees() {
    return heading * 180.0 / M_PI;
}

void odom::ResetPosition() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
    last_left_pos = 0.0;
    last_right_pos = 0.0;
}

double odom::GetDistanceToPoint(double target_x, double target_y) {
    double delta_x = target_x - x;
    double delta_y = target_y - y;
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

double odom::GetAngleToPoint(double target_x, double target_y) {
    double delta_x = target_x - x;
    double delta_y = target_y - y;
    double target_angle = atan2(delta_y, delta_x);
    double angle_error = target_angle - heading;
    
    // Normalize to -π to π
    return atan2(sin(angle_error), cos(angle_error));
}