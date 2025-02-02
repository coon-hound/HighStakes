#include "odom.h"
#include <cmath>

odom::odom() : 
    x(0.0),
    y(0.0),
    heading(+0.0),
    last_left_pos(0.0),
    last_right_pos(0.0),
    initial_heading(0.0) {}

double odom::encoder_to_inches(double encoder_counts) {
    return (encoder_counts / ENCODER_COUNTS_PER_REVOLUTION) * GEAR_RATIO * WHEEL_CIRCUMFERENCE;
}

void odom::UpdatePosition() {
    double left_distance = encoder_to_inches(left_motor_pos - last_left_pos);
    double right_distance = encoder_to_inches(right_motor_pos - last_right_pos);
    
    double distance = (left_distance + right_distance) / 2.0;
    
    double raw_heading = Imu.heading(degrees);
    
    heading = raw_heading * (M_PI / 180.0) + initial_heading;
    heading = fmod(heading, 2 * M_PI);

    if (heading < 0) {
        heading += 2 * M_PI;
    }
    
    // Calculate position changes - modified to make y-axis point up
    double delta_x = distance * sin(heading);  // Changed from cos to sin
    double delta_y = distance * cos(heading);  // Changed from sin to cos
    
    x += delta_x;
    y += delta_y;
    
    last_left_pos = left_motor_pos;
    last_right_pos = right_motor_pos;
}

// void odom::UpdatePosition() {
//     double left_distance = encoder_to_inches(left_motor_pos - last_left_pos);
//     double right_distance = encoder_to_inches(right_motor_pos - last_right_pos);
    
//     double distance = (left_distance + right_distance) / 2.0;
    
//     double raw_heading = Imu.heading(degrees);
    
//     heading = raw_heading * (M_PI / 180.0) + initial_heading;
//     heading = fmod(heading, 2 * M_PI);

//     if (heading < 0) {
//         heading += 2 * M_PI;
//     }
    
//     // Calculate position changes
//     double delta_x = distance * cos(heading);
//     double delta_y = distance * sin(heading);
    
//     x += delta_x;
//     y += delta_y;
    
//     last_left_pos = left_motor_pos;
//     last_right_pos = right_motor_pos;
    
//     // printf("x = %f, y = %f, heading = %f\n", x, y, heading * 180.0/M_PI);
// }

void odom::SetX(double new_value){
    x = new_value;
}

void odom::SetY(double new_value) {
    y = new_value;
}

void odom::SetHeadingDegrees(double new_degrees) {
    new_degrees = fmod(new_degrees, 360.0);
    if (new_degrees < 0) {
        new_degrees += 360.0;
    }
    
    heading = new_degrees * (M_PI / 180.0);
    initial_heading = heading;
}

void odom::SetPosition(double new_x, double new_y, double new_heading_degrees) {
    SetX(new_x);
    SetY(new_y);
    SetHeadingDegrees(new_heading_degrees);
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
    
    // Calculate target angle from positive y-axis (clockwise positive) 
    double target_angle = atan2(delta_x, delta_y);  // Order remains the same
    
    // Normalize target_angle to 0 to 2π to match heading format
    target_angle = fmod(target_angle, 2 * M_PI);
    if (target_angle < 0) {
        target_angle += 2 * M_PI;
    }
    
    // Calculate error (heading is already normalized in UpdatePosition)
    double angle_error = target_angle - heading;
    
    // Normalize to -π to π for the shortest rotation
    if (angle_error > M_PI) {
        angle_error -= 2 * M_PI;
    } else if (angle_error < -M_PI) {
        angle_error += 2 * M_PI;
    }
    
    // Convert to degrees before returning
    return angle_error * 180.0 / M_PI;
}

double odom::GetAngleToPointBack(double target_x, double target_y) {
    double delta_x = target_x - x;  
    double delta_y = target_y - y;
    
    // Calculate target angle from positive y-axis (clockwise positive)
    double target_angle = atan2(delta_x, delta_y);  
    
    // Add π (180 degrees) to make the back face the point
    target_angle += M_PI;
    
    // Normalize target_angle to 0 to 2π to match heading format
    target_angle = fmod(target_angle, 2 * M_PI);
    if (target_angle < 0) {
        target_angle += 2 * M_PI;
    }
    
    // Calculate error (heading is already normalized in UpdatePosition)
    double angle_error = target_angle - heading;
    
    // Normalize to -π to π for the shortest rotation
    if (angle_error > M_PI) {
        angle_error -= 2 * M_PI;
    } else if (angle_error < -M_PI) {
        angle_error += 2 * M_PI;
    }
    
    // Convert to degrees before returning
    return angle_error * 180.0 / M_PI;
}

// double odom::GetAngleToPoint(double target_x, double target_y) {
//     double delta_x = target_x - x;  
//     double delta_y = target_y - y;
    
//     // Calculate target angle from positive y-axis (clockwise positive) 
//     double target_angle = atan2(delta_x, delta_y);
    
//     // Normalize target_angle to 0 to 2π to match heading format
//     target_angle = fmod(target_angle, 2 * M_PI);
//     if (target_angle < 0) {
//         target_angle += 2 * M_PI;
//     }
    
//     // Calculate error (heading is already normalized in UpdatePosition)
//     double angle_error = target_angle - heading;
    
//     // Normalize to -π to π for the shortest rotation
//     if (angle_error > M_PI) {
//         angle_error -= 2 * M_PI;
//     } else if (angle_error < -M_PI) {
//         angle_error += 2 * M_PI;
//     }
    
//     // Convert to degrees before returning
//     return angle_error * 180.0 / M_PI;
// }
