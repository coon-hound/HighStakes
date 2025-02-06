#include "odom.h"
#include "definitions.h"
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

void odom::SetCornerPosition() {
    double center_x = 0, center_y = 0;

    if (heading >= 3 * M_PI / 2 && heading < 2 * M_PI){
        // Top Left
        center_x = 1/4 * (2 * ROBOT_WIDTH * cos(heading) - 2 * ROBOT_LENGTH * sin(heading));
        center_y = 1/4 * (-2 * ROBOT_LENGTH * cos(heading) + 2 * ROBOT_WIDTH * sin(heading) + 576);

    } else if (heading >= 0 && heading < M_PI/2) {
        // Top Right
        center_x = 1/4 * (-2 * ROBOT_LENGTH * sin(heading) - 2 * ROBOT_WIDTH * cos(heading) + 576);
        center_y = 1/4 * (-2 * ROBOT_LENGTH * cos(heading) + 2 * ROBOT_WIDTH * sin(heading) + 576);

    } else if (heading >= M_PI/2 && heading < M_PI) {
        // Bottom Right
        center_x = 1/4 * (-2 * ROBOT_LENGTH * sin(heading) + 2 * ROBOT_WIDTH * cos(heading) + 576);
        center_y = 1/4 * (2 * ROBOT_WIDTH *  sin(heading) - 2 * ROBOT_LENGTH * cos(heading));

    } else if (heading >= M_PI && heading < 3 * M_PI/2) {
        // Bottom Left
        center_x = 1/4 * (-2 * ROBOT_LENGTH * sin(heading) - 2 * ROBOT_WIDTH * cos(heading));
        center_y = 1/4 * (-(ROBOT_LENGTH) * cos(heading) + ROBOT_LENGTH * cos(heading + 180) - 2 * ROBOT_WIDTH * sin(a));
    }


    /*
    if (heading >= 0 && heading < M_PI/2) {
        // top right
        center_x = 144 - ((ROBOT_WIDTH * cos(heading) - ROBOT_LENGTH * cos(M_PI/2 - heading)) / 2); 
        center_y = 144 - ((ROBOT_WIDTH * sin(heading) - ROBOT_LENGTH * sin(M_PI/2 - heading)) / 2); 

    } else if (heading >= M_PI/2 && heading < M_PI) {
        // bottom right
        center_x = 144 - ((ROBOT_WIDTH * cos(heading) - ROBOT_LENGTH * cos(M_PI/2 - heading)) / 2); 
        center_y = ((ROBOT_WIDTH * sin(heading) - ROBOT_LENGTH * sin(M_PI/2 - heading)) / 2); 
    } else if (heading >= M_PI && heading < 3 * M_PI / 2) {
        // bottom left
        center_x = ((ROBOT_WIDTH * cos(heading) - ROBOT_LENGTH * cos(M_PI/2 - heading)) / 2); 
        center_y = ((ROBOT_WIDTH * sin(heading) - ROBOT_LENGTH * sin(M_PI/2 - heading)) / 2); 
    } else if (heading >= 3 * M_PI / 2 && heading < 2 * M_PI) {
        // top left
        center_x = ((ROBOT_WIDTH * cos(heading) - ROBOT_LENGTH * cos(M_PI/2 - heading)) / 2); 
        center_y = 144 - ((ROBOT_WIDTH * sin(heading) - ROBOT_LENGTH * sin(M_PI/2 - heading)) / 2); 
    }
    */

    SetX(center_x);
    SetY(center_y);
}

// void odom::SetCornerPosition() {
//     float halfWidth = ROBOT_WIDTH / 2.0f;
//     float halfLength = ROBOT_LENGTH / 2.0f;

//     // Calculate base position assuming bottom-left corner (0,0)
//     float baseX = halfWidth * cosf(heading) - halfLength * sinf(heading);
//     float baseY = halfWidth * sinf(heading) + halfLength * cosf(heading);

//     double heading_deg = heading * 180.0 / M_PI;

//     // Determine which corner based on heading and adjust coordinates
//     if (heading_deg > 270 && heading_deg <= 360) {
//         // Top-left corner: flip Y
//         SetX(baseX);
//         SetY(144.0f - baseY);
//     }
//     else if (heading_deg > 0 && heading_deg <= 90) {
//         // Top-right corner: flip both X and Y
//         SetX(144.0f - baseX);
//         SetY(144.0f - baseY);
//     }
//     else if (heading_deg > 90 && heading_deg <= 180) {
//         // Bottom-right corner: flip X
//         SetX(144.0f - baseX);
//         SetY(baseY);
//     }
//     else {  // 180-270 degrees, bottom-left
//         SetX(baseX);
//         SetY(baseY);
//     }
// }

// void odom::SetCornerPosition() {
//     float halfWidth = ROBOT_WIDTH / 2.0f;
//     float frontDist = sqrtf(pow(ROBOT_LENGTH/2, 2) + pow(ROBOT_WIDTH/2, 2));
//     float frontAngle = atan2f(halfWidth, ROBOT_LENGTH/2);
    
//     float mathAngle = (M_PI/2 - heading);
    
//     float centerX = frontDist * (sin(mathAngle - frontAngle) + sin(mathAngle + frontAngle));
//     float centerY = frontDist * (cos(mathAngle - frontAngle) + cos(mathAngle + frontAngle));
    
//     double heading_deg = heading * 180.0 / M_PI;

//     if (heading_deg > 270 && heading_deg <= 360) {
//         // Facing left - use top left corner
//         centerY = 144.0f - centerY;
//     }
//     else if (heading_deg > 0 && heading_deg <= 90) {
//         // Facing up - use top right corner
//         centerX = 144.0f - centerX;
//         centerY = 144.0f - centerY;
//     }
//     else if (heading_deg > 90 && heading_deg <= 180) {
//         // Facing right - use bottom right corner
//         centerX = 144.0f - centerX;
//     }
//     else {  // heading_deg > 180 && heading_deg <= 270
//         // Facing down - use bottom left corner
//         // Both x and y are already correct from 0
//     }
    
//     SetX(centerX);
//     SetY(centerY);
// }

// void odom::SetCornerPosition() {
//     // Half the robot's width (distance from center to side)
//     float halfWidth = ROBOT_WIDTH / 2.0f;
    
//     // Calculate the distances from the robot's center to its front corners
//     float frontDist = sqrtf(pow(ROBOT_LENGTH/2, 2) + pow(ROBOT_WIDTH/2, 2));
//     float frontAngle = atan2f(halfWidth, ROBOT_LENGTH/2);
    
//     // Calculate the position of the robot's center based on the two front corners touching walls
//     float centerX = frontDist * (sin(heading - frontAngle) + sin(heading + frontAngle));
//     float centerY = frontDist * (cos(heading - frontAngle) + cos(heading + frontAngle));
    
//     // Since the above calculates from origin, adjust for the corner we're using

//     double heading_deg = heading * 180.0 / M_PI;

//     if (heading_deg > 0 && heading_deg <= 90) {
//         centerX = 144.0f - centerX;
//         centerY = 144.0f - centerY;
//         // top right
//     }
//     else if (heading_deg > 180 && heading_deg <= 270) {
//         // bottom right
//         // centerX already correct (from 0)
//         // centerY already correct (from 0)
//     }
//     else if (heading_deg > 270 && heading_deg <= 360) {
//         // Top left corner
//         // centerX already correct (from 0)
//         centerY = 144.0f - centerY;
//     }
//     else {  // heading <= 45 || heading > 315
//         // bottom right
//         centerX = 144.0f - centerX;
//         // centerY already correct (from 0)
//     }

//     heading = heading * M_PI / 180;
//     // Set the robot's position
//     SetX(centerX);
//     SetY(centerY);
// }

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
