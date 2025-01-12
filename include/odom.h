#ifndef ODOM_H
#define ODOM_H

#include "definitions.h"
#include <cmath>

class odom {
private:
    // Constants
    static constexpr double WHEEL_DIAMETER = 3.25; // inches
    static constexpr double GEAR_RATIO = 48.0/72.0; // 48:72
    static constexpr double ENCODER_COUNTS_PER_REVOLUTION = 360.0;
    static constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
    
    // Robot physical properties
    static constexpr double TRACK_WIDTH = 13.62; // inches
    
    // Position variables
    double x;
    double y;
    double heading;
    
    // Previous encoder values
    double last_left_pos;
    double last_right_pos;
    
    // Private helper method

public:
    odom();
    
    double encoder_to_inches(double encoder_counts);
    void UpdatePosition();
    double GetX();
    double GetY();
    double GetHeadingRadians();
    double GetHeadingDegrees();
    void ResetPosition();
    double GetDistanceToPoint(double target_x, double target_y);
    double GetAngleToPoint(double target_x, double target_y);
};

#endif // ODOM_H