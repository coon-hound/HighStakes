#include "pid.h"
#include <stdio.h>
#include <algorithm>
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

PID::PID(double p, double i, double d, double min, double max, double tolerance, double i_max, double i_range, double d_tolerance) 
    : kp(p), ki(i), kd(d), output_min(min), output_max(max), error_tolerance(tolerance), I_max(i_max), I_range(i_range), D_tolerance(d_tolerance) {
    reset();
}

void PID::reset() {
    setpoint = 0.0;
    error_sum = 0.0;
    last_error = 0.0;
    arrived = false;
}

void PID::setSetpoint(double new_setpoint) {
    setpoint = new_setpoint;
}

double PID::calculate(double measurement) {
    double error = setpoint - measurement;
    
    double P = kp * error;

    error_sum += error;
    
    double I = ki * error_sum;
   
    double D = kd * (error - last_error);
    last_error = error;

    // if speed isn't too high
    if (fabs(error) < error_tolerance) {
        arrived = 1; 
    }

    double output = P + I + D;
    
    if (output > output_max) {
        output = output_max;
    }
    if (output < output_min) {
        output = output_min;
    }
    
    return output;
}

double PID::calculate_with_limits(double measurement) {
    double error = setpoint - measurement;
    
    // Normalize error to -180 to +180 range
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    double P = kp * error;
    
    // Calculate I term first
    if (fabs(P) > I_range || fabs(error) < error_tolerance || error_sum * error <= 0) {
        error_sum = 0;
    }
    else {
        error_sum += error;
    }
    
    double I = ki * error_sum;
    
    // Apply I term limits after calculating it
    if (fabs(I) > I_max) {
        I = sign(I) * I_max; 
    }
   
    double D = kd * (error - last_error);
    last_error = error;

    // Check if we've arrived at target
    if (fabs(error) < error_tolerance && fabs(D) < D_tolerance)  {
        arrived = 1; 
    }

    double output = P + I + D;
    
    // Apply output limits
    if (output > output_max) {
        output = output_max;
    }
    if (output < output_min) {
        output = output_min;
    }
    
    return output;
}

void PID::setGains(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
}

void PID::setOutputLimits(double min, double max) {
    output_min = min;
    output_max = max;
}

bool PID::hasArrived() {
    return arrived;
}