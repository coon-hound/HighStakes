#ifndef PID_H
#define PID_H

class PID {
private:
    double kp; 
    double ki;
    double kd;
    
    double setpoint;   
    double error_sum; 
    double last_error;
    double output_min;
    double output_max;

    double P;
    double I;
    double D;

    double I_max; // prevent overshoot
    double I_range; // delay start of accumulation

    double D_tolerance; // braking power threshold
                        // the speed of which is acceptable to brake at

    double error_tolerance; // tolerance of which to stop

    bool arrived;

public:
    PID(double p, double i, double d, double min, double max, double tolerance, double i_max = 100, double i_range = 20, double d_tolerance = 1e-6);
    
    void reset();
    
    void setSetpoint(double new_setpoint);
    
    double calculate(double measurement);
    double calculate_with_limits(double measurement);
    
    void setGains(double p, double i, double d);
    void setOutputLimits(double min, double max);

    bool hasArrived();
};

#endif