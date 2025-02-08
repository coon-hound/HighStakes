#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <vex.h>
#include <deque>
#include <cmath>
#include "port_config.h"
#include "pid.h"
#include "odom.h"
using namespace vex;
class odom; 

extern brain       Brain;
extern controller  Controller;
extern timer       Timer;

extern motor       Intake;
extern motor       Ladybrown1;
extern motor       Ladybrown2;

extern odom        Odom; 
extern inertial    Imu;

extern motor       RightMotor1;
extern motor       RightMotor2;
extern motor       RightMotor3;

extern motor       LeftMotor1;
extern motor       LeftMotor2;
extern motor       LeftMotor3;

extern optical     Color;
extern distance    LadybrownDistance;

extern int         Team_Color;

extern bool        Ring_Fired;

extern bool        macro_mode;

extern bool        Store_Mode;
extern bool        Store_Mode_Move_Intake;

// drivetrain

extern bool Toggle_Doinker;
extern pneumatics  Doinker;

extern int Right_Power;
extern int Left_Power;

#define right_motor_pos ((RightMotor1.position(rotationUnits::deg) + RightMotor2.position(rotationUnits::deg) + RightMotor3.position(rotationUnits::deg))/3)
#define left_motor_pos ((LeftMotor1.position(rotationUnits::deg) + LeftMotor2.position(rotationUnits::deg) + LeftMotor3.position(rotationUnits::deg))/3)

extern PID Forward_PID;
extern PID Turn_PID;

extern double prev_right_position;
extern double prev_left_position;

// intake

extern pneumatics  Intake_Lift;
extern bool Toggle_Intake_Lift;

extern int         Intake_Power;

#define RED 51
#define BLUE 52
#define CHECKING 53

#define intake_pos (Intake.position(rotationUnits::deg))
const double period  = 17.0 * 60.0; // 1020
#define stage (floor(intake_pos / period))
#define hook_pos (intake_pos - stage * period)

struct Ring {
    int hooked_stage;
    int color;
    int initial_color;

    bool entered_color_range;
    bool color_checked;

    Ring() {
        hooked_stage = stage;
        color = CHECKING;
        initial_color = CHECKING;
        entered_color_range = false;
        color_checked = false;
    }

    double getPos() {
        double pos = intake_pos - hooked_stage * period;
        return pos;
    }
};

extern std::deque<Ring> ring_queue;

// ladybrown

extern bool Ladybrown_Has_Ring;
extern PID ladybrown_pid;

#define LOAD_HEIGHT 80
#define DESCORE_HEIGHT 470
#define DESCORE_TWO_HEIGHT 500
#define FLOAT_HEIGHT 170

#define LADYBROWN_DETECT_DISTANCE 200

extern int         Ladybrown_Power;
extern int         Ladybrown_Arm_Height;

#define ladybrown_pos (Ladybrown1.position(rotationUnits::deg))

// motor 

extern void MoveMotor(motor m, int pct);

// mogo

extern pneumatics  Mogomech;
extern bool Toggle_Mogo;

enum auton_strategy {
    BRing,
    BAWP,
    BMogo,
    RRing,
    RAWP,
    RMogo,
    ASkills
};

extern auton_strategy auton_strat;

#endif