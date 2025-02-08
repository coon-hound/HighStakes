#include "definitions.h"

using namespace vex;

brain       Brain;
controller  Controller;
timer       Timer;

pneumatics  Doinker(DOINKER);
bool Toggle_Doinker = false;

motor       Intake(port::INTAKE, ratio6_1, true);
motor 		Ladybrown1(port::LB1, ratio18_1, true);
motor 		Ladybrown2(port::LB2, ratio18_1, false);

motor       RightMotor1(port::RIGHT_MOTOR1, ratio6_1, false);
motor       RightMotor2(port::RIGHT_MOTOR2, ratio6_1, false);
motor       RightMotor3(port::RIGHT_MOTOR3, ratio6_1, true);

motor       LeftMotor1(port::LEFT_MOTOR1, ratio6_1, true);
motor       LeftMotor2(port::LEFT_MOTOR2, ratio6_1, true);
motor       LeftMotor3(port::LEFT_MOTOR3, ratio6_1, false);

odom 		Odom;
inertial    Imu(port::IMU);

optical     Color(port::COLOR);
distance	LadybrownDistance(port::LB_DISTANCE);

int 		Right_Power = 0;
int			Left_Power = 0;

double prev_right_position = 0;
double prev_left_position = 0;

PID			Forward_PID(6.5, 25.0, 30.0, -100, 100, 1.0, 100.0, 20.0, 0.2);			
PID 		Turn_PID(4.00, 10.0, 30.0, -100, 100, 3, 100.0, 20.0, 0.1);
// PID 		Turn_PID(3.50, 0.0, 30.0, -100, 100, 3);
pneumatics  Intake_Lift(INTAKE_LIFT);
bool Toggle_Intake_Lift = false;

int 		Team_Color = RED;
std::deque<Ring> ring_queue;
int 		Intake_Power = 0;

int         Ladybrown_Power = 0;

bool 		macro_mode = false;
bool		Store_Mode = false;
bool		Store_Mode_Move_Intake = false;

bool        Ring_Fired = false;
bool        Intake_Flag = false;
bool        Outtake_Flag = false;

PID 		ladybrown_pid(0.75, 0, 3.5, -100, 100, 0);

bool 		Ladybrown_Has_Ring = false;

int 		Ladybrown_Arm_Height = -1;

pneumatics	Mogomech(MOGO);   
bool Toggle_Mogo = false;

void MoveMotor(motor m, int pct) {
	m.spin(fwd, 128 * pct, voltageUnits::mV);	
}