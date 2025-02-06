#include "auton_task.h"
#include "definitions.h"
#include "auton_functions.h"

void BlueMogo() {
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(122.80);
    Odom.SetY(38.18);
    Odom.SetHeadingDegrees(251.29);

    target_distance = Odom.GetDistanceToPoint(72, 24);

    Intake_Power = 100;
    ForwardTillDestination(100, target_distance - 18, 0); 
    target_distance = Odom.GetDistanceToPoint(72, 24);
    Toggle_Doinker = 1;
    PIDForward(target_distance - 18);

    Intake_Power = 0;    

    this_thread::sleep_for(100);

    ForwardTillDestination(-100, 15, 0);
    PIDForward(-5);
    Toggle_Doinker = 0;

    this_thread::sleep_for(250);

    // reintake mogo
    target_heading = Odom.GetAngleToPointBack(71, 24);
    printf("target_heading = %f\n", target_heading);
    PIDTurn(target_heading);
    // score ring
    ForwardTillDestination(-30, 17, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    this_thread::sleep_for(100);

    target_heading = Odom.GetAngleToPoint(144.0, 0.0);
    target_distance = Odom.GetDistanceToPoint(144.0, 0.0);

    PIDTurn(target_heading);
    ForwardTimer(50, 2000, 0);

    // second ring
    ForwardTillDestination(-10, 7, 0); 
    ForwardTimer(50, 1000, 0);

    // third ring
    ForwardTillDestination(-10, 7, 0); 
    ForwardTimer(50, 1000, 0);


    Odom.SetCornerPosition();
    printf("x = %f, y = %f, heading%f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());
    
    PIDForward(-10);


    target_heading = Odom.GetAngleToPoint(130.0, 30.0);
    target_distance = Odom.GetDistanceToPoint(130.0, 30.0);

    PIDTurn(target_heading);
    Toggle_Mogo = 0;
    Intake_Power = 100;
    PIDForward(target_distance);
    Intake_Power = 0;

    target_heading = Odom.GetAngleToPointBack(96, 48);
    target_distance = Odom.GetDistanceToPoint(96, 48);

    PIDTurn(target_heading);
    PIDForward(-(target_distance - 10));
    ForwardTillDestination(-30, 5, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    PIDForward(5);

    target_heading = Odom.GetAngleToPoint(100, 25);
    target_distance =  Odom.GetDistanceToPoint(100, 25);

    PIDTurn(target_heading);
    PIDForward(target_distance);
    



    while(!Controller.ButtonDown.pressing()) {
        this_thread::sleep_for(10);
    }
}

void BlueRing() {
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(130.50);
    Odom.SetY(83.52);
    Odom.SetHeadingDegrees(129.96);

    Ladybrown_Power = 100;
    this_thread::sleep_for(300);
    PIDForward(-10);
    Ladybrown_Power = -100;


    target_heading = Odom.GetAngleToPointBack(96, 96);
    target_distance = Odom.GetDistanceToPoint(96, 96);

    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 20, 0);
    // score ring
    ForwardTillDestination(-30, 16.5, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    target_heading = Odom.GetAngleToPoint(75, 115);
    target_distance = Odom.GetDistanceToPoint(75, 115);

    PIDTurn(target_heading);
    PIDForward(target_distance - 12);

    PIDTurnAbsolute(0);
    Intake_Power = 100;
    ForwardTillDestination(50, 9, 0);
    // move backward so I turn into the wall fucking up everything 
    PIDForward(5);
    PIDForward(-5);
    // ForwardTillDestination(-30, 5, 0);

    target_heading = Odom.GetAngleToPoint(96, 120);
    target_distance = Odom.GetDistanceToPoint(96, 120);

    PIDTurn(target_heading);
    ForwardTillDestination(50, target_distance, 0);

    target_heading = Odom.GetAngleToPoint(125, 138.3);
    target_distance = Odom.GetDistanceToPoint(125, 138.3);

    // PIDTurn(target_heading);
    // printf("curr_heading = %f\n", Odom.GetHeadingDegrees());
    // ForwardTillDestination(50, target_distance - 5, 0);

    // ForwardTimer(30, 1000, 0);
    ForwardTimer(70, 1500, -45);

    printf("after 1st curr_heading = %f\n", Odom.GetHeadingDegrees());

    // second ring
    ForwardTillDestination(-20, 7, 0); 
    this_thread::sleep_for(500);
    ForwardTimer(50, 750, 0);

    printf("after 2nd curr_heading = %f\n", Odom.GetHeadingDegrees());

    // third ring
    ForwardTillDestination(-20, 7, 0); 
    this_thread::sleep_for(500);
    ForwardTimer(50, 750, 0);

    Odom.SetCornerPosition(); 
    printf("x = %f, y = %f, heading%f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());

    PIDForward(-10);

    // PIDForward(-16);
    // PIDTurnAbsolute(172);

    // Odom.SetX(126);
    // Odom.SetY(127);


    target_heading = Odom.GetAngleToPoint(120, 72);
    target_distance = Odom.GetDistanceToPoint(120, 72);

    PIDTurn(target_heading);

    printf("target_heading %f\n", target_heading);
    printf("final_heading = %f\n", Odom.GetHeadingDegrees());

    Toggle_Intake_Lift = 1;

    PIDForward(target_distance + 10);

    Toggle_Intake_Lift = 0;
    this_thread::sleep_for(500);
    PIDForward(-10);

    while (1) {
        this_thread::sleep_for(10);
    }
}

void AutonSkills() {
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(17);
    Odom.SetY(72);
    Odom.SetHeadingDegrees(270);

    // Score Wall Stake
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    Ladybrown_Power = -100;
    PIDForward(-10);
    
    // 24, 48 (Mogo)
    target_heading = Odom.GetAngleToPointBack(24, 48);
    target_distance = Odom.GetDistanceToPoint(24, 48);
    PIDTurn(target_heading);
    ForwardTillDestination(-30, 15, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;
    PIDForward(-5);

    // 48, 48
    target_heading = Odom.GetAngleToPoint(48, 48);
    target_distance = Odom.GetDistanceToPoint(48, 48);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);
    this_thread::sleep_for(100);

    // 72, 72
    target_heading = Odom.GetAngleToPoint(72, 72);
    target_distance = Odom.GetDistanceToPoint(72, 72);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 7, 0);
    PIDForward(5);

    // 96, 48
    target_heading = Odom.GetAngleToPoint(96, 48);
    target_distance = Odom.GetDistanceToPoint(96, 48);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 96, 24
    target_heading = Odom.GetAngleToPoint(96, 24);
    target_distance = Odom.GetDistanceToPoint(96, 24);
    PIDTurn(target_heading);
    this_thread::sleep_for(250);
    PIDForward(10);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(2);

    // 72, 12
    target_heading = Odom.GetAngleToPoint(72, 12);
    target_distance = Odom.GetDistanceToPoint(72, 12);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);
    PIDTurnAbsolute(180);

    ForwardTimer(30, 1000, 0);

    // Lady Brown Wall Stake

    // Wall stakes alignment - y: 10.43 x: 72 heading:180 (close)
    Odom.SetX(72);
    Odom.SetY(10.43);
    Odom.SetHeadingDegrees(180);

    Intake_Power = 0;
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    this_thread::sleep_for(100);
    Intake_Power = 100;
    PIDForward(-10);
    ForwardTimer(30, 500, 0);
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    Ladybrown_Power = -100;
    this_thread::sleep_for(100);
    PIDForward(-10);

    // 120, 12
    target_heading = Odom.GetAngleToPoint(120, 12);
    target_distance = Odom.GetDistanceToPoint(120, 12);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 120, 24
    target_heading = Odom.GetAngleToPoint(120, 24);
    target_distance = Odom.GetDistanceToPoint(120, 24);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 132, 24
    target_heading = Odom.GetAngleToPoint(132, 24);
    target_distance = Odom.GetDistanceToPoint(132, 24);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 136, 12 (Clear Corner)
    target_heading = Odom.GetAngleToPoint(136, 12);
    target_distance = Odom.GetDistanceToPoint(136, 12);
    PIDTurn(target_heading);
    PIDForward(target_distance - 7);
    this_thread::sleep_for(50);
    Toggle_Doinker = 1;
    this_thread::sleep_for(300);
    Turn_PID.setGains(6, 15, 30);
    PIDTurnAbsolute(0);
    Turn_PID.setGains(4, 10, 30);
    Toggle_Doinker = 0;

    // Release Mogo Bottom Right
    target_heading = Odom.GetAngleToPointBack(144, 6);
    target_distance = Odom.GetDistanceToPoint(144, 6);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 5, 0);
    Toggle_Mogo = 0;
    PIDForward(10);

    //72, 72
    target_heading = Odom.GetAngleToPoint(72, 72);
    target_distance = Odom.GetDistanceToPoint(72, 72);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 

    // 96, 96
    target_heading = Odom.GetAngleToPoint(96, 96);
    target_distance = Odom.GetDistanceToPoint(96, 96);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 72 132
    target_heading = Odom.GetAngleToPoint(72, 132);
    target_distance = Odom.GetDistanceToPoint(72, 132);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    target_heading = Odom.GetAngleToPoint(72, 144);
    PIDTurn(target_heading);
    ForwardTimer(30, 500, 0);

    // Wall stakes alignment - y: 133.57 x: 72 heading: 0 (far side)
    Odom.SetX(72);
    Odom.SetY(133.57);
    Odom.SetHeadingDegrees(0);

    Intake_Power = 0;
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    this_thread::sleep_for(100);
    Intake_Power = 100;
    PIDForward(-10);
    ForwardTimer(30, 500, 0);
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    this_thread::sleep_for(100);
    PIDForward(-10);

    // 120, 132
    target_heading = Odom.GetAngleToPoint(120, 132);
    target_distance = Odom.GetDistanceToPoint(120, 132);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 132, 120
    target_heading = Odom.GetAngleToPoint(132, 120);
    target_distance = Odom.GetDistanceToPoint(132, 120);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 96, 132
    target_heading = Odom.GetAngleToPointBack(96, 132);
    target_distance = Odom.GetDistanceToPoint(96, 132);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 10, 0);
    PIDForward(-5);
    Toggle_Mogo = 1;

    // 136, 136
    target_heading = Odom.GetAngleToPoint(136, 136);
    target_distance = Odom.GetDistanceToPoint(136, 136);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);
    this_thread::sleep_for(50);
    Toggle_Doinker = 1;
    this_thread::sleep_for(300);

    // 140, 140
    target_heading = Odom.GetAngleToPointBack(140, 140);
    target_distance = Odom.GetDistanceToPoint(140, 140);
    Turn_PID.setGains(6, 15, 30);
    PIDTurn(target_heading);
    Turn_PID.setGains(4, 10, 30);
    Toggle_Doinker = 0;
    ForwardTillDestination(-60, target_distance - 10, 0);
    PIDForward(-5);
    Toggle_Mogo = 0;

    // 132, 72
    target_heading = Odom.GetAngleToPoint(132, 72);
    target_distance = Odom.GetDistanceToPoint(132, 72);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 144, 72
    target_heading = Odom.GetAngleToPoint(144, 72);
    target_distance = Odom.GetDistanceToPoint(144, 72);
    PIDTurn(target_heading);
    ForwardTimer(30, 500, 0);

    Intake_Power = 0;
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    this_thread::sleep_for(100);
    Intake_Power = 100;
    PIDForward(-10);
    ForwardTimer(30, 500, 0);
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(500);
    Ladybrown_Power = -100; 
    this_thread::sleep_for(100);
    PIDForward(-10);

    // 120, 72 (Get Mogo)
    PIDForward(-2);
    Toggle_Mogo = 1;

    // 96, 120
    target_heading = Odom.GetAngleToPoint(96, 120);
    target_distance = Odom.GetDistanceToPoint(96, 120);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 48, 120
    target_heading = Odom.GetAngleToPoint(48, 120);
    target_distance = Odom.GetDistanceToPoint(48, 120);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 48, 96
    target_heading = Odom.GetAngleToPoint(48, 96);
    target_distance = Odom.GetDistanceToPoint(48, 96);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 24, 120
    target_heading = Odom.GetAngleToPoint(24, 120);
    target_distance = Odom.GetDistanceToPoint(24, 120);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 12, 120
    target_heading = Odom.GetAngleToPoint(12, 120);
    target_distance = Odom.GetDistanceToPoint(12, 120);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 24, 132
    target_heading = Odom.GetAngleToPoint(24, 132);
    target_distance = Odom.GetDistanceToPoint(24, 132);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    Intake_Power = 0;

    // Top Left Corner
    // 0, 144
    target_heading = Odom.GetAngleToPointBack(0, 144);
    PIDTurn(target_heading);
    ForwardTimer(-60, 500, 0);
    Toggle_Mogo = 0;

    // 24, 96
    target_heading = Odom.GetAngleToPointBack(24, 96);
    target_distance = Odom.GetDistanceToPoint(24, 96);
    PIDForward(10);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 10, 0);
    PIDForward(-5);
    Toggle_Mogo = 1;

    // 48, 24
    Intake_Power = 100;
    target_heading = Odom.GetAngleToPoint(48, 24);
    target_distance = Odom.GetDistanceToPoint(48, 24);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 24, 24
    target_heading = Odom.GetAngleToPoint(24, 24);
    target_distance = Odom.GetDistanceToPoint(24, 24);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 12, 24
    target_heading = Odom.GetAngleToPoint(12, 24);
    target_distance = Odom.GetDistanceToPoint(12, 24);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 24, 12
    target_heading = Odom.GetAngleToPoint(24, 12);
    target_distance = Odom.GetDistanceToPoint(24, 12);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // Bottom Left Corner
    // 0, 0
    target_heading = Odom.GetAngleToPointBack(0, 0);
    PIDTurn(target_heading);
    ForwardTimer(-60, 500, 0);
    
    // Hang
    // 60, 60
    macro_mode = 1;
    Ladybrown_Arm_Height = DESCORE_HEIGHT;
    target_heading = Odom.GetAngleToPointBack(60, 60);
    target_distance = Odom.GetDistanceToPoint(60, 60);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance, 0);

}

void OrientationTest(){
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(17);
    Odom.SetY(72);
    Odom.SetHeadingDegrees(270);

    PIDTurnAbsolute(0);
}