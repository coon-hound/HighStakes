#include "auton_task.h"
#include "definitions.h"
#include "auton_functions.h"

void BlueMogo() {
    Team_Color = BLUE;
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(122.80);
    Odom.SetY(38.18);
    Odom.SetHeadingDegrees(251.29);

    target_distance = Odom.GetDistanceToPoint(72, 24);

    Intake_Power = 100;
    ForwardTillDestination(100, target_distance - 22, 0); 
    target_distance = Odom.GetDistanceToPoint(72, 24);
    Toggle_Doinker = 1;
    PIDForward(2);

    Intake_Power = 0;    

    this_thread::sleep_for(100);

    ForwardTillDestination(-100, 20, 0);
    PIDForward(-5);
    Toggle_Doinker = 0;

    this_thread::sleep_for(250);

    // reintake mogo
    target_heading = Odom.GetAngleToPointBack(71, 29);
    printf("target_heading = %f\n", target_heading);
    PIDTurn(target_heading);
    // score ring
    ForwardTillDestination(-60, 15, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    this_thread::sleep_for(100);

    target_heading = Odom.GetAngleToPoint(144.0, 0.0);
    target_distance = Odom.GetDistanceToPoint(144.0, 0.0);

    PIDTurn(target_heading);
    ForwardTimer(80, 1500, 0);

    // second ring
    // ForwardTillDestination(-40, 7, 0); 
    // PIDForward(-7);
    ForwardTimer(-50, 300, 0);

    ForwardTimer(60, 750, 0);

    // third ring
    // ForwardTillDestination(-40, 7, 0); 
    ForwardTimer(-50, 300, 0);
    // PIDForward(-7);
    ForwardTimer(60, 750, 0);


    Odom.SetCornerPosition();
    printf("x = %f, y = %f, heading%f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());
    
    PIDForward(-10);

    Intake_Power = -100;
    this_thread::sleep_for(100);
    Intake_Power = 100;


    target_heading = Odom.GetAngleToPoint(130.0, 30.0);
    target_distance = Odom.GetDistanceToPoint(130.0, 30.0);

    PIDTurn(target_heading);
    this_thread::sleep_for(500);
    Toggle_Mogo = 0;
    Intake_Power = 100;
    PIDForward(target_distance + 15);
    Intake_Power = 0;

    target_heading = Odom.GetAngleToPointBack(96, 48);
    target_distance = Odom.GetDistanceToPoint(96, 48);

    PIDTurn(target_heading);
    ForwardTillDestination(-30, target_distance - 5, 0);
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
    Team_Color = BLUE;
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(132.58);
    Odom.SetY(85.70);
    Odom.SetHeadingDegrees(149.74);

    Ladybrown_Power = 100;
    this_thread::sleep_for(300);
    PIDForward(-10);
    Ladybrown_Power = -100;


    target_heading = Odom.GetAngleToPointBack(96, 96);
    target_distance = Odom.GetDistanceToPoint(96, 96);

    PIDTurn(target_heading + 5);
    ForwardTillDestination(-60, target_distance - 20, 0);
    // score ring
    ForwardTillDestination(-30, 18, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    PIDForward(8);

    target_heading = Odom.GetAngleToPoint(75, 120);
    target_distance = Odom.GetDistanceToPoint(75, 120);

    PIDTurn(target_heading);
    PIDForward(target_distance - 11);

    PIDTurnAbsolute(0);
    Intake_Power = 100;
    ForwardTillDestination(70, 9, 0);
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
    ForwardTimer(80, 1500, -5);

    printf("after 1st curr_heading = %f\n", Odom.GetHeadingDegrees());

    // second ring
    // ForwardTillDestination(-40, 7, 0); 
    // PIDForward(-7);
    ForwardTimer(-50, 300, 0);

    ForwardTimer(60, 750, 0);

    // third ring
    // ForwardTillDestination(-40, 7, 0); 
    ForwardTimer(-50, 300, 0);
    // PIDForward(-7);
    ForwardTimer(60, 750, 0);

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

    ForwardTillDestination(100, target_distance + 10, 0);

    Toggle_Intake_Lift = 0;
    this_thread::sleep_for(500);
    PIDForward(-10);

    while (1) {
        this_thread::sleep_for(10);
    }
}

void RedRing() {
    Team_Color = RED;
    double target_heading = 0;
    double target_distance = 0;

    Odom.SetX(132.58);
    Odom.SetY(85.70);
    Odom.SetHeadingDegrees(149.74);

    Ladybrown_Power = 100;
    this_thread::sleep_for(300);
    PIDForward(-10);
    Ladybrown_Power = -100;


    target_heading = Odom.GetAngleToPointBack(96, 96);
    target_distance = Odom.GetDistanceToPoint(96, 96);

    PIDTurn(target_heading + 5);
    ForwardTillDestination(-60, target_distance - 20, 0);
    // score ring
    ForwardTillDestination(-30, 18, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    PIDForward(8);

    target_heading = Odom.GetAngleToPoint(75, 120);
    target_distance = Odom.GetDistanceToPoint(75, 120);

    PIDTurn(target_heading);
    PIDForward(target_distance - 11);

    PIDTurnAbsolute(0);
    Intake_Power = 100;
    ForwardTillDestination(70, 9, 0);
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
    ForwardTimer(80, 1500, -5);

    printf("after 1st curr_heading = %f\n", Odom.GetHeadingDegrees());

    // second ring
    // ForwardTillDestination(-40, 7, 0); 
    // PIDForward(-7);
    ForwardTimer(-50, 300, 0);

    ForwardTimer(60, 750, 0);

    // third ring
    // ForwardTillDestination(-40, 7, 0); 
    ForwardTimer(-50, 300, 0);
    // PIDForward(-7);
    ForwardTimer(60, 750, 0);

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

    ForwardTillDestination(100, target_distance + 10, 0);

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
    this_thread::sleep_for(500);
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
    // PIDForward(10);
     
    ForwardTillDestination(60, target_distance - 10, 0);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT;
    PIDForward(5);

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
    printf("1 x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());
    Intake_Power = 0;
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(600);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    this_thread::sleep_for(100);
    Intake_Power = 100;
    PIDForward(-10);
    ForwardTimer(30, 500, 0);

    printf(" 3x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());

    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(600);
    Ladybrown_Power = -100;
    this_thread::sleep_for(100);

    Odom.SetX(72);
    Odom.SetY(10.43);
    //Odom.SetHeadingDegrees(180);

    PIDForward(-10);
    printf(" 4x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());

    // 120, 12
    target_heading = Odom.GetAngleToPoint(120, 12);
    target_distance = Odom.GetDistanceToPoint(120, 12);
    printf(" 5x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());
    printf("target_heading = %f\n", target_heading);
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

    // 144, 0
    target_heading = Odom.GetAngleToPoint(144, 0);
    PIDTurn(target_heading); 
    ForwardTimer(60, 500, 0);

    Odom.SetCornerPosition();

    // 136, 12 (Clear Corner)
    PIDForward(-8);
    this_thread::sleep_for(50);
    Toggle_Doinker = 1;
    this_thread::sleep_for(300);
    Turn_PID.setGains(6, 15, 30);
    PIDTurnAbsolute(0);
    Turn_PID.setGains(4, 10, 30);
    Toggle_Doinker = 0;

    // Release Mogo Bottom Right
    target_heading = Odom.GetAngleToPointBack(140, 4);
    target_distance = Odom.GetDistanceToPoint(140, 4);
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

    // Wall stakes alignment - y: 133.62 x: 72 heading:0 (false)
    printf("1 x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());
    Intake_Power = 0;
    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(600);
    macro_mode = 1;
    Ladybrown_Arm_Height = LOAD_HEIGHT; 
    this_thread::sleep_for(100);
    Intake_Power = 100;
    PIDForward(-10);
    ForwardTimer(30, 500, 0);

    printf(" 3x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());

    macro_mode = 0;
    Ladybrown_Power = 100;
    this_thread::sleep_for(600);
    Ladybrown_Power = -100;
    this_thread::sleep_for(100);

    Odom.SetX(72);
    Odom.SetY(133.62);
    //Odom.SetHeadingDegrees(180);

    PIDForward(-10);
    printf(" 4x = %f\n, y = %f, heading = %f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());

    // 96, 96
    target_heading = Odom.GetAngleToPointBack(96, 96);
    target_distance = Odom.GetDistanceToPoint(96, 96);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 10, 0);
    PIDForward(-5);

    // 132, 96
    target_heading = Odom.GetAngleToPointBack(132, 96);
    target_distance = Odom.GetDistanceToPoint(132, 96);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 10, 0);
    PIDForward(-5);
    Toggle_Mogo = 1;

    // Clearing Top Right Corner
    // 136, 136
    target_heading = Odom.GetAngleToPoint(136, 136);
    target_distance = Odom.GetDistanceToPoint(136, 136);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);

    // 144, 144
    target_heading = Odom.GetAngleToPoint(144, 144);
    target_distance = Odom.GetDistanceToPoint(144, 144);

    ForwardTimer(60, 500, 0);

    Odom.SetCornerPosition();

    /*
    // 136, 12 (Clear Corner)
    PIDForward(-8);
    this_thread::sleep_for(50);
    Toggle_Doinker = 1;
    this_thread::sleep_for(300);
    Turn_PID.setGains(6, 15, 30);
    PIDTurnAbsolute(270);
    Turn_PID.setGains(4, 10, 30);
    Toggle_Doinker = 0;
    */ 

   PIDForward(-8);

    // Release Mogo Top Right
    target_heading = Odom.GetAngleToPointBack(140, 140);
    target_distance = Odom.GetDistanceToPoint(140, 140);
    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 5, 0);
    Toggle_Mogo = 0;
    PIDForward(10);

    // 132, 72
    target_heading = Odom.GetAngleToPoint(132, 72);
    target_distance = Odom.GetDistanceToPoint(132, 72);
    PIDTurn(target_heading);
    ForwardTillDestination(60, target_distance - 10, 0);
    PIDForward(5);

    // 120, 72 (Get Mogo)
    Odom.GetAngleToPointBack(96, 72);
    PIDTurn(target_heading);
    PIDForward(-15);
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