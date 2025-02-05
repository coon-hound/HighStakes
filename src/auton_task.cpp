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