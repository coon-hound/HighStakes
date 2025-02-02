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
    ForwardTillDestination(100, target_distance - 19, 0); 
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

    target_heading = Odom.GetAngleToPoint(120.0, 8.0);
    target_distance = Odom.GetDistanceToPoint(120, 8.0);

    PIDTurn(target_heading);
    ForwardTillDestination(50, target_distance - 3, 0); 
    ForwardTimer(30, 1000, 0);

    // second ring
    ForwardTillDestination(-20, 7, 0); 
    ForwardTimer(50, 750, 0);

    // third ring
    ForwardTillDestination(-20, 7, 0); 
    ForwardTimer(50, 750, 0);
    
    PIDForward(-10);


    target_heading = Odom.GetAngleToPoint(130.0, 30.0);
    target_distance = Odom.GetDistanceToPoint(130.0, 30.0);

    PIDTurn(target_heading);
    Toggle_Mogo = 0;
    Intake_Power = 100;
    PIDForward(target_distance + 10);
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
    this_thread::sleep_for(500);
    Ladybrown_Power = -100;
    this_thread::sleep_for(1000);

    PIDForward(-10);

    target_heading = Odom.GetAngleToPointBack(96, 96);
    target_distance = Odom.GetDistanceToPoint(96, 96);

    PIDTurn(target_heading);
    ForwardTillDestination(-60, target_distance - 20, 0);
    // score ring
    ForwardTillDestination(-30, 15, 0);
    Intake_Power = 100;
    Toggle_Mogo = 1;

    target_heading = Odom.GetAngleToPoint(75, 115);
    target_distance = Odom.GetDistanceToPoint(75, 115);

    PIDTurn(target_heading);
    PIDForward(target_distance - 7);

    PIDTurnAbsolute(0);
    Intake_Power = 100;
    ForwardTillDestination(50, 20, 0);

    target_heading = Odom.GetAngleToPoint(96, 120);
    target_distance = Odom.GetDistanceToPoint(96, 120);

    PIDTurn(target_heading);
    ForwardTillDestination(50, target_distance, 0);

    target_heading = Odom.GetAngleToPoint(125, 138.3);
    target_distance = Odom.GetDistanceToPoint(125, 138.3);

    PIDTurn(target_heading);
    ForwardTillDestination(50, target_distance - 5, 0);

    ForwardTimer(30, 1000, 0);

    // second ring
    ForwardTillDestination(-20, 7, 0); 
    ForwardTimer(50, 750, 0);

    // third ring
    ForwardTillDestination(-20, 7, 0); 
    ForwardTimer(50, 750, 0);
    
    PIDForward(-10);

    target_heading = Odom.GetAngleToPoint(120, 70);
    target_distance = Odom.GetDistanceToPoint(120, 70);

    PIDTurn(target_heading);

    Toggle_Intake_Lift = 1;

    PIDForward(target_distance);

    // ForwardTillDestination(100, target_distance, 0);

    



}