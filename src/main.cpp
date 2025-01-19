/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Aaron                                                     */
/*    Created:      1/3/2025, 3:13:07 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "port_config.h"
#include "definitions.h"
#include "autonfunctions.h"
#include "threads.h"
#include "pid.h"
#include <stdio.h>

// wait 3 secs before moving the robot for the gyro to calibrate

using namespace vex;

thread HOOK_HELPER(hook_helper_thread);
thread HOOKS(hook_thread);
thread LADYBROWN(lb_thread);
thread DRIVETRAIN(drivetrain_thread);
thread ODOM(odom_thread);


int main() {
    printf("poop\n");
    Color.setLightPower(100);
    Color.objectDetectThreshold(128);
    Color.setLight(ledState::on);
    Intake.resetPosition();
    Ladybrown1.resetPosition();
    Ladybrown2.resetPosition();

    RightMotor1.resetPosition();
    RightMotor2.resetPosition();
    RightMotor3.resetPosition();

    LeftMotor1.resetPosition();
    LeftMotor2.resetPosition();
    LeftMotor3.resetPosition();


    Ladybrown1.setStopping(coast);
    Ladybrown2.setStopping(coast);


    bool prev_l1_state = false;
    bool prev_l2_state = false;

    bool prev_A_state = false;

    int descore_state = 0;

    // Right_Power = 10;
    // Left_Power = 10;
    // this_thread::sleep_for(2000);

    // ForwardTillDestination(100.0, 100.0, 0); 
    // PIDForward(10);
    
    // PIDTurn(90);
    
    // while (1) {
    //     ForwardWithCorrection(20, 0, 1.0, 10);
    //     this_thread::sleep_for(10);
    // }
    // Right_Power = 0;
    // Left_Power = 0;
    while(1) {
        int axis3_power = Controller.Axis3.position();
        int axis1_power = Controller.Axis1.position();

        Right_Power = axis3_power - axis1_power; 
        Left_Power = axis3_power + axis1_power;

        if (Controller.ButtonR1.pressing()) {
            Intake_Power = 100;
            if (ladybrown_pos > FLOAT_HEIGHT - 20) {
                Store_Mode = !Controller.ButtonR2.pressing();
            }
        } else if (Controller.ButtonR2.pressing()) {
            Intake_Power = -100;
        } else {
            Intake_Power = 1;
        } 

        if (Controller.ButtonB.pressing()) {
            Intake_Lift.close();
        } else {
            Intake_Lift.open();
        }

        // mogo control code
        if (Controller.ButtonA.pressing() != prev_A_state && prev_A_state != 1){
            Toggle_Mogo = !Toggle_Mogo;
        }
        
        prev_A_state = Controller.ButtonA.pressing(); 


        // printf("lb pos = %f\n", ladybrown_pos);

        // Ladybrown_Power = ladybrown_pid.calculate(ladybrown_pos);

        if (Controller.ButtonL1.pressing()) {
            Ladybrown_Power = 100;
        } else if (Controller.ButtonL2.pressing()) {
            Ladybrown_Power = -100;
        } else {
            Ladybrown_Power = 0;
        }

        if(Controller.ButtonL1.pressing() != prev_l1_state && !prev_l1_state) {
            if (Controller.ButtonL2.pressing()) {
                printf("trigger\n");
                macro_mode = 1;
                Ladybrown_Arm_Height = LOAD_HEIGHT;
            } else {
                macro_mode = 0;
            }
        }
        
        if(Controller.ButtonL2.pressing() != prev_l2_state && !prev_l2_state) {
            if (Controller.ButtonL1.pressing()) {
                printf("trigger\n");
                macro_mode = 1;
                descore_state ++;
                printf("descore_state = %d\n", descore_state);
                if (descore_state == 1) {
                    Ladybrown_Arm_Height = DESCORE_HEIGHT;
                } else if (descore_state >= 2) {
                    Ladybrown_Arm_Height = DESCORE_TWO_HEIGHT;
                }
            } else {
                descore_state = 0;
                macro_mode = 0;
            }
        }

        if (macro_mode) {
            ladybrown_pid.setSetpoint(Ladybrown_Arm_Height);
        }

        prev_l1_state = Controller.ButtonL1.pressing();
        prev_l2_state = Controller.ButtonL2.pressing();
        this_thread::sleep_for(10);
    }

    return 0;
}
