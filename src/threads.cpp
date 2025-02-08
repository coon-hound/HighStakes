#include "threads.h"
#include "definitions.h"

// 1 red, 0 blue
static bool detectRed(double hue) {
    if (hue <= 170 || hue >= 250) {
        return true;
    }

    return false;
}

/*
    nearobject and queue is empty | push
    !queue.empty near object and curr_stage != queue 
*/

int hook_helper_thread() {
    int ini_hue;
    while (1) {
        // if (!(!ring_queue.empty() && ring_queue.front().hooked_stage == stage) && hook_pos > 500 && hook_pos < 950 && Color.isNearObject()) {

        // push rings, make sure to push above position 700 so that the previous ring clears the sensor. 
        if (!(!ring_queue.empty() && ring_queue.front().hooked_stage == stage) && hook_pos > 700 && Color.isNearObject()) {
            Ring R;

            R.color = Team_Color;
            printf("pushed, encoder_value = %f, hook_pos = %f\n", intake_pos, hook_pos);
            ring_queue.push_front(R);
        }

// 650 - 950

        if (!ring_queue.empty() && ring_queue.front().getPos() > 400 && ring_queue.front().getPos() < 950) {
            // always take the second reading of color to ensure accuracy
            int hue = Color.hue();
            if (!ring_queue.front().entered_color_range) {
                // first reading
                ini_hue = hue;
                ring_queue.front().entered_color_range = true;
                bool isRed = detectRed(hue);
                if (isRed) {
                    ring_queue.front().color = RED;
                } else {
                    ring_queue.front().color = BLUE;
                }

                ring_queue.front().color_checked = true;
            }
            // if (ini_hue != hue && ring_queue.front().entered_color_range && !ring_queue.front().color_checked) {
            //     // second reading
            //     bool isRed = detectRed(hue);
            //     printf("hue = %d\n", hue);
            //     if (isRed == true) {
            //         ring_queue.front().color = RED;
            //     }
            //     else {
            //         ring_queue.front().color = BLUE;
            //     }

            //     ring_queue.front().color_checked = true;
            // } 
        }

        // pop rings
        if (!ring_queue.empty()) {
            // outtake
            if (ring_queue.front().getPos() < 30) {
                ring_queue.pop_front();
            }

            // poop
            if (ring_queue.back().getPos() > (2 * period) - 360) {
                Ring_Fired = 1;
               
            }
        }


        if (!ring_queue.empty()) {
            Brain.Screen.printAt(10, 80, "color %d\n", ring_queue.front().color);
        }
        Brain.Screen.printAt(10, 20, "%f\n", intake_pos);
        Brain.Screen.printAt(10, 60, "%d\n", ring_queue.size());

        this_thread::sleep_for(10);
    }
    return 0;
}

int hook_thread() {
    double prev_intake_pos = -50;
    double intake_start_time = 0;
    double prev_intake_power = 0;
    while (1) {
        if (!Ring_Fired) {
            // normal movement
            // store the ring at a stopping position
            // anti-jam

            if (Intake_Power > 10 && prev_intake_power <= 10) {
                intake_start_time = Timer.time();
            }

            // if (Intake_Power > 10 && fabs(intake_pos - prev_intake_pos) < .001 && Timer.time() - intake_start_time > 1000) {
            //     printf("intake_pos = %f, prev_intake_pos = %f\n", intake_pos, prev_intake_pos);
            //     printf("delta = %f\n", intake_pos - prev_intake_pos);
            //     MoveMotor(Intake, -100);
            //     this_thread::sleep_for(250);
            // }
            if (Store_Mode && Intake_Power > 0 && !ring_queue.empty() && ring_queue.back().getPos() > period + 60) {
                MoveMotor(Intake, 1);
            }
            else if (Ladybrown_Has_Ring && ladybrown_pos <= FLOAT_HEIGHT - 20) {
                MoveMotor(Intake, 1);
            }
            else {
                MoveMotor(Intake, Intake_Power);
            }
        } else {
            // eject ring
            if (ring_queue.empty()) {
                Ring_Fired = 0;
            }
            else {
                // if the ladybrown is at load height
                if (ladybrown_pos > LOAD_HEIGHT - 20 && ladybrown_pos < LOAD_HEIGHT + 20) {
                    // move the intake to a certain position
                    while (ring_queue.back().getPos() < (2 * period)-60) {
                        MoveMotor(Intake, 100);
                        this_thread::sleep_for(10);
                    }
                    MoveMotor(Intake, 100);
                    this_thread::sleep_for(100);

                    MoveMotor(Intake, -10);
                    this_thread::sleep_for(50);

                    MoveMotor(Intake, 100);
                    this_thread::sleep_for(50);

                    if (LadybrownDistance.objectDistance(mm) < LADYBROWN_DETECT_DISTANCE) {
                        Ladybrown_Has_Ring = 1;
                        MoveMotor(Intake, -10);
                        this_thread::sleep_for(100);
                    }
                }
                else if (ring_queue.back().color != Team_Color){
                    // fire sequence 
                    while (ring_queue.back().getPos() < (2 * period)-20) {
                        MoveMotor(Intake, 100);
                        this_thread::sleep_for(10);
                    }

                    MoveMotor(Intake, -100);
                    this_thread::sleep_for(100);
                }
                else {
                    // score sequence
                    MoveMotor(Intake, 100);
                    this_thread::sleep_for(250);
                }

                printf("size of queue before %d\n", ring_queue.size());
                printf("pop\n");
                ring_queue.pop_back();
                printf("size of queue after %d\n", ring_queue.size());
                Ring_Fired = 0;
            }

        }
        
        if (Toggle_Intake_Lift) {
            Intake_Lift.close();
        } else {
            Intake_Lift.open();
        }

        prev_intake_pos = intake_pos;
        prev_intake_power = Intake_Power;

        this_thread::sleep_for(10);
    }

    return 0;
}

/*
--------------------------------------------------- BUG ----------------------------------------
If we lift manually early before the feeding has finished will it be 
overwritten by the lb thread to store height?
*/

int lb_thread() {
    double prev_Ladybrown_Pos = 0;

    // hold L1 and lcikc L2 descore
    // hold l2 and click l1, load

    bool prev_Ladybrown_Has_Ring = false;

    while (1) {
        if (ladybrown_pos < 0) {
            Ladybrown1.resetPosition();
            Ladybrown2.resetPosition();
        } 

        if (ladybrown_pos > FLOAT_HEIGHT - 20 && prev_Ladybrown_Pos <= FLOAT_HEIGHT - 20) {
            Store_Mode = 1;
        } else if (ladybrown_pos <= FLOAT_HEIGHT - 20 && prev_Ladybrown_Pos > FLOAT_HEIGHT - 20) {
            Store_Mode = 0;
        } 

        // printf("Store_Mode =%d\n", Store_Mode);

        if (Ladybrown_Has_Ring && !prev_Ladybrown_Has_Ring) {
            macro_mode = 1;
            Ladybrown_Arm_Height = FLOAT_HEIGHT;
        }

        if (Ladybrown_Has_Ring && LadybrownDistance.objectDistance(mm) > LADYBROWN_DETECT_DISTANCE || ladybrown_pos > FLOAT_HEIGHT - 20) {
            Ladybrown_Has_Ring = false;
        }

        if (!macro_mode) {
            // manual control
            if (Ladybrown_Power <= 0 && ladybrown_pos < 100) {
                MoveMotor(Ladybrown1, -10);
                MoveMotor(Ladybrown2, -10);
            }
            else {
                MoveMotor(Ladybrown1, Ladybrown_Power);
                MoveMotor(Ladybrown2, Ladybrown_Power);
            }
        } else {
            // pid control
            ladybrown_pid.setSetpoint(Ladybrown_Arm_Height);
            Ladybrown_Power = ladybrown_pid.calculate(ladybrown_pos);
            MoveMotor(Ladybrown1, Ladybrown_Power);
            MoveMotor(Ladybrown2, Ladybrown_Power);
        }

       

      

        prev_Ladybrown_Pos = ladybrown_pos;
        prev_Ladybrown_Has_Ring = Ladybrown_Has_Ring; 


        this_thread::sleep_for(10);
    
    }
    return 0;
}

int drivetrain_thread() {
    while (1) {
        // printf("in thread: Right_power = %f, Left_power = %f\n", Right_Power, Left_Power);
        MoveMotor(RightMotor1, Right_Power);
        MoveMotor(RightMotor2, Right_Power);
        MoveMotor(RightMotor3, Right_Power);

        MoveMotor(LeftMotor1, Left_Power);
        MoveMotor(LeftMotor2, Left_Power);
        MoveMotor(LeftMotor3, Left_Power);

        if (Toggle_Mogo) {
            Mogomech.open();
        } else {
            Mogomech.close();
        }

        if (Toggle_Doinker) {
            Doinker.open();
        } else {
            Doinker.close();
        }

        this_thread::sleep_for(10);
    }
    return 0;
}

int odom_thread() {
    while(1) {
        Odom.UpdatePosition(); 

        // printf("x = %f, y = %f, heading%f\n", Odom.GetX(), Odom.GetY(), Odom.GetHeadingDegrees());

        this_thread::sleep_for(10);
    }
    return 0;
}

/*
    finish programming
    finish auton routing and routes
    finish meeting notes
    finish talbe of contents
    look at history and find all the tabs that have been editted since 4pm, and 
    get rid of all blank pages after printing
    number each page 
    binder it
*/