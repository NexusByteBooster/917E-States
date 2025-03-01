// files
#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include <iostream>
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "autons/contest.hpp"
#include "autons/quals.hpp"
#include "autons/tune.hpp"
#include "autons/skills.hpp"
#include "autons/ringSide.hpp"

// runtime variables
bool sort = false;
double fwd;
double turning;
float up;
float down;
bool clamped = false;
bool hooked = false;
int armState = 0;
int separationState = 0;
int autoSelector = 0;

void printTelemetry() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %.1f %.1f %.1f", leftMotors.get_temperature(0),
                            leftMotors.get_temperature(1), leftMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %.1f %.1f %.1f", rightMotors.get_temperature(0),
                            rightMotors.get_temperature(1), rightMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %.1f", intake.motor.get_temperature());

        switch (autoSelector) {
            case 0: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Skills"); break;
            case 1: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: AWP Red"); break;
            case 2: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: AWP Blue"); break;
            case 3: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Half AWP Red"); break;
            case 4: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Half AWP Blue"); break;
            case 5: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Mogo Rush Red"); break;
            case 6: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Mogo Rush Blue"); break;
            case 7: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Elim Red Top Side"); break;
            case 8: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Elim Blue Top Side"); break;
            default: break;
        }

        std::cout << pose.x << " " << pose.y << " " << imu.get_rotation() << pose.theta << std::endl;
        switch (intake.ring) {
            case Intake::Ring::BLUE: controller.print(1, 1, "%s", "SEPARATING BLUE"); break;
            case Intake::Ring::RED: controller.print(1, 1, "%s", "SEPARATING RED"); break;
            case Intake::Ring::NONE: controller.print(1, 1, "%s", "SEPARATING NONE"); break;
            default: break;
        }

        pros::delay(200);
    }
}
void init_separation(Intake::Ring ring) {
    intakeMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    topSort.set_led_pwm(100);
    intake.ring = ring;
    if (intake.ring == Intake::Ring::BLUE) {
        separationState = 1;
    } else if (intake.ring == Intake::Ring::RED) {
        separationState = 0;
    } else if (intake.ring == Intake::Ring::NONE) {
        separationState = 2;
    }
}
void initialize() {
    pros::delay(1000);
    chassis.calibrate(); // calibrate the chassis
    pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values
    pros::Task task {[=] { intake.intakeControl(); }}; // create a task to control the intake
    arm.initialize(); // initialize the cata object
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // halfAwpRed();
    // halfAwpBlue();
    // mogoRushRed();
    // mogoRushBlue();
    // awpRed();
    // awpBlue();
    // elimRedTopSide();
    // elimBlueTopSide();
    // prog();
    // chassis.moveFor(12,2000,{},false);

    switch (autoSelector) {
        case 0: skills(); break;
        case 1: awpRed(); break;
        case 2: awpBlue(); break;
        case 3: halfAwpRed(); break;
        case 4: halfAwpBlue(); break;
        case 5: mogoRushRed(); break;
        case 6: mogoRushBlue(); break;
        case 7: elimRedTopSide(); break;
        case 8: elimBlueTopSide(); break;
        default: break;
    }
}

void arcadeCurve(pros::controller_analog_e_t power, pros::controller_analog_e_t turn, pros::Controller mast, float f) {
    up = mast.get_analog(power);
    down = mast.get_analog(turn);
    fwd = (exp(-f / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-f / 10))) * up;
    turning = -1 * down;
    leftMotors.move(fwd * 0.9 - turning);
    rightMotors.move(fwd * 0.9 + turning);
}

void opAsyncButtons() {
    while (true) {
        // toggle clamp
        if (controller.get_digital(DIGITAL_Y)) {
            clamped = !clamped;
            clamp.set_value(clamped);
            pros::delay(500);
        }

        if (controller.get_digital(DIGITAL_B)) {
            hooked = !hooked;
            hook.set_value(hooked);
            pros::delay(500);
        }

        if (controller.get_digital(DIGITAL_R2)) {
            armState++;
            if (armState > 2) { armState = 0; }
            switch (armState) {
                case 0: arm.retract(5); break;
                case 1: arm.loadWallstake(32); break;
                case 2:
                    intake.set(Intake::IntakeState::OUTTAKE, 50);
                    arm.scoreWallstake();
                    intake.set(Intake::IntakeState::STOPPED);
                    break;
                default: break;
            }
        }
        if (controller.get_digital(DIGITAL_A)) {
            arm.scoreWallstake(185);
            armState = 1;
        }

        pros::delay(20);
    }
}
void switchSeparation() {
    if (controller.get_digital(DIGITAL_UP)) {
        separationState++;
        if (separationState > 2) { separationState = 0; }
        switch (separationState) {
            case 0: intake.setSeparation(Intake::Ring::RED); break;
            case 1: intake.setSeparation(Intake::Ring::BLUE); break;
            case 2: intake.setSeparation(Intake::Ring::NONE); break;
        }
        pros::delay(500);
    }
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);

    while (true) {
        switchSeparation();
        arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, controller, 9.6);

        if (controller.get_digital(DIGITAL_R1)) // intake
        {
            intake.set(Intake::IntakeState::INTAKING);
        }
        else if (controller.get_digital(DIGITAL_R2)) // outtake
        {
            intake.set(Intake::IntakeState::OUTTAKE);
        }
        else if (controller.get_digital(DIGITAL_L1) == false && controller.get_digital(DIGITAL_L2) == false &&
            controller.get_digital(DIGITAL_R2) == false) // stop intake
        {
            intake.set(Intake::IntakeState::STOPPED);
        }
        if (controller.get_digital(DIGITAL_LEFT)) {
            autoSelector++;
            pros::delay(1000);
            if (autoSelector > 8) { autoSelector = 0; }
        }
        
        pros::delay(10);
    }
}