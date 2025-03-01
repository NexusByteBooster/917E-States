#include "config.hpp"

inline void mogoRushRed() {
    chassis.setPose(-54, -36, 90);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPose(-10, -43, 90, 2000, {.maxSpeed = 90}, true);
    // arm.loadWallstake();
    chassis.waitUntil(34);
    hook.set_value(true);
    // chassis.moveFor(30,2000,{.forwards = false},true);
    // chassis.waitUntil(25);
    // intake.set(Intake::IntakeState::STOPPED);
    // chassis.waitUntilDone();
    chassis.turnToHeading(-8, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    hook.set_value(false);
    // chassis.turnToPoint(24,-15,2000,{.forwards = false},false);
    chassis.moveToPose(-20, -17, 180, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(33);
    clamp.set_value(true);
    chassis.moveToPoint(-10, -57, 2000, {.maxSpeed = 70}, false); // was 4,-61
    chassis.turnToHeading(+90 + 39, 2000, {}, false); // was -90-42
    chassis.moveFor(7.1, 2000, {}, false); // 7.2
    chassis.turnToHeading(90 + 45, 1000, {}, false); // was 48
    intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    intake.set(Intake::IntakeState::OUTTAKE, 90);
    pros::delay(200);
    chassis.moveFor(10, 2000, {.forwards = false});
    chassis.waitUntil(5);
    // arm.retract();
    chassis.turnToPoint(-58, -26, 2000, {}, true);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(-59, -25, -90, 2000, {}, true);
    chassis.moveFor(25, 2000, {.forwards = false}, false);
    pros::delay(500);
    clamp.set_value(false);
    // chassis.moveFor(10, 2000, {},false);
    chassis.turnToHeading(180, 2000, {}, false);
}

inline void mogoRushBlue() {
    chassis.setPose(54, -36, -90);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPose(12, -50, -105, 2000, {.maxSpeed = 100}, true);
    // arm.loadWallstake();
    chassis.waitUntil(34);
    hook.set_value(true);
    // chassis.moveFor(30,2000,{.forwards = false},true);
    // chassis.waitUntil(25);
    // intake.set(Intake::IntakeState::STOPPED);
    // chassis.waitUntilDone();
    chassis.turnToHeading(90, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}, false);
    hook.set_value(false);
    // chassis.turnToPoint(24,-15,2000,{.forwards = false},false);
    chassis.moveToPoint(14, -47, 2000, {.forwards = false, .maxSpeed = 90, .minSpeed = 40, .earlyExitRange = 10},
                        false);
    chassis.moveToPose(23, -19, -180, 2000, {.forwards = false, .maxSpeed = 70}, true); // was 23,-20 // was 21, -14.5
    chassis.waitUntil(32);
    clamp.set_value(true);
    chassis.moveToPoint(7, -60, 3000, {.maxSpeed = 80}, false); // was 4,-61
    chassis.turnToHeading(-90 - 48, 2000, {}, false); // was -90-48
    chassis.moveFor(3, 2000, {}, false);
    chassis.turnToHeading(-90 - 47, 500, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    intake.set(Intake::IntakeState::OUTTAKE, 90);
    pros::delay(200);
    chassis.moveFor(10, 2000, {.forwards = false});
    chassis.waitUntil(5);
    // arm.retract();
    chassis.turnToPoint(30, -35, 2000, {}, true);
    intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPose(58,-28, 90,2000,{},false);
    chassis.moveToPoint(30, -35, 2000, {.minSpeed = 50, .earlyExitRange = 6}, false);
    chassis.moveToPoint(58, -28, 2000, {}, false);
    chassis.moveFor(15, 2000, {.forwards = false}, false);
    pros::delay(500);
    clamp.set_value(false);
    chassis.moveFor(10, 2000, {}, false);
}