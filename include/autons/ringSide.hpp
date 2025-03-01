#include "config.hpp"

inline void elimBlueTopSide() {
    chassis.setPose(56, 11.5, 120);

    chassis.moveToPoint(21, 27, 2000, {.forwards = false, .maxSpeed = 50});
    // chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    chassis.waitUntil(30);
    clamp.set_value(true);

    intake.set(Intake::IntakeState::INTAKING, 127);

    chassis.turnToPoint(14, 40, 2000, {}, false);
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPose(-8.4, 50,0, 5000,{.minSpeed = 20},false);
    chassis.moveToPoint(14, 40, 2000, {.minSpeed = 50}, false);
    pros::delay(250);
    chassis.moveToPoint(8, 52, 2000, {}, false);
    // chassis.moveFor(7,2000,{.forwards = false},false);

    chassis.turnToPoint(26, 48, 2000, {}, true); // 30

    chassis.moveToPoint(26, 48, 2000, {.minSpeed = 80}, false);
    pros::delay(1000);
    intake.set(Intake::IntakeState::OUTTAKE, 127);

    chassis.moveToPose(60, 17, 180, 2000, {}, false);
    // chassis.waitUntil(5);
    // intake.set(Intake::IntakeState::OUTTAKE,127);
    // chassis.waitUntilDone();
    hook.set_value(true);
    pros::delay(250);
    chassis.moveFor(10, 2000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::INTAKING);
    hook.set_value(false);
    chassis.moveToPoint(50, 14, 2000, {}, false); // -51,12

    chassis.turnToPoint(54, 54, 2000, {}, false); //-53
    chassis.moveToPose(54, 54, 24, 2000, {}, false);
    hook.set_value(true);
    pros::delay(500);
    chassis.turnToHeading(-90, 2000, {.minSpeed = 127}, false);

    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}

inline void elimRedTopSide() {
    chassis.setPose(-56, 11.5, -120);

    chassis.moveToPoint(-21, 27, 2000, {.forwards = false, .maxSpeed = 50});
    // chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    chassis.waitUntil(30);
    clamp.set_value(true);

    intake.set(Intake::IntakeState::INTAKING, 127);

    chassis.turnToPoint(-12, 40, 2000, {}, false);
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPose(-8.4, 50,0, 5000,{.minSpeed = 20},false);
    chassis.moveToPoint(-12, 40, 2000, {.minSpeed = 50}, false);
    pros::delay(250);
    chassis.moveToPoint(-10, 52, 2000, {}, false);
    chassis.moveFor(7, 2000, {.forwards = false}, false);

    chassis.turnToPoint(-28, 48, 2000, {}, true); // 30

    chassis.moveToPoint(-26, 48, 2000, {.minSpeed = 50}, false);
    chassis.moveToPose(-45, 17, 180, 2000, {}, false);

    hook.set_value(true);
    pros::delay(100);
    chassis.moveFor(10, 2000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::INTAKING);
    hook.set_value(false);
    chassis.moveToPoint(-51, 14, 2000, {}, false); // -51,12

    chassis.turnToPoint(-54, 60, 2000, {}, false); //-53
    chassis.moveToPoint(-54, 61, 2000, {}, true); //-53
    chassis.waitUntil(10);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToHeading(-75, 2000, {}, true);
    chassis.waitUntil(60);
    hook.set_value(true);
    pros::delay(500);
    chassis.swingToHeading(180, DriveSide::LEFT, 2000, {.minSpeed = 127}, false);

    // chassis.turnToPoint(-68,69,2000,{},false);
    // chassis.moveFor(60,2000,{.maxSpeed = 127},true);
    // chassis.waitUntil(10);
    // intake.set(Intake::IntakeState::OUTTAKE,127);
    // pros::delay(1000);
    // intake.set(Intake::IntakeState::INTAKING,127);
    // chassis.moveFor(10,2000,{.maxSpeed = 127},true);
    // chassis.moveFor(10,2000,{.forwards = false},true);
    // chassis.moveFor(5,2000,{},false);
    // chassis.moveFor(5,2000,{.forwards = false},false);

    // chassis.moveToPoint(-12,63,3000,{},false);
    // intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // chassis.moveFor(10,2000,{.forwards = false},false);
    // arm.retract();
    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}