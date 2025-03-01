#include "config.hpp"

inline void awpRed() {
    chassis.setPose(-56, 11.5, -120);
    // arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(-20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    // arm.retract();
    //  chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(-10, 43, 2000, {}, false);
    pros::delay(30);

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(-40, 0, 2000, {}, false);
    chassis.moveToPoint(-40, 0, 3000,
                        {
                            .maxSpeed = 90,
                            .minSpeed = 50,
                            .earlyExitRange = 5,
                        },
                        false);

    // chassis.cancelAllMotions();
    // while(true){
    //     pros::delay(200);
    // }

    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(-44, -46, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(-21, -25, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-21, -25, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(25);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);

    // scoring next ring
    chassis.moveToPoint(-23, -50, 2000, {.maxSpeed = 65}, false);

    // touching bar
    chassis.turnToPoint(-13, -24, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-13, -24, 2000, {.minSpeed = 60}, false);
}

inline void awpBlue() {
    chassis.setPose(56, 11.5, 120);
    // arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    // arm.retract();
    //  chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(10, 43, 2000, {}, false);
    pros::delay(30);

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(40, 0, 2000, {}, false);
    chassis.moveToPoint(40, 0, 3000,
                        {
                            .maxSpeed = 90,
                            .minSpeed = 50,
                            .earlyExitRange = 5,
                        },
                        false);

    // chassis.cancelAllMotions();
    // while(true){
    //     pros::delay(200);
    // }

    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(44, -46, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(21, -25, 1000, {.forwards = false}, false);
    chassis.moveToPoint(21, -25, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(25);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);

    // scoring next ring
    chassis.moveToPoint(23, -50, 2000, {.maxSpeed = 65}, false);

    // touching bar
    chassis.turnToPoint(13, -24, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}

inline void halfAwpRed() {
    chassis.setPose(-56, 11.5, -120);
    // arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(-20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    // arm.retract();
    //  chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(-10, 43, 2000, {}, false);
    pros::delay(30);

    // touch bar
    chassis.turnToPoint(-22, 26, 2000, {}, false);
    chassis.moveToPoint(-22, 26, 3000,
                        {
                            .maxSpeed = 80,
                        },
                        false);
    chassis.moveToPoint(-19, 10, 2000, {.maxSpeed = 70});
}

inline void halfAwpBlue() {
    chassis.setPose(56, 11.5, 120);
    // arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    // arm.retract();
    //  chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(10, 43, 2000, {}, false);
    pros::delay(30);

    // touch bar
    chassis.turnToPoint(22, 26, 2000, {}, false);
    chassis.moveToPoint(22, 26, 3000,
                        {
                            .maxSpeed = 80,
                        },
                        false);
    chassis.moveToPoint(19, 10, 2000, {.maxSpeed = 70});
}