#include "config.hpp"

inline void tune(){
    chassis.setPose(0,0,0);
    chassis.moveFor(24, 10000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveFor(24, 10000, {.forwards=false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveFor(24, 10000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveFor(24, 10000, {.forwards=false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.angularPID.setGains(5.2, 0, 45);
    chassis.turnToHeading(90, 10000);
    chassis.waitUntilDone();

    chassis.turnToHeading(0,10000);
    chassis.waitUntilDone();
    chassis.lateralPID.setGains(26,0,150);

    chassis.moveToPoint(0,-30, 10000, {.forwards=false, .maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.moveToPoint(0,0, 10000, {.maxSpeed = 80});
    chassis.waitUntilDone();
}