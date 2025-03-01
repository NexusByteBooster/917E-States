#include <cmath>
#include "pros/rtos.hpp"
#include "lemlib/exitcondition.hpp"

namespace lemlib {
ExitCondition::ExitCondition(const float range, const int time)
    : range(range),
      time(time) {}

bool ExitCondition::getExit() { return done; }

bool ExitCondition::update(const float input) {
    const int curTime = pros::millis(); // get starting time
    if (std::fabs(input) > range) startTime = -1;
    else if (startTime == -1) startTime = curTime;
    else if (curTime >= startTime + time) done = true;
    return done; // return if exit condition has been met for a certain period since the starting time
}

void ExitCondition::reset() {
    startTime = -1;
    done = false;
}
} // namespace lemlib