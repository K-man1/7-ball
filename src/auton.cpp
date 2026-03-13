#include "robot.h"

// ── Distance resets ──────────────────────────────────────────────────────────────
    void resetWithDistance(ResetWalls walls) {
        double heading = chassis.getPose().theta;

        // Determine which sensor faces which wall based on current heading
        // Normalize heading to 0-360
        double h = fmod(heading, 360.0);
        if (h < 0) h += 360.0;

        // Each sensor's effective facing = its physical offset + robot heading
        // At 0°:   left→left wall, back→bottom wall
        // At 90°:  left→top wall,  back→left wall
        // At 180°: left→right wall, back→top wall
        // At 270°: left→bottom wall, back→right wall

        auto getXSensor = [&]() -> double {
    if (h >= 315 || h < 45)   return dist_left.get() / 25.4;  // facing up,    left→left wall
    if (h >= 45  && h < 135)  return dist_back.get() / 25.4;  // facing right, back→left wall
    if (h >= 135 && h < 225)  return dist_right.get() / 25.4; // facing down,  right→left wall
    return dist_back.get() / 25.4;                             // facing left (270°), back→right wall
};

auto getYSensor = [&]() -> double {
    if (h >= 315 || h < 45)   return dist_back.get() / 25.4;  // facing up,    back→bottom wall
    if (h >= 45  && h < 135)  return dist_left.get() / 25.4;  // facing right, left→top wall
    if (h >= 135 && h < 225)  return dist_back.get() / 25.4;  // facing down,  back→top wall
    return dist_left.get() / 25.4;                             // facing left (270°), left→bottom wall
};

        if (walls == ResetWalls::LEFT || walls == ResetWalls::LEFT_TOP || walls == ResetWalls::LEFT_BOTTOM) {
            double newX = -(FIELD_SIZE / 2.0) + getXSensor() + DIST_LEFT_LATERAL_OFFSET;
            chassis.setPose(newX, chassis.getPose().y, heading);
        }

        if (walls == ResetWalls::RIGHT || walls == ResetWalls::RIGHT_TOP || walls == ResetWalls::RIGHT_BOTTOM) {
            double newX = (FIELD_SIZE / 2.0) - getXSensor() - DIST_RIGHT_LATERAL_OFFSET;
            chassis.setPose(newX, chassis.getPose().y, heading);
        }

        if (walls == ResetWalls::TOP || walls == ResetWalls::LEFT_TOP || walls == ResetWalls::RIGHT_TOP) {
            double newY = (FIELD_SIZE / 2.0) - getYSensor() - DIST_LEFT_LATERAL_OFFSET;
            chassis.setPose(chassis.getPose().x, newY, heading);
        }

        if (walls == ResetWalls::BOTTOM || walls == ResetWalls::LEFT_BOTTOM || walls == ResetWalls::RIGHT_BOTTOM) {
            double newY = -(FIELD_SIZE / 2.0) + getYSensor() + DIST_BACK_FB_OFFSET;
            chassis.setPose(chassis.getPose().x, newY, heading);
        }
    }
// ── Mechanism helpers ─────────────────────────────────────────────────────────
void load() {
    intake.move(127);
    up.set_value(false);
    down.set_value(true);
}

void score() {
    up.set_value(true);
    down.set_value(true);
    intake.move(-127);
    pros::delay(75);
    intake.move(127);
}

void middle(int speed) {
    intake.move(speed);
    up.set_value(false);
    down.set_value(false);
}

void outtake() {
    intake.move(-100);
}


// ── Autonomous ────────────────────────────────────────────────────────────────
void autonomous() {
    //set start position
    chassis.setPose(-54.5, 18, 90);
    load();
    pros::lcd::print(3, "left: %f", dist_left.get() / 25.4);
    pros::lcd::print(4, "back: %f", dist_back.get() / 25.4);
    //Intake 4 blocks 
    chassis.moveToPose(-23, 27, 315, 2000, {.maxSpeed = 75});
    //go into middle goal
    chassis.turnToHeading(315,1000);
    intake.move(0);
    chassis.moveToPose(-9.29,9.48,315,3000,{.forwards=false});
    pros::delay(2000);
    middle(75);
    pros::delay(3000);

    //go to loader & goal axis
    chassis.moveToPoint(-47, 48, 3000);
    //turn to align to loader
    chassis.turnToHeading(-90, 3000);
    //GO INTO LOADER
    will.extend();
    chassis.moveToPoint(-60, chassis.getPose().y, 3000);
    load();
    pros::delay(2000);
    chassis.moveToPoint(-47, 48, 3000);
    pros::delay(500);
    will.retract();

    

    intake.move(0);
    //go into alley
    chassis.moveToPose(-30, 60.7, -90, 3000, {.forwards=false, .maxSpeed=50});
    //go through alley
    chassis.turnToHeading(270,3000);
    chassis.moveToPoint(30, 60.7, 5000, {.forwards=false, .maxSpeed=80});
    chassis.waitUntilDone();
    //align to long goal (1)
    
    chassis.moveToPoint(51, 52.5, 3000, {.forwards=false});
    chassis.turnToHeading(90, 3000);
    
    //go into long goal (1)
    chassis.moveToPose(23.949, 48.4, 90, 3000, {.forwards=false});
    chassis.waitUntilDone();
    //score in long goal (1)
    
    score();
    will.extend();
    pros::delay(2000);



    //go into loader (2)
    chassis.moveToPose(66, 47.1, 90, 3000, {.maxSpeed=75});
    load();
    pros::delay(2000);

    //score again (2)
    chassis.moveToPose(24.537, 48.4, 90, 3000, {.forwards=false});
    pros::delay(1500);
    score();
    pros::delay(2000);
    will.retract();

    //algin with parking barrier
    chassis.moveToPose(64, 10, 180, 3000);

    //park
    load();
    chassis.moveToPoint(64, -55, 3000);
    chassis.turnToHeading(-90, 3000);
    // Move backward for 1 second
    int32_t start = 0;
    while (start < 500) {
        chassis.arcade(-45, 0); // negative = backward
        start++;
        pros::delay(10);
    }
    chassis.arcade(0, 0); // stop
    resetWithDistance(ResetWalls::RIGHT_BOTTOM);
    intake.move(0);
    chassis.moveToPoint(22, -30, 3000);
    will.extend();
    chassis.turnToHeading(135, 3000);
    chassis.moveToPose(9, -9, 135, 3000, {.forwards=false});
    chassis.moveToPoint(8.5, -8.5, 1000);
    middle(75);
    pros::delay(2000);
    middle(30);
}
