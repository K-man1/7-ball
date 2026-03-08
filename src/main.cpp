#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include "src/monte.hpp"

#include <iostream>

using namespace lemlib;


pros::MotorGroup intake({14, -17});

pros::adi::Pneumatics will('D', false);
pros::adi::Pneumatics wing('B', false);
pros::adi::Pneumatics up('F', false);
pros::adi::Pneumatics down('G', false);

// ─── Drivetrain ───────────────────────────────────────────────────────────────

pros::MotorGroup leftMotors({-13, 12, 11},  pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, -19, -18}, pros::MotorGearset::blue);

pros::Imu imu(10);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors,
                              12, lemlib::Omniwheel::NEW_325, 450, 8);

lemlib::ControllerSettings lateral_controller(5.6, 0, 4.5, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(1.68, 0, 11.65, 1.1, 1, 300, 3, 500, 0);

lemlib::OdomSensors odom_sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        odom_sensors, &throttleCurve, &steerCurve);

// ─── Distance Sensors ─────────────────────────────────────────────────────────

pros::Distance dist_right(8);
pros::Distance dist_back(2);
pros::Distance dist_left(3);

// ─── Mechanism helpers ────────────────────────────────────────────────────────

void load() {
    intake.move(127);
    up.set_value(false);
    down.set_value(true);
}

void score() {
    up.set_value(true);
    down.set_value(true);
    intake.move(-127);
    pros::delay(250);
    intake.move(127);
}

void middle() {
    intake.move(90);
    up.set_value(false);
    down.set_value(false);
}

void outtake() {
    intake.move(-100);
}





/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate(); // calibrate sensors
  chassis.setPose(0, 0, 0);

  // /*
  pros::Task screenTask([&]() {
    while (true) {

      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      // Print distance sensor values in inches
      pros::lcd::print(4, "N: %.2f in", dNorth.get_distance() / 25.4);
      pros::lcd::print(5, "E: %.2f in", dEast.get_distance() / 25.4);
      pros::lcd::print(6, "S: %.2f in", dNorthW.get_distance() / 25.4);
      pros::lcd::print(7, "W: %.2f in", dWest.get_distance() / 25.4);

      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}",
                                    chassis.getPose()); // log file to sd card
      // delay to save resources
      pros::delay(50);
    }
  });
  // */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // unclamp
  // clamp.retract();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() { }



void solo_awp() {
    chassis.setPose(0, 0, 0);
    startMCL(chassis);

    chassis.moveToPoint(0, 32.6, 3000);
    will.toggle();
    load();
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(19, 33.1, 1000, {.maxSpeed = 45});
    chassis.waitUntilDone();
    pros::delay(2100);

    chassis.moveToPoint(-22, 34.8, 2200, {.forwards = false, .maxSpeed = 90});
    pros::delay(400);
    score();
    pros::delay(3000);

    will.toggle();
    chassis.turnToHeading(210, 800);
    load();

    chassis.moveToPoint(-21, -32, 1570, {.maxSpeed = 75});
    chassis.waitUntilDone();
    will.extend();

    chassis.turnToHeading(135, 600);
    chassis.moveToPoint(-5, -64, 830);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(-23, -63, 1100, {.forwards = false});
    pros::delay(500);

    score();
    pros::delay(2500);

    chassis.moveToPose(27, -63, 90, 2000, {.maxSpeed = 55});
    load();
    pros::delay(1500);
    pros::delay(1500);

    chassis.moveToPoint(0, -63, 1000, {.forwards = false});
    chassis.moveToPoint(-23, -63, 1100, {.forwards = false});
    score();
    pros::delay(2500);
    will.toggle();
    pros::delay(500);

    chassis.moveToPose(-45, -21, 135, 5000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(100);
    middle();
    will.toggle();
    pros::delay(5000);
}

void auton_skills() {
    chassis.setPose(0, 0, 0);
    startMCL(chassis);

    wing.extend();
    load();
    chassis.moveToPoint(-5, 21, 5000);
    chassis.turnToHeading(225, 2000);
    chassis.moveToPoint(7, 36, 5000, {.forwards = false});
    chassis.waitUntilDone();
    middle();
    pros::delay(2000);
    chassis.moveToPoint(0, -29, 5000);

    intake.move(0);
    // rest of skills auton goes here...
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
    solo_awp();
}

// Create MCL instance using the existing distance sensors

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    stopMCL();

    while (true) {
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
            up.set_value(false);
            down.set_value(true);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
            pros::delay(100);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
            up.set_value(true);
            down.set_value(true);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            up.set_value(false);
            down.set_value(false);
            intake.move(75);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-100);
        } else {
            up.set_value(false);
            down.set_value(true);
            intake.move(0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            will.toggle();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            wing.toggle();
        }

        pros::delay(25);
    }
}