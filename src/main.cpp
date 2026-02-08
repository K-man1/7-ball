#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <iostream>


pros::MotorGroup left_motor_group({-5, 6, 7}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motor_group({8, -9, -10}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// initialize function. Runs on program startup
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {

    while (true) {
		pros::lcd::print(0, "RIGHT 8: %d", right_motor_group.get_power_all()[0]);
		pros::lcd::print(1, "LEFT 5: %d", left_motor_group.get_power_all()[0]);

		pros::lcd::print(2, "RIGHT 9: %d", right_motor_group.get_power_all()[1]);
		pros::lcd::print(3, "LEFT 6: %d", left_motor_group.get_power_all()[1]);

		pros::lcd::print(4, "RIGHT 10: %d", right_motor_group.get_power_all()[2]);
		pros::lcd::print(5, "LEFT 7: %d", left_motor_group.get_power_all()[2]);
		pros::delay(100);
	}
}
	);
}

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}