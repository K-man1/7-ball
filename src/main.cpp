#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include <cmath>

// user-operated motors
// Port 1 = Scorer
pros::MotorGroup intake({14, -17});
// Port 2 & 3 = Intake (use MotorGroup to control both together)


pros::adi::Pneumatics will('D', false);


pros::adi::Pneumatics wing('A', false);


pros::adi::Pneumatics up('F', false);


pros::adi::Pneumatics down('G', false);



#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/image/lv_image.h"


// Declare & define functions to set the background color
inline void blue_background() { lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x00008B), 0); }


void display_img_from_c_array() {
    // create a variable for the c array (image)
    LV_IMAGE_DECLARE(logo);


    // declare and define the image object
    lv_obj_t* img = lv_image_create(lv_screen_active());


    // set the source data for the image
    lv_image_set_src(img,&logo);


    // (Optional) set the image's alignment
    lv_obj_align(img,LV_ALIGN_CENTER,0,0); // centered in the screen
}


void display_img_from_file(const void * src) {
    // declare and define the image object
    lv_obj_t* img = lv_image_create(lv_screen_active());


    // set the source data for the image
    lv_image_set_src(img,src);


    // (Optional) set the image's alignment
    lv_obj_align(img,LV_ALIGN_CENTER,0,0); // centered in the screen
}



// pros::adi::DigitalOut wing('C');
// bool wing_state = true;


// motor groups
pros::MotorGroup leftMotors({-13, 12, 11}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({20,-19,-18}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


// Inertial Sensor on port 20
pros::Imu imu(10);


// tracking wheels


// vertical tracking wheel encoder. Rotation sensor, port 19, reversed
pros::Rotation verticalEnc(16);


// vertical tracking wheel. 2.75" diameter, 0" offset
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.25);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 8. If we had traction wheels, it would have been 8
);


// lateral motion controller
lemlib::ControllerSettings lateral_controller(5.6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              4.5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);




// angular PID controller
lemlib::ControllerSettings angular_controller(1.68, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11.65, // derivative gain (kD)
                                              1.1, // anti windup
                                              1, // small error range, in inches
                                              300, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);




// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);


// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttleCurve, 
                        &steerCurve
);


lemlib::Pose pose = chassis.getPose();


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    blue_background(); // set background color
    // display_img_from_c_array();


    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading


            // delay to save resources
            pros::delay(100);
        }
    });




    chassis.calibrate(); // calibrate sensors
   
}







/**
 * Runs while the robot is disabled
 */
void disabled() {}


/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(path2loader2_txt); // '.' replaced with "_" to make c++ happy










/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


 void load() {
    intake.move(127);
    up.set_value(false);
    down.set_value(true);
}


void score() {
    // reverse briefly
    up.set_value(true);
    down.set_value(true);
    intake.move(-127);
    pros::delay(250);
    // then score
    intake.move(127);
   
}


void middle() {
    intake.move(127);
    up.set_value(false);
    down.set_value(false);
}


void outtake() {
    intake.move(-100);
}




void autonomous() {
     // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
 
    // chassis.moveToPoint(0, 48, 3000);
    chassis.moveToPoint(0, 31.6, 1000);


    will.toggle();
    load();
    //Turn to face loader
    chassis.turnToHeading(89, 450);
    // Go into loader
    chassis.moveToPoint(19, 31.6, 800, {.maxSpeed = 45});


    //STAY HERE UNTIL OPTICAL SENSOR SEES BLUE. ADD CODE HERE.
   


    //Loader to goal


    //Back from loader into goal
    chassis.moveToPoint(-22, 34.8, 2200, {.forwards = false, .maxSpeed = 90});
    pros::delay(400);
    score();




    //leave goal and turn
    will.toggle();
    chassis.turnToHeading(210, 800);
    load();


    //Get first corner blocks


    //get second trio of corner blocks
    chassis.moveToPoint(-21, -32, 1570, {.maxSpeed = 75});


    //turn to algin 2 second part of auto
    chassis.turnToHeading(135, 600);


    //go to align to 2nd part of auto
    chassis.moveToPoint(-5, -64, 830);


    //turn to face goal
    chassis.turnToHeading(90, 500);


    //move back into goal and score
    chassis.moveToPoint(-23, -63, 1100, {.forwards = false});
    pros::delay(500);

    score();
    will.toggle();
    pros::delay(1000);
    chassis.setPose(-23, -63, chassis.getPose().theta);

    //go into loader
    chassis.moveToPose(21, -63, 90, 1000, {.maxSpeed = 65});
    load();
    chassis.turnToHeading(93, 100);
    chassis.turnToHeading(87, 100);
    chassis.turnToHeading(93, 100);
    chassis.turnToHeading(87, 100);
    chassis.turnToHeading(93, 100);
    chassis.turnToHeading(87, 100);


    //back away
    chassis.moveToPoint(0, -63, 1000, {.forwards = false});

    //go to middle goal
    chassis.moveToPose(-45, -21, 135, 5000, {.forwards = false});
    pros::delay(1500);
    intake.move(-127);
    pros::delay(100);
    middle();
   
    // //GO to middle goal
    // middle_goal.set_value(false);
    // chassis.moveToPose(-30, -21, 45, 5000, {.forwards = false});
    // intake.move(80);
    // scorer.move(50);
}


/**
 * Runs in driver control
 */




// ---------- Mechanism Functions ----------






pros::Controller controller(pros::E_CONTROLLER_MASTER);


// ---------- Heading Lock Variables ----------
bool headingLock = false;
double lockedHeading = 0;

// Snap heading to nearest multiple of 90
double nearest90(double heading) {
    return round(heading / 90.0) * 90.0;
}

void opcontrol() {
    while (true) {
        // ---------- Toggle Heading Lock ----------
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            headingLock = !headingLock;
            if (headingLock) {
                // Snap current heading to nearest 90 degrees
                double currentHeading = chassis.getPose().theta;
                lockedHeading = nearest90(currentHeading);
                chassis.turnToHeading(lockedHeading, 100);
            }
        }

        // ---------- Driver Inputs ----------
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // If heading is locked, force straight driving
        if (headingLock) {
            rightX = 0; // no turning allowed
        }

        // Move the robot
        chassis.arcade(leftY, rightX);

        // ---------- Mechanism Controls ----------
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
            up.set_value(false);
            down.set_value(true);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
            pros::delay(100);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
            up.set_value(true);
            down.set_value(true);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127);
            up.set_value(false);
            down.set_value(false);
            pros::delay(300);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
            up.set_value(false);
            down.set_value(false);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-100);
        }
        else {
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