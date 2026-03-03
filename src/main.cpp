#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "mcl.hpp"

// ─── Motors & Pneumatics ──────────────────────────────────────────────────────

pros::MotorGroup intake({14, -17});

pros::adi::Pneumatics will('D', false);
pros::adi::Pneumatics wing('A', false);
pros::adi::Pneumatics up('F', false);
pros::adi::Pneumatics down('G', false);

// ─── Display ─────────────────────────────────────────────────────────────────

#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/image/lv_image.h"

inline void blue_background() {
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x00008B), 0);
}

void display_img_from_c_array() {
    LV_IMAGE_DECLARE(logo);
    lv_obj_t* img = lv_image_create(lv_screen_active());
    lv_image_set_src(img, &logo);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
}

void display_img_from_file(const void* src) {
    lv_obj_t* img = lv_image_create(lv_screen_active());
    lv_image_set_src(img, src);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
}

// ─── Drivetrain ───────────────────────────────────────────────────────────────

pros::MotorGroup leftMotors({-13, 12, 11},   pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, -19, -18},  pros::MotorGearset::blue);

pros::Imu imu(10);

pros::Rotation verticalEnc(16);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.25);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors,
                              12, lemlib::Omniwheel::NEW_325, 450, 8);

lemlib::ControllerSettings lateral_controller(5.6, 0, 4.5, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(1.68, 0, 11.65, 1.1, 1, 300, 3, 500, 0);

lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttleCurve, &steerCurve);

lemlib::Pose pose = chassis.getPose();

// ─── Distance Sensors ─────────────────────────────────────────────────────────
// TODO: Replace port numbers with your actual sensor ports.

pros::Distance dist_front(3);
pros::Distance dist_right(4);
pros::Distance dist_back(5);
pros::Distance dist_left(6);

// ─── MCL ─────────────────────────────────────────────────────────────────────

MCL<MCL_PARTICLE_COUNT> mcl;

// TODO: Measure your robot and fill in the actual offsets.
// offset_x, offset_y = inches from center of rotation (+ = right/forward)
// heading_deg = direction sensor faces (0=forward, 90=right, 180=back, 270=left)
std::array<SensorConfig, 4> sensor_configs = {{
    { 0.0f,  7.0f,   0.0f},  // front
    { 7.0f,  0.0f,  90.0f},  // right
    { 0.0f, -7.0f, 180.0f},  // back
    {-7.0f,  0.0f, 270.0f},  // left
}};

std::array<pros::Distance*, 4> dist_ptrs = {
    &dist_front, &dist_right, &dist_back, &dist_left
};

float prev_x = 0.0f;
float prev_y = 0.0f;

// Sets chassis pose AND re-initializes MCL particles around that position.
// Call this anywhere you currently call chassis.setPose().
void set_pose_and_init_mcl(float x, float y, float theta, float spread = 2.0f) {
    chassis.setPose(x, y, theta);
    mcl.init(x, y, spread);
    prev_x = x;
    prev_y = y;
}

// ─── initialize ───────────────────────────────────────────────────────────────

void initialize() {
    pros::lcd::initialize();
    blue_background();
    // display_img_from_c_array();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f",     chassis.getPose().x);
            pros::lcd::print(1, "Y: %f",     chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(100);
        }
    });

    chassis.calibrate();

    // MCL background task — corrects x/y from distance sensors at 100Hz
    pros::Task mcl_task([&]() {
        while (true) {
            mcl_update(chassis, mcl, prev_x, prev_y, dist_ptrs, sensor_configs);

            lemlib::Pose p = chassis.getPose();
            prev_x = p.x;
            prev_y = p.y;

            pros::delay(10);
        }
    });
}

// ─── disabled / competition_initialize ───────────────────────────────────────

void disabled() {}
void competition_initialize() {}

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

// ─── autonomous ───────────────────────────────────────────────────────────────

void autonomous() {
    set_pose_and_init_mcl(0, 0, 0);

    chassis.moveToPoint(0, 32.6, 3000);

    will.toggle();
    load();
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(19, 33.1, 1000, {.maxSpeed = 45});
    chassis.waitUntilDone();
    pros::delay(2100);

    // STAY HERE UNTIL OPTICAL SENSOR SEES BLUE. ADD CODE HERE.

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

    // Re-init MCL at this known position to reset any particle drift
    set_pose_and_init_mcl(-23, -63, chassis.getPose().theta);

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

// ─── opcontrol ────────────────────────────────────────────────────────────────

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    while (true) {
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
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