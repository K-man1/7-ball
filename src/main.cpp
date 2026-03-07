#define MCL_LOG_SD

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
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

// pros::Rotation verticalEnc(16);
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.25);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors,
                              12, lemlib::Omniwheel::NEW_325, 450, 8);

lemlib::ControllerSettings lateral_controller(5.6, 0, 4.5, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(1.68, 0, 11.65, 1.1, 1, 300, 3, 500, 0);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttleCurve, &steerCurve);

lemlib::Pose pose = chassis.getPose();

// ─── Distance Sensors ─────────────────────────────────────────────────────────

pros::Distance dist_right(8);
pros::Distance dist_back(2);
pros::Distance dist_left(3);

// ─── MCL ─────────────────────────────────────────────────────────────────────

MCL<MCL_PARTICLE_COUNT> mcl;

// offset_x, offset_y = inches from center of rotation (+ = right/forward)
// heading_deg = direction sensor faces (0=forward, 90=right, 180=back, 270=left)
std::array<SensorConfig, 3> sensor_configs = {{
    { 5.5f,  4.0f,  90.0f},  // right — 5.5" right, 4" in front
    {-4.0f, -3.75f, 180.0f}, // back  — 4" left of center, 3.75" behind
    {-5.5f,  5.0f, 270.0f},  // left  — 5.5" left, 5" in front
}};

std::array<pros::Distance*, 3> dist_ptrs = {
    &dist_right, &dist_back, &dist_left
};

float prev_x = 0.0f;
float prev_y = 0.0f;

// ─── Position helpers ─────────────────────────────────────────────────────────

void set_pose_and_init_mcl(float x, float y, float theta, float spread = 2.0f) {
    chassis.setPose(x, y, theta);
    mcl.init(x, y, spread);
    prev_x = x;
    prev_y = y;
}

// Reset position from a single sensor against a known wall.
// offset = perpendicular distance from sensor to robot's edge facing that wall.
// Call once per wall you want to reset from — explicitly specify which wall.
enum Wall { TOP, BOTTOM, LEFT_WALL, RIGHT_WALL };

void reset_from_wall(pros::Distance& sensor, float offset, Wall wall) {
    float dist = sensor.get() / 25.4f;
    if (dist <= 0 || dist >= 48.0f) return;

    lemlib::Pose p = chassis.getPose();
    float total = dist + offset;

    switch (wall) {
        case BOTTOM:     set_pose_and_init_mcl(p.x,            FIELD_MIN + total, p.theta); break;
        case TOP:        set_pose_and_init_mcl(p.x,            FIELD_MAX - total, p.theta); break;
        case LEFT_WALL:  set_pose_and_init_mcl(FIELD_MIN + total, p.y,            p.theta); break;
        case RIGHT_WALL: set_pose_and_init_mcl(FIELD_MAX - total, p.y,            p.theta); break;
    }
}

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

// ─── Auton selector ──────────────────────────────────────────────────────────

int selected_auton = 0;
const int NUM_AUTONS = 2;
const char* auton_names[] = {"AWP", "Skills"};

// ─── Auton routines ───────────────────────────────────────────────────────────

void solo_awp() {
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

    // at this point: facing 90°, right sensor sees bottom wall, back sensor sees left wall
    reset_from_wall(dist_right, 4.0f,  BOTTOM);
    reset_from_wall(dist_back,  3.75f, LEFT_WALL);

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
    set_pose_and_init_mcl(0, 0, 0);
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
    // at this point: facing 90°, right sensor sees bottom wall, back sensor sees left wal
    // rest of skills auton goes here...
}

// ─── disabled / competition_initialize ───────────────────────────────────────

void disabled() {}
void competition_initialize() {}

// ─── autonomous ───────────────────────────────────────────────────────────────

void autonomous() {
    solo_awp();
}

// ─── initialize ───────────────────────────────────────────────────────────────

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() {
    pros::lcd::initialize();
    blue_background();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f",     chassis.getPose().x);
            pros::lcd::print(1, "Y: %f",     chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "px: %f py: %f", prev_x, prev_y);
            pros::lcd::print(4, "Auton: %s (%d)", auton_names[selected_auton], selected_auton);
            pros::delay(100);
        }
    });

    pros::lcd::register_btn0_cb([]() {
        selected_auton = (selected_auton - 1 + NUM_AUTONS) % NUM_AUTONS;
    });
    pros::lcd::register_btn2_cb([]() {
        selected_auton = (selected_auton + 1) % NUM_AUTONS;
    });

    chassis.calibrate();

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

// ─── opcontrol ────────────────────────────────────────────────────────────────

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
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            up.set_value(false);
            down.set_value(false);
            intake.move(75);
            
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