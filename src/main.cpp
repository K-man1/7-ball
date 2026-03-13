#include "robot.h"

// ── Display helpers ───────────────────────────────────────────────────────────
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

// ── initialize ────────────────────────────────────────────────────────────────
void initialize() {
    pros::lcd::initialize();
    wing.extend();
    // blue_background(); // set background color
    // display_img_from_c_array();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f",     chassis.getPose().x);
            pros::lcd::print(1, "Y: %f",     chassis.getPose().y);
            pros::lcd::print(2, "bob: %f", chassis.getPose().theta);
            pros::delay(100);
        }
    });

    chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}
