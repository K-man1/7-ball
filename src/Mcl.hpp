#pragma once

/**
 * mcl.hpp — Monte Carlo Localization
 * Integrated for 30000S-RoboSapiens (VRC Push Back 2025-2026)
 *
 * Plugs into LemLib odometry (dx/dy from chassis.getPose()) and
 * four VEX Distance Sensors.
 *
 * Usage:
 *   1. Call mcl.init(x, y, spread) whenever you call chassis.setPose().
 *   2. In your odometry/control loop, call mcl_update(chassis, mcl, sensors).
 *   3. The updated x/y is written back into the chassis pose automatically.
 *
 * Tune MCL_PARTICLE_COUNT (start at 500, raise toward 2000 if time budget allows).
 * Tune MCL_TOLERANCE to how close odometry prediction must be to sensor reading
 * before the reading is trusted.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"

// ─── Tuning constants ────────────────────────────────────────────────────────

// Number of particles. ~500 is fast; ~2000 is more accurate.
static constexpr size_t MCL_PARTICLE_COUNT = 500;

// Max allowed difference (inches) between predicted and actual sensor reading
// before we drop the reading as noise/obstacle. Tune to ~6–12.
static constexpr float MCL_TOLERANCE = 10.0f;

// ─── VEX field constants ─────────────────────────────────────────────────────

static constexpr float FIELD_SIZE = 140.42f;
static constexpr float HALF_SIZE  = 0.5f * FIELD_SIZE;
static constexpr float FIELD_MIN  = -HALF_SIZE;
static constexpr float FIELD_MAX  =  HALF_SIZE;

// Field obstacles (loader, goal, center bar) — used to drop readings that
// are aimed at field elements rather than the perimeter walls.
static constexpr float LOADER_X       = 47.0f;
static constexpr float LOADER_RADIUS  = 3.0f;
static constexpr float LOADER_PADDING = 0.5f;
static constexpr float LOADER_WIDTH   = LOADER_RADIUS * 2.0f + LOADER_PADDING * 2.0f;
static constexpr float LOADER_LENGTH  = LOADER_RADIUS * 2.0f + LOADER_PADDING;

static constexpr std::array<std::pair<float,float>, 4> LOADERS = {{
    {-LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f},
    {-LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f},
    { LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f},
    { LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f},
}};

static constexpr float GOAL_X       = 48.0f;
static constexpr float GOAL_Y       = 23.0f;
static constexpr float GOAL_PADDING = 1.0f;
static constexpr float GOAL_WIDTH   = 6.0f  + GOAL_PADDING * 2.0f;
static constexpr float GOAL_LENGTH  = 3.0f  + GOAL_PADDING * 2.0f;

static constexpr std::array<std::pair<float,float>, 4> GOALS = {{
    {-GOAL_X, -GOAL_Y},
    {-GOAL_X,  GOAL_Y},
    { GOAL_X,  GOAL_Y},
    { GOAL_X, -GOAL_Y},
}};

static constexpr float CENTER_PADDING = 1.0f;
static constexpr float CENTER_WIDTH   = 21.0f + CENTER_PADDING * 2.0f;
static constexpr float CENTER_LENGTH  = 21.0f + CENTER_PADDING * 2.0f;
static constexpr std::pair<float,float> CENTER_POS = {0.0f, 0.0f};

// ─── XorShift32 PRNG ─────────────────────────────────────────────────────────

struct XorShift32 {
    uint32_t state;

    inline XorShift32(uint32_t seed = pros::micros())
        : state(seed == 0 ? 0x12345678u : seed) {}

    inline uint32_t next_u32() {
        uint32_t x = state;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        state = x; return x;
    }

    inline float next_f32() {
        return (next_u32() >> 8) * (1.0f / (1u << 24));
    }

    inline float range_f32(float lo, float hi) {
        return lo + (hi - lo) * next_f32();
    }

    // Box–Muller transform
    inline float gaussian(float std_dev) {
        float u1 = std::max(next_f32(), 1e-12f);
        float u2 = next_f32();
        float r  = std::sqrt(-2.0f * std::log(u1));
        return r * std::cos(2.0f * M_PI * u2) * std_dev;
    }
};

// ─── Geometry helpers ─────────────────────────────────────────────────────────

struct MCLPoint { float x, y; };

struct MCLLine {
    MCLPoint start, end;

    // Returns distance along ray from start to the nearest intersection with
    // an axis-aligned rectangle, or nullopt if no forward intersection.
    inline std::optional<float>
    square_intersect_distance(float cx, float cy, float w, float h) const {
        float hw = w * 0.5f, hh = h * 0.5f;
        float rx = start.x - cx, ry = start.y - cy;
        float dx = end.x - start.x, dy = end.y - start.y;
        float best_t = std::numeric_limits<float>::infinity();

        if (std::abs(dx) > 1e-6f) {
            float inv_dx = 1.0f / dx;
            float tx = (dx > 0.0f ? hw : -hw) - rx;
            float t  = tx * inv_dx;
            if (t >= 0.0f && std::abs(ry + t * dy) <= hh) best_t = t;
        }

        if (std::abs(dy) > 1e-6f) {
            float inv_dy = 1.0f / dy;
            float ty = (dy > 0.0f ? hh : -hh) - ry;
            float t  = ty * inv_dy;
            if (t >= 0.0f && t < best_t && std::abs(rx + t * dx) <= hw) best_t = t;
        }

        if (best_t < std::numeric_limits<float>::infinity())
            return best_t * std::hypot(dx, dy);
        return std::nullopt;
    }
};

// ─── Sensor reading capture ───────────────────────────────────────────────────

struct MCLReading {
    float    recorded;   // actual sensor reading (inches)
    float    inv_var;    // -0.5 / (std_dev^2) — precomputed for weight calc
    MCLPoint rel_pos;    // sensor origin relative to robot center (rotated)
    MCLPoint proj_pt;    // a second point along sensor ray direction

    MCLReading(float rec, float std_dev, MCLPoint rel, MCLPoint proj)
        : recorded(rec),
          inv_var(-0.5f / (std_dev * std_dev)),
          rel_pos(rel),
          proj_pt(proj) {}

    // Predict what this sensor would read if robot were at particle_pos.
    inline std::optional<float> predict(MCLPoint particle_pos) const {
        MCLLine ray{
            {rel_pos.x + particle_pos.x, rel_pos.y + particle_pos.y},
            {proj_pt.x + particle_pos.x, proj_pt.y + particle_pos.y}
        };
        return ray.square_intersect_distance(0.0f, 0.0f, FIELD_SIZE, FIELD_SIZE);
    }
};

// ─── MCL core (Structure of Arrays, fixed N on stack) ────────────────────────

template <size_t N>
struct MCL {
    float particle_x[N];
    float particle_y[N];
    float particle_weights[N];

    float temp_x[N];
    float temp_y[N];
    float temp_weights[N];

    // Pre-resample snapshot for debugging/SD logging
    float presample_x[N];
    float presample_y[N];
    float presample_weights[N];

    XorShift32 rng;

    MCL() : rng(pros::micros()) {
        std::fill(particle_x,         particle_x         + N, 0.0f);
        std::fill(particle_y,         particle_y         + N, 0.0f);
        std::fill(particle_weights,   particle_weights   + N, 1.0f / N);
        std::fill(temp_x,             temp_x             + N, 0.0f);
        std::fill(temp_y,             temp_y             + N, 0.0f);
        std::fill(temp_weights,       temp_weights       + N, 0.0f);
        std::fill(presample_x,        presample_x        + N, 0.0f);
        std::fill(presample_y,        presample_y        + N, 0.0f);
        std::fill(presample_weights,  presample_weights  + N, 0.0f);
    }

    // ── Step 1: Init ─────────────────────────────────────────────────────────
    // Call this whenever you call chassis.setPose().
    // spread = how far particles are distributed around (x, y); ~2–3 inches.
    void init(float x, float y, float spread = 2.0f) {
        rng = XorShift32(pros::micros());
        for (size_t i = 0; i < N; i++) {
            particle_x[i] = std::clamp(x + rng.range_f32(-spread, spread),
                                       FIELD_MIN, FIELD_MAX);
            particle_y[i] = std::clamp(y + rng.range_f32(-spread, spread),
                                       FIELD_MIN, FIELD_MAX);
            particle_weights[i] = 1.0f / N;
        }
    }

    // ── Step 2: Predict ───────────────────────────────────────────────────────
    // dx, dy = odometry delta since last call (inches).
    // std_dev = noise; a good value is std::hypot(dx, dy) / 4.0f.
    void predict(float dx, float dy, float std_dev) {
        for (size_t i = 0; i < N; i++) {
            particle_x[i] = std::clamp(particle_x[i] + dx + rng.gaussian(std_dev),
                                       FIELD_MIN, FIELD_MAX);
            particle_y[i] = std::clamp(particle_y[i] + dy + rng.gaussian(std_dev),
                                       FIELD_MIN, FIELD_MAX);
        }
    }

    // ── Step 3: Update ────────────────────────────────────────────────────────
    void update(const std::vector<MCLReading>& readings) {
        float max_weight = 0.0f;

        for (size_t i = 0; i < N; i++) {
            float weight = 1.0f;

            for (const auto& r : readings) {
                auto pred = r.predict({particle_x[i], particle_y[i]});
                if (pred.has_value()) {
                    float err = r.recorded - pred.value();
                    weight *= std::exp(r.inv_var * err * err);
                    if (weight == 0.0f) break;
                } else {
                    weight = 0.0f;
                    break;
                }
            }

            if (!std::isfinite(weight) || weight < 0.0f) weight = 0.0f;
            particle_weights[i] = weight;
            if (weight > max_weight) max_weight = weight;
        }

        // Normalize
        if (max_weight <= 0.0f) {
            float uniform = 1.0f / N;
            for (size_t i = 0; i < N; i++) particle_weights[i] = uniform;
            return;
        }

        float sum = 0.0f;
        for (size_t i = 0; i < N; i++) {
            particle_weights[i] /= max_weight;
            sum += particle_weights[i];
        }

        if (sum <= 0.0f) {
            float uniform = 1.0f / N;
            for (size_t i = 0; i < N; i++) particle_weights[i] = uniform;
            return;
        }

        float inv_sum = 1.0f / sum;
        for (size_t i = 0; i < N; i++) particle_weights[i] *= inv_sum;
    }

    // ── Step 4: Estimate ──────────────────────────────────────────────────────
    MCLPoint estimate() const {
        float ex = 0.0f, ey = 0.0f;
        for (size_t i = 0; i < N; i++) {
            ex += particle_x[i] * particle_weights[i];
            ey += particle_y[i] * particle_weights[i];
        }
        return {ex, ey};
    }

    // ── Step 5: Resample ──────────────────────────────────────────────────────
    void resample() {
        // Save pre-resample state for SD logging
        std::copy(particle_x,       particle_x       + N, presample_x);
        std::copy(particle_y,       particle_y       + N, presample_y);
        std::copy(particle_weights, particle_weights + N, presample_weights);

        float inv_n = 1.0f / N;
        float offset = rng.next_f32() * inv_n;
        float cumulative = particle_weights[0];
        size_t idx = 0;

        for (size_t i = 0; i < N; i++) {
            float sample = offset + i * inv_n;
            while (sample > cumulative && idx < N - 1) {
                idx++;
                cumulative += particle_weights[idx];
            }
            size_t safe_idx = std::min(idx, N - 1);
            temp_x[i]       = particle_x[safe_idx];
            temp_y[i]       = particle_y[safe_idx];
            temp_weights[i] = inv_n;
        }

        std::copy(temp_x,       temp_x       + N, particle_x);
        std::copy(temp_y,       temp_y       + N, particle_y);
        std::copy(temp_weights, temp_weights + N, particle_weights);
    }
};

// ─── Convenience integration function ────────────────────────────────────────
//
// Call this once per odometry tick, right after your odometry updates.
//
// Parameters:
//   chassis      — your LemLib chassis object
//   mcl          — your MCL<MCL_PARTICLE_COUNT> instance
//   prev_x/prev_y — robot position from the PREVIOUS tick (update after calling)
//   sensors      — array of your 4 pros::Distance sensors
//   sensor_offsets — relative (x, y, heading_deg) for each sensor from robot center
//   robot_theta_rad — current heading in radians (from IMU, which LemLib manages)
//
// After calling, update prev_x/prev_y to chassis.getPose().x/y.
//
struct SensorConfig {
    float offset_x;       // inches from robot center
    float offset_y;
    float heading_deg;    // direction sensor points, 0 = forward
};

template <size_t S>
inline void mcl_update(
    lemlib::Chassis&                         chassis,
    MCL<MCL_PARTICLE_COUNT>&                 mcl,
    float                                    prev_x,
    float                                    prev_y,
    std::array<pros::Distance*, S>           dist_sensors,
    std::array<SensorConfig, S>              sensor_configs,
    float                                    mcl_tolerance = MCL_TOLERANCE
) {
    lemlib::Pose pose = chassis.getPose();
    float theta_rad = pose.theta * (M_PI / 180.0f);

    // ── Predict ──────────────────────────────────────────────────────────────
    float dx = pose.x - prev_x;
    float dy = pose.y - prev_y;
    mcl.predict(dx, dy, std::hypot(dx, dy) / 4.0f);

    // ── Build readings ────────────────────────────────────────────────────────
    std::vector<MCLReading> readings;
    readings.reserve(4);

    for (size_t s = 0; s < S; s++) {
        pros::Distance* sensor = dist_sensors[s];
        const SensorConfig& cfg = sensor_configs[s];

        int raw_mm = sensor->get();
        if (raw_mm <= 0 || raw_mm >= 9999) continue;           // invalid reading

        float d = raw_mm / 25.4f;                              // mm → inches

        // Sensor position rotated into world frame
        float cos_t = std::cos(theta_rad), sin_t = std::sin(theta_rad);
        float rel_x = cfg.offset_x * cos_t - cfg.offset_y * sin_t;
        float rel_y = cfg.offset_x * sin_t + cfg.offset_y * cos_t;

        // A second point in the direction the sensor is pointing
        float sensor_heading_rad = (cfg.heading_deg * M_PI / 180.0f) + theta_rad;
        float proj_x = rel_x + std::cos(sensor_heading_rad);
        float proj_y = rel_y + std::sin(sensor_heading_rad);

        MCLPoint rel_pos{rel_x, rel_y};
        MCLPoint proj_pt{proj_x, proj_y};

        // Check if reading would pass through a field obstacle
        MCLLine ray{
            {rel_x + pose.x, rel_y + pose.y},
            {proj_x + pose.x, proj_y + pose.y}
        };
        bool hits_obstacle =
            std::any_of(LOADERS.begin(), LOADERS.end(), [&](const auto& l) {
                return ray.square_intersect_distance(l.first, l.second, LOADER_WIDTH, LOADER_LENGTH).has_value();
            }) ||
            std::any_of(GOALS.begin(), GOALS.end(), [&](const auto& g) {
                return ray.square_intersect_distance(g.first, g.second, GOAL_WIDTH, GOAL_LENGTH).has_value();
            }) ||
            ray.square_intersect_distance(CENTER_POS.first, CENTER_POS.second, CENTER_WIDTH, CENTER_LENGTH).has_value();

        if (hits_obstacle) continue;

        // VEX accuracy: ±15mm below 200mm, ±5% above
        float bound  = d < 7.874015f ? 0.590551f : 0.05f * d;
        float std_dev = std::max(bound / 3.0f, 1e-6f);

        // Tolerance filter — drop if odometry and sensor wildly disagree
        MCLReading candidate(d, std_dev, rel_pos, proj_pt);
        auto predicted = candidate.predict({pose.x, pose.y});
        if (!predicted.has_value()) continue;
        if (std::abs(d - predicted.value()) > mcl_tolerance) continue;

        readings.push_back(candidate);
    }

    // Only run update/estimate/resample if we have valid readings
    if (!readings.empty()) {
        mcl.update(readings);
        MCLPoint est = mcl.estimate();

        // ── SD card logging (for tuning only) ────────────────────────────────
        // To enable: #define MCL_LOG_SD at the top of main.cpp before #include "mcl.hpp"
        // To tune: pull /usd/mcl_log.csv off the SD card and run visualize_mcl.py
        // IMPORTANT: delete mcl_log.csv from the SD card between runs
        // IMPORTANT: comment out MCL_LOG_SD once tuning is done — file I/O is slow
#ifdef MCL_LOG_SD
        static uint32_t log_ticks = 0;
        if (log_ticks < 300) { // only log first 3 seconds (300 ticks at 100Hz)
            FILE* f = fopen("/usd/mcl_log.csv", "a");
            if (f) {
                for (size_t i = 0; i < MCL_PARTICLE_COUNT; i++) {
                    fprintf(f, "P,%f,%f,%f\n",
                        mcl.presample_x[i],
                        mcl.presample_y[i],
                        mcl.presample_weights[i]);
                }
                fprintf(f, "E,%f,%f\n", est.x, est.y);
                fclose(f);
            }
            log_ticks++;
        }
#endif

        mcl.resample();

        // Write corrected x/y back into chassis pose, keeping LemLib's heading
        chassis.setPose(est.x, est.y, pose.theta);
    }
}