#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <random>
#include <cmath>
#include <utility>
#include "constants.h"  // must define WINDOW_WIDTH, WINDOW_HEIGHT, MIN_SPEED, CACHE_LINE_SIZE, NUM_BOIDS

//
// Common helper: return a velocity vector whose magnitude is at least MIN_SPEED.
// Used by both single-Boid and array-Boid variants.
//
inline std::pair<float, float> initializeVelocity() {
    static std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * static_cast<float>(M_PI));
    float angle = angleDist(generator);

    // Base velocity of magnitude MIN_SPEED in a random direction
    float vx = std::cos(angle) * MIN_SPEED;
    float vy = std::sin(angle) * MIN_SPEED;

    // Add a small random “jitter” in [-1, +1] to each component
    std::uniform_real_distribution<float> jitterDist(-1.0f, 1.0f);
    vx += jitterDist(generator);
    vy += jitterDist(generator);

    // If the new speed is below MIN_SPEED, re‐normalize up to exactly MIN_SPEED
    float speed = std::sqrt(vx * vx + vy * vy);
    if (speed < MIN_SPEED) {
        float factor = MIN_SPEED / speed;
        vx *= factor;
        vy *= factor;
    }
    return {vx, vy};
}

//
// Common helper: generate a random float in [min, max].
//
inline float randomFloat(float min, float max) {
    static std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(generator);
}

//
// ——————————————————————————————————————————————————————————————————————————
// Single‐Boid (unpacked) and Single‐Boid (padded) structs
// ——————————————————————————————————————————————————————————————————————————
//

struct Boid {
    float x, y;    // Position
    float vx, vy;  // Velocity

    // Initialize at (px, py), then pick a random velocity ≥ MIN_SPEED
    Boid(float px, float py)
        : x(px), y(py)
    {
        auto vel = initializeVelocity();
        vx = vel.first;
        vy = vel.second;
    }

    // Default‐construct at random screen position, then pick a random velocity
    Boid()
        : x(randomFloat(0.0f, WINDOW_WIDTH)),
          y(randomFloat(0.0f, WINDOW_HEIGHT))
    {
        auto vel = initializeVelocity();
        vx = vel.first;
        vy = vel.second;
    }

    // Overwrite position/velocity
    void update(float& newVx, float& newVy, float& newX, float& newY) {
        vx = newVx;
        vy = newVy;
        x  = newX;
        y  = newY;
    }
};

struct alignas(CACHE_LINE_SIZE) PadBoid {
    float x, y;    // Position
    float vx, vy;  // Velocity
    // Pad out to at least one full cache line:
    char pad[ (CACHE_LINE_SIZE > sizeof(float)*4) ? (CACHE_LINE_SIZE - sizeof(float)*4) : 1 ];

    PadBoid(float px, float py)
        : x(px), y(py)
    {
        auto vel = initializeVelocity();
        vx = vel.first;
        vy = vel.second;
    }

    PadBoid()
        : x(randomFloat(0.0f, WINDOW_WIDTH)),
          y(randomFloat(0.0f, WINDOW_HEIGHT))
    {
        auto vel = initializeVelocity();
        vx = vel.first;
        vy = vel.second;
    }
    PadBoid(PadBoid& other)
        : x(other.x), y(other.y), vx(other.vx), vy(other.vy) {}

    void update(float& newVx, float& newVy, float& newX, float& newY) {
        vx = newVx;
        vy = newVy;
        x  = newX;
        y  = newY;
    }
};

//
// ——————————————————————————————————————————————————————————————————————————
// Single‐Parameters (unpacked) and Single‐Parameters (padded) structs
// ——————————————————————————————————————————————————————————————————————————
//

struct Parameters {
    float close_dx, close_dy;
    float avg_vx,    avg_vy;
    float avg_x,     avg_y;
    float neighboring_count;

    Parameters()
        : close_dx(0.0f), close_dy(0.0f),
          avg_vx(0.0f),   avg_vy(0.0f),
          avg_x(0.0f),    avg_y(0.0f),
          neighboring_count(0.0f)
    {}

    void reset() {
        close_dx         = 0.0f;
        close_dy         = 0.0f;
        avg_vx           = 0.0f;
        avg_vy           = 0.0f;
        avg_x            = 0.0f;
        avg_y            = 0.0f;
        neighboring_count = 0.0f;
    }
};

struct alignas(CACHE_LINE_SIZE) PadParameters {
    float close_dx, close_dy;
    float avg_vx,    avg_vy;
    float avg_x,     avg_y;
    float neighboring_count;
    // Pad out so the struct occupies exactly one cache line (or more):
    char pad[ (CACHE_LINE_SIZE > sizeof(float)*7) ? (CACHE_LINE_SIZE - sizeof(float)*7) : 1 ];

    PadParameters()
        : close_dx(0.0f), close_dy(0.0f),
          avg_vx(0.0f),   avg_vy(0.0f),
          avg_x(0.0f),    avg_y(0.0f),
          neighboring_count(0.0f)
    {}

    void reset() {
        close_dx         = 0.0f;
        close_dy         = 0.0f;
        avg_vx           = 0.0f;
        avg_vy           = 0.0f;
        avg_x            = 0.0f;
        avg_y            = 0.0f;
        neighboring_count = 0.0f;
    }
};

//
// ——————————————————————————————————————————————————————————————————————————
// Array‐of‐Boids (unpacked) and Array‐of‐Boids (padded) structs
// ——————————————————————————————————————————————————————————————————————————
//

struct BoidsList {
    float x[NUM_BOIDS];
    float y[NUM_BOIDS];
    float vx[NUM_BOIDS];
    float vy[NUM_BOIDS];
    // Pad so that the entire arrays of floats plus this pad field fit in (or exceed) one cache line:
    char pad[ (CACHE_LINE_SIZE > sizeof(float)*4*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*4*NUM_BOIDS) : 1 ];

    BoidsList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i]   = randomFloat(0.0f, WINDOW_WIDTH);
            y[i]   = randomFloat(0.0f, WINDOW_HEIGHT);
            auto vel = initializeVelocity();
            vx[i]  = vel.first;
            vy[i]  = vel.second;
        }
    }

    // “Copy constructor” (just copies elementwise)
    BoidsList(const BoidsList& other) {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i]  = other.x[i];
            y[i]  = other.y[i];
            vx[i] = other.vx[i];
            vy[i] = other.vy[i];
        }
    }

    void update(int& idx, float& newVx, float& newVy, float& newX, float& newY) {
        vx[idx] = newVx;
        vy[idx] = newVy;
        x[idx]  = newX;
        y[idx]  = newY;
    }
};

struct alignas(CACHE_LINE_SIZE) PadBoidsList {
    float x[NUM_BOIDS];
    char pad1[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float y[NUM_BOIDS];
    char pad2[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float vx[NUM_BOIDS];
    char pad3[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float vy[NUM_BOIDS];
    char pad4[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];

    PadBoidsList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i]   = randomFloat(0.0f, WINDOW_WIDTH);
            y[i]   = randomFloat(0.0f, WINDOW_HEIGHT);
            auto vel = initializeVelocity();
            vx[i]  = vel.first;
            vy[i]  = vel.second;
        }
    }

    PadBoidsList(const PadBoidsList& other) {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i]  = other.x[i];
            y[i]  = other.y[i];
            vx[i] = other.vx[i];
            vy[i] = other.vy[i];
        }
    }

    void update(int& idx, float& newVx, float& newVy, float& newX, float& newY) {
        vx[idx] = newVx;
        vy[idx] = newVy;
        x[idx]  = newX;
        y[idx]  = newY;
    }
};

//
// ——————————————————————————————————————————————————————————————————————————
// Array‐of‐Parameters (unpacked) and Array‐of‐Parameters (padded) structs
// ——————————————————————————————————————————————————————————————————————————
//

struct ParametersList {
    float close_dx[NUM_BOIDS];
    float close_dy[NUM_BOIDS];
    float avg_vx[NUM_BOIDS];
    float avg_vy[NUM_BOIDS];
    float avg_x[NUM_BOIDS];
    float avg_y[NUM_BOIDS];
    float neighboring_count[NUM_BOIDS];

    ParametersList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            close_dx[i]         = 0.0f;
            close_dy[i]         = 0.0f;
            avg_vx[i]           = 0.0f;
            avg_vy[i]           = 0.0f;
            avg_x[i]            = 0.0f;
            avg_y[i]            = 0.0f;
            neighboring_count[i] = 0.0f;
        }
    }

    // Reset only one boid’s slot
    void reset(int& idx) {
        close_dx[idx]         = 0.0f;
        close_dy[idx]         = 0.0f;
        avg_vx[idx]           = 0.0f;
        avg_vy[idx]           = 0.0f;
        avg_x[idx]            = 0.0f;
        avg_y[idx]            = 0.0f;
        neighboring_count[idx] = 0.0f;
    }
};

struct alignas(CACHE_LINE_SIZE) PadParametersList {
    float close_dx[NUM_BOIDS];
    char pad1[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float close_dy[NUM_BOIDS];
    char pad2[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float avg_vx[NUM_BOIDS];
    char pad3[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float avg_vy[NUM_BOIDS];
    char pad4[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float avg_x[NUM_BOIDS];
    char pad5[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float avg_y[NUM_BOIDS];
    char pad6[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];
    float neighboring_count[NUM_BOIDS];
    char pad7[ (CACHE_LINE_SIZE > sizeof(float)*NUM_BOIDS) ? (CACHE_LINE_SIZE - sizeof(float)*NUM_BOIDS) : 1 ];

    PadParametersList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            close_dx[i]         = 0.0f;
            close_dy[i]         = 0.0f;
            avg_vx[i]           = 0.0f;
            avg_vy[i]           = 0.0f;
            avg_x[i]            = 0.0f;
            avg_y[i]            = 0.0f;
            neighboring_count[i] = 0.0f;
        }
    }

    void reset(int& idx) {
        close_dx[idx]         = 0.0f;
        close_dy[idx]         = 0.0f;
        avg_vx[idx]           = 0.0f;
        avg_vy[idx]           = 0.0f;
        avg_x[idx]            = 0.0f;
        avg_y[idx]            = 0.0f;
        neighboring_count[idx] = 0.0f;
    }
};

#endif // STRUCTURES_H
