#include <random>
#include "constants.h"

#ifndef STRUCTURES_H
#define STRUCTURES_H

inline float randomFloat(float min, float max) {
    static std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<float> distribution(min, max);
    return distribution(generator);
}

struct Boid {
    float x, y;   // Position
    float vx, vy; // Velocity

    Boid(const float px, const float py) : x(px), y(py) {
        const std::pair<float,float> speed = initializeVelocity();
        vx = speed.first;
        vy = speed.second;
    }

    Boid() : x(randomFloat(0, WINDOW_WIDTH)), y(randomFloat(0, WINDOW_HEIGHT)) {
        const std::pair<float, float> speed = initializeVelocity();
        vx = speed.first;
        vy = speed.second;
    }

    std::pair<float, float> initializeVelocity() {
        float angle = randomFloat(0, 2 * M_PI);
        vx = std::cos(angle) * MIN_SPEED;
        vy = std::sin(angle) * MIN_SPEED;

        vx += randomFloat(-1.0f, 1.0f);
        vy += randomFloat(-1.0f, 1.0f);

        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }

    void update(float& vx, float& vy, float& x, float& y) {
        this->vx = vx;
        this->vy = vy;
        this->x = x;
        this->y = y;
    }
};

struct alignas(CACHE_LINE_SIZE) PadBoid {
    float x;
    float y;
    float vx;
    float vy;
    char pad[CACHE_LINE_SIZE > sizeof(float) * 4 ? CACHE_LINE_SIZE - sizeof(float) * 4 : 1];

    PadBoid(const float px, const float py) : x(px), y(py) {
        const std::pair<float,float> speed = initializeVelocity();
        vx = speed.first;
        vy = speed.second;
    }

    PadBoid() : x(randomFloat(0, WINDOW_WIDTH)), y(randomFloat(0, WINDOW_HEIGHT)) {
        const std::pair<float, float> speed = initializeVelocity();
        vx = speed.first;
        vy = speed.second;
    }

    std::pair<float, float> initializeVelocity() {
        float angle = randomFloat(0, 2 * M_PI);
        vx = std::cos(angle) * MIN_SPEED;
        vy = std::sin(angle) * MIN_SPEED;

        vx += randomFloat(-1.0f, 1.0f);
        vy += randomFloat(-1.0f, 1.0f);

        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }

    void update(float& vx, float& vy, float& x, float& y) {
        this->vx = vx;
        this->vy = vy;
        this->x = x;
        this->y = y;
    }
};

// Similarly remove alignas from Parameters for debugging
struct Parameters {
    float close_dx, close_dy, avg_vx, avg_vy, avg_x, avg_y, neighboring_count;

    Parameters() : close_dx(0), close_dy(0), avg_vx(0), avg_vy(0), avg_x(0), avg_y(0), neighboring_count(0) {}

    void reset() {
        close_dx = 0;
        close_dy = 0;
        avg_vx = 0;
        avg_vy = 0;
        avg_x = 0;
        avg_y = 0;
        neighboring_count = 0;
    }
};

struct alignas(CACHE_LINE_SIZE) PadParameters{
    float close_dx;
    float close_dy;
    float avg_vx;
    float avg_vy;
    float avg_x;
    float avg_y;
    float neighboring_count;
    char pad[CACHE_LINE_SIZE > sizeof(float) * 7 ? CACHE_LINE_SIZE - sizeof(float ) * 7 : 1];

    PadParameters() : close_dx(0), close_dy(0), avg_vx(0), avg_vy(0), avg_x(0), avg_y(0), neighboring_count(0) {}

    void reset() {
        close_dx = 0;
        close_dy = 0;
        avg_vx = 0;
        avg_vy = 0;
        avg_x = 0;
        avg_y = 0;
        neighboring_count = 0;
    }
};


struct BoidsList {
    float x[NUM_BOIDS];
    float y[NUM_BOIDS];
    float vx[NUM_BOIDS];
    float vy[NUM_BOIDS];
    char pad[CACHE_LINE_SIZE > sizeof(float) * 4 * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * 4 * NUM_BOIDS : 1];

    BoidsList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i] = randomFloat(0, WINDOW_WIDTH);
            y[i] = randomFloat(0, WINDOW_HEIGHT);
            const std::pair<float,float> speed = initializeVelocity(i);
            vx[i] = speed.first;
            vy[i] = speed.second;
        }
    }

    std::pair<float, float> initializeVelocity(int i) {
        float angle = randomFloat(0, 2 * M_PI);
        float vx = std::cos(angle) * MIN_SPEED;
        float vy = std::sin(angle) * MIN_SPEED;

        vx += randomFloat(-1.0f, 1.0f);
        vy += randomFloat(-1.0f, 1.0f);

        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }

    void update(int& i, float& vx, float& vy, float& x, float& y) {
        this->vx[i] = vx;
        this->vy[i] = vy;
        this->x[i] = x;
        this->y[i] = y;
    }

};

struct alignas(CACHE_LINE_SIZE) PadBoidsList{
    float x[NUM_BOIDS];
    char pad[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float y[NUM_BOIDS];
    char pad2[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float vx[NUM_BOIDS];
    char pad3[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float vy[NUM_BOIDS];
    char pad4[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];

    PadBoidsList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i] = randomFloat(0, WINDOW_WIDTH);
            y[i] = randomFloat(0, WINDOW_HEIGHT);
            const std::pair<float,float> speed = initializeVelocity(i);
            vx[i] = speed.first;
            vy[i] = speed.second;
        }
    }

    std::pair<float, float> initializeVelocity(int i) {
        float angle = randomFloat(0, 2 * M_PI);
        float vx = std::cos(angle) * MIN_SPEED;
        float vy = std::sin(angle) * MIN_SPEED;

        vx += randomFloat(-1.0f, 1.0f);
        vy += randomFloat(-1.0f, 1.0f);

        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }

    void update(int& i, float& vx, float& vy, float& x, float& y){
        this->vx[i] = vx;
        this->vy[i] = vy;
        this->x[i] = x;
        this->y[i] = y;
    }

};

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
            close_dx[i] = 0;
            close_dy[i] = 0;
            avg_vx[i] = 0;
            avg_vy[i] = 0;
            avg_x[i] = 0;
            avg_y[i] = 0;
            neighboring_count[i] = 0;
        }
    }

    void reset(int& i) {
        this->close_dx[i] = 0;
        this->close_dy[i] = 0;
        this->avg_vx[i] = 0;
        this->avg_vy[i] = 0;
        this->avg_x[i] = 0;
        this->avg_y[i] = 0;
        this->neighboring_count[i] = 0;

    }

};

struct alignas(CACHE_LINE_SIZE) PadParametersList{
    float close_dx[NUM_BOIDS];
    char pad[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float close_dy[NUM_BOIDS];
    char pad2[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float avg_vx[NUM_BOIDS];
    char pad3[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float avg_vy[NUM_BOIDS];
    char pad4[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float avg_x[NUM_BOIDS];
    char pad5[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float avg_y[NUM_BOIDS];
    char pad6[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * NUM_BOIDS : 1];
    float neighboring_count[NUM_BOIDS];
    char pad7[CACHE_LINE_SIZE > sizeof(float) * NUM_BOIDS ? CACHE_LINE_SIZE - sizeof(float) * 7 * NUM_BOIDS : 1];

    PadParametersList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            close_dx[i] = 0;
            close_dy[i] = 0;
            avg_vx[i] = 0;
            avg_vy[i] = 0;
            avg_x[i] = 0;
            avg_y[i] = 0;
            neighboring_count[i] = 0;
        }
    }

    void reset(int& i) {
        this->close_dx[i] = 0;
        this->close_dy[i] = 0;
        this->avg_vx[i] = 0;
        this->avg_vy[i] = 0;
        this->avg_x[i] = 0;
        this->avg_y[i] = 0;
        this->neighboring_count[i] = 0;
    }

};

#endif //STRUCTURES_H
