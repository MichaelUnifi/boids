#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <omp.h>

const int CACHE_LINE_SIZE = 64;


const int NUM_MEASUREMENTS = 10;
const int NUM_ITERATIONS = 5000;
const int NUM_BOIDS = 1000;

// Constants for Boid behavior
const float TURN_FACTOR = 0.2f;
const float MAX_SPEED = 6.0f;
const float MIN_SPEED = 3.0f;
const float CENTERING_FACTOR = 0.005f;
const float AVOID_FACTOR = 0.015f;
const float PROTECTED_RANGE = 15.0f;
const float MATCHING_FACTOR = 0.01f;
const float VISUAL_RANGE = 60.0f;

const int WINDOW_WIDTH = 720;
const int WINDOW_HEIGHT = 480;
const int TOP_MARGIN = 380;
const int BOTTOM_MARGIN = 100;
const int LEFT_MARGIN = 100;
const int RIGHT_MARGIN = 620;

// Random value helper
float randomFloat(float min, float max) {
    return min + static_cast<float>(random()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Boid structure with position, velocity, and initialization
struct alignas(CACHE_LINE_SIZE) Boid {//TODO decide if aos or soa
    alignas(CACHE_LINE_SIZE) float  x, y;   // Position
    alignas(CACHE_LINE_SIZE) float vx, vy; // Velocity
    char pad[CACHE_LINE_SIZE > sizeof(float) * 4 ? CACHE_LINE_SIZE - sizeof(float) * 4 : 1];

    Boid(const float px,const float py){
        x = px;
        y = py;
        const std::pair<float,float> speed = initializeVelocity();
        vx = speed.first;
        vy = speed.second;
    }

    Boid() : x(randomFloat(0, WINDOW_WIDTH)), y(randomFloat(0, WINDOW_HEIGHT)) {
        const std::pair<float,float> speed = initializeVelocity();
        vx = speed.first;
        vy = speed.second;
    }

    std::pair<float,float> initializeVelocity() {
        float angle = randomFloat(0, 2 * M_PI);
        vx = std::cos(angle) * MIN_SPEED + randomFloat(-1.0f, 1.0f);
        vy = std::sin(angle) * MIN_SPEED + randomFloat(-1.0f, 1.0f);

        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }
};

// Parameters structure with padding to avoid false sharing
struct alignas(CACHE_LINE_SIZE) Parameters {
    alignas(CACHE_LINE_SIZE) float close_dx;
    alignas(CACHE_LINE_SIZE) float close_dy;
    alignas(CACHE_LINE_SIZE) float avg_vx;
    alignas(CACHE_LINE_SIZE) float avg_vy;
    alignas(CACHE_LINE_SIZE) float avg_x;
    alignas(CACHE_LINE_SIZE) float avg_y;
    alignas(CACHE_LINE_SIZE) float neighboring_count;
    char pad[CACHE_LINE_SIZE > sizeof(float) * 7 ? CACHE_LINE_SIZE - sizeof(float ) * 7 : 1];

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

void getParameters(Boid& b, Boid boids[], Parameters& params) {
    for (int j = 0; j < NUM_BOIDS; ++j) {
        Boid& other = boids[j];
        if (&b == &other) continue;

        float dx = b.x - other.x;
        float dy = b.y - other.y;

        if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
            float squared_dist = dx * dx + dy * dy;

            if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                params.close_dx += dx;
                params.close_dy += dy;
            } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                params.avg_x += other.x;
                params.avg_y += other.y;
                params.avg_vx += other.vx;
                params.avg_vy += other.vy;
                params.neighboring_count += 1;
            }
        }
    }
}

void update(Boid& boid, Parameters& params) {
    if (params.neighboring_count > 0) {
        params.avg_x /= params.neighboring_count;
        params.avg_y /= params.neighboring_count;
        params.avg_vx /= params.neighboring_count;
        params.avg_vy /= params.neighboring_count;

        boid.vx += (params.avg_x - boid.x) * CENTERING_FACTOR + (params.avg_vx - boid.vx) * MATCHING_FACTOR;
        boid.vy += (params.avg_y - boid.y) * CENTERING_FACTOR + (params.avg_vy - boid.vy) * MATCHING_FACTOR;
        boid.vx += params.close_dx * AVOID_FACTOR;
        boid.vy += params.close_dy * AVOID_FACTOR;
    }

    if (boid.y > TOP_MARGIN) boid.vy -= TURN_FACTOR;
    if (boid.x > RIGHT_MARGIN) boid.vx -= TURN_FACTOR;
    if (boid.x < LEFT_MARGIN) boid.vx += TURN_FACTOR;
    if (boid.y < BOTTOM_MARGIN) boid.vy += TURN_FACTOR;

    float speed = std::sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    if (speed > MAX_SPEED) {
        boid.vx = (boid.vx / speed) * MAX_SPEED;
        boid.vy = (boid.vy / speed) * MAX_SPEED;
    } else if (speed < MIN_SPEED) {
        boid.vx = (boid.vx / speed) * MIN_SPEED;
        boid.vy = (boid.vy / speed) * MIN_SPEED;
    }

    boid.x += boid.vx;
    boid.y += boid.vy;
}

int main() {
    Boid boids[NUM_BOIDS];
    Parameters params[NUM_BOIDS];

    auto start = std::chrono::high_resolution_clock::now();

#pragma omp parallel shared(boids) private(params)
    {
        Parameters parameters;
        for (int j = 0; j < NUM_ITERATIONS; ++j) {

#pragma omp for schedule(dynamic)
            for (int k = 0; k < NUM_BOIDS; ++k) {
                for (int i = 0; i < NUM_BOIDS; ++i) {
                    if (i != k) continue;

                    float dx = boids[k].x - boids[i].x;
                    float dy = boids[k].y - boids[i].y;

                    if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                        float squared_dist = dx * dx + dy * dy;

                        if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                            parameters.close_dx += dx;
                            parameters.close_dy += dy;
                        } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                            parameters.avg_x += boids[k].x;
                            parameters.avg_y += boids[k].y;
                            parameters.avg_vx += boids[k].vx;
                            parameters.avg_vy += boids[k].vy;
                            parameters.neighboring_count += 1;
                        }
                    }
                }
            }

#pragma omp for schedule(dynamic)
        for (int k = 0; k < NUM_BOIDS; ++k) {
            update(boids[k], params[k]);//TODO direct implementation
        }
    }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel time: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}



/*std::unordered_map<int, std::chrono::duration<double>> map;
    for (int i = 0; i < NUM_MEASUREMENTS; ++i) {

        map[i] = end - start;
    }

    saveMapToJSON(map, "map_data.json");*/