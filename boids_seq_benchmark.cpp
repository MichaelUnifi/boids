#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>


const int NUM_MEASUREMENTS = 10;
const int NUM_ITERATIONS = 1000;
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

void saveMapToJSON(const std::unordered_map<int, std::chrono::duration<double>>& myMap, const std::string& filename) {

    nlohmann::json j_map;
    for (const auto& pair : myMap) {
        j_map[std::to_string(pair.first)] = pair.second.count(); // Convert duration to double (seconds)
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        file << j_map.dump(4); // Pretty-print with 4 spaces indentation
        file.close();
        std::cout << "Map saved to " << filename << " successfully." << std::endl;
    } else {
        std::cerr << "Could not open file for writing." << std::endl;
    }
}

// Random value helper
float randomFloat(float min, float max) {
    return min + static_cast<float>(random()) / (static_cast<float>(RAND_MAX / (max - min)));
}


struct Boid {
    float x, y;   // Position
    float vx, vy; // Velocity

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
        // Generate a random angle in radians for the direction
        float angle = randomFloat(0, 2 * M_PI);

        // Set initial velocity components based on the minimum speed and angle
        vx = std::cos(angle) * MIN_SPEED;
        vy = std::sin(angle) * MIN_SPEED;

        // Add some random fluctuation to the velocity
        vx += randomFloat(-1.0f, 1.0f);
        vy += randomFloat(-1.0f, 1.0f);

        // Ensure the speed meets the minimum requirement
        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }
};

struct BoidsList {
    float x[NUM_BOIDS];
    float y[NUM_BOIDS];
    float vx[NUM_BOIDS];
    float vy[NUM_BOIDS];

    BoidsList() {
        for (int i = 0; i < NUM_BOIDS; ++i) {
            x[i] = randomFloat(0, WINDOW_WIDTH);
            y[i] = randomFloat(0, WINDOW_HEIGHT);
            const std::pair<float,float> speed = initializeVelocity();
            vx[i] = speed.first;
            vy[i] = speed.second;
        }
    }

    static std::pair<float,float> initializeVelocity() {
        // Generate a random angle in radians for the direction
        float angle = randomFloat(0, 2 * M_PI);

        float vx = std::cos(angle) * MIN_SPEED;
        float vy = std::sin(angle) * MIN_SPEED;

        // Add some random fluctuation to the velocity
        vx += randomFloat(-1.0f, 1.0f);
        vy += randomFloat(-1.0f, 1.0f);

        // Speed control
        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed < MIN_SPEED) {
            vx = (vx / speed) * MIN_SPEED;
            vy = (vy / speed) * MIN_SPEED;
        }
        return std::make_pair(vx, vy);
    }

};

struct Parameters {
    float close_dx;
    float close_dy;
    float avg_vx;
    float avg_vy;
    float avg_x;
    float avg_y;
    float neighboring_count;

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

    void reset() {
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
};

void aos_getParameters(Boid& b, Boid boids[], Parameters& params) {
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

void soa_getParameters(BoidsList& b, ParametersList& params) {

    for (int i = 0; i < NUM_BOIDS; ++i) {
    params.reset();
        for (int j = 0; j < NUM_BOIDS; ++j) {
            if (i == j) continue;

            float dx = b.x[i] - b.x[j];
            float dy = b.y[i] - b.y[j];

            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                float squared_dist = dx * dx + dy * dy;

                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                    params.close_dx[i] += dx;
                    params.close_dy[i] += dy;
                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                    params.avg_x[i] += b.x[j];
                    params.avg_y[i] += b.y[j];
                    params.avg_vx[i] += b.vx[j];
                    params.avg_vy[i] += b.vy[j];
                    params.neighboring_count[i] += 1;
                }
            }
        }
    }
}


void aos_update(Boid& boid, Parameters& params) {
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

void soa_update(int i, BoidsList& boids, ParametersList& params) {
    if (params.neighboring_count[i] > 0) {
        params.avg_x[i] /= params.neighboring_count[i];
        params.avg_y[i] /= params.neighboring_count[i];
        params.avg_vx[i] /= params.neighboring_count[i];
        params.avg_vy[i] /= params.neighboring_count[i];
        boids.vx[i] += (params.avg_x[i] - boids.x[i]) * CENTERING_FACTOR + (params.avg_vx[i] - boids.vx[i]) * MATCHING_FACTOR;
        boids.vy[i] += (params.avg_y[i] - boids.y[i]) * CENTERING_FACTOR + (params.avg_vy[i] - boids.vy[i]) * MATCHING_FACTOR;
        boids.vx[i] += params.close_dx[i] * AVOID_FACTOR;
        boids.vy[i] += params.close_dy[i] * AVOID_FACTOR;
    }

    if (boids.y[i] > TOP_MARGIN) boids.vy[i] -= TURN_FACTOR;
    if (boids.x[i] > RIGHT_MARGIN) boids.vx[i] -= TURN_FACTOR;
    if (boids.x[i] < LEFT_MARGIN) boids.vx[i] += TURN_FACTOR;
    if (boids.y[i] < BOTTOM_MARGIN) boids.vy[i] += TURN_FACTOR;

    float speed = std::sqrt(boids.vx[i] * boids.vx[i] + boids.vy[i] * boids.vy[i]);
    if (speed > MAX_SPEED) {
        boids.vx[i] = (boids.vx[i] / speed) * MAX_SPEED;
        boids.vy[i] = (boids.vy[i] / speed) * MAX_SPEED;
    } else if (speed < MIN_SPEED) {
        boids.vx[i] = (boids.vx[i] / speed) * MIN_SPEED;
        boids.vy[i] = (boids.vy[i] / speed) * MIN_SPEED;
    }

    boids.x[i] += boids.vx[i];
    boids.y[i] += boids.vy[i];
}

int main() {

    std::unordered_map<int, std::chrono::duration<double>> aos_map;
    std::unordered_map<int, std::chrono::duration<double>> soa_map;

    for(int l = 10; l<NUM_BOIDS; l+=10) {

        std::chrono::duration<double> aos_values[NUM_MEASUREMENTS];
        std::chrono::duration<double> soa_values[NUM_MEASUREMENTS];

        for(int t = 0; t < NUM_MEASUREMENTS; t++) {
            //sequential aos
            Boid boids_aos[NUM_BOIDS];
            Parameters parameters_aos[NUM_BOIDS];

            for (int i = 0; i < NUM_BOIDS; ++i) {
                boids_aos[i] = Boid();
            }
            auto start_aos = std::chrono::high_resolution_clock::now();

            for(int j=0; j<NUM_ITERATIONS; j++) {
                for (int i = 0; i < NUM_BOIDS; ++i) {
                    aos_getParameters(boids_aos[i], boids_aos, parameters_aos[i]);
                }

                for (int i = 0; i < NUM_BOIDS; ++i) {
                    aos_update(boids_aos[i], parameters_aos[i]);
                }
            }

            auto end_aos = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_aos = end_aos - start_aos;
            aos_values[t] = elapsed_aos;

            //sequential soa
            BoidsList boids_soa;
            ParametersList parameters_soa;

            auto start_soa = std::chrono::high_resolution_clock::now();

            for(int i=0; i<NUM_ITERATIONS; i++) {
                soa_getParameters(boids_soa, parameters_soa);
            }
            for (int i = 0; i < NUM_BOIDS; ++i) {
                soa_update(i, boids_soa, parameters_soa);
            }

            auto end_soa = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_soa = end_soa - start_soa;
            soa_values[t] = elapsed_soa;
        }
        aos_map[l] = std::accumulate(aos_values, aos_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0)) / NUM_MEASUREMENTS;
        soa_map[l] = std::accumulate(soa_values, soa_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0)) / NUM_MEASUREMENTS;

        std::cout<<"Number of boids: "<<l<<std::endl;
    }


    saveMapToJSON(aos_map, "aos_map_data.json");
    saveMapToJSON(soa_map, "soa_map_data.json");

    return 0;
}
