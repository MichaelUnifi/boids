#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>

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


struct alignas(CACHE_LINE_SIZE) Boid {
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
                    parameters_aos[i].reset();
                    for (int k = 0; k < NUM_BOIDS; ++k) {
                        if (i != k) continue;

                        float dx = boids_aos[i].x - boids_aos[k].x;
                        float dy = boids_aos[i].y - boids_aos[k].y;

                        if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                            float squared_dist = dx * dx + dy * dy;

                            if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                                parameters_aos[i].close_dx += dx;
                                parameters_aos[i].close_dy += dy;
                            } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                                parameters_aos[i].avg_x += boids_aos[k].x;
                                parameters_aos[i].avg_y += boids_aos[k].y;
                                parameters_aos[i].avg_vx += boids_aos[k].vx;
                                parameters_aos[i].avg_vy += boids_aos[k].vy;
                                parameters_aos[i].neighboring_count += 1;
                            }
                        }
                    }
                }

                for (int i = 0; i < NUM_BOIDS; ++i) {
                    if (parameters_aos[i].neighboring_count > 0) {
                        parameters_aos[i].avg_x /= parameters_aos[i].neighboring_count;
                        parameters_aos[i].avg_y /= parameters_aos[i].neighboring_count;
                        parameters_aos[i].avg_vx /= parameters_aos[i].neighboring_count;
                        parameters_aos[i].avg_vy /= parameters_aos[i].neighboring_count;

                        boids_aos[i].vx += (parameters_aos[i].avg_x - boids_aos[i].x) * CENTERING_FACTOR + (parameters_aos[i].avg_vx - boids_aos[i].vx) * MATCHING_FACTOR;
                        boids_aos[i].vy += (parameters_aos[i].avg_y - boids_aos[i].y) * CENTERING_FACTOR + (parameters_aos[i].avg_vy - boids_aos[i].vy) * MATCHING_FACTOR;
                        boids_aos[i].vx += parameters_aos[i].close_dx * AVOID_FACTOR;
                        boids_aos[i].vy += parameters_aos[i].close_dy * AVOID_FACTOR;
                    }

                    if (boids_aos[i].y > TOP_MARGIN) boids_aos[i].vy -= TURN_FACTOR;
                    if (boids_aos[i].x > RIGHT_MARGIN) boids_aos[i].vx -= TURN_FACTOR;
                    if (boids_aos[i].x < LEFT_MARGIN) boids_aos[i].vx += TURN_FACTOR;
                    if (boids_aos[i].y < BOTTOM_MARGIN) boids_aos[i].vy += TURN_FACTOR;

                    float speed = std::sqrt(boids_aos[i].vx * boids_aos[i].vx + boids_aos[i].vy * boids_aos[i].vy);
                    if (speed > MAX_SPEED) {
                        boids_aos[i].vx = (boids_aos[i].vx / speed) * MAX_SPEED;
                        boids_aos[i].vy = (boids_aos[i].vy / speed) * MAX_SPEED;
                    } else if (speed < MIN_SPEED) {
                        boids_aos[i].vx = (boids_aos[i].vx / speed) * MIN_SPEED;
                        boids_aos[i].vy = (boids_aos[i].vy / speed) * MIN_SPEED;
                    }

                    boids_aos[i].x += boids_aos[i].vx;
                    boids_aos[i].y += boids_aos[i].vy;
                }
            }

            auto end_aos = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_aos = end_aos - start_aos;
            aos_values[t] = elapsed_aos;

            //sequential soa
            BoidsList boids_soa;
            ParametersList parameters_soa;

            auto start_soa = std::chrono::high_resolution_clock::now();

            for(int i = 0; i < NUM_ITERATIONS; i++) {
                parameters_soa.reset();//here it's a global reset, so we do it only once
                for (int j = 0; j < NUM_BOIDS; ++j) {
                    for(int k = 0; k < NUM_BOIDS; ++k) {
                        if (k != j) continue;

                        float dx = boids_soa.x[j] - boids_soa.x[k];
                        float dy = boids_soa.y[j] - boids_soa.y[k];

                        if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                            float squared_dist = dx * dx + dy * dy;

                            if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                                parameters_soa.close_dx[j] += dx;
                                parameters_soa.close_dy[j] += dy;
                            } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                                parameters_soa.avg_x[j] += boids_soa.x[k];
                                parameters_soa.avg_y[j] += boids_soa.y[k];
                                parameters_soa.avg_vx[j] += boids_soa.vx[k];
                                parameters_soa.avg_vy[j] += boids_soa.vy[k];
                                parameters_soa.neighboring_count[j] += 1;
                            }
                        }
                    }

                }
                for (int j = 0; i < NUM_BOIDS; ++i) {
                    if (parameters_soa.neighboring_count[i] > 0) {
                        parameters_soa.avg_x[i] /= parameters_soa.neighboring_count[i];
                        parameters_soa.avg_y[i] /= parameters_soa.neighboring_count[i];
                        parameters_soa.avg_vx[i] /= parameters_soa.neighboring_count[i];
                        parameters_soa.avg_vy[i] /= parameters_soa.neighboring_count[i];
                        boids_soa.vx[i] += (parameters_soa.avg_x[i] - boids_soa.x[i]) * CENTERING_FACTOR + (parameters_soa.avg_vx[i] - boids_soa.vx[i]) * MATCHING_FACTOR;
                        boids_soa.vy[i] += (parameters_soa.avg_y[i] - boids_soa.y[i]) * CENTERING_FACTOR + (parameters_soa.avg_vy[i] - boids_soa.vy[i]) * MATCHING_FACTOR;
                        boids_soa.vx[i] += parameters_soa.close_dx[i] * AVOID_FACTOR;
                        boids_soa.vy[i] += parameters_soa.close_dy[i] * AVOID_FACTOR;
                    }

                    if (boids_soa.y[i] > TOP_MARGIN) boids_soa.vy[i] -= TURN_FACTOR;
                    if (boids_soa.x[i] > RIGHT_MARGIN) boids_soa.vx[i] -= TURN_FACTOR;
                    if (boids_soa.x[i] < LEFT_MARGIN) boids_soa.vx[i] += TURN_FACTOR;
                    if (boids_soa.y[i] < BOTTOM_MARGIN) boids_soa.vy[i] += TURN_FACTOR;

                    float speed = std::sqrt(boids_soa.vx[i] * boids_soa.vx[i] + boids_soa.vy[i] * boids_soa.vy[i]);
                    if (speed > MAX_SPEED) {
                        boids_soa.vx[i] = (boids_soa.vx[i] / speed) * MAX_SPEED;
                        boids_soa.vy[i] = (boids_soa.vy[i] / speed) * MAX_SPEED;
                    } else if (speed < MIN_SPEED) {
                        boids_soa.vx[i] = (boids_soa.vx[i] / speed) * MIN_SPEED;
                        boids_soa.vy[i] = (boids_soa.vy[i] / speed) * MIN_SPEED;
                    }

                    boids_soa.x[i] += boids_soa.vx[i];
                    boids_soa.y[i] += boids_soa.vy[i];
                }
            }

            auto end_soa = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_soa = end_soa - start_soa;
            soa_values[t] = elapsed_soa;
        }


    std::cout<<"aos average time: "<<std::accumulate(aos_values, aos_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0)).count()/NUM_MEASUREMENTS<<std::endl;
    std::cout<<"soa average time: "<<std::accumulate(soa_values, soa_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0)).count()/NUM_MEASUREMENTS<<std::endl;

    return 0;
}
