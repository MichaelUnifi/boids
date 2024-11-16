#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <omp.h>

constexpr int NUM_MEASUREMENTS = 10;
constexpr int NUM_ITERATIONS = 1;
constexpr int NUM_BOIDS = 40000;

constexpr float TURN_FACTOR = 0.2f;
constexpr float MAX_SPEED = 6.0f;
constexpr float MIN_SPEED = 3.0f;
constexpr float CENTERING_FACTOR = 0.005f;
constexpr float AVOID_FACTOR = 0.015f;
constexpr float PROTECTED_RANGE = 15.0f;
constexpr float MATCHING_FACTOR = 0.01f;
constexpr float VISUAL_RANGE = 60.0f;

constexpr int WINDOW_WIDTH = 720;
constexpr int WINDOW_HEIGHT = 480;
constexpr int TOP_MARGIN = 380;
constexpr int BOTTOM_MARGIN = 100;
constexpr int LEFT_MARGIN = 100;
constexpr int RIGHT_MARGIN = 620;

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

float randomFloat(float min, float max) {
    static std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<float> distribution(min, max);
    return distribution(generator);
}

struct Boid {
    float x, y;   // Position
    float vx, vy; // Velocity

    Boid(const float px,const float py){
        x = px;
        y = py;
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
};

// Remove alignas to check if alignment causes the segfault
struct PadBoid {
    float x, y;   // Position
    float vx, vy; // Velocity

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

int main() {
    std::unordered_map<int,std::chrono::duration<double>> aos_values;

    PadBoid boids_aos[NUM_BOIDS];
    Parameters parameters_aos[NUM_BOIDS];
    for(int t = 1; t <= NUM_MEASUREMENTS; ++t) {
        auto start_aos = std::chrono::high_resolution_clock::now();
        int num_threads = t*2;
        omp_set_num_threads(num_threads);
        std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
        #pragma omp parallel for
        for (int i = 0; i < NUM_BOIDS; ++i) {
            boids_aos[i] = PadBoid();
        }
        #pragma omp parallel shared(boids_aos) private(parameters_aos)
        {
            for(int i = 0; i<NUM_ITERATIONS; i++) {
                #pragma omp for nowait schedule(static) collapse(1)
                for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
                    parameters_aos[j].reset();
                    for (int k = 0; k < NUM_BOIDS; ++k) {
                        if (j == k) continue;// avoid self-comparison

                        float dx = boids_aos[j].x - boids_aos[k].x;
                        float dy = boids_aos[j].y - boids_aos[k].y;

                        if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                            float squared_dist = dx * dx + dy * dy;

                            if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                                parameters_aos[j].close_dx += dx;
                                parameters_aos[j].close_dy += dy;
                            } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                                parameters_aos[j].avg_x += boids_aos[k].x;
                                parameters_aos[j].avg_y += boids_aos[k].y;
                                parameters_aos[j].avg_vx += boids_aos[k].vx;
                                parameters_aos[j].avg_vy += boids_aos[k].vy;
                                parameters_aos[j].neighboring_count += 1;
                            }
                        }
                    }
                }
                #pragma omp for schedule(static) nowait
                for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
                    if (parameters_aos[j].neighboring_count > 0) {
                        parameters_aos[j].avg_x /= parameters_aos[j].neighboring_count;
                        parameters_aos[j].avg_y /= parameters_aos[j].neighboring_count;
                        parameters_aos[j].avg_vx /= parameters_aos[j].neighboring_count;
                        parameters_aos[j].avg_vy /= parameters_aos[j].neighboring_count;

                        float new_vx = boids_aos[j].vx;
                        float new_vy = boids_aos[j].vy;
                        float new_x = boids_aos[j].x;
                        float new_y = boids_aos[j].y;
                        new_vx += (parameters_aos[j].avg_x - boids_aos[j].x) * CENTERING_FACTOR + (parameters_aos[j].avg_vx - boids_aos[j].vx) * MATCHING_FACTOR;
                        new_vy += (parameters_aos[j].avg_y - boids_aos[j].y) * CENTERING_FACTOR + (parameters_aos[j].avg_vy - boids_aos[j].vy) * MATCHING_FACTOR;
                        new_vx += parameters_aos[j].close_dx * AVOID_FACTOR;
                        new_vy += parameters_aos[j].close_dy * AVOID_FACTOR;

                        float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);
                        if (new_y > TOP_MARGIN) new_vy -= TURN_FACTOR;
                        if (new_x > RIGHT_MARGIN) new_vx -= TURN_FACTOR;
                        if (new_x < LEFT_MARGIN) new_vx += TURN_FACTOR;
                        if (new_y < BOTTOM_MARGIN) new_vy += TURN_FACTOR;

                        if (speed > MAX_SPEED) {
                            new_vx = (new_vx / speed) * MAX_SPEED;
                            new_vy = (new_vy / speed) * MAX_SPEED;
                        } else if (speed < MIN_SPEED) {
                            new_vx = (new_vx / speed) * MIN_SPEED;
                            new_vy = (new_vy / speed) * MIN_SPEED;
                        }

                        #pragma omp critical //all parameters must be updated at the same time, otherwise inconsistencies could (will) arise
                        {
                            boids_aos[j].update(new_vx, new_vy, new_x, new_y);
                        }
                    }
                }
            }

        }
        auto end_aos = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_aos = end_aos - start_aos;
        std::cout<<"parallel aos elapsed time - "<<num_threads<<" threads: "<<elapsed_aos.count()<<std::endl;
        aos_values[num_threads] = elapsed_aos;
    }
        saveMapToJSON(aos_values, "parallel_aos.json");
        for (const auto& pair : aos_values) {
            std::cout << "Threads: " << pair.first << ", Elapsed time: " << pair.second.count() << "s\n";
        }
        return 0;

}