#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <fstream>
#include <string>
#include <omp.h>
#include "structures.h"
#include <nlohmann/json.hpp>


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

void getParametersSOA(PadBoidsList* boids, PadParametersList* parameters) {
    #pragma omp for schedule(static) nowait
    for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
        parameters->reset(j);
        for (int k = 0; k < NUM_BOIDS; ++k) {
            if (j == k) continue;// avoid self-comparison

            float dx = boids->x[j] - boids->x[k];
            float dy = boids->y[j] - boids->y[k];

            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                float squared_dist = dx * dx + dy * dy;

                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                    parameters->close_dx[j] += dx;
                    parameters->close_dy[j] += dy;
                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                    parameters->avg_x[j] += boids->x[k];
                    parameters->avg_y[j] += boids->y[k];
                    parameters->avg_vx[j] += boids->vx[k];
                    parameters->avg_vy[j] += boids->vy[k];
                    parameters->neighboring_count[j] += 1;
                }
            }
        }
    }
}

void updateSOA(PadBoidsList* boids, PadParametersList* parameters) {
    #pragma omp for schedule(static)
    for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
        if (parameters->neighboring_count[j] > 0) {
            parameters->avg_x[j] /= parameters->neighboring_count[j];
            parameters->avg_y[j] /= parameters->neighboring_count[j];
            parameters->avg_vx[j] /= parameters->neighboring_count[j];
            parameters->avg_vy[j] /= parameters->neighboring_count[j];

            float new_vx = boids->vx[j];
            float new_vy = boids->vy[j];
            float new_x = boids->x[j];
            float new_y = boids->y[j];
            new_vx += (parameters->avg_x[j] - boids->x[j]) * CENTERING_FACTOR + (parameters->avg_vx[j] - boids->vx[j]) * MATCHING_FACTOR;
            new_vy += (parameters->avg_y[j] - boids->y[j]) * CENTERING_FACTOR + (parameters->avg_vy[j] - boids->vy[j]) * MATCHING_FACTOR;
            new_vx += parameters->close_dx[j] * AVOID_FACTOR;
            new_vy += parameters->close_dy[j] * AVOID_FACTOR;

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

            new_x = new_x + new_vx;
            new_y = new_y + new_vy;

            boids->update(j, new_vx, new_vy, new_x, new_y);
        }
    }
}

void getParametersAOS(PadBoid* boids, PadParameters* parameters) {
    #pragma omp for schedule(static) nowait
    for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
        parameters[j].reset();
        for (int k = 0; k < NUM_BOIDS; ++k) {
            if (j == k) continue;// avoid self-comparison

            float dx = boids[j].x - boids[k].x;
            float dy = boids[j].y - boids[k].y;

            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                float squared_dist = dx * dx + dy * dy;

                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                    parameters[j].close_dx += dx;
                    parameters[j].close_dy += dy;
                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                    parameters[j].avg_x += boids[k].x;
                    parameters[j].avg_y += boids[k].y;
                    parameters[j].avg_vx += boids[k].vx;
                    parameters[j].avg_vy += boids[k].vy;
                    parameters[j].neighboring_count += 1;
                }
            }
        }
    }
}

void updateAOS(PadBoid* boids, PadParameters* parameters) {
    #pragma omp for schedule(static)
    for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
        if (parameters[j].neighboring_count > 0) {
            parameters[j].avg_x /= parameters[j].neighboring_count;
            parameters[j].avg_y /= parameters[j].neighboring_count;
            parameters[j].avg_vx /= parameters[j].neighboring_count;
            parameters[j].avg_vy /= parameters[j].neighboring_count;

            float new_vx = boids[j].vx;
            float new_vy = boids[j].vy;
            float new_x = boids[j].x;
            float new_y = boids[j].y;
            new_vx += (parameters[j].avg_x - boids[j].x) * CENTERING_FACTOR + (parameters[j].avg_vx - boids[j].vx) * MATCHING_FACTOR;
            new_vy += (parameters[j].avg_y - boids[j].y) * CENTERING_FACTOR + (parameters[j].avg_vy - boids[j].vy) * MATCHING_FACTOR;
            new_vx += parameters[j].close_dx * AVOID_FACTOR;
            new_vy += parameters[j].close_dy * AVOID_FACTOR;

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

            new_x = new_x + new_vx;
            new_y = new_y + new_vy;

            boids[j].update(new_vx, new_vy, new_x, new_y);
        }
    }
}


int main() {

    std::cout<<"Starting benchmark"<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> Values;



    for(int t = 1; t <= NUM_THREADS_INCREMENTS; ++t) {
        int numThreads = t*2;
        omp_set_num_threads(numThreads);
        std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
        PadBoidsList* soaBoids = new PadBoidsList();
        PadParametersList* soaParameters = new PadParametersList();
        #pragma omp for schedule(static)
        for(int i = 0; i < NUM_BOIDS; ++i) { //first touch initialization
            soaBoids->x[i] = static_cast<float>(rand() % WINDOW_WIDTH);
            soaBoids->y[i] = static_cast<float>(rand() % WINDOW_HEIGHT);
            soaBoids->vx[i] = static_cast<float>(rand() % 10 - 5);
            soaBoids->vy[i] = static_cast<float>(rand() % 10 - 5);
        }
        Values[numThreads] = std::chrono::duration<double>(0);
        Values[20 + numThreads] = std::chrono::duration<double>(0);
        for(int i = 0; i < NUM_MEASUREMENTS; i++) {
            auto padStartSoa = std::chrono::high_resolution_clock::now();
            #pragma omp parallel num_threads(numThreads) shared(soaBoids, numThreads) firstprivate(soaParameters) default(none)
            {
                getParametersSOA(soaBoids, soaParameters);
                updateSOA(soaBoids, soaParameters);
            }
            auto padEndSoa = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> padElapsedSoa = padEndSoa - padStartSoa;
            Values[20 + numThreads] += padElapsedSoa;
        }
        delete soaBoids;
        delete soaParameters;
        PadBoid* aosBoids = new PadBoid[NUM_BOIDS];
        PadParameters* aosParameters = new PadParameters[NUM_BOIDS];
        #pragma omp for schedule(static)
        for(int i = 0; i < NUM_BOIDS; ++i) { //first touch initialization
            aosBoids[i] = PadBoid(static_cast<float>(rand() % WINDOW_WIDTH), static_cast<float>(rand() % WINDOW_HEIGHT));
        }
        Values[20 + numThreads] /= NUM_MEASUREMENTS;
        std::cout<<"parallel padded soa elapsed time - "<<numThreads<<" threads: "<<Values[20 + numThreads].count()<<std::endl;
        for (int i = 0; i < NUM_MEASUREMENTS; i++) {
            auto padStartAos = std::chrono::high_resolution_clock::now();
            #pragma omp parallel num_threads(numThreads) shared(aosBoids, numThreads) firstprivate(aosParameters) default(none)
            {
                getParametersAOS(aosBoids, aosParameters);
                updateAOS(aosBoids, aosParameters);
            }
            auto padEndAos = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> padElapsedAos = padEndAos - padStartAos;
            Values[numThreads] += padElapsedAos;
        }
        Values[numThreads] /= NUM_MEASUREMENTS;
        std::cout<<"parallel padded aos elapsed time - "<<numThreads<<" threads: "<<Values[numThreads].count()<<std::endl;
        std::cout<<"----------------------------------------"<<std::endl;

        delete[] aosBoids;
        delete[] aosParameters;
    }


    saveMapToJSON(Values, "omp_boids.json");

    return 0;
}