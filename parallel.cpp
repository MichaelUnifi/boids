#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <omp.h>
#include "structures.h"


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

void getParameters(PadBoidsList* boids, PadParametersList* parameters) {

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

void update(PadBoidsList* boids, PadParametersList* parameters) {
    #pragma omp for schedule(static) nowait
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
            boids->update(j, new_vx, new_vy, new_x, new_y);
        }
    }
}


int main() {

    std::cout<<"Starting padded soa benchmark"<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> pad_soa_values;

    PadBoidsList* boids = new PadBoidsList();
    PadParametersList* parameters = new PadParametersList();
    for(int t = 1; t <= NUM_THREADS_INCREMENTS; ++t) {
        int num_threads = t*2;
        omp_set_num_threads(num_threads);
        std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
        pad_soa_values[num_threads] = std::chrono::duration<double>(0);
        for(int i = 0; i<NUM_MEASUREMENTS; i++) {
            auto pad_start_soa = std::chrono::high_resolution_clock::now();
            #pragma omp parallel shared(boids) firstprivate(parameters) num_threads(num_threads) //firstprivate here gives an initialized copy of the parameters to each thread
            {
                PadBoidsList* local_boids;
                local_boids = new PadBoidsList(*boids);
                getParameters(local_boids, parameters);
                update(local_boids, parameters);
                #pragma omp for
                for(int j = 0; j < NUM_BOIDS; j++) {
                    #pragma omp critical
                    {
                        boids->update(j, local_boids->vx[j], local_boids->vy[j], local_boids->x[j], local_boids->y[j]);
                    }
                }
            }

            auto pad_end_soa = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> pad_elapsed_soa = pad_end_soa - pad_start_soa;
            pad_soa_values[num_threads] += pad_elapsed_soa/NUM_MEASUREMENTS; //normalizing the time by the number

        }
        std::cout<<"parallel padded soa elapsed time - "<<num_threads<<" threads: "<<pad_soa_values[t*2].count()<<std::endl;
    }
    saveMapToJSON(pad_soa_values, "best_pad_parallel_soa.json");


    return 0;
}