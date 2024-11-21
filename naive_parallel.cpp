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


int main() {

    std::cout<<"Naive parallel implementation"<<std::endl;

    //aos benchmark
    std::cout<<"Starting aos benchmark"<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> aos_values;

    {
        Boid boids_aos[NUM_BOIDS];
        Parameters parameters_aos[NUM_BOIDS];
        for(int t = 1; t <= NUM_THREADS_INCREMENTS; ++t) {
            int num_threads = t*2;
            omp_set_num_threads(num_threads);
            std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
            aos_values[num_threads] = std::chrono::duration<double>(0);
            for (int i = 0; i < NUM_BOIDS; ++i) {
                boids_aos[i] = Boid();
            }
            for(int i = 0; i<NUM_MEASUREMENTS; i++) {
                auto start_aos = std::chrono::high_resolution_clock::now();
                #pragma omp parallel num_threads(num_threads)
                {
                    #pragma omp for schedule(static)
                    for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
                        #pragma omp critical
                        {parameters_aos[j].reset();}
                        for (int k = 0; k < NUM_BOIDS; ++k) {
                            if (j == k) continue;// avoid self-comparison

                            float dx = boids_aos[j].x - boids_aos[k].x;
                            float dy = boids_aos[j].y - boids_aos[k].y;

                            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                                float squared_dist = dx * dx + dy * dy;

                                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                                    #pragma omp critical
                                    {
                                        parameters_aos[j].close_dx += dx;
                                        parameters_aos[j].close_dy += dy;
                                    }
                                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                                    #pragma omp critical
                                    {
                                        parameters_aos[j].avg_x += boids_aos[k].x;
                                        parameters_aos[j].avg_y += boids_aos[k].y;
                                        parameters_aos[j].avg_vx += boids_aos[k].vx;
                                        parameters_aos[j].avg_vy += boids_aos[k].vy;
                                        parameters_aos[j].neighboring_count += 1;
                                    }
                                }
                            }
                        }
                    }
                    #pragma omp for schedule(static)
                    for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
                        if (parameters_aos[j].neighboring_count > 0) {
                            #pragma omp critical
                            {
                                parameters_aos[j].avg_x /= parameters_aos[j].neighboring_count;
                                parameters_aos[j].avg_y /= parameters_aos[j].neighboring_count;
                                parameters_aos[j].avg_vx /= parameters_aos[j].neighboring_count;
                                parameters_aos[j].avg_vy /= parameters_aos[j].neighboring_count;
                            }

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
                auto end_aos = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_aos = end_aos - start_aos;
                aos_values[num_threads] += elapsed_aos/NUM_MEASUREMENTS; //normalizing the time by the number
            }
            std::cout<<"parallel aos elapsed time - "<<num_threads<<" threads: "<<aos_values[t*2].count()<<std::endl;
        }
    }
    saveMapToJSON(aos_values, "naive_parallel_aos.json");

    //padded aos benchmark
    std::cout<<"Starting padded aos benchmark"<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> pad_aos_values;

    {
        PadBoid pad_boids_aos[NUM_BOIDS];
        PadParameters pad_parameters_aos[NUM_BOIDS];
        for(int t = 1; t <= NUM_THREADS_INCREMENTS; ++t) {
            int num_threads = t*2;
            omp_set_num_threads(num_threads);
            std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
            pad_aos_values[num_threads] = std::chrono::duration<double>(0);
            for (int i = 0; i < NUM_BOIDS; ++i) {
                pad_boids_aos[i] = PadBoid();
            }
            for(int i = 0; i<NUM_MEASUREMENTS; i++) {
                auto pad_start_aos = std::chrono::high_resolution_clock::now();
                #pragma omp parallel num_threads(num_threads)
                {
                    #pragma omp for schedule(static)
                    for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
                        #pragma omp critical
                        {pad_parameters_aos[j].reset();}
                        for (int k = 0; k < NUM_BOIDS; ++k) {
                            if (j == k) continue;// avoid self-comparison

                            float dx = pad_boids_aos[j].x - pad_boids_aos[k].x;
                            float dy = pad_boids_aos[j].y - pad_boids_aos[k].y;

                            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                                float squared_dist = dx * dx + dy * dy;

                                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                                    #pragma omp critical
                                    {
                                        pad_parameters_aos[j].close_dx += dx;
                                        pad_parameters_aos[j].close_dy += dy;
                                    }
                                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                                    #pragma omp critical
                                    {
                                        pad_parameters_aos[j].avg_x += pad_boids_aos[k].x;
                                        pad_parameters_aos[j].avg_y += pad_boids_aos[k].y;
                                        pad_parameters_aos[j].avg_vx += pad_boids_aos[k].vx;
                                        pad_parameters_aos[j].avg_vy += pad_boids_aos[k].vy;
                                        pad_parameters_aos[j].neighboring_count += 1;
                                    }
                                }
                            }
                        }
                    }
                    #pragma omp for schedule(static)
                    for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
                        if (pad_parameters_aos[j].neighboring_count > 0) {
                            #pragma omp critical
                            {
                                pad_parameters_aos[j].avg_x /= pad_parameters_aos[j].neighboring_count;
                                pad_parameters_aos[j].avg_y /= pad_parameters_aos[j].neighboring_count;
                                pad_parameters_aos[j].avg_vx /= pad_parameters_aos[j].neighboring_count;
                                pad_parameters_aos[j].avg_vy /= pad_parameters_aos[j].neighboring_count;
                            }
                            float new_vx = pad_boids_aos[j].vx;
                            float new_vy = pad_boids_aos[j].vy;
                            float new_x = pad_boids_aos[j].x;
                            float new_y = pad_boids_aos[j].y;
                            new_vx += (pad_parameters_aos[j].avg_x - pad_boids_aos[j].x) * CENTERING_FACTOR + (pad_parameters_aos[j].avg_vx - pad_boids_aos[j].vx) * MATCHING_FACTOR;
                            new_vy += (pad_parameters_aos[j].avg_y - pad_boids_aos[j].y) * CENTERING_FACTOR + (pad_parameters_aos[j].avg_vy - pad_boids_aos[j].vy) * MATCHING_FACTOR;
                            new_vx += pad_parameters_aos[j].close_dx * AVOID_FACTOR;
                            new_vy += pad_parameters_aos[j].close_dy * AVOID_FACTOR;

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
                                pad_boids_aos[j].update(new_vx, new_vy, new_x, new_y);
                            }
                        }
                    }
                }
                auto pad_end_aos = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> pad_elapsed_aos = pad_end_aos - pad_start_aos;
                pad_aos_values[num_threads] += pad_elapsed_aos/NUM_MEASUREMENTS; //normalizing the time by the number
            }
            std::cout<<"parallel padded aos elapsed time - "<<num_threads<<" threads: "<<pad_aos_values[t*2].count()<<std::endl;
        }
    }
    saveMapToJSON(pad_aos_values, "naive_parallel_pad_aos.json");

    //parallel soa benchmark
    std::cout<<"Starting soa benchmark"<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> soa_values;

    {
        BoidsList boids_soa;
        ParametersList parameters_soa;
        for(int t = 1; t <= NUM_THREADS_INCREMENTS; ++t) {
            int num_threads = t*2;
            omp_set_num_threads(num_threads);
            std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
            soa_values[num_threads] = std::chrono::duration<double>(0);
            for(int i = 0; i<NUM_MEASUREMENTS; i++) {
                auto start_soa = std::chrono::high_resolution_clock::now();
                #pragma omp parallel num_threads(num_threads)
                {
                    #pragma omp for schedule(static)
                    for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
                        #pragma omp critical
                        {parameters_soa.reset(j);}
                        for (int k = 0; k < NUM_BOIDS; ++k) {
                            if (j == k) continue;// avoid self-comparison

                            float dx = boids_soa.x[j] - boids_soa.x[k];
                            float dy = boids_soa.y[j] - boids_soa.y[k];

                            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                                float squared_dist = dx * dx + dy * dy;

                                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                                    #pragma omp critical
                                    {
                                        parameters_soa.close_dx[j] += dx;
                                        parameters_soa.close_dy[j] += dy;
                                    }
                                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                                    #pragma omp critical
                                    {
                                        parameters_soa.avg_x[j] += boids_soa.x[k];
                                        parameters_soa.avg_y[j] += boids_soa.y[k];
                                        parameters_soa.avg_vx[j] += boids_soa.vx[k];
                                        parameters_soa.avg_vy[j] += boids_soa.vy[k];
                                        parameters_soa.neighboring_count[j] += 1;
                                    }
                                }
                            }
                        }
                    }
                    #pragma omp for schedule(static)
                    for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
                        if (parameters_soa.neighboring_count[j] > 0) {
                            #pragma omp critical
                            {
                                parameters_soa.avg_x[j] /= parameters_soa.neighboring_count[j];
                                parameters_soa.avg_y[j] /= parameters_soa.neighboring_count[j];
                                parameters_soa.avg_vx[j] /= parameters_soa.neighboring_count[j];
                                parameters_soa.avg_vy[j] /= parameters_soa.neighboring_count[j];
                            }

                            float new_vx = boids_soa.vx[j];
                            float new_vy = boids_soa.vy[j];
                            float new_x = boids_soa.x[j];
                            float new_y = boids_soa.y[j];
                            new_vx += (parameters_soa.avg_x[j] - boids_soa.x[j]) * CENTERING_FACTOR + (parameters_soa.avg_vx[j] - boids_soa.vx[j]) * MATCHING_FACTOR;
                            new_vy += (parameters_soa.avg_y[j] - boids_soa.y[j]) * CENTERING_FACTOR + (parameters_soa.avg_vy[j] - boids_soa.vy[j]) * MATCHING_FACTOR;
                            new_vx += parameters_soa.close_dx[j] * AVOID_FACTOR;
                            new_vy += parameters_soa.close_dy[j] * AVOID_FACTOR;

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
                                boids_soa.update(j, new_vx, new_vy, new_x, new_y);
                            }
                        }
                    }
                }
                auto end_soa = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_soa = end_soa - start_soa;
                soa_values[num_threads] += elapsed_soa/NUM_MEASUREMENTS; //normalizing the time by the number

            }
            std::cout<<"parallel soa elapsed time - "<<num_threads<<" threads: "<<soa_values[t*2].count()<<std::endl;
        }
    }
    saveMapToJSON(soa_values, "naive_parallel_soa.json");

    //padded soa benchmark
    std::cout<<"Starting padded soa benchmark"<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> pad_soa_values;

    {
        PadBoidsList pad_boids_soa;
        PadParametersList pad_parameters_soa;
        for(int t = 1; t <= NUM_THREADS_INCREMENTS; ++t) {
            int num_threads = t*2;
            omp_set_num_threads(num_threads);
            std::cout<<"Running with "<<omp_get_max_threads()<<" threads"<<std::endl;
            pad_soa_values[num_threads] = std::chrono::duration<double>(0);
            for(int i = 0; i<NUM_MEASUREMENTS; i++) {
                auto pad_start_soa = std::chrono::high_resolution_clock::now();
                #pragma omp parallel num_threads(num_threads)
                {
                    #pragma omp for schedule(static)
                    for (int j = 0; j < NUM_BOIDS; ++j) {//get the parameters
                        #pragma omp critical
                        {
                            pad_parameters_soa.reset(j);
                        }
                        for (int k = 0; k < NUM_BOIDS; ++k) {
                            if (j == k) continue;// avoid self-comparison

                            float dx = pad_boids_soa.x[j] - pad_boids_soa.x[k];
                            float dy = pad_boids_soa.y[j] - pad_boids_soa.y[k];

                            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                                float squared_dist = dx * dx + dy * dy;

                                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {// check if the boid is too close
                                    #pragma omp critical
                                    {
                                        pad_parameters_soa.close_dx[j] += dx;
                                        pad_parameters_soa.close_dy[j] += dy;
                                    }
                                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {// check if the other boid is visible
                                    #pragma omp critical
                                    {
                                        pad_parameters_soa.avg_x[j] += pad_boids_soa.x[k];
                                        pad_parameters_soa.avg_y[j] += pad_boids_soa.y[k];
                                        pad_parameters_soa.avg_vx[j] += pad_boids_soa.vx[k];
                                        pad_parameters_soa.avg_vy[j] += pad_boids_soa.vy[k];
                                        pad_parameters_soa.neighboring_count[j] += 1;
                                    }
                                }
                            }
                        }
                    }
                    #pragma omp for schedule(static)
                    for(int j = 0; j < NUM_BOIDS; ++j) {//update the boids
                        if (pad_parameters_soa.neighboring_count[j] > 0) {
                            #pragma omp critical
                            {
                                pad_parameters_soa.avg_x[j] /= pad_parameters_soa.neighboring_count[j];
                                pad_parameters_soa.avg_y[j] /= pad_parameters_soa.neighboring_count[j];
                                pad_parameters_soa.avg_vx[j] /= pad_parameters_soa.neighboring_count[j];
                                pad_parameters_soa.avg_vy[j] /= pad_parameters_soa.neighboring_count[j];
                            }

                            float new_vx = pad_boids_soa.vx[j];
                            float new_vy = pad_boids_soa.vy[j];
                            float new_x = pad_boids_soa.x[j];
                            float new_y = pad_boids_soa.y[j];
                            new_vx += (pad_parameters_soa.avg_x[j] - pad_boids_soa.x[j]) * CENTERING_FACTOR + (pad_parameters_soa.avg_vx[j] - pad_boids_soa.vx[j]) * MATCHING_FACTOR;
                            new_vy += (pad_parameters_soa.avg_y[j] - pad_boids_soa.y[j]) * CENTERING_FACTOR + (pad_parameters_soa.avg_vy[j] - pad_boids_soa.vy[j]) * MATCHING_FACTOR;
                            new_vx += pad_parameters_soa.close_dx[j] * AVOID_FACTOR;
                            new_vy += pad_parameters_soa.close_dy[j] * AVOID_FACTOR;

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
                                pad_boids_soa.update(j, new_vx, new_vy, new_x, new_y);
                            }
                        }
                    }
                }
                auto pad_end_soa = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> pad_elapsed_soa = pad_end_soa - pad_start_soa;
                pad_soa_values[num_threads] += pad_elapsed_soa/NUM_MEASUREMENTS; //normalizing the time by the number
            }
            std::cout<<"parallel padded soa elapsed time - "<<num_threads<<" threads: "<<pad_soa_values[t*2].count()<<std::endl;
        }
    }
    saveMapToJSON(pad_soa_values, "naive_pad_parallel_soa.json");


    return 0;
}