#include <cmath>
#include <iostream>
#include <chrono>
#include <random>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
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


void aos_getParameters(PadBoid* boids, PadParameters* parameters) {

    for (int i = 0; i < NUM_BOIDS; ++i) {
        parameters[i].reset();
        for (int k = 0; k < NUM_BOIDS; ++k) {
            if (i == k) continue;

            float dx = boids[i].x - boids[k].x;
            float dy = boids[i].y - boids[k].y;

            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                float squared_dist = dx * dx + dy * dy;

                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                    parameters[i].close_dx += dx;
                    parameters[i].close_dy += dy;
                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                    parameters[i].avg_x += boids[k].x;
                    parameters[i].avg_y += boids[k].y;
                    parameters[i].avg_vx += boids[k].vx;
                    parameters[i].avg_vy += boids[k].vy;
                    parameters[i].neighboring_count += 1;
                }
            }
        }
    }

}


void aos_update(PadBoid* boid, PadParameters* params) {
    for(int i = 0; i < NUM_BOIDS; ++i) {
        if (params->neighboring_count > 0) {
            params->avg_x /= params->neighboring_count;
            params->avg_y /= params->neighboring_count;
            params->avg_vx /= params->neighboring_count;
            params->avg_vy /= params->neighboring_count;

            boid->vx += (params->avg_x - boid->x) * CENTERING_FACTOR + (params->avg_vx - boid->vx) * MATCHING_FACTOR;
            boid->vy += (params->avg_y - boid->y) * CENTERING_FACTOR + (params->avg_vy - boid->vy) * MATCHING_FACTOR;
            boid->vx += params->close_dx * AVOID_FACTOR;
            boid->vy += params->close_dy * AVOID_FACTOR;
        }

        if (boid->y > TOP_MARGIN) boid->vy -= TURN_FACTOR;
        if (boid->x > RIGHT_MARGIN) boid->vx -= TURN_FACTOR;
        if (boid->x < LEFT_MARGIN) boid->vx += TURN_FACTOR;
        if (boid->y < BOTTOM_MARGIN) boid->vy += TURN_FACTOR;

        float speed = std::sqrt(boid->vx * boid->vx + boid->vy * boid->vy);
        if (speed > MAX_SPEED) {
            boid->vx = (boid->vx / speed) * MAX_SPEED;
            boid->vy = (boid->vy / speed) * MAX_SPEED;
        } else if (speed < MIN_SPEED) {
            boid->vx = (boid->vx / speed) * MIN_SPEED;
            boid->vy = (boid->vy / speed) * MIN_SPEED;
        }

        boid->x += boid->vx;
        boid->y += boid->vy;
    }

}

void soa_getParameters(PadBoidsList& boids, PadParametersList& parameters) {
    for (int i = 0; i < NUM_BOIDS; ++i) {
        parameters.reset(i);
        for (int k = 0; k < NUM_BOIDS; ++k) {
            if (k == i) continue;

            float dx = boids.x[i] - boids.x[k];
            float dy = boids.y[i] - boids.y[k];

            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                float squared_dist = dx * dx + dy * dy;

                if (squared_dist < PROTECTED_RANGE * PROTECTED_RANGE) {
                    parameters.close_dx[i] += dx;
                    parameters.close_dy[i] += dy;
                } else if (squared_dist < VISUAL_RANGE * VISUAL_RANGE) {
                    parameters.avg_x[i] += boids.x[k];
                    parameters.avg_y[i] += boids.y[k];
                    parameters.avg_vx[i] += boids.vx[k];
                    parameters.avg_vy[i] += boids.vy[k];
                    parameters.neighboring_count[i] += 1;
                }
            }
        }
}
}

void soa_update(PadBoidsList& boids, PadParametersList& parameters) {
    for(int i = 0; i < NUM_BOIDS; i++) {
        if (parameters.neighboring_count[i] > 0) {
            parameters.avg_x[i] /= parameters.neighboring_count[i];
            parameters.avg_y[i] /= parameters.neighboring_count[i];
            parameters.avg_vx[i] /= parameters.neighboring_count[i];
            parameters.avg_vy[i] /= parameters.neighboring_count[i];
            boids.vx[i] += (parameters.avg_x[i] - boids.x[i]) * CENTERING_FACTOR + (parameters.avg_vx[i] - boids.vx[i]) * MATCHING_FACTOR;
            boids.vy[i] += (parameters.avg_y[i] - boids.y[i]) * CENTERING_FACTOR + (parameters.avg_vy[i] - boids.vy[i]) * MATCHING_FACTOR;
            boids.vx[i] += parameters.close_dx[i] * AVOID_FACTOR;
            boids.vy[i] += parameters.close_dy[i] * AVOID_FACTOR;
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

}


int main() {

    std::chrono::duration<double> aos_values[NUM_MEASUREMENTS];
    std::chrono::duration<double> soa_values[NUM_MEASUREMENTS];
    //sequential aos
    PadBoid boids_aos[NUM_BOIDS];
    PadParameters parameters_aos[NUM_BOIDS];

    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids_aos[i] = PadBoid();
    }
    for(int t = 0; t < NUM_MEASUREMENTS; t++) {

        auto start_aos = std::chrono::high_resolution_clock::now();

        aos_getParameters(boids_aos, parameters_aos);
        aos_update(boids_aos, parameters_aos);


        auto end_aos = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_aos = end_aos - start_aos;
        aos_values[t] = elapsed_aos;

    }
    std::cout<<"aos average time: "<<std::accumulate(aos_values, aos_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0)).count()/NUM_MEASUREMENTS<<std::endl;

    PadBoidsList boids_soa;
    PadParametersList parameters_soa;

    for(int t = 0; t < NUM_MEASUREMENTS; t++) {//sequential soa



        auto start_soa = std::chrono::high_resolution_clock::now();

        soa_getParameters(boids_soa, parameters_soa);
        soa_update(boids_soa, parameters_soa);

        auto end_soa = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_soa = end_soa - start_soa;
        soa_values[t] = elapsed_soa;
    }
    std::cout<<"soa average time: "<<std::accumulate(soa_values, soa_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0)).count()/NUM_MEASUREMENTS<<std::endl;
    std::unordered_map<int,std::chrono::duration<double>> sequential_values;
    sequential_values[0] = std::accumulate(aos_values, aos_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0))/NUM_MEASUREMENTS;
    sequential_values[1] = std::accumulate(soa_values, soa_values + NUM_MEASUREMENTS, std::chrono::duration<double>(0))/NUM_MEASUREMENTS;
    saveMapToJSON(sequential_values, "boids_sequential_results.json");

    return 0;
}
