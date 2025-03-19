//
// Created by michael on 11/13/24.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

constexpr int CACHE_LINE_SIZE = 64;

constexpr int NUM_THREADS_INCREMENTS = 10;
constexpr int NUM_MEASUREMENTS = 100;
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

#endif //CONSTANTS_H
