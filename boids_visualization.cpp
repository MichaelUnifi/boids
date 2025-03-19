#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <chrono>
#include <random>

const int NUM_BOIDS = 300;

// Constants for Boid behavior, note that the number of boids has been reduced to 300 for a simple visualization
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

// Boid struct
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
    Parameters parameters[NUM_BOIDS];

    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids[i] = Boid();
    }

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Boids Simulation");
    auto start = std::chrono::high_resolution_clock::now();

    while (window.isOpen()) {
        auto event = sf::Event();
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        for (int i = 0; i < NUM_BOIDS; ++i) {
            parameters[i].reset();
            getParameters(boids[i], boids, parameters[i]);
        }

        for (int i = 0; i < NUM_BOIDS; ++i) {
            update(boids[i], parameters[i]);
        }

        window.clear();
        for (int i = 0; i < NUM_BOIDS; ++i) {
            // Create a line (rectangle) to represent the boid's orientation
            float length = 10.0f; // Line length
            sf::RectangleShape boidShape(sf::Vector2f(length, 2.0f)); // Width of 2 for thin line
            boidShape.setFillColor(sf::Color(255, 105, 180));

            // Position the line at the boid's location
            boidShape.setPosition(boids[i].x, boids[i].y);

            // Calculate the angle in degrees from the velocity vector
            float angle = std::atan2(boids[i].vy, boids[i].vx) * 180 / M_PI;
            boidShape.setRotation(angle);

            window.draw(boidShape);
        }

        window.display();
        sf::sleep(sf::milliseconds(10));
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulation Time: " << elapsed.count() << " seconds\n";

    return 0;
}
