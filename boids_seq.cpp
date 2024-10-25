#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

// Constants for Boid behavior
const float TURN_FACTOR = 0.2f;
const float MAX_SPEED = 6.0f;
const float MIN_SPEED = 3.0f;
const float CENTERING_FACTOR = 0.0005f;
const float AVOID_FACTOR = 0.05f;
const float PROTECTED_RANGE = 8.0f;
const float MATCHING_FACTOR = 0.05f;
const float VISUAL_RANGE = 40.0f;


const int NUM_BOIDS = 100;


// Window dimensions
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 800;

// Margin to keep boids within window bounds
const int MARGIN = 100;

// Random value helper
float randomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Boid struct
struct Boid {
    float x, y;   // Position
    float vx, vy; // Velocity

    Boid(float px, float py) : x(px), y(py), vx(randomFloat(-1, 1)), vy(randomFloat(-1, 1)) {}

    Boid() : x(randomFloat(0, WINDOW_WIDTH)), y(randomFloat(0, WINDOW_HEIGHT)), vx(randomFloat(-1, 1)), vy(randomFloat(-1, 1)) {}

};

struct Parameters {
    float close_dx;
    float close_dy;
    float avg_vx;
    float avg_vy;
    float avg_x;
    float avg_y;
    float neighboring_count;

    Parameters(float dx, float dy, float vx, float vy, float x, float y, float nc) : close_dx(dx), close_dy(dy), avg_vx(vx), avg_vy(vy), avg_x(x), avg_y(y), neighboring_count(nc) {}

    Parameters() : close_dx(0), close_dy(0), avg_vx(0), avg_vy(0), avg_x(0), avg_y(0), neighboring_count(0) {}

};

Parameters getParameters(Boid& b, std::vector<Boid>& boids) {
    float close_dx = 0;
    float close_dy = 0;
    float avg_vx = 0;
    float avg_vy = 0;
    float avg_x = 0;
    float avg_y = 0;
    float neighboring_count = 0;


    for (Boid& other : boids) {
        if (&b == &other) continue;

        float dx = b.x - other.x;
        float dy = b.y - other.y;

        if(std::abs(dx)<VISUAL_RANGE && std::abs(dy)<VISUAL_RANGE){
            float squared_dist= dx*dx + dy*dy;

            if(squared_dist<PROTECTED_RANGE*PROTECTED_RANGE){
                close_dx += dx;
                close_dy += dy;
            }
            else if(squared_dist<VISUAL_RANGE*VISUAL_RANGE) {
                avg_x += other.x;
                avg_y += other.y;
                avg_vx += other.vx;
                avg_vy += other.vy;
                neighboring_count+=1;
            }
        }
    }


    Parameters parameters = Parameters(close_dx, close_dy, avg_vx, avg_vy, avg_x, avg_y, neighboring_count);

    return parameters;

}


int main() {
    //TODO: Implement logic to smartly calculate parameters (array of parameters?)
    Boid boids[NUM_BOIDS];
    Parameters parameters[NUM_BOIDS];

    // Initialize Boids
    for (int i = 0; i < 100; ++i) {
        boids[i] = Boid();
    }

    // Create SFML window
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Boids Simulation");

    // Benchmarking start
    auto start = std::chrono::high_resolution_clock::now();

    // Main loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Apply rules sequentially
        applyBoidRules(boids);

        // Update positions
        for (Boid& b : boids) {
            b.avoidZeroVelocity(); // Ensure no boid gets stuck
            b.update();
        }

        // Clear the window and draw the boids as dots
        window.clear();

        for (Boid& b : boids) {
            // Create a circle shape for each boid
            sf::CircleShape boidShape(4.0f);  // Set radius of the boid's dot (adjust size as needed)
            boidShape.setFillColor(sf::Color::White);
            boidShape.setPosition(b.x, b.y);
            window.draw(boidShape);
        }

        window.display();
    }

    // Benchmarking end
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulation Time: " << elapsed.count() << " seconds\n";

    return 0;
}

// Sequential function to apply boid rules
void applyBoidRules(std::vector<Boid>& boids) {
    for (int i = 0; i < boids.size(); ++i) {
        Boid& b = boids[i];

        // Call your rule implementations here
        auto separation = ruleSeparation(b, boids);
        auto alignment = ruleAlignment(b, boids);
        auto cohesion = ruleCohesion(b, boids);

        // Update velocity based on rules
        b.vx += separation.first + alignment.first + cohesion.first;
        b.vy += separation.second + alignment.second + cohesion.second;

        // Limit force (optional: ensure no boid moves too fast)
        float mag = std::sqrt(b.vx * b.vx + b.vy * b.vy);
        if (mag > MAX_FORCE) {
            b.vx = (b.vx / mag) * MAX_FORCE;
            b.vy = (b.vy / mag) * MAX_FORCE;
        }
    }
}

// Implement your rules here:

// Rule 1: Separation - Steer to avoid crowding local flockmates
std::pair<float, float> ruleSeparation(Boid& b, std::vector<Boid>& boids) {
    // Your separation logic goes here
    int close_dx = 0;
    int close_dy = 0;

    for (Boid& other : boids) {
        if (&b == &other) continue;

        float dx = b.x - other.x;
        float dy = b.y - other.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < PROTECTED_RANGE) {
            close_dx += dx;
            close_dy += dy;
        }
    }
    return {0, 0}; // Placeholder, update with your logic
}

// Rule 2: Alignment - Steer towards the average heading of local flockmates
std::pair<float, float> ruleAlignment(Boid& b, std::vector<Boid>& boids) {
    // Your alignment logic goes here
    return {0, 0}; // Placeholder, update with your logic
}

// Rule 3: Cohesion - Steer to move toward the average position of local flockmates
std::pair<float, float> ruleCohesion(Boid& b, std::vector<Boid>& boids) {
    // Your cohesion logic goes here
    return {0, 0}; // Placeholder, update with your logic
}
