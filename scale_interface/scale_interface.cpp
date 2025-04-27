#include "scale_interface.h"
#include <chrono>
#include <thread>

scale_interface::scale_interface()
    : _running(false), _scale_angle(0.0f) {}

scale_interface::~scale_interface() {
    stop();
}

float scale_interface::get_scale_angle() const {
    return _scale_angle.load(std::memory_order_relaxed);
}

void scale_interface::reset() {

}

void scale_interface::start() {
    if (_running) return;
    _running = true;
    _reader_thread = std::thread(&scale_interface::reader_loop, this);
}

void scale_interface::stop() {
    if (!_running) return;
    _running = false;
    if (_reader_thread.joinable()) {
        _reader_thread.join();
    }
}

void scale_interface::reader_loop() {
    const float angle_increment = 5.0f;  // Angle to increase each time
    const float final_angle = 30.0f;     // Final steady angle
    const int steps = 6;                 // Number of steps to reach 30 degrees
    const float step_duration = 2.0f;    // Duration for each step in seconds

    int step_count = 0;

    while (_running && step_count < steps) {
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Increase the angle by 5 degrees every 2 seconds
        float target_angle = (step_count + 1) * angle_increment;
        _scale_angle.store(target_angle, std::memory_order_relaxed);

        step_count++;
    }

    // After 6 steps, stay steady at 30 degrees
    _scale_angle.store(final_angle, std::memory_order_relaxed);

    _running = false;
}