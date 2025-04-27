// scale_interface.cpp : Defines the entry point for the application.
//

#include "scale_interface.h"

#include "scale_interface.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    scale_interface scale;
    scale.start();

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        float elapsed_sec = std::chrono::duration<float>(now - start_time).count();

        float angle = scale.get_scale_angle();
        std::cout << "Time: " << elapsed_sec << "s, Angle: " << angle << " degrees" << std::endl;

        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    scale.stop();

    std::cout << "Test completed!" << std::endl;
    return 0;
}
