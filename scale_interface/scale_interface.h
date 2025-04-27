#pragma once

#include <thread>
#include <atomic>

class scale_interface {
public:
    scale_interface();
    ~scale_interface();

    float get_scale_angle() const;
    void reset();
    void start();
    void stop();

private:
    void reader_loop();

    static constexpr const char* _serial_port = "/dev/ttyUSB0";

    std::atomic<bool> _running;
    std::thread _reader_thread;
    std::atomic<float> _scale_angle;
};
