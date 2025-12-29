#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <algorithm>

class scale_interface {
public:
    scale_interface();
    ~scale_interface() noexcept;

    // Non-copyable, non-movable
    scale_interface(const scale_interface&) = delete;
    scale_interface& operator=(const scale_interface&) = delete;
    scale_interface(scale_interface&&) = delete;
    scale_interface& operator=(scale_interface&&) = delete;

    float get_scale_angle() const;
    int get_last_weight_reading() const;
    int get_tare() const;
    bool is_ready() const;

    void reset();
    void start();
    void stop();

    //private:

    void trigger_tare();
    void empty_scale();

    void sync_servo();

    static constexpr const char* _serial_port = "/dev/ttyUSB0";

    struct SharedData {

        std::atomic<float>   servo_goal{ 0 };
        std::atomic<float>   servo_actual{ 28.0 };
        std::atomic<bool>    running{ false };
        std::atomic<float>   target_angle{ 60.0f };
        std::atomic<int>     tare{ 0 };
        std::atomic<bool>    is_resetting{ false };




        std::mutex           mtx;
        std::condition_variable cv;
    };

    std::shared_ptr<SharedData> _data;
    std::thread           _reader_thread;
    std::thread           _servo_thread;

};
