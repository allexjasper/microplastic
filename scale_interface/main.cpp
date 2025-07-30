// scale_interface.cpp : Defines the entry point for the application.
//

#include "scale_interface.h"

#include "scale_interface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <csignal>

#include <boost/asio.hpp>
#include <iostream>
#include <string>

namespace asio = boost::asio;
using asio::steady_timer;
using asio::serial_port;
using namespace std::chrono_literals;

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <memory>

namespace asio = boost::asio;
using asio::serial_port;
using asio::steady_timer;
using namespace std::chrono_literals;

class SerialAsyncHandler {
public:
    SerialAsyncHandler(asio::io_context& io, const std::string& port_name, unsigned int baud_rate)
        : serial_(io, port_name),
        timer_(io),
        read_buffer_(),
        motor_state_(false) {
        serial_.set_option(serial_port::baud_rate(baud_rate));
        start_read();
        start_motor_cycle();
    }

private:
    serial_port serial_;
    steady_timer timer_;
    asio::streambuf read_buffer_;
    bool motor_state_;

    void start_read() {
        asio::async_read_until(serial_, read_buffer_, '\n',
            [this](const boost::system::error_code& ec, std::size_t /*bytes*/) {
                if (!ec) {
                    std::istream is(&read_buffer_);
                    std::string line;
                    std::getline(is, line);
                    std::cout << "Received: " << line << std::endl;
                    start_read();  // continue reading
                }
                else {
                    std::cerr << "Read error: " << ec.message() << std::endl;
                }
            });
    }

    void start_motor_cycle() {
        timer_.expires_after(2s);
        timer_.async_wait([this](const boost::system::error_code& ec) {
            if (ec) {
                std::cerr << "Timer error: " << ec.message() << std::endl;
                return;
            }

            auto command = std::make_shared<std::string>(motor_state_ ? "m,0,257\n" : "m,1,257\n");

            asio::async_write(serial_, asio::buffer(*command),
                [this, command](const boost::system::error_code& ec, std::size_t /*bytes*/) {
                    if (ec) {
                        std::cerr << "Write error: " << ec.message() << std::endl;
                    }
                    // command remains alive through shared_ptr until this lambda ends
                });

            motor_state_ = !motor_state_;
            start_motor_cycle();  // reschedule next toggle
            });
    }
};


int main() {
    scale_interface scale;
    scale.start();


    try {
        asio::io_context io;
        SerialAsyncHandler handler(io, "COM6", 115200);
        io.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    }


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
