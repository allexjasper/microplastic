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
#include <boost/regex.hpp>

#include <boost/asio.hpp>
#include <iostream>
#include <string>


#include "dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "COM5"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
//#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
//#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
//#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


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

std::atomic<int> goal_pos;


int mapWeightToServo(int weight3)
{
    // Input range
    const int in_min = -250000;
    const int in_max = -400000;  // smaller number
    // Output range
    const int out_min = 50;
    const int out_max = 550;

    // Ensure weight3 stays inside input range
    if (weight3 > in_min) weight3 = in_min;
    if (weight3 < in_max) weight3 = in_max;

    // Linear interpolation
    double scale = double(out_max - out_min) / double(in_max - in_min);
    return int(out_min + (weight3 - in_min) * scale);
}


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

                    boost::regex re(
                        R"(hx,(-?\d+),(-?\d+),(-?\d+),(-?\d+),fwd:(-?\d+),bkw:(-?\d+).*)"
                    );

                    boost::smatch match;
                    if (boost::regex_match(line, match, re)) {
                        int weight1 = std::stoi(match[1]);
                        int weight2 = std::stoi(match[2]);
                        int weight3 = std::stoi(match[3]);
                        int weight4 = std::stoi(match[4]);
                        int ticks_forward = std::stoi(match[5]);
                        int ticks_backward = std::stoi(match[6]);

                        std::cout << "w1=" << weight1
                            << " w2=" << weight2
                            << " w3=" << weight3
                            << " w4=" << weight4
                            << " fwd=" << ticks_forward
                            << " bkw=" << ticks_backward
                            << std::endl;

                        goal_pos.store(mapWeightToServo(weight3));
                    }
                    else
                    {
                        std::cout << "Received: " << line << std::endl;
                    }

         
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


std::atomic<bool> running(true);


void servo_thread()
{
    // Initialize PortHandler instance
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    int32_t dxl_present_position = 0;               // Present position

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        return;
    }

    // Enable Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
    {
        printf("Failed to enable torque\n");
        return;
    }
    else
    {
        printf("Dynamixel has been successfully connected\n");
    }

    while (running)
    {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, goal_pos.load(), &dxl_error);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION,
            (uint32_t*)&dxl_present_position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            printf("read fail\n");
        else
            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, 50, dxl_present_position);
    }


    /*
    while (running)
    {
        // Move to position 50
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, 50, &dxl_error);

        while (running && std::abs(dxl_present_position - 50) > 3)
        {
            dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION,
                (uint32_t*)&dxl_present_position, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
                printf("read fail\n");
            else
                printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, 50, dxl_present_position);
        }

        // Move to position 540
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, 540, &dxl_error);

        while (running && std::abs(dxl_present_position - 540) > 3)
        {
            dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION,
                (uint32_t*)&dxl_present_position, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
                printf("read fail\n");
            else
                printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, 540, dxl_present_position);
        }
    }
    */

    // Cleanup
    portHandler->closePort();
}




int main() {
    scale_interface scale;
    scale.start();

    goal_pos.store(50);


    std::thread t(servo_thread);


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
