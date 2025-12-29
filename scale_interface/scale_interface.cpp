


#pragma warning(push)

// 2. Disable the specific warning C4668
#pragma warning(disable: 4668)
#pragma warning(disable: 4800)
#pragma warning(disable: 4251)


#include "scale_interface.h"
#include <random>
#include <chrono>
#include <thread>

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
#include <boost/asio.hpp>
#include <boost/regex.hpp>
//#include <boost/circular_buffer.hpp> // Requires Boost
#include <thread>
#include <iostream>
#include <string>
#include <atomic>
#include <chrono>
#include <vector>
#include <algorithm> // for nth_element
#include <mutex>     // for std::mutex

#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <thread>
#include <iostream>
#include <string>
#include <atomic>
#include <chrono>

#include "dynamixel_sdk.h"     

#pragma warning(pop)

int maxWeight = 90000 * 4;
float maxAngle = 65.0;


// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY       112

constexpr uint16_t ADDR_PRO_MAX_POS_LIMIT = 48; // 4 Bytes
constexpr uint16_t ADDR_PRO_MIN_POS_LIMIT = 52; // 4 Bytes

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "COM4"      // Check which port is being used on your controller
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



namespace asio = boost::asio;
using namespace std::chrono_literals;



namespace asio = boost::asio;
using namespace std::chrono_literals;

#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <thread>
#include <iostream>
#include <string>
#include <atomic>
#include <chrono>
#include <vector>
#include <algorithm> // for nth_element
#include <mutex>     
#include <deque>     // Replaces circular_buffer

namespace asio = boost::asio;
using namespace std::chrono_literals;

class SerialAsyncHandler {
public:
    SerialAsyncHandler(const std::string& port_name, unsigned int baud_rate)
        : motor_direction_(false),
        motor_speed_(false),
        io_(),
        serial_(io_, port_name),
        timer_(io_),
        read_buffer_()

        // history_buffer_ initializes empty by default
    {
        serial_.set_option(asio::serial_port::baud_rate(baud_rate));
    }

    ~SerialAsyncHandler() {
        stop();
    }

    void start() {
        if (io_thread_.joinable()) return;

        io_.restart();

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            history_buffer_.clear();
        }

        start_read();
        start_motor_cycle();

        io_thread_ = std::thread([this]() {
            io_.run();
            });
    }

    void stop() {
        io_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

    std::atomic<int> last_reading{ 0 };

    int compute_tare() {
        std::vector<int> temp_view;

        // 1. Copy data under lock
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            if (history_buffer_.empty()) {
                return 0;
            }
            // Copy deque to vector for sorting
            // We copy because we cannot rearrange the live history buffer 
            // (it must remain in chronological order for the sliding window).
            temp_view.assign(history_buffer_.begin(), history_buffer_.end());
        }

        // 2. Find Median
        size_t n = temp_view.size() / 2;
        std::nth_element(temp_view.begin(), temp_view.begin() + n, temp_view.end());

        return temp_view[n];
    }


    std::atomic<bool> motor_direction_;
    std::atomic<bool> motor_speed_;
private:
    asio::io_context io_;
    asio::serial_port serial_;
    asio::steady_timer timer_;
    asio::streambuf read_buffer_;

    std::thread io_thread_;

    std::mutex buffer_mutex_;
    std::deque<int> history_buffer_; // Changed to std::deque
    const size_t max_history_size_ = 20;

    void start_read() {
        asio::async_read_until(serial_, read_buffer_, '\n',
            [this](const boost::system::error_code& ec, std::size_t /*bytes*/) {
                if (!ec) {
                    std::istream is(&read_buffer_);
                    std::string line;
                    std::getline(is, line);

                    boost::regex re(R"(hx,(-?\d+),(-?\d+),(-?\d+),(-?\d+).*)");
                    boost::smatch match;

                    if (boost::regex_search(line, match, re)) {
                        int total = std::stoi(match[1]) + std::stoi(match[2]) +
                            std::stoi(match[3]) + std::stoi(match[4]);

                        last_reading.store(total);

                        {
                            std::lock_guard<std::mutex> lock(buffer_mutex_);

                            // Add new reading
                            history_buffer_.push_back(total);

                            // Enforce sliding window size manually
                            if (history_buffer_.size() > max_history_size_) {
                                history_buffer_.pop_front();
                            }
                        }

                        /*
                        float angle = 0;


                        if()
                        int weight = std::max(get_last_weight_reading() - get_tare(), 0);

                        angle = ((float)weight / (float)maxWeight) * maxAngle;
                        */


                    }
                    start_read();
                }
                else if (ec != asio::error::operation_aborted) {
                    std::cerr << "Read error: " << ec.message() << std::endl;
                }
            });
    }

    // controll the motion of the linear motor
    void start_motor_cycle() {
        timer_.expires_after(100ms);
        timer_.async_wait([this](const boost::system::error_code& ec) {
            if (ec == asio::error::operation_aborted) return;


            std::string speedStr = motor_speed_.load() ? "257" : "0";
            std::string dirStr = motor_direction_.load() ? "0" : "1";

            auto command = std::make_shared<std::string>("m," + dirStr + ", " + speedStr + "\n");
            asio::async_write(serial_, asio::buffer(*command),
                [this, command](const boost::system::error_code& ec, std::size_t) {
                    if (ec && ec != asio::error::operation_aborted) std::cerr << "Write error\n";
                });

            start_motor_cycle();
            });
    }
};

std::atomic<bool> running(true);




SerialAsyncHandler* g_serialAsyncHandler = NULL;


scale_interface::scale_interface()
    : _data(std::make_shared<SharedData>())
{
    try
    {
        g_serialAsyncHandler = new SerialAsyncHandler("COM3", 115200);
    }
    catch (...)
    {

    }
}


void scale_interface::empty_scale()
{
    _data->is_resetting.store(true);
    _data->tare.store(0);
    _data->servo_goal.store(0.0);
    std::thread t([this]() {
        if (g_serialAsyncHandler)
        {
            // up
            g_serialAsyncHandler->motor_direction_.store(true);
            g_serialAsyncHandler->motor_speed_.store(true);
            std::this_thread::sleep_for(std::chrono::seconds(10));

            //down
            g_serialAsyncHandler->motor_direction_.store(false);
            g_serialAsyncHandler->motor_speed_.store(true);
            std::this_thread::sleep_for(std::chrono::seconds(10));

            //stop
            g_serialAsyncHandler->motor_direction_.store(false);
            g_serialAsyncHandler->motor_speed_.store(false);

            _data->is_resetting.store(false);
            // finally trage
            trigger_tare();
        }
        });

    t.detach();
}

void scale_interface::trigger_tare()
{
    std::thread t([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (g_serialAsyncHandler)
            _data->tare.store(g_serialAsyncHandler->compute_tare());
        });

    t.detach();
}

scale_interface::~scale_interface() noexcept {
    stop();

}



float scale_interface::get_scale_angle() const {

    float angle = _data->servo_actual.load();
    return angle;
}

int scale_interface::get_last_weight_reading() const
{
    return g_serialAsyncHandler->last_reading.load();
}

bool scale_interface::is_ready() const
{
    return !_data->is_resetting.load() && _data->tare != 0;
}

int scale_interface::get_tare() const
{
    return _data->tare.load();
}

void scale_interface::reset() {
    empty_scale();
    //stop();
    //_data->scale_angle.store(0.0f, std::memory_order_relaxed);
    //_data->target_angle.store(60.0f, std::memory_order_relaxed);    
}

void scale_interface::start() {

    bool expected = false;
    if (!_data->running.compare_exchange_strong(expected, true)) {
        return;
    }

    if (g_serialAsyncHandler)
        g_serialAsyncHandler->start();

    empty_scale();
    sync_servo();

}

void scale_interface::stop() {

    _data->running.store(false);


    {
        std::lock_guard<std::mutex> lock(_data->mtx);
        _data->cv.notify_one();
    }


    if (_reader_thread.joinable()) {
        if (std::this_thread::get_id() != _reader_thread.get_id()) {
            _reader_thread.join();
        }
        else {
            _reader_thread.detach();
        }
        _reader_thread = std::thread();
    }
    if (_servo_thread.joinable())
        _servo_thread.join();
}

#include <cmath>    // For std::round
#include <cstdint>  // For int32_t

// Constants
constexpr float DXL_RESOLUTION = 4096.0f;
constexpr float DXL_MAX_ANGLE = 360.0f;

// Convert Degrees (float) to Servo Position (int)
int32_t degreesToPosition(float degrees) {
    float position = (degrees / DXL_MAX_ANGLE) * DXL_RESOLUTION;
    return static_cast<int32_t>(std::round(position));
}

// Convert Servo Position (int) to Degrees (float)
float positionToDegrees(int32_t position) {
    return (static_cast<float>(position) / DXL_RESOLUTION) * DXL_MAX_ANGLE;
}

void scale_interface::sync_servo()
{

    //_data->servo_actual = 25.0;
    //_data->servo_actual.store(25.0, std::memory_order_relaxed);

    auto l([this]()
        {
            //std::this_thread::sleep_for(std::chrono::seconds(5));
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


            // 1. Convert angles to integer steps (0 - 4095)
            int32_t minPos = degreesToPosition(0);
            int32_t maxPos = degreesToPosition(maxAngle + 5.0);

            // Enable Torque
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);


            // 3. Write Minimum Position Limit
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, DXL_ID, ADDR_PRO_MIN_POS_LIMIT, (uint32_t)minPos, &dxl_error
            );
            if (dxl_comm_result != COMM_SUCCESS) {
                std::cout << "Failed to set Min Limit!" << std::endl;
            }

            // 4. Write Maximum Position Limit
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, DXL_ID, ADDR_PRO_MAX_POS_LIMIT, (uint32_t)maxPos, &dxl_error
            );
            if (dxl_comm_result != COMM_SUCCESS) {
                std::cout << "Failed to set Max Limit!" << std::endl;
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

            // Set Profile Velocity to 1 (Lowest possible setting: ~0.229 rev/min)
            // Note: Setting this to 0 would use the Maximum speed.
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, DXL_ID, ADDR_PRO_PROFILE_VELOCITY, 7, &dxl_error);

            // Check for communication errors (optional but recommended)
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0) {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            while (_data->running)
            {
                float goalAngle = 0;


                if (is_ready() && get_tare() != 0)
                {
                    int weight = std::max(get_last_weight_reading() - get_tare(), 0);
                    goalAngle = std::min(((float)weight / (float)maxWeight) * maxAngle, maxAngle);

                    std::cout << "goal angle: " << goalAngle << std::endl;

                }
                _data->servo_goal.store(goalAngle);


                int goalPos = degreesToPosition(_data->servo_goal.load());

                dxl_comm_result = packetHandler->write4ByteTxRx(
                    portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, goalPos, &dxl_error);

                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                dxl_comm_result = packetHandler->read4ByteTxRx(
                    portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION,
                    (uint32_t*)&dxl_present_position, &dxl_error);


                float actualPos = positionToDegrees(dxl_present_position);
                _data->servo_actual.store(actualPos);


                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
                    printf("read fail\n");
                else
                {
                    std::cout << "goal: " << goalPos << "actual: " << dxl_present_position << std::endl;
                    _data->servo_actual.store(actualPos);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }



            // Cleanup
            portHandler->closePort();
        });

    _servo_thread = std::thread(l);
}

