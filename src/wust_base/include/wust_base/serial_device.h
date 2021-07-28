#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

#include <boost/asio.hpp>
#include <string>
#include <mutex>
#include <ros/ros.h>

#include <thread>
#include <chrono>

class SerialDevice {
private:
    std::string serial_device_name;
    boost::asio::io_service io_s;
    boost::asio::serial_port sp;
    std::mutex serial_r_mux;
    std::mutex serial_w_mux;
    void flushSerialBuffer();
    void openSerialDevice();


public:
    explicit SerialDevice(const std::string &serial_name) : sp(io_s), serial_device_name(serial_name) {
        openSerialDevice();
    }

    /// Write some data.
    /// \param buffer
    /// \param length
    /// \return The number of bytes transferred.
    size_t write(uint8_t *buffer, size_t length);

    /// Read some data.
    /// \param buffer
    /// \param length
    /// \return The number of bytes transferred.
    size_t read(uint8_t *buffer, size_t length);
};


#endif //SERIAL_DEVICE_H
