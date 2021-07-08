#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

#include <boost/asio.hpp>
#include <string>
#include <mutex>


class SerialDevice {
private:
    boost::asio::io_service io_s;
    boost::asio::serial_port sp;
    std::mutex serial_r_mux;
    std::mutex serial_w_mux;
    void flushSerialBuffer();


public:
    explicit SerialDevice(const std::string &serial_name) : sp(io_s) {
        boost::system::error_code ec;
        sp.open(serial_name, ec);
        if (ec) {
            // TODO: Handle the open error here

        } else {

            sp.set_option(boost::asio::serial_port::baud_rate(115200));
            sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
            sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
            sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
            sp.set_option(boost::asio::serial_port::character_size(8));
            flushSerialBuffer();
        }
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
