#include "wust_base/serial_device.h"

size_t SerialDevice::write(uint8_t *buffer, size_t length)
{
    std::lock_guard<std::mutex> guard(serial_w_mux);
    size_t len;
    try
    {
        len = boost::asio::write(sp, boost::asio::buffer(buffer, length));
    }
    catch(const boost::system::system_error& e)
    {
        ROS_INFO("Write error :%s, retrying to open device...", e.what());
        openSerialDevice();
    }
    return len;
}

size_t SerialDevice::read(uint8_t *buffer, size_t length)
{
    std::lock_guard<std::mutex> guard(serial_r_mux);
    size_t len;
    try
    {
        len = boost::asio::read(sp, boost::asio::buffer(buffer, length));
    }
    catch(const boost::system::system_error& e)
    {
        ROS_INFO("Read error :%s, retrying to open device...", e.what());
        openSerialDevice();
    }
    return len;
}

void SerialDevice::flushSerialBuffer()
{
    std::lock_guard<std::mutex> guard(serial_r_mux);
    ::tcflush(sp.lowest_layer().native_handle(), TCIFLUSH);
}

void SerialDevice::openSerialDevice()
{
    boost::system::error_code ec;
    while (true)
    {
        sp.open(serial_device_name, ec);
        if (ec)
        {
            sp.close();
            ROS_INFO("Cannot open %s, error code: %d, retrying...", serial_device_name.c_str(), ec.value());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else
        {
            sp.set_option(boost::asio::serial_port::baud_rate(115200));
            sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
            sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
            sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
            sp.set_option(boost::asio::serial_port::character_size(8));
            flushSerialBuffer();
            ROS_INFO("Open %s successful!", serial_device_name.c_str());
            break;
        }
    }
}
