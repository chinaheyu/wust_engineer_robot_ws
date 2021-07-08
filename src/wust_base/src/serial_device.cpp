#include "wust_base/serial_device.h"

size_t SerialDevice::write(uint8_t *buffer, size_t length) {
    std::lock_guard<std::mutex> guard(serial_w_mux);
    return boost::asio::write(sp, boost::asio::buffer(buffer, length));
}

size_t SerialDevice::read(uint8_t *buffer, size_t length) {
    std::lock_guard<std::mutex> guard(serial_r_mux);
    return boost::asio::read(sp, boost::asio::buffer(buffer, length));
}

void SerialDevice::flushSerialBuffer() {
    std::lock_guard<std::mutex> guard(serial_r_mux);
    ::tcflush(sp.lowest_layer().native_handle(), TCIFLUSH);
}