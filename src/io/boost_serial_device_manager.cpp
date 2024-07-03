#include "afoo/io/boost_serial_device_manager.hpp"

#include "afoo/io/boost_serial_device.hpp"
#include "afoo/io/status.hpp"

#include <map>
#include <mutex>
#include <string>
#include <tuple>

namespace afoo::io {

Status BoostSerialDeviceManager::initializeDevice(const std::string& port) {
  std::lock_guard<std::mutex> lock(deviceMutex_);
  if (deviceMap_.contains(port) && deviceMap_[port]->isInitialized()) {
    RCLCPP_DEBUG(logger_, "Initializing device.");
    return deviceMap_[port]->initialize();
  }
  RCLCPP_DEBUG(logger_, "Device already initialized or not created.");
  return {STATUS::SUCCESS};  // Already initialized or device not created
}

Status BoostSerialDeviceManager::openDevice(const std::string& port) {
  std::lock_guard<std::mutex> lock(deviceMutex_);
  if (deviceMap_.contains(port) && deviceMap_[port]->isOpened()) {
    RCLCPP_DEBUG(logger_, "Opening device.");
    return deviceMap_[port]->open();
  }
  RCLCPP_WARN(logger_, "Device already open or not created.");
  return {STATUS::ERROR, ERROR::NONE};  // Device already open or not created
}

Status BoostSerialDeviceManager::closeDevice(const std::string& port) {
  std::lock_guard<std::mutex> lock(deviceMutex_);
  if (deviceMap_.contains(port) && deviceMap_[port]->isOpened()) {
    RCLCPP_DEBUG(logger_, "Closing device.");
    return deviceMap_[port]->close();
  }
  RCLCPP_DEBUG(logger_, "Device was not open or not created.");
  return {STATUS::SUCCESS};  // Device was not open or not created
}

std::tuple<Status, std::optional<std::string>>
BoostSerialDeviceManager::readFromDevice(const std::string& port,
                                         const std::string& request) {
  std::lock_guard<std::mutex> lock(deviceMutex_);
  if (!deviceMap_.contains(port)) {
    RCLCPP_ERROR(logger_, "Device not created, cannot read.");
    return {Status(STATUS::ERROR, ERROR::COMMUNICATION_ERROR), {}};
  }
  RCLCPP_DEBUG(logger_, "Reading from device: %s", request.c_str());
  return deviceMap_[port]->read(request);
}

Status BoostSerialDeviceManager::createDevice(const std::string& port,
                                              uint32_t baudRate) {
  std::lock_guard<std::mutex> lock(deviceMutex_);
  RCLCPP_DEBUG(logger_, "BoostSerialDeviceManager::createDevice ");
  if (!deviceMap_.contains(port)) {
    RCLCPP_DEBUG(logger_, "Creating device at port: with baud rate: %s",
                 port.c_str());
    try {
      deviceMap_[port] = std::make_unique<BoostSerialDevice>(port, baudRate);
      auto status = deviceMap_[port]->open();
      if (!status.isSuccess()) {
        RCLCPP_ERROR(logger_, "Failed to open newly created device.");
        return status;
      }
      RCLCPP_DEBUG(logger_, "Device created and opened successfully.");
      return {STATUS::SUCCESS};
    } catch (const boost::system::system_error& e) {
      const boost::system::error_code& ec = e.code();
      if (ec == boost::asio::error::already_open) {
        RCLCPP_WARN(logger_,
                    "Device already open, attempting to close and reopen.");
        deviceMap_[port]->close();        // Try to close it first
        return deviceMap_[port]->open();  // Attempt to reopen
      } else if (ec == boost::asio::error::no_such_device ||
                 ec.value() == ENOENT) {
        std::string errorMessage = "Please make sure the device " + port +
                                   " is correct and accessible.";
        RCLCPP_ERROR(logger_, "No such device: %s", port.c_str());
        return {STATUS::ERROR, ERROR::ERROR_OPENING_DEVICE, errorMessage};
      } else {
        RCLCPP_ERROR(logger_, "Communication error while creating device: %s",
                     e.what());
        return {STATUS::ERROR, ERROR::COMMUNICATION_ERROR,
                std::string(e.what())};
      }
    }
  }
  return {STATUS::SUCCESS};  // Device already created
}

Status BoostSerialDeviceManager::writeToDevice(const std::string& port,
                                               std::string&& data) {
  std::lock_guard<std::mutex> lock(deviceMutex_);
  if (!deviceMap_.contains(port)) {
    RCLCPP_ERROR(logger_, "Device not created, cannot write data.");
    return {STATUS::ERROR, ERROR::SENSOR_FAILURE};
  }

  RCLCPP_DEBUG(logger_, "Writing to device: %s", toHexString(data).c_str());
  return deviceMap_[port]->write(data);
}

}  // namespace afoo::io
