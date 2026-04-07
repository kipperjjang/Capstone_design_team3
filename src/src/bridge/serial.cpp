#include "bridge/serial.hpp"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

Serial::Serial(const PortConfig &config) : config_(config) {
  // Initialize serial port
  if (!openSerialPort()) return;

  std::cout << "Serial port opened and configured: " << config_.name << " ("
            << config_.port << ")" << std::endl;
}

ssize_t Serial::readSerial(uint8_t *buffer, std::size_t size) {
  if (buffer == nullptr || size == 0) {
    return 0;
  }

  if (!isOpen()) {
    std::cerr << "Serial port is not open: " << config_.name << std::endl;
    return -1;
  }

  while (true) {
    const ssize_t bytes_read = ::read(serial_port_fd_, buffer, size);
    if (bytes_read >= 0) {
      return bytes_read;
    }

    if (errno == EINTR) {
      continue;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }

    std::cerr << "Failed to read serial port " << config_.name << ": "
              << std::strerror(errno) << std::endl;
    return -1;
  }
}

ssize_t Serial::writeSerial(const uint8_t *data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return 0;
  }

  if (!isOpen()) {
    std::cerr << "Serial port is not open: " << config_.name << std::endl;
    return -1;
  }

  std::size_t total_bytes_written = 0;
  while (total_bytes_written < size) {
    const ssize_t bytes_written =
      ::write(serial_port_fd_, data + total_bytes_written, size - total_bytes_written);
    if (bytes_written > 0) {
      total_bytes_written += static_cast<std::size_t>(bytes_written);
      continue;
    }

    if (bytes_written < 0 && errno == EINTR) {
      continue;
    }

    if (bytes_written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      usleep(1000);
      continue;
    }

    std::cerr << "Failed to write serial port " << config_.name << ": "
              << std::strerror(errno) << std::endl;
    return -1;
  }

  return static_cast<ssize_t>(total_bytes_written);
}

ssize_t Serial::writeSerial(const std::vector<uint8_t> &data) {
  if (data.empty()) {
    return 0;
  }

  return writeSerial(data.data(), data.size());
}

static bool configureDataBits(termios2 *uart_config, int data_bits) {
  if (uart_config == nullptr) {
    return false;
  }

  switch (data_bits) {
    case 5:
      uart_config->c_cflag |= CS5;
      return true;
    case 6:
      uart_config->c_cflag |= CS6;
      return true;
    case 7:
      uart_config->c_cflag |= CS7;
      return true;
    case 8:
      uart_config->c_cflag |= CS8;
      return true;
    default:
      return false;
  }
}

static bool configureParity(termios2 *uart_config, int parity) {
  if (uart_config == nullptr) {
    return false;
  }

  switch (parity) {
    case 0:
      return true;
    case 1:
      uart_config->c_cflag |= (PARENB | PARODD);
      uart_config->c_iflag |= INPCK;
      return true;
    case 2:
      uart_config->c_cflag |= PARENB;
      uart_config->c_iflag |= INPCK;
      return true;
    default:
      return false;
  }
}

bool Serial::openSerialPort() {
  serial_port_fd_ = open(config_.port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_port_fd_ < 0) {
    std::cerr << "Failed to open serial port " << config_.name << " at "
              << config_.port << ": " << std::strerror(errno) << std::endl;
    return false;
  }

  if (!configureSerialPort()) {
    closeSerialPort();
    return false;
  }

  return true;
}

bool Serial::configureSerialPort() {
  if (!isOpen()) {
    return false;
  }
  
  if (config_.baud <= 0) {
    std::cerr << "Invalid baud rate for " << config_.name << std::endl;
    return false;
  }

  if (config_.stop_bits != 1 && config_.stop_bits != 2) {
    std::cerr << "Invalid stop bit count for " << config_.name << std::endl;
    return false;
  }

  if (config_.parity < 0 || config_.parity > 2) {
    std::cerr << "Invalid parity configuration for " << config_.name << std::endl;
    return false;
  }

  termios2 uart_config{};
  if (ioctl(serial_port_fd_, TCGETS2, &uart_config) < 0) {
    std::cerr << "Could not read serial configuration for " << config_.name << ": "
              << std::strerror(errno) << std::endl;
    return false;
  }

  uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP |
                           IXON | IXOFF | IXANY);
  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  uart_config.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS | CBAUD);
  uart_config.c_cflag |= (CREAD | CLOCAL | BOTHER);

  if (!configureDataBits(&uart_config, config_.data_bits)) {
    std::cerr << "Invalid data bit count for " << config_.name << std::endl;
    return false;
  }

  if (!configureParity(&uart_config, config_.parity)) {
    std::cerr << "Invalid parity configuration for " << config_.name << std::endl;
    return false;
  }

  if (config_.stop_bits == 2) {
    uart_config.c_cflag |= CSTOPB;
  }

  uart_config.c_ispeed = static_cast<speed_t>(config_.baud);
  uart_config.c_ospeed = static_cast<speed_t>(config_.baud);
  uart_config.c_cc[VMIN] = 0;
  uart_config.c_cc[VTIME] = 1;

  if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
    std::cerr << "Could not set serial configuration for " << config_.name << ": "
              << std::strerror(errno) << std::endl;
    return false;
  }

  ioctl(serial_port_fd_, TCFLSH, TCIOFLUSH);
  return true;
}

void Serial::closeSerialPort() {
  if (!isOpen()) {
    return;
  }

  close(serial_port_fd_);
  serial_port_fd_ = -1;
}
