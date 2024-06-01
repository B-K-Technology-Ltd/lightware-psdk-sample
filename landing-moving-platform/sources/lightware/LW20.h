//
// Created by LightWare.
// LW20/c Interface
//

#pragma once

#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <atomic>
#include <thread>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#include <fcntl.h>
#include <string.h>

class LW20 {

#define I2C_SLAVE	0x0703

public:
    LW20() {

    }

    ~LW20() {
        this->disconnect();
    }

    void connect(int i2cAddress) {
        _deviceAddress = i2cAddress;

        const char *device = "/dev/i2c-1"; // This may change, wiringPi has a good discovery for this, but it's not available yet on pi5.
        _fd = open (device, O_RDWR);

        if (_fd == -1) {
            std::cerr << "I2C Bus file could not be opened" << std::endl;
        } else {
            std::cerr << "I2C Bus opened on FD: " << _fd << std::endl;

            if (ioctl (_fd, I2C_SLAVE, i2cAddress) < 0) {
                std::cerr << "Unable to select I2C device: " << strerror(errno) << std::endl;
            } else {
                _threadRunning.store(true, std::memory_order_relaxed);
                _runningThread = std::thread(&LW20::loop, this);
            }
        }
    }

    void disconnect() {
        _threadRunning.store(false, std::memory_order_relaxed);
        close(_fd);
    }

    int latestDistance() {
        return this->_latestDistance.load(std::memory_order_relaxed);
    }

private:
    std::atomic<bool> _threadRunning{false};
    std::thread _runningThread;

    int _deviceAddress = 0x66; // default factory value
    int _fd;

    std::atomic<int> _latestDistance{0}; // in cm

    // Loop running in background thread.
    void loop() {
        while (_threadRunning.load(std::memory_order_relaxed))  {
            unsigned char byte[2];
            int res = read(_fd, byte, 2);

            if (res == -1) {
                std::cout << "I2C Device with address " << _deviceAddress << " was not available" << std::endl;
            } else {
                int distanceRead = (byte[0] << 8) | byte[1];
                std::cout << "[" << _deviceAddress << "] Distance: " << distanceRead << "cm" << std::endl;

                _latestDistance.store(distanceRead, std::memory_order_relaxed);
            }

            usleep(250);
        }
    }
};

