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

class SF30D {

#define I2C_SLAVE	0x0703

public:
    SF30D() {

    }

    ~SF30D() {
        this->disconnect();
    }

    void connect() {
        _fd = open (_device, O_RDWR | O_NOCTTY | O_SYNC);

        if (_fd == -1) {
            std::cerr << "USB device could not be opened" << std::endl;
        } else {
            std::cerr << "USB device opened on FD: " << _fd << std::endl;

            _threadRunning.store(true, std::memory_order_relaxed);
            _runningThread = std::thread(&LW20::loop, this);
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

    const char *_device = "/dev/ttyUSB0";
    int _fd;

    std::atomic<int> _latestDistance{0}; // in cm


    // Loop running in background thread.
    void loop() {
        while (_threadRunning.load(std::memory_order_relaxed))  {
            unsigned char byte[2];
            int res = read(_fd, byte, 2);

            if (res == -1) {
                std::cout << "USB Device " << _device << " was not available" << std::endl;
            } else {
                int distanceRead = (byte[0] << 8) | byte[1];
                std::cout << "[" << _device << "] Distance: " << distanceRead << "cm" << std::endl;

                _latestDistance.store(distanceRead, std::memory_order_relaxed);
            }

            usleep(250);
        }
    }
};

