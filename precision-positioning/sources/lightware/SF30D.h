//
// Created by LightWare.
// SF30/D Interface
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
            _runningThread = std::thread(&SF30D::loop, this);
        }
    }

    void disconnect() {
        _threadRunning.store(false, std::memory_order_relaxed);
        close(_fd);
    }

    float latestDistance() {
        return this->_latestDistance.load(std::memory_order_relaxed);
    }

private:
    std::atomic<bool> _threadRunning{false};
    std::thread _runningThread;

    const char *_device = "/dev/ttyACM0"; // To figure this out, plug the device on the Pi then immediately run the `dmesg` command to see which device was assigned.
    int _fd = -1;

    std::atomic<float> _latestDistance{0.0}; // in m

    int32_t readData(uint8_t *Buffer, int32_t BufferSize) {
        if (_fd < 0) {
            std::cout << "Can't read from null coms";
            return -1;
        }

        errno = 0;
        int readBytes = read(_fd, Buffer, BufferSize);

        return readBytes;
    }

    float getNextReading() {
        char line[64];
        int lineSize = 0;

        while (true) {
            char recvData;
            if (this->readData((uint8_t*)&recvData, 1) == 1) {
                if (recvData == '\n') {
                    line[lineSize] = 0;
                    float distance = atof(line);
                    return distance;
                } else if (isdigit(recvData) || recvData == '.') {
                    line[lineSize++] = recvData;

                    if (lineSize == sizeof line) {
                        lineSize = 0;
                    }
                }
            }
        }
    }

    // Loop running in background thread.
    void loop() {
        while (_threadRunning.load(std::memory_order_relaxed))  {
            float distanceRead = this->getNextReading();
            std::cout << "[" << _device << "] Distance: " << distanceRead << " m" << std::endl;

            _latestDistance.store(distanceRead, std::memory_order_relaxed);
        }
    }
};

