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
//#include <asm/ioctl.h>

#include <fcntl.h>
#include <string.h>

#include "lwNx.h"

class SF30D: lwSerialPort {

public:

    SF30D() { }

    ~SF30D() {
        this->disconnect();
    }

private:
    std::atomic<bool> _initiated{false};
    std::atomic<bool> _useUSB{false};
    std::atomic<bool> _useSerial{false};

    const char *_device = 0;
    int _fd = -1;
    int _bitRate = 0;

public:
    void initForUSB(const char *device) {
        this->_initiated.store(true, std::memory_order_relaxed);
        this->_useUSB.store(true, std::memory_order_relaxed);
        this->_useSerial.store(false, std::memory_order_relaxed);

        this->_device = device;
        std::cout << "Initialized for USB connection to " << device << std::endl;
    }

    void initForSerial(int bitRate, const char *device) {
        this->_initiated.store(true, std::memory_order_relaxed);
        this->_useUSB.store(false, std::memory_order_relaxed);
        this->_useSerial.store(true, std::memory_order_relaxed);

        this->_device = device;
        this->_bitRate = bitRate;
        std::cout << "Initialized for Serial connection to " << device << " at " << bitRate << "bauds" << std::endl;
    }

private:
    void _connectUSB() {

        if (_initiated == false || _useUSB == false) {
            std::cerr << "Please init for USB before you try to connect to USB" << std::endl;
            return;
        }

        _fd = open(_device, O_RDWR | O_NOCTTY | O_SYNC );

        if (_fd == -1) {
            std::cerr << "USB device could not be opened" << std::endl;
        } else {
            std::cout << "USB device opened on FD: " << _fd << std::endl;

            _threadRunning.store(true, std::memory_order_relaxed);
            _runningThread = std::thread(&SF30D::loop, this);
        }
    }

    int32_t _convertBaudRate(int32_t BitRate) {
        switch (BitRate) {
            case 115200: { return B115200; }
            case 230400: { return B230400; }
            case 460800: { return B460800; }
            case 500000: { return B500000; }
            case 576000: { return B576000; }
            case 921600: { return B921600; }
            default:
                return B115200;
        }
    }

    void _connectSerial() {

        if (_initiated == false || _useSerial == false) {
            std::cerr << "Please init for USB before you try to connect to USB" << std::endl;
            return;
        }

        _fd = open((const char *) _device, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);

        if (_fd == -1) {
            std::cerr << "Serial device could not be opened" << std::endl;
        } else {
            std::cerr << "Serial device opened on FD: " << _fd << std::endl;

            struct termios tty;
            memset(&tty, 0, sizeof(tty));
            if (tcgetattr(_fd, &tty) != 0) {
                std::cerr << "Error from tcgetattr" << std::endl;
                return;
            }

            int32_t BitRate = _convertBaudRate(_bitRate);

            cfsetospeed(&tty, BitRate);
            cfsetispeed(&tty, BitRate);

            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag |= 0;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_iflag &= ~IGNBRK;
            tty.c_iflag &= ~ICRNL;
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 1;

            if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
                std::cerr << "Error from tcsetattr" << std::endl;
                return;
            }

            std::cout << "Serial device connected and configured" << std::endl;

            // Handshake activates the device
            this->performHandShake();

            return;
        }
    }

public:
    void connect() {
        if (this->_useUSB) {
            this->_connectUSB();
        } else if (this->_useSerial) {
            this->_connectSerial();
        } else {
            std::cerr << "Wrong initiation state, please init this object before connecting";
        }
    }

    void disconnect() {
        _threadRunning.store(false, std::memory_order_relaxed);
        if (_fd >= 0) {
            close(_fd);
        }
        _fd = -1;
    }

    float latestDistance() {
        return this->_latestDistance.load(std::memory_order_relaxed);
    }

private:
    std::atomic<bool> _threadRunning{false};
    std::thread _runningThread;

    int writeData(uint8_t *Buffer, int32_t BufferSize) {
        if (_fd < 0) {
            std::cout << "Can't write to null coms" << std::endl;
            return -1;
        }

        int writtenBytes = write(_fd, Buffer, BufferSize);

        if (writtenBytes != BufferSize)
        {
            // std::cout << "Could not send all bytes!" << std::endl;
            return -1;
        }

        //	std::cout << "Sent " << BufferSize << " bytes" << std::endl
        return writtenBytes;
    }

    int32_t readData(uint8_t *Buffer, int32_t BufferSize) {
        if (_fd < 0) {
            std::cout << "Can't read from null coms" << std::endl;
            return -1;
        }

        errno = 0;
        int readBytes = read(_fd, Buffer, BufferSize);

        return readBytes;
    }



    // Data received from the sensor.

    float _distance = 0.0; // in m
    unsigned int _strength = 0; // in percentage
    unsigned int _temperature = 0; // in degrees

    std::atomic<float> _latestDistance{0.0}; // in m


    void _getNextUSBReading() {
        char line[64];
        int lineSize = 0;

        while (true) {
            char recvData;
            if (this->readData((uint8_t*)&recvData, 1) == 1) {
                if (recvData == '\n') {
                    line[lineSize] = 0;
                    this->_distance = atof(line);
                    return;
                } else if (isdigit(recvData) || recvData == '.') {
                    line[lineSize++] = recvData;

                    if (lineSize == sizeof line) {
                        lineSize = 0;
                    }
                }
            }
        }
    }

    uint16_t readInt16(uint8_t *Buffer, uint32_t Offset) {
        uint16_t result;
        result = (Buffer[Offset + 0] << 0) | (Buffer[Offset + 1] << 8);
        return result;
    }

    void _getNextSerialReading() {

        while (true) {
            lwResponsePacket response;

            if (lwnxRecvPacket(this, 44, &response, 1000)) {
                uint16_t firstReturnRaw = readInt16(response.data, 4);
                uint16_t firstReturnFiltered = readInt16(response.data, 6);
                uint16_t firstReturnStrength = readInt16(response.data, 8);

                uint16_t lastReturnRaw = readInt16(response.data, 10);
                uint16_t lastReturnFiltered = readInt16(response.data, 12);
                uint16_t lastReturnStrength = readInt16(response.data, 14);

                uint16_t backgroundNoise = readInt16(response.data, 16);
                uint16_t temperature = readInt16(response.data, 18) / 100;

                _distance = static_cast<float>(firstReturnFiltered)/100;
                _strength = firstReturnStrength;
                _temperature = temperature;
                return;
            }
        }
    }

    void getNextReading() {
        if (this->_useUSB) {
            this->_getNextUSBReading();
        } else if (this->_useSerial) {
            this->_getNextSerialReading();
        } else {
            std::cerr << "Wrong initiation state, please init this object before connecting";
        }
    }


    // Loop running in background thread.
    void loop() {
        while (_threadRunning.load(std::memory_order_relaxed))  {
            this->getNextReading();
            std::cout << "[" << _device << "] Distance: " << _distance << " m" << std::endl;

            _latestDistance.store(_distance, std::memory_order_relaxed);
        }
    }


public:
    // Serial Commands

    std::string getModelName() {
        if (!_initiated || !_useSerial) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return "";
        }

        char modelName[16]{};
        //        if (!lwnxCmdReadString(this, LWNXProductNameCmdID, modelName)) {
        //            std::cerr << "No Response to command" << std::endl;
        //	    return "";
        //        }

        while(lwnxCmdReadString(this, LWNXProductNameCmdID, modelName) == false) {
            usleep(50);
        }


        printf("Model: %.16s\n", modelName);
        return std::string(modelName);
    }

    uint32_t getHardwareVersion() {
        if (_initiated == false || _useSerial == false) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return 0;
        }

        // Read the hardware version. (Command 1: Hardware version)
        uint32_t hardwareVersion = 0;
        // if (!lwnxCmdReadUInt32(this, 1, &hardwareVersion)) {
        //     std::cerr << "No Response to command" << std::endl;
        //     return 0;
        // }

        while (lwnxCmdReadUInt32(this, 1, &hardwareVersion) == false) {
            usleep(50);
        }


        printf("Hardware: %d\n", hardwareVersion);
        return hardwareVersion;
    }

    uint32_t  getFirmwareVersion(){
        if (_initiated == false || _useSerial == false) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return 0;
        }

        // Read the firmware version. (Command 2: Firmware version)
        uint32_t firmwareVersion = 0;
        // if (!lwnxCmdReadUInt32(this, 2, &firmwareVersion)) {
        //     std::cerr << "No Response to command" << std::endl;
        //     return 0;
        // }
        while (lwnxCmdReadUInt32(this, 2, &firmwareVersion) == false) {
            usleep(50);
        }

        char firmwareVersionStr[16];
        lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);
        printf("Firmware: %.16s (%d)\n", firmwareVersionStr, firmwareVersion);
        return firmwareVersion;
    }

    std::string getSerialNumber() {
        if (!_initiated || !_useSerial) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return "";
        }

        char serialNumber[16]{};
        // if (!lwnxCmdReadString(this, 3, serialNumber)) {
        //     std::cerr << "No Response to command" << std::endl;
        //     return "";
        // }
        while (!lwnxCmdReadString(this, 3, serialNumber) == false) {
            usleep(50);
        }

        printf("Serial: %.16s\n", serialNumber);
        return std::string(serialNumber, 16);
    }

    void performHandShake() {
        this->getModelName();
        this->getHardwareVersion();
        this->getFirmwareVersion();
        this->getSerialNumber();
        std::cout << "Handshake Complete" << std::endl;
    }

    void setUpdateRate(uint8_t rate) {
        if (_initiated == false || _useSerial == false) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return;
        }

        // Set the output rate to 78 readings per second. (Command 76: Update rate)
        //     if (!lwnxCmdWriteUInt8(this, LWNXUpdateRateCmdID, rate)) {
        //         std::cerr << "No Response to command" << std::endl;
        //     }

        while (lwnxCmdWriteUInt8(this, LWNXUpdateRateCmdID, rate) == false) {
            usleep(50);
        }

        std::cout << "Rate Updated" << std::endl;
    }

    void setDistanceOutputConfig(uint32_t config) {
        if (_initiated == false || _useSerial == false) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return;
        }

        // Set distance output to include all information (0xFFFFFFFF): (Command 29: Distance output)
        //     if (!lwnxCmdWriteUInt32(this, LWNXDistanceOutputCmdID, config)) {
        //         std::cerr << "No Response to command" << std::endl;
        //     }
        while (lwnxCmdWriteUInt32(this, LWNXDistanceOutputCmdID, config) == false) {
            usleep(50);
        }

        std::cout << "Distance Output Config Set" << std::endl;
    }

    void enableStreaming() {
        if (_initiated == false || _useSerial == false) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return;
        }

        // Enable streaming of point data. (Command 30: Stream)
        // if (!lwnxCmdWriteUInt32(this, LWNXStreamCmdID, 5)) {
        //     std::cerr << "No Response to command" << std::endl;
        //     return;
        // }

        while (lwnxCmdWriteUInt32(this, LWNXStreamCmdID, 5) == false) {
            usleep(50);
        }

        // Loop start fetching info
        _threadRunning.store(true, std::memory_order_relaxed);
        _runningThread = std::thread(&SF30D::loop, this);

        std::cout << "Streaming Started" << std::endl;
    }

    void disableStreaming() {
        if (_initiated == false || _useSerial == false) {
            std::cerr << "Can't call serial command. Please init for serial first" << std::endl;
            return;
        }

        // Disable streaming of point data. (Command 30: Stream)
        if (!lwnxCmdWriteUInt32(this, LWNXStreamCmdID, 0)) {
            std::cerr << "No Response to command" << std::endl;
            return;
        }

        // Killing the read thread
        _threadRunning.store(false, std::memory_order_relaxed);
    }
};

