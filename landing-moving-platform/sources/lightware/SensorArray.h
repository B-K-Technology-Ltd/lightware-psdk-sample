//
// Created by LightWare.
// Software representation of the LW20/c quad sensor array for platform detection
//

#pragma once

#include "LW20.h"
#include <limits>

class SensorArray {

public:
    SensorArray() {
        this->_sensor1 = new LW20();
        this->_sensor2 = new LW20();
        this->_sensor3 = new LW20();
        this->_sensor4 = new LW20();
    }

    ~SensorArray() {
        delete this->_sensor1;
        delete this->_sensor2;
        delete this->_sensor3;
        delete this->_sensor4;
    }

    // Connects the entire array and start receiving distances
    void connect() {
        this->_sensor1->connect(0x66);
        this->_sensor2->connect(0x67);
        this->_sensor3->connect(0x68);
        this->_sensor4->connect(0x69);
    }

    // Average distance in cm
    int averageDistance() {
        int sensor1LastRead = this->_sensor1->latestDistance();
        int sensor2LastRead = this->_sensor2->latestDistance();
        int sensor3LastRead = this->_sensor3->latestDistance();
        int sensor4LastRead = this->_sensor4->latestDistance();

        return (sensor1LastRead + sensor2LastRead + sensor3LastRead + sensor4LastRead)/4;
    }

    // Return the maximum differences between the closest and the furthest away hit
    int maxDelta() {
        int sensor1LastRead = this->_sensor1->latestDistance();
        int sensor2LastRead = this->_sensor2->latestDistance();
        int sensor3LastRead = this->_sensor3->latestDistance();
        int sensor4LastRead = this->_sensor4->latestDistance();

        int distances[4] = {sensor1LastRead, sensor2LastRead, sensor3LastRead, sensor4LastRead};
        int lowest = std::numeric_limits<int>::max();
        int highest = std::numeric_limits<int>::min();

        for(int i = 0; i < size; ++i) {
            if(distances[i] < lowest) {
                lowest = distances[i];
            }
            if(distances[i] > highest) {
                highest = distances[i];
            }
        }

        return highest - lowest;
    }


private:
    LW20 *_sensor1;
    LW20 *_sensor2;
    LW20 *_sensor3;
    LW20 *_sensor4;

};
