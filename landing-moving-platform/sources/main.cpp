//
// Created by LightWare
//

#include "lightware/SensorArray.h"
#include "dji/application/application.hpp"


// Some globals to make things easier
Application *vehicle = NULL;
SensorArray *sensorArray = NULL;

void clean_exit(int code) {
    // Stop anything that needs stopping.
    // Clean anything that needs cleaning.

    if (vehicle != NULL) {
        delete vehicle;
    }
    if (sensorArray != NULL) {
        delete sensorArray;
    }
    std::cout << "Exiting..." << std::endl;
    exit(code);
}

int main(int argc, char **argv) {
    // Create the vehicle instance to connect
    // We are reusing the Application object from DJI's sample for clarity only.
    vehicle = new Application(argc, argv);

    sensorArray = new SensorArray();
    sensorArray->connect();

    // Check loop
    bool checkingToLand = true;
    int flatEnough = 10; // in cm; 10cm -> ~15° plane

    while (checkingToLand) {
        int maxDelta = sensorArray->maxDelta();

        // Condition to determine a landing opportunity.
        if (maxDelta <= flatEnough && maxDelta > 0) {
            checkingToLand = false;
            break;
        }
    }

    if (!checkingToLand) {
        // We exited the loop flagging we aren't checking to land, so we are landing.
        std::cout << "Landing..." << std::endl;

        vehicle->land();
    }
}
