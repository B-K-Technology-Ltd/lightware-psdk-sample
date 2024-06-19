//
// Created by LightWare
//

#include "lightware/SF30D.h"
#include "dji/application/application.hpp"


// Some globals to make things easier
Application *vehicle = NULL;
SF30D *sensor = NULL;

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

    sensor = new SF30D();
    sensor->connect();

    // Check loop
    bool active = true;
    int closeEnough = 50; // in cm; 50cm

    while (active) {
        int currentDistance = sensor->latestDistance();

        // Condition to determine a landing opportunity.
        if (currentDistance <= closeEnough) {
            // We exited the loop flagging we aren't checking to land, so we are landing.
            std::cout << "Stopping..." << std::endl;

            vehicle->emergencyBreak();
            break;
        }
        usleep(250);
    }
}
