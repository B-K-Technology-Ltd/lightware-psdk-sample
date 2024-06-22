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
    if (sensor != NULL) {
        delete sensor;
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
    float closeEnough = 0.5; // in m; 50cm

    while (active) {
        float currentDistance = sensor->latestDistance();

        // Condition to determine a landing opportunity.
        if (currentDistance <= closeEnough && currentDistance > 0 && currentDistance < 250) { // 0 and 250 are false reads
            // We exited the loop flagging we aren't checking to land, so we are landing.
            std::cout << "Stopping Vehicle..." << std::endl;

            vehicle->emergencyBreak();
        }
        usleep(250);
    }
}
