#LightWare Sample
## Landing on a Moving Vehicle

This sample code is to accompany the content explaining how to leverage the LW20/c and the DJI M350 to develop a logic to land on a moving vehicle.
The robotic element to make this a production-viable system is hyper simplified by design and should not be considered as production-ready in its current state.

## Principles

This code is establishing a connection with the DJI M350 using the Payload SDK and observes in a control loop the results of four LW20/C until the results reflect of a relative flat surface identified as a commit moment to finally send command to the aircraft to land.

## Project Structure

| Folder | Description                                            |
| -- |--------------------------------------------------------|
| libraries | Contains all the libraries dependencies that are local |
| sources | Contains all source code for the project, including headers |
| sources/dji | Contains the PayloadSDK Sample - modded to work with this path structure |
| sources/lightware | Contains the LightWare LW20/c implementation to interface with the sensors |


## LW20/c Interface

This project uses a software defined representation of the sensor configuration as an 'array'.

This allows the main code to be structured, but isn't strictly required for function.

## Installing dependencies

### DJI

This project uses the PayloadSDK sample. You'll need to configure it and install the dependencies as covered here:

https://developer.dji.com/doc/payload-sdk-tutorial/en/quick-start/quick-guide/raspberry-pi.html


## Compiling Instructions

This project uses a traditional CMake flow. From the root folder (where the CMakeLists.txt file is):

```shell
mkdir build;
cd build;
cmake ..;
make -j4;
make install;
```

## Running the app

From the build folder:

```shell
./lander
```