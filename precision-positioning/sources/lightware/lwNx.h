//----------------------------------------------------------------------------------------------------------------------------------
// The LWNX protocol is a binary based protocol for reading and writing data to LightWare devices.
//----------------------------------------------------------------------------------------------------------------------------------
#pragma once

#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "platformLinux.h"

#define PACKET_START_BYTE	0xAA
#define PACKET_TIMEOUT		200
#define PACKET_RETRIES		4

class lwResponsePacket {
	public:
		uint8_t data[1024];
		int32_t size;
		int32_t payloadSize;
		uint8_t parseState;

		lwResponsePacket();
};

class lwSerialPort {
public:
    virtual int writeData(uint8_t *Buffer, int32_t BufferSize) = 0;
    virtual int32_t readData(uint8_t *Buffer, int32_t BufferSize) = 0;
};

//----------------------------------------------------------------------------------------------------------------------------------
// Commands IDs
//----------------------------------------------------------------------------------------------------------------------------------

// Product Name - R:16bytes
#define LWNXProductNameCmdID            0

// Hardware Version - R:4bytes
#define LWNXHardwareVersionCmdID        1

// Firmware Version - R:4bytes
#define LWNXFirmwareVersionCmdID        2

// Serial Number - R:16bytes
#define LWNXSerialNumberCmdID           3

// Human readable text message
#define LWNXUTF8TextMessageCmdID        7

// 16 byte store for user data - R:16bytes; W:16bytes; Persists
#define LWNXUserDataCmdID               9

// Next usable safety token - R:2bytes
#define LWNXTokenCmdID                  10

// Store persistable parameters - W:2bytes
#define LWNXSaveParametersCmdID         12

// Restart the unit - W:2bytes
#define LWNXResetCmdID                  14

// Upload firmware file pages - R:4bytes; W:130bytes
#define LWNXStageFirmwareCmdID          16

// Apply staged firmware - R:4bytes; W:0bytes
#define LWNXCommitFirmwareCmdID         17

// Distance output configuration - R:4bytes; W:4bytes; Persists
#define LWNXDistanceOutputCmdID         29

typedef enum: uint32_t  {
    // Bit	Output
    LWNXDistanceOutputParamFirstReturnRaw = 0xF0000000,      //	First return raw
    LWNXDistanceOutputParamFirstReturnFilter = 0x0F000000,   //	First return filter
    LWNXDistanceOutputParamFirstReturnStrength = 0x00F00000, //	First return strength
    LWNXDistanceOutputParamLastReturnRaw = 0x000F0000,       //	Last return raw
    LWNXDistanceOutputParamLastReturnFilter = 0x0000F000,    //	Last return filter
    LWNXDistanceOutputParamLastReturnStrength = 0x00000F00,  //	Last return strength
    LWNXDistanceOutputParamBackgroundNoise = 0x000000F0,     //	Background noise
    LWNXDistanceOutputParamTemperature = 0x0000000F,         //	Temperature
} LWNXDistanceOutputParam;

//Current data stream type - R:4bytes; W:4bytes
#define LWNXStreamCmdID                 30

// Full Speed Measurement distance data in cm - R:~
#define LWNXFullSpeedDistanceInCmCmdID  40

// Measurement distance data in cm - R:~
#define LWNXDistanceDataInCmCmdID       44

// Is laser firing? - R:1byte; W:1byte
#define LWNXLaserFiringCmdID            50

// Measured temperature - R:4bytes
#define LWNXTemperatureCmdID            55

// Data sampling update rate - R:2bytes; W:2bytes; Persists
#define LWNXUpdateRateCmdID             76

typedef enum : uint8_t {
//    Command value	Update rate samples/second
    LWNXUpdateRateParam20010 = 0,
    LWNXUpdateRateParam10005 = 1,
    LWNXUpdateRateParam5002 = 2,
    LWNXUpdateRateParam2501 = 3,
    LWNXUpdateRateParam1250 = 4,
    LWNXUpdateRateParam625 = 5,
    LWNXUpdateRateParam312 = 6,
    LWNXUpdateRateParam156 = 7,
    LWNXUpdateRateParam78 = 8,
    LWNXUpdateRateParam39 = 9,
} LWNXUpdateRateParam;

// Select First or Last Return - R:2bytes; W:2bytes; Persists
#define LWNXReturnModeCmdID             77

// Analog Data update rate - R:2bytes; W:2bytes; Persists
#define LWNXAnalogUpdateRateCmdID       80

// Measured background noise - R:4bytes
#define LWNXNoiseCmdID                  85

// Serial baud rate - R:1byte; W:1byte; Persists
#define LWNXBaudRateCmdID               91

// I2C address - R:1byte; W:1byte; Persists
#define LWNXI2CAddressCmdID             92



//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
// Create a CRC-16-CCITT 0x1021 hash of the specified data.
uint16_t lwnxCreateCrc(uint8_t* Data, uint16_t Size);

// Breaks an integer firmware version into Major, Minor, and Patch.
void lwnxConvertFirmwareVersionToStr(uint32_t Version, char* String);

//----------------------------------------------------------------------------------------------------------------------------------
// LWNX protocol implementation.
//----------------------------------------------------------------------------------------------------------------------------------
// Prepare a response packet for a new incoming response.
void lwnxInitResponsePacket(lwResponsePacket* Response);

// Waits to receive a packet of specific command id.
// Does not return until a response is received or a timeout occurs.
bool lwnxRecvPacket(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutMs);

// Returns true if full packet was received, otherwise finishes immediately and returns false while waiting for more data.
bool lwnxRecvPacketNoBlock(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response);

// Composes and sends a packet.
void lwnxSendPacketBytes(lwSerialPort* Serial, uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize);

// Handle both the sending and receving of a command. 
// Does not return until a response is received or all retries have expired.
bool lwnxHandleManagedCmd(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, bool Write = false, uint8_t* WriteData = NULL, uint32_t WriteSize = 0);

//----------------------------------------------------------------------------------------------------------------------------------
// Command functions.
//----------------------------------------------------------------------------------------------------------------------------------
// Issue read commands.
bool lwnxCmdReadInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t* Response);
bool lwnxCmdReadInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t* Response);
bool lwnxCmdReadInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t* Response);

bool lwnxCmdReadUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response);
bool lwnxCmdReadUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t* Response);
bool lwnxCmdReadUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t* Response);

bool lwnxCmdReadString(lwSerialPort* Serial, uint8_t CommandId, char* Response);
bool lwnxCmdReadData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize);

// Issue write commands.
bool lwnxCmdWriteInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t Value);
bool lwnxCmdWriteInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t Value);
bool lwnxCmdWriteInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t Value);

bool lwnxCmdWriteUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t Value);
bool lwnxCmdWriteUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t Value);
bool lwnxCmdWriteUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t Value);

bool lwnxCmdWriteString(lwSerialPort* Serial, uint8_t CommandId, char* String);
bool lwnxCmdWriteData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Data, uint32_t DataSize);
