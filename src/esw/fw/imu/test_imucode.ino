// MCP2517/8 send a CAN frame
// CAN FD Shield - https://www.longan-labs.cc/1030012.html
// CANBed FD - https://www.longan-labs.cc/1030009.html

#include <SPI.h>
#include "mcp2518fd_can.h"

// pins for CAN-FD Shield
//const int SPI_CS_PIN = 9;
//const int CAN_INT_PIN = 2;

// pins for CANBed FD
const int SPI_CS_PIN = 17;
const int CAN_INT_PIN = 7;

mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

void setup() {
    Serial.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN20_1000KBPS)) {
        Serial.println("CAN init fail, retry...");
        delay(100);
    }
    Serial.println("CAN init ok!");
}

unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};

// struct CanFdMessageId {
//         std::uint8_t destination{};
//         std::uint8_t source : 7 {};
//         bool replyRequired : 1 {};
//     };

void loop() {
    CAN.sendMsgBuf((0x10 << 8) | (0x60 << 1), 1, 8, stmp);
    delay(10);
    CAN.sendMsgBuf((0x10 << 8) | (0x60 << 1), 1, 8, stmp);
    delay(500);                       // send data per 100ms
    Serial.println("CAN BUS sendMsgBuf ok!");
}

// END FILE
