#pragma once

#include "I2C.h"         // for I2C and IOFailure
#include <assert.h>      // for assert
#include <cmath>         // for M_PI
#include <limits>        // for numeric limits
#include <mutex>         // for mutex
#include <ros/console.h> // for ROS_ERROR
#include <string.h>      // for string and memcpy

#define OFF_OP 0x00
#define OFF_WB 0
#define OFF_RB 0

#define ON_OP 0x01
#define ON_WB 0
#define ON_RB 0

#define OPEN_OP 0x02
#define OPEN_WB 4
#define OPEN_RB 0

#define OPEN_PLUS_OP 0x03
#define OPEN_PLUS_WB 4
#define OPEN_PLUS_RB 4

#define CLOSED_OP 0x04
#define CLOSED_WB 8
#define CLOSED_RB 0

#define CLOSED_PLUS_OP 0x05
#define CLOSED_PLUS_WB 8
#define CLOSED_PLUS_RB 4

#define CONFIG_PWM_OP 0x06
#define CONFIG_PWM_WB 2
#define CONFIG_PWM_RB 0

#define CONFIG_K_OP 0x07
#define CONFIG_K_WB 12
#define CONFIG_K_RB 0

#define QUAD_OP 0x08
#define QUAD_WB 0
#define QUAD_RB 4

#define ADJUST_OP 0x09
#define ADJUST_WB 4
#define ADJUST_RB 0

#define ABS_ENC_OP 0x0A
#define ABS_ENC_WB 0
#define ABS_ENC_RB 4

#define LIMIT_OP 0x0B
#define LIMIT_WB 0
#define LIMIT_RB 1

#define CALIBRATED_OP 0x0C
#define CALIBRATED_WB 0
#define CALIBRATED_RB 1

#define LIMIT_ON_OP 0x0D
#define LIMIT_ON_WB 1
#define LIMIT_ON_RB 0

#define UINT8_POINTER_T reinterpret_cast<uint8_t*>

#define CALIBRATED_BOOL 0xFF

/*
Virtual Controllers store information about various
controller-specific parameters (such as encoder cpr).
The virtual Controller class also has functions representing
the possible transactions that can be had with the physical controller.
The virtual Controller will not attempt to communicate with its
physical controller unless "activated" by an appropriate ROS
message relayed by ROSHandler.h.
(e.g. A virtual RA Controller will never attempt to communicate
with its physical RA controller unless an RA-related ROS message is
sent. This is to prevent multiple virtual Controller objects from
trying to contact the same physical Controller object.)
*/


class Controller {
public:
    float quadCPR = std::numeric_limits<float>::infinity();
    float kP = 0.01f;
    float kI = 0.0f;
    float kD = 0.0f;
    float inversion = 1.0f;

    // REQUIRES: _name is the name of the motor,
    // mcuID is the mcu id of the controller which dictates the slave address,
    // _motorID is the motor id of the motor that is to be controlled,
    // motorMaxVoltage is the max allowed voltage of the motor,
    // and driverVoltage is the input voltage of the driver.
    // 0 < motorMaxVoltage <= driverVoltage <= 36.
    // It is very important that the driverVoltage is 100% accurate,
    // or else you will end up giving too much voltage to a motor,
    // which is dangerous.
    // MODIFIES: Controller object
    // EFFECTS: Creates a virtual controller object
    // that when live, will control the
    // physical controller (the STM32).
    Controller(
            std::string& _name,
            uint8_t mcuID,
            uint8_t _motorID,
            float _motorMaxVoltage,
            float _driverVoltage);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns true if Controller is live.
    bool isControllerLive() const;

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns last saved value of angle.
    // Expect a value between -M_PI and M_PI.
    float getCurrentAngle() const;

    // REQUIRES: -1.0 <= input <= 1.0
    // MODIFIES: currentAngle. Also makes controller live if not already.
    // EFFECTS: Sends an open loop command scaled to PWM limits
    // based on allowed voltage of the motor. Also updates angle.
    void moveOpenLoop(float input);

private:
    // REQUIRES: nothing
    // MODIFIES: isLive
    // EFFECTS: If not already live,
    // configures the physical controller.
    // Then makes live.
    void makeLive();

    uint8_t deviceAddress;
    uint8_t motorID;
    uint8_t motorIDRegMask;
    float motorMaxVoltage;
    float driverVoltage;
    std::string name;

    float currentAngle;

    bool isLive = false;
};
