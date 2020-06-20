/**
 * @file DisplayControl.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Display view models and controllers
 * @version 0.1
 * @date 2020-06-13
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef DISPLAY_CONTROL_H
#define DISPLAY_CONTROL_H

#include "FaultHandling.h"
#include "MotionStateMachine.h"

#include <Adafruit_LiquidCrystal.h>

namespace Display {

/**
 * 20x4 display template
 *
 * +--------------------+
 * |                    |
 * |                    |
 * |                    |
 * |                    |
 * +--------------------+
 */

/**
 * Normal Operation
 *
 * +--------------------+
 * |LOWERING   P:-360.00|
 * |SYS OK     R:-360.00|
 * | 1    2    3    4   |
 * | OK   OK   HALT HALT|
 * +--------------------+
 *
 * +--------------------+
 * |STOPPED    P:-360.00|
 * |SYS OK     R:-360.00|
 * | 1    2    3    4   |
 * | HALT HALT HALT HALT|
 * +--------------------+
 *
 * +--------------------+
 * |RAISING    P:-360.00|
 * |SYS OK     R:-360.00|
 * | 1    2    3    4   |
 * | OK   OK   OK   OK  |
 * +--------------------+
 */

/**
 * FAULT
 *
 * +--------------------+
 * | /!\ Fault Detected | <--- Fault header
 * |INCLINOMETER_INIT   | <--- error code
 * | (re-boot required) | <--- Recoverable or non-recoverable fault
 * |Incl. 1 start failed| <--- Short human-readable
 * +--------------------+
 *
 * +--------------------+
 * | /!\ Fault Detected | <--- Fault header
 * |INCLINOMETER_UNREADY| <--- error code
 * | (please try again) | <--- Recoverable or non-recoverable fault
 * |Sensor timed out    | <--- Short human-readable
 * +--------------------+
 */

typedef struct {
    Motion::MotionStateMachine::STATE motionState;
    double pitch;
    double roll;
    bool ram1;
    bool ram2;
    bool ram3;
    bool ram4;
    bool enable;
    Motion::MovementDirection dirn;
    Fault::Type faultType;
} SystemDisplayState;

typedef union {
    struct __attribute__((packed)) {
        char line1[21];
        char line2[21];
        char line3[21];
        char line4[21];
    } line_struct;
    char array[4][21];
} DisplayableText;

class Controller {
  public:
    Controller() : lcd{0} {};
    bool begin();
    void writeRaw(DisplayableText& t);
    void update(SystemDisplayState &state);
  private:
    Adafruit_LiquidCrystal lcd;
};

class AbstractDisplayView {
  public:
    virtual DisplayableText Render(const SystemDisplayState &dispState) = 0;
    void BlankDisplayableText(DisplayableText *text);
};

class DisplayViewFault : public AbstractDisplayView {
  public:
    DisplayableText Render(const SystemDisplayState &dispState) override;
};

class DisplayViewNormal : public AbstractDisplayView {
  public:
    DisplayableText Render(const SystemDisplayState &dispState) override;
};

} // namespace Display

#endif // DISPLAY_CONTROL_H