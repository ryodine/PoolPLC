/**
 * @file DisplayControl.cpp
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Display view models and controllers
 * @version 0.1
 * @date 2020-06-16
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "DisplayControl.h"

#include <Arduino.h>
#include <cstdio>
#include <cstring>

#define OK_OR_HALT_OR_OFF(corner, enable)                                      \
    ((enable)) ? (((corner)) ? "HALT" : "OK") : "OFF"

using namespace Display;

void AbstractDisplayView::BlankDisplayableText(DisplayableText *text)
{
    memset(text, " ", sizeof(DisplayableText));
}

DisplayableText DisplayViewFault::Render(const SystemDisplayState &dispState)
{
    DisplayableText text;
    BlankDisplayableText(&text);
    strncpy(text.line_struct.line1, "!! Fault Detected !!", 21);
    strncpy(text.line_struct.line2, Fault::ecodeName[dispState.faultType], 21);
    strncpy(text.line_struct.line3, Fault::ecodeHelpText[dispState.faultType],
            21);
    if (dispState.faultType > Fault::FATAL_END_SENTINEL) {
        strncpy(text.line_struct.line4, " (please try again) ", 21);
    }
    else {
        strncpy(text.line_struct.line4, " (re-boot required) ", 21);
    }
    return text;
}

DisplayableText DisplayViewNormal::Render(const SystemDisplayState &dispState)
{
    DisplayableText text;
    BlankDisplayableText(&text);
    char pitch[8];
    char roll[8];
    dtostrf(dispState.pitch, 7, 2, pitch);
    dtostrf(dispState.roll, 7, 2, roll);
    snprintf(text.line_struct.line1, 21, "%-10s P:%7s",
             Motion::k_motionStateNames[dispState.motionState], pitch);
    if (dispState.motionState == Motion::MotionStateMachine::STATE_MOVING) {
        if (dispState.dirn == Motion::MovementDirection::RAISE) {
            snprintf(text.line_struct.line2, 21, "RAISING    R:%7s", roll);
        }
        else if (dispState.dirn == Motion::MovementDirection::LOWER) {
            snprintf(text.line_struct.line2, 21, "LOWERING   R:%7s", roll);
        }
        else {
            snprintf(text.line_struct.line2, 21, "OFF        R:%7s", roll);
        }
    }
    else if (dispState.motionState ==
             Motion::MotionStateMachine::STATE_FAULTED) {
        snprintf(text.line_struct.line2, 21, "CAN RESUME R:%7s", roll);
    }
    else {
        snprintf(text.line_struct.line2, 21, "SYSTEM OK  R:%7s", roll);
    }
    strncpy(text.line_struct.line3, "-1-- -2-- -3-- -4-- ", 21);
    snprintf(text.line_struct.line4, 21, "%-4s %-4s %-4s %-4s ",
             OK_OR_HALT_OR_OFF(dispState.ram1, dispState.enable),
             OK_OR_HALT_OR_OFF(dispState.ram2, dispState.enable),
             OK_OR_HALT_OR_OFF(dispState.ram3, dispState.enable),
             OK_OR_HALT_OR_OFF(dispState.ram4, dispState.enable));

    return text;
}

bool Controller::begin()
{
    lcd.begin(20, 4);
    lcd.setBacklight(HIGH);
    lcd.setCursor(0, 0);
}

void Controller::update(SystemDisplayState &state)
{
    DisplayableText t;
    if (state.faultType != Fault::ALL_OK) {
        DisplayViewFault faultDisp;
        t = faultDisp.Render(state);
    }
    else {
        DisplayViewNormal normalDisp;
        t = normalDisp.Render(state);
    }
    writeRaw(t);
}

void Controller::writeRaw(DisplayableText &t)
{
    lcd.setCursor(0, 0);
    lcd.print(t.line_struct.line1);
    lcd.setCursor(0, 1);
    lcd.print(t.line_struct.line2);
    lcd.setCursor(0, 2);
    lcd.print(t.line_struct.line3);
    lcd.setCursor(0, 3);
    lcd.print(t.line_struct.line4);
}