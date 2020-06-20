#ifndef PIN_MAPPINGS_GUARD_H
#define PIN_MAPPINGS_GUARD_H

// Status LED
#define STATUS_PIN CONTROLLINO_D7

// FAULT LED
#define FAULT_PIN CONTROLLINO_D6

// Zero sensors button input
#define ZERO_BUTTON CONTROLLINO_A0

// Pin for enabling raise mode on controller
#define RAISE_BUTTON CONTROLLINO_A1

// Pin for enabling lower mode on controller
#define LOWER_BUTTON CONTROLLINO_A2

// Pin for disabling to TOP RIGHT ram
#define OUT_TR CONTROLLINO_D16

// Pin for disabling to TOP LEFT ram
#define OUT_TL CONTROLLINO_D17

// Pin for disabling to BOTTOM LEFT ram
#define OUT_BL CONTROLLINO_D18

// Pin for disabling to BOTTOM RIGHT ram
#define OUT_BR CONTROLLINO_D19

// Pin for cutting power to the main motor
#define OUT_ENABLE CONTROLLINO_D23

#endif // PIN_MAPPINGS_GUARD_H