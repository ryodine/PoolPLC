#ifndef PIN_MAPPINGS_GUARD_H
#define PIN_MAPPINGS_GUARD_H

// Status LED
#define STATUS_PIN CONTROLLINO_D7

// FAULT LED
#define FAULT_PIN CONTROLLINO_D6

// Zero sensors button input
#define ZERO_BUTTON CONTROLLINO_A0

// Zero sensors button input
#define REFLASH_ACEINNA CONTROLLINO_A3

// Pin for enabling raise mode on controller
#define RAISE_BUTTON CONTROLLINO_A1

// Pin for enabling lower mode on controller
#define LOWER_BUTTON CONTROLLINO_A2

//! Raising ram control
#define OUT_TR_RSE CONTROLLINO_D16
#define OUT_TL_RSE CONTROLLINO_D17
#define OUT_BL_RSE CONTROLLINO_D18
#define OUT_BR_RSE CONTROLLINO_D19

//! Lowering ram control
#define OUT_TR_LWR CONTROLLINO_D20
#define OUT_TL_LWR CONTROLLINO_D21
#define OUT_BL_LWR CONTROLLINO_D22
#define OUT_BR_LWR CONTROLLINO_D23

// Pin for cutting power to the main motor
#define OUT_ENABLE CONTROLLINO_D15

#endif // PIN_MAPPINGS_GUARD_H