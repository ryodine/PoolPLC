/**
 * @file FaultHandling.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Manages fault conditions and reporting
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef POOL_FAULT_HANDLING_H
#define POOL_FAULT_HANDLING_H

#ifndef ENUM_TO_STRING
#define __ENUM_TO_STRING__(x) #x
#define ENUM_TO_STRING(x)     __ENUM_TO_STRING__(x)
#endif

namespace Fault {

/**
 * @brief List of system faults
 */
enum Type {
    // 20 char limit:     //
    //--------------------//
    ZERO,
    INCLINOMETER_INIT,
    INCLINOMETER_INIT2,
    FRAM_INIT,
    FATAL_END_SENTINEL,

    INCLINOMETER_UNREADY,
    INCL_IMPLAUS_READ,
    ACCEL_PARITY_FAILURE,
    TOO_MUCH_TILT,
    RETRYABLE_END_SENTINEL,
    ALL_OK
};

const char ecodeName[ALL_OK + 1][21] = {ENUM_TO_STRING(ZERO),
                                        ENUM_TO_STRING(INCLINOMETER_INIT),
                                        ENUM_TO_STRING(INCLINOMETER_INIT2),
                                        ENUM_TO_STRING(FRAM_INIT),
                                        ENUM_TO_STRING(FATAL_END_SENTINEL),
                                        ENUM_TO_STRING(INCLINOMETER_UNREADY),
                                        ENUM_TO_STRING(INCL_IMPLAUS_READ),
                                        ENUM_TO_STRING(ACCEL_PARITY_FAILURE),
                                        ENUM_TO_STRING(TOO_MUCH_TILT),
                                        ENUM_TO_STRING(RETRYABLE_END_SENTINEL),
                                        ENUM_TO_STRING(ALL_OK)};

constexpr char ecodeHelpText[ALL_OK + 1][21] = {
    //   20 char limit:     //
    //"--------------------"//
    "",
    "Incl. 1 start failed",
    "Incl. 2 start failed",
    "Memory module failed",
    "",
    "Sensor timed out    ",
    "Sensor bad reading  ",
    "Sensor disagreement ",
    "Too much tilt!"
    "",
    "NO SYSTEM ERRORS"};

/**
 * @brief List of system events that can unlatch one or more recoverable faults
 */
enum FaultUnlatchEvent {
    INCLINOMETER_DATA_RECEIVE = 0,
    MOVEMENT_COMMAND_END,
    FAULT_UNLATCH_EVENT_SIZE
};

// clang-format off
constexpr bool k_faultUnlatchMapping[FAULT_UNLATCH_EVENT_SIZE][RETRYABLE_END_SENTINEL - FATAL_END_SENTINEL] = {
//  INCL_NR  IMPLAUS  PARITY
    {true,    false,   false}, // INCLINOMETER_DATA_RECEIVE
    {false,   true,    false} // MOVEMENT_COMMAND_END
};
// clang-format on

/**
 * @brief Fault handler singleton
 */
class Handler {
  public:
    /**
     * @brief Get the pointer to the instance of the fault handler
     *
     * @return Handler* shared instance
     */
    static Handler *instance();

    /**
     * @brief Set the Fault Code
     *
     * @param fault the fault to latch
     */
    void setFaultCode(Type fault);

    /**
     * @brief Checks if there is at least one minor (recoverable) fault latched
     *
     * @return true if there is at least one minor fault
     * @return false if there aren't any minor faults
     */
    bool hasMinorFault() { return hasFaultOfType(FATAL_END_SENTINEL, ALL_OK); };

    /**
     * @brief Checks if there is at least one major (non-recoverable) fault
     * latched
     *
     * @return true if there is at least one major fault
     * @return false if there aren't any major faults
     */
    bool hasMajorFault() { return hasFaultOfType(ZERO, FATAL_END_SENTINEL); };

    /**
     * @brief Checks if there is at least one fault latched
     *
     * @return true if there is at least one fault
     * @return false if there aren't any faults
     */
    bool hasFault() { return hasFaultOfType(ZERO, ALL_OK); };

    /**
     * @brief Serial logs the current faults
     *
     */
    void printFaultReport();

    /**
     * @brief unlatch a fault condition directly
     *
     * @param fault the fault condition to unlatch
     */
    void unlatchFaultCode(Type fault);

    /**
     * @brief Unlatches all faults that are mapped to this fault unlatch event.
     *
     * @param event the event that was triggered
     */
    void onFaultUnlatchEvent(FaultUnlatchEvent event);

    /**
     * @brief Finds the next fault, starting at start
     *
     * @param start starting fault
     * @return int next fault, or -1 if none
     */
    int nextFault(Type start);

  private:
    bool faults[ALL_OK];
    static Handler *inst;
    int minorFaultState = 0;
    unsigned long lastTs;

    bool hasFault(Type fault);
    bool hasFaultOfType(Type start, Type end);
    int numFaults();
    Handler();
};
}; // namespace Fault

#endif
