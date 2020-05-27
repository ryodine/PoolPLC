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

namespace Fault {

/**
 * @brief List of system faults
 */
enum Type {
    ZERO,
    INCLINOMETER_INIT,
    INCLINOMETER_INIT2,
    FRAM_INIT,
    FATAL_END_SENTINEL,

    INCLINOMETER_NOT_READY,
    INCLINOMETER_IMPLAUSIBLE_READING,
    ACCEL_PARITY_FAILURE,
    RETRYABLE_END_SENTINEL,
    ALL_OK
};

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
     * @brief Checks if there is at least one major (non-recoverable) fault latched
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
     * @brief periodic function that writes fault flash codes to an indicator.
     * 
     * Note - will only flash the first fault found.
     * 
     * 2 short flashes plus n long flashes for recoverable faults
     * 5 short flashes plus n long flashes for non-recoverable faults
     * 
     * @param LEDPIN pin to digital write error code patterns to
     */
    void faultFlasherPeriodic(const int LEDPIN);

    /**
     * @brief unlatch a fault condition
     * 
     * @param fault the fault condition to unlatch
     */
    void unlatchFaultCode(Type fault);

  private:
    bool faults[ALL_OK];
    static Handler *inst;
    int minorFaultState = 0;
    unsigned long lastTs;

    bool hasFault(Type fault);
    bool hasFaultOfType(Type start, Type end);
    int numFaults();
    int nextFault(Type start);
    void printFaultReport();
    Handler();
};
}; // namespace Fault

#endif
