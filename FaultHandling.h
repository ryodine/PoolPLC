#ifndef POOL_FAULT_HANDLING_H
#define POOL_FAULT_HANDLING_H

namespace Fault {
  enum Type {
    ZERO,
    ACCEL_INIT,
    FRAM_INIT,
    FATAL_END_SENTINEL,
    ACCEL_NOT_READY,
    ACCEL_IMPLAUSIBLE_READING,
    ACCEL_PARITY_FAILURE,
    RETRYABLE_END_SENTINEL,
    ALL_OK
  };

  class Handler {
    public:
      static Handler* instance();
      void setFaultCode(Type fault);
      bool hasMinorFault() { return hasFaultOfType(FATAL_END_SENTINEL, ALL_OK); };
      bool hasMajorFault() { return hasFaultOfType(ZERO, FATAL_END_SENTINEL); };
      bool hasFault() { return hasFaultOfType(ZERO, ALL_OK); };
      void faultFlasherPeriodic(const int LEDPIN);
      void unlatchFaultCode(Type fault);
    private:
      bool faults[ALL_OK];
      static Handler* inst;
      int minorFaultState = 0;
      unsigned long lastTs;

      bool hasFault(Type fault);
      bool hasFaultOfType(Type start, Type end);
      int numFaults();
      int nextFault(Type start);
      void printFaultReport();
      Handler();
  };
};

#endif
