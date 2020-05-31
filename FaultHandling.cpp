#include "FaultHandling.h"
#include "Arduino.h"

Fault::Handler *Fault::Handler::inst = 0;

Fault::Handler *Fault::Handler::instance()
{
    if (inst == 0) {
        inst = new Handler();
    }
    return inst;
}

Fault::Handler::Handler()
{
    for (int i = 0; i < ALL_OK; i++) {
        faults[i] = false;
    }
}

void Fault::Handler::setFaultCode(Type fault)
{
    if (!this->faults[fault]) {
        this->faults[fault] = true;
        this->minorFaultState = 0;
        this->lastTs = millis();
    }
}

void Fault::Handler::unlatchFaultCode(Type fault)
{
    this->faults[fault] = false;
}

void Fault::Handler::onFaultUnlatchEvent(FaultUnlatchEvent event)
{
    bool* evtTriggers = k_faultUnlatchMapping[event];
    for (int i = 0; i < RETRYABLE_END_SENTINEL - FATAL_END_SENTINEL; ++i) {
        if (evtTriggers[i]) {
            unlatchFaultCode(FATAL_END_SENTINEL + 1 + i);
        }
    }
}

bool Fault::Handler::hasFault(Type fault) { return this->faults[fault]; }

bool Fault::Handler::hasFaultOfType(Type start, Type end)
{
    for (int i = start; i < end; i++) {
        if (hasFault((Type)i)) {
            return true;
        }
    }
    return false;
}

int Fault::Handler::numFaults()
{
    int count = 0;
    for (int i = 0; i < ALL_OK; i++) {
        if (faults[i] == true) {
            count++;
        }
    }
    return count;
}

int Fault::Handler::nextFault(Type start)
{
    for (int i = start; i < ALL_OK; i++) {
        if (faults[i] == true) {
            return i;
        }
    }
    return -1;
}

void Fault::Handler::printFaultReport()
{
    Serial.println("FAULT REPORT:");
    for (int i = ZERO + 1; i < FATAL_END_SENTINEL; i++) {
        Serial.print("Fatal Fault #");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(faults[i]);
    }
    for (int i = FATAL_END_SENTINEL + 1; i < RETRYABLE_END_SENTINEL; i++) {
        Serial.print("Recoverable Fault #");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(faults[i]);
    }
}

void Fault::Handler::faultFlasherPeriodic(const int LEDPIN)
{
    if (!hasFault()) {
        digitalWrite(LEDPIN, HIGH);
        return;
    }

    if (hasMajorFault()) {
        int fault = nextFault(ZERO);
        while (true) {
            for (int i = 0; i < 5; i++) {
                digitalWrite(LEDPIN, LOW);
                delay(100);
                digitalWrite(LEDPIN, HIGH);
                delay(200);
                digitalWrite(LEDPIN, LOW);
                delay(100);
            }
            for (int i = 0; i < fault; i++) {
                digitalWrite(LEDPIN, LOW);
                delay(100);
                digitalWrite(LEDPIN, HIGH);
                delay(1000);
                digitalWrite(LEDPIN, LOW);
                delay(100);
            }
            printFaultReport();
        }
    }

    if (hasMinorFault()) {
        int fault = nextFault(FATAL_END_SENTINEL);
        unsigned long millisDiff = millis() - lastTs;
        if (millisDiff > 100 || millisDiff < 0) {
            minorFaultState++;
            lastTs = millis();
        }
        int fs = minorFaultState % (fault * 6 + 4);
        if (fs < 4) {
            digitalWrite(LEDPIN, fs % 2 == 0);
        }

        if (fs >= 4) {
            digitalWrite(LEDPIN, fs % 6 <= 4);
        }

        if (fs == (fault * 6 + 3)) {
            printFaultReport();
        }
    }
}
