#ifndef MANCHESTERSTATEMACHINE_H
#define MANCHESTERSTATEMACHINE_H

#define SYNC true
#define STANDARD IEEE

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "ManchesterEncoding.h"
#include <stdbool.h>
#include <stdint.h>

#define GAP_TIME_US 104
#define MANCHESTER_MAX_DATA_SIZE 10
#define IDLE_CONDITION_MIN_REPEAT 4

enum State {
    STATE_IDLE,
    STATE_START_CONDITION,
    STATE_START_BIT_DETECTION,
    STATE_DATA_RECEIVING,
	  STATE_STOP_CONDITION
};

void setOutputSerial(bool _useSerial);
void setInputPin(uint8_t _inputPin);
void setOutputPin(uint8_t _outputPin);
void setupManchesterTimerIRQ();
enum StatusMessages stateDataReceivingHandler();
void stateIdleHandler();
void stateStartCondtionHandler();
void stateStartBitDetectionHandler();
void stopConditionHandler();
void manchesterStateMachine();
void storeDecodedBits();


#endif