#include "manchesterStateMachine.h"

static bool outputEncodedFlag = false;
static enum StatusMessages status;
static unsigned char decodedDataResult[MANCHESTER_MAX_DATA_SIZE];
static uint8_t decodedDataLen = 0;
static bool foundFirstBit = false;
static bool pinState;
static bool rcvDataBitsFlag = false;
static enum State currentState = STATE_IDLE;
static int consecutiveLowCount = 0;
static bool foundStartBitZero = false;
static int bytesReceivedCnt = 0;
static unsigned char bytesReceived[MANCHESTER_MAX_DATA_SIZE * 2] = {0};
static unsigned char byteReceived = 0;
static int bitIdx = BYTE_SIZE;

static uint8_t inputPin = 6;
static uint8_t outputPin = 8;
static bool useSerial = false;


/**
 * @brief Decide either to activate serial output or not.
 *
 * This function check the user demand to print the results on other function with the Serial port.
 *
 * @param _useSerial Boolean indicating whether to use serial communication (true) or not (false).
 */
void setOutputSerial(bool _useSerial)
{
	useSerial = _useSerial;
	if (useSerial)
	{
		Serial.begin(115200);
		while (!Serial);
	}
}

/**
 * @brief Set the input pin for receiving Manchester encoded data.
 *
 * This function sets the input pin for receiving Manchester encoded data.
 *
 * @param _inputPin The input pin number.
 */
void setInputPin(uint8_t _inputPin)
{
	pinMode(_inputPin, INPUT);
	inputPin = _inputPin;
}

/**
 * @brief Decide either to write decoded data to gpio.
 *
 * This function check the user demand to write decoded data to gpio.
 *
 * @param _outputPin Boolean indicating whether to use gpio write (true) or not (false).
 */
void setOutputPin(uint8_t _outputPin)
{
	outputPin = _outputPin;
	pinMode(outputPin, OUTPUT);
	digitalWrite(outputPin, LOW);
  outputEncodedFlag = true;
}

/**
 * @brief Setup timer register.
 *
 * This function sets up the timer register to generate interrupt every GAP_TIME_US in order to "catch" the data. 
 * GAP_TIME_US = 1/baud
 */
void setupManchesterTimerIRQ()
{
	NVIC_EnableIRQ(TIMER0_IRQn);
	NRF_TIMER0->TASKS_STOP = 1;
	
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // use timer mode - increment timer counter every tick
	NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
	NRF_TIMER0->PRESCALER = 4; // 16MHz / (2^4) = 1MHz -> 1us
	
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->CC[0] = GAP_TIME_US;
	NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos; // enable compare[0] event

	NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos); // clears timer on comapre[0] event
}

extern "C"
{
  /**
  * @brief Timer interrupt handler.
  */
  void TIMER0_IRQHandler()
  {
    if (NRF_TIMER0->EVENTS_COMPARE[0] && NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk)
    {
      NRF_TIMER0->EVENTS_COMPARE[0] = 0;
      if (SYNC && foundFirstBit)
      {
        NRF_TIMER0->TASKS_CLEAR = 1;
        NRF_TIMER0->CC[0] = GAP_TIME_US;
        foundFirstBit = false;
      }
      pinState = digitalRead(inputPin);
	    consecutiveLowCount = pinState ? 0 : consecutiveLowCount+1;
      if (rcvDataBitsFlag && STATE_DATA_RECEIVING)
      {
		    storeDecodedBits();
      }
    }
  }
}

/**
 * @brief Store decoded Manchester bits.
 *
 * This function stores the decoded Manchester bits into the 'bytesReceived' array.
 */
void storeDecodedBits()
{
	byteReceived |= (pinState << --bitIdx);
	if (outputEncodedFlag)
    digitalWrite(outputPin, pinState);
	if (bitIdx == 0)
	{
	  bytesReceived[bytesReceivedCnt++] = byteReceived;
	  bitIdx = BYTE_SIZE;
	  byteReceived = 0;
	}
	if (consecutiveLowCount >= IDLE_CONDITION_MIN_REPEAT) // end of transmission
	{
	  rcvDataBitsFlag = false;
	}
}

/**
 * @brief Manchester state machine.
 *
 * This function implements the Manchester state machine, handling different states during decoding.
 */
void manchesterStateMachine()
{
  switch (currentState)
  {
    case STATE_IDLE:
        stateIdleHandler();
        break;

    case STATE_START_CONDITION:
        // State handeld by GPIO interrupt stateStartCondtionHandler()
        break;

    case STATE_START_BIT_DETECTION:
        
        stateStartBitDetectionHandler();
        break;

    case STATE_DATA_RECEIVING:
        status = stateDataReceivingHandler();
        break;
		
    case STATE_STOP_CONDITION:
      stopConditionHandler();
      break;

    default:
        currentState = STATE_IDLE;
        break;
  }
}

/**
 * @brief Handle the stop condition of the Manchester state machine.
 *
 * This function handles the stop condition of the Manchester state machine, printing the decoded data if using serial communication.
 */

void stopConditionHandler()
{
	if (useSerial)
	{
		switch (status)
		{
			case OK:
				Serial.print("Encoded Data: ");
				for (int i=0; i<decodedDataLen; i++)
				{
					Serial.print(decodedDataResult[i], HEX);
				}
				Serial.println();
				break;
			case ERROR_STANDARD:
				Serial.println("Error: Unknown conversion standard");
				break;
			case ERROR_SIZE:
				Serial.println("Error: Data size not fitting Manchester encoding");
				break;
			case INVALID_ENCODE:
				Serial.println("Error: Data not fitting Manchester encoding");
				break;
			default:
				break;
		}
	}
	currentState = STATE_IDLE;
}

/**
 * @brief Handle the state of receiving data during Manchester decoding.
 *
 * This function handles the state of receiving data during Manchester decoding, and decodes the received data when complete.
 *
 * @return The status of the decoding operation.
 */
enum StatusMessages stateDataReceivingHandler()
{
	status = WORKING;
  if (rcvDataBitsFlag == false)
  {
    currentState = STATE_IDLE;
    // parse data:
    status = decodeManchester(bytesReceived, bytesReceivedCnt, decodedDataResult, STANDARD);
    if (status == OK)
    {
      decodedDataLen = (bytesReceivedCnt / 2);
    }
    currentState = STATE_STOP_CONDITION;
  }
  return status;
}

/**
 * @brief Handle the idle state of the Manchester state machine.
 *
 * This function handles the idle state of the Manchester state machine, waiting for the start condition.
 */
void stateIdleHandler()
{
  if (consecutiveLowCount >= IDLE_CONDITION_MIN_REPEAT)
  {
    attachInterrupt(inputPin, stateStartCondtionHandler, RISING);
    currentState = STATE_START_CONDITION;
  }

}

/**
 * @brief Handle the start condition of the Manchester state machine.
 *
 * This function handles the start condition of the Manchester state machine, transitioning to the start bit detection state.
 */
void stateStartCondtionHandler()
{
    currentState = STATE_START_BIT_DETECTION;
    pinState = 1;
    consecutiveLowCount = 0;
    if (SYNC)
      NRF_TIMER0->CC[0] = GAP_TIME_US + (GAP_TIME_US/2); // move the sample time to middle of the signal - ensure getting correct data
    else
      NRF_TIMER0->CC[0] = GAP_TIME_US;
    
	  NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;
    detachInterrupt(inputPin);
    foundFirstBit = true;
}

/**
 * @brief Handle the start bit detection state of the Manchester state machine.
 *
 * This function handles the start bit detection state of the Manchester state machine, transitioning to the data receiving state when the start bits(0->1) is detected.
 */
void stateStartBitDetectionHandler()
{
    if (!pinState && !foundStartBitZero)
    {
      foundStartBitZero = true;
    }
    else if (pinState && foundStartBitZero)
    {
      foundStartBitZero = false;
      currentState = STATE_DATA_RECEIVING;
      bitIdx = BYTE_SIZE;
      byteReceived = 0;
      bytesReceivedCnt = 0;
	    decodedDataLen = 0;
      rcvDataBitsFlag = true;
    }
}
