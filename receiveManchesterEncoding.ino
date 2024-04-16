#include "manchesterStateMachine.h"


#define INPUT_PIN 2 // make sure to not use pin 6 when using Serial.print.
#define OUTPUT_PIN 3 // make sure to not use pin 8 when using Serial.print.
#define USE_SERIAL true

void setup() {
  setOutputSerial(USE_SERIAL);
  setInputPin(INPUT_PIN);
  setOutputPin(OUTPUT_PIN);
  setupManchesterTimerIRQ();
  attachInterrupt(INPUT_PIN, stateStartCondtionHandler, RISING);

}


void loop() {
  manchesterStateMachine();  
}
