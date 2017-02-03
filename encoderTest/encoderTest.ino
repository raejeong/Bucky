#include "Arduino.h"
#include "digitalWriteFast.h"

#define ENCODER_INTERRUPT 0
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 4
#define ENCODER_REVERSED

volatile bool encoderSet;
volatile long encoderTicks = 0;
volatile long encoderTicksOld = 0;
unsigned long tPeriod = 0;
unsigned long tOld = 0;
float motorSpeed = 0;

void HandleMotorInterruptA()
{
  encoderSet = digitalReadFast(ENCODER_PIN_B);
  #ifdef ENCODER_REVERSED
    encoderTicks -= encoderSet ? -1 : +1;
  #else
    encoderTicks += encoderSet ? -1 : +1;
  #endif
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
  	pinMode(ENCODER_PIN_A, INPUT);
  	digitalWrite(ENCODER_PIN_A, LOW);
  	pinMode(ENCODER_PIN_B, INPUT);
  	digitalWrite(ENCODER_PIN_B, LOW);
  	attachInterrupt(ENCODER_INTERRUPT, HandleMotorInterruptA, RISING);

   	Serial.begin(115200);

    tOld = millis();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    tPeriod = millis() - tOld;
    tOld = millis();

    motorSpeed = (encoderTicks - encoderTicksOld)*1000*2*3.14/(tPeriod*7.0);
   
    Serial.println(motorSpeed);

    encoderTicksOld = encoderTicks;


    delay(100);
}


