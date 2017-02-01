#include "I2Cdev.h"
#include "Arduino.h"
#include "digitalWriteFast.h"
#include "Utility.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define INTERRUPT_PIN 3
#define PWM_PIN 5
#define DIR_PIN 6
#define MAX_VOLTAGE 12
#define MIN_VOLTAGE 1
#define ENCODER_INTERRUPT 2
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 4
#define ENCODER_REVERSED

float robotAngle;
float robotAngleRef;
float robotAngleError;
float Kp = 100;
float Kd = 0.2;
float torque;
unsigned long tPeriod = 0;
unsigned long tOld = 0;
float kMotor = 0.2;
float motorVoltage = 0;
float motorSpeed = 0;
volatile bool encoderSet;
volatile long encoderTicks = 0;
long encoderTicksOld = 0;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void runMotor(float voltageInput)
{ 
  	int voltageSign = sign(voltageInput);
  	if(voltageSign == -1)
  	{
    	voltageSign = 0;
  	}

  	voltageInput = constrain(voltageInput,MIN_VOLTAGE,MAX_VOLTAGE)*10;		// multiply by 10 to keep higher resolution
  	int motorPWM = map((int)voltageInput,MIN_VOLTAGE*10,MAX_VOLTAGE*10,0,255);
    digitalWrite(DIR_PIN, voltageSign);
    analogWrite(PWM_PIN, motorPWM);
}

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
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
  	pinMode(ENCODER_PIN_A, INPUT);
  	digitalWrite(ENCODER_PIN_A, LOW);
  	pinMode(ENCODER_PIN_B, INPUT);
  	digitalWrite(ENCODER_PIN_B, LOW);
  	attachInterrupt(ENCODER_INTERRUPT, HandleMotorInterruptA, RISING);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   	Serial.begin(115200);

    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(2);

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    tOld = millis();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        robotAngle = ypr[1];

       	robotAngleError = robotAngleRef - robotAngle;

        tPeriod = millis() - tOld;
        tOld = millis();

        torque = Kp*robotAngleError + Kd*gyro[1];

        motorSpeed = (encoderTicks - encoderTicksOld)/tPeriod/8;
        encoderTicksOld = encoderTicks;

        motorVoltage = torque + kMotor*motorSpeed;

        Serial.print("data\t");
        Serial.print(robotAngleError);
        Serial.print("\t");
        Serial.println(motorVoltage);
        
        runMotor(3);
    }
}

