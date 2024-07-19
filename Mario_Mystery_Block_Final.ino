//Include required libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SimpleTimer.h>
#include <Servo.h>
#include <AccelStepper.h>
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#include <FastLED.h>

#define OUTPUT_READABLE_REALACCEL

//Debug
#define DEBUG true

//Define Pin Lications
#define INTERRUPT_PIN 2         // use pin 2 on Arduino Uno & most boards /*Change to the pin that you will use on your arduino*/
#define LED_PIN 13              // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define HEARTBEAT_PIN 3         // Use pin 3 for heartbeat to verify Arduino is working
#define STEPPER_PIN_STEP 26     // STEP pin
#define STEPPER_PIN_DIR 24      // DIR pin
#define STEPPER_PIN_ENABLE 25   // ENABLE pin (optional, set to -1 if not used)
#define HOMING_SWITCH_PIN 5   //Homing Limit Switch
#define LOAD_SWITCH_PIN 7     //Loaded Limit Switch
#define SERVO_SIGNAL_PIN 9      //Define our servo signal pin. Must use a PWM pin (OC2B)
#define LED_PIN     32 //used both pin 5 and 32

#define STEPS_PER_REVOLUTION 200  // Steps per revolution of your stepper motor
#define NUM_LEDS    71
#define BRIGHTNESS  200
#define LED_TYPE    SK6812
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

#define STATE_WAITING_FOR_PUNCH 1
#define STATE_START_PUNCH 2

//#define STATE_DELAY_AFTER_PUNCH 100
#define STATE_PUNCH_RESET_INIT 100
#define STATE_PUNCH_RESET 101
#define STATE_PUNCH_LATCH 102
#define STATE_PUNCH_LOAD 103
#define STATE_IDLE 104
#define STATE_PUNCH_INIT 105
#define STATE_PUNCH_EXECUTE 106
#define STATE_PUNCH__POST_EXECUTE 107
//#define TEST_STEPPER_MOTOR 200


//Create peripheral objects
MPU6050 mpu;
Servo servoMotor;
AccelStepper stepper(1, STEPPER_PIN_STEP, STEPPER_PIN_DIR);
DFRobotDFPlayerMini myDFPlayer;
CRGB leds[NUM_LEDS];

//Declare functions
void switchLED(bool on);

//Declare global variables
uint16_t stepperLocation;
bool blinkState = false;
bool punchState = false;
uint8_t state;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

//Setup timers
SimpleTimer heartbeatTimer(1000);
SimpleTimer punchTimer(3000);
SimpleTimer programTimers[10];

//Setup constants
uint16_t ACCELEROMETER_TRIGGER = 50;
int16_t STEPPER_SPEED = 400;
int16_t STEPPER_ACCELERATION = 1000;
uint16_t SERVO_OPEN_LATCH_ANGLE = 45;
uint16_t SERVO_CLOSE_LATCH_ANGLE = 0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {


    delay(3000); //Delay 3 seconds upon starting up
    Serial.begin(115200); //Setup serial port for debugging
    /* Setup Peripheral Devices */

    //Servo Motor
    servoMotor.attach(SERVO_SIGNAL_PIN); //Assign the PWM pin to the servoMotor object
    servoMotor.write(SERVO_OPEN_LATCH_ANGLE); //Have to do this immediately. If you initialize the pin and don't do this, it tries to pull to a random position

    //Limit Switches
    pinMode(HOMING_SWITCH_PIN, INPUT_PULLUP);
    pinMode(LOAD_SWITCH_PIN, INPUT_PULLUP);
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(HEARTBEAT_PIN, OUTPUT);


    //Setpper Motor
      stepper.setMaxSpeed(STEPPER_SPEED);      // Adjust to your motor's max speed
  stepper.setAcceleration(STEPPER_ACCELERATION);  // Acceleration in steps per second per second
    pinMode(STEPPER_PIN_ENABLE, OUTPUT);
    digitalWrite(STEPPER_PIN_ENABLE, HIGH); // LOW enables the motor, HIGH disables (if using ENABLE pin)


    //Sound - DF1 Mini
    Serial1.begin(9600);
    if(DEBUG){
      //Serial.println();
      //Serial.println(F("DFRobot DFPlayer Mini Demo"));
      //Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
    }
    
    
    if (!myDFPlayer.begin(Serial1, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
      if(DEBUG){
        Serial.println(F("Unable to begin:"));
        Serial.println(F("1.Please recheck the connection!"));
        Serial.println(F("2.Please insert the SD card!"));
      }
      while(true){
        delay(0); // Code to compatible with ESP8266 watch dog.
      }
    }
    if(DEBUG){
      Serial.println(F("DFPlayer Mini online."));
    }
    
    myDFPlayer.volume(30);

    //Lights
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );

    //Accelerometer Setup
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setWireTimeout(3000, true); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

        devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
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

    delay(1000);

  /** Set Peripherals to initial state **/
    //Servo Motor
    servoMotor.write(SERVO_OPEN_LATCH_ANGLE);

    //Play Sound
    myDFPlayer.playFolder(01, 006);
    delay(3000);
    myDFPlayer.playFolder(01, 010);

    //Turn off lights
    switchLED(false);

    //Setup timers
    programTimers[2].setInterval(1000); //Reset Init
    programTimers[3].setInterval(1000); //Timer before latch
    programTimers[4].setInterval(2000); //Timer after latch
    programTimers[5].setInterval(1); //Timer after punch
    programTimers[6].setInterval(500); //Timer after release
    programTimers[7].setInterval(1000); //Timer after release

  /** Reset Timer **/
    programTimers[2].reset();

    delay(9000);
    //Go to start state
    state = STATE_PUNCH_RESET_INIT;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    //if (!dmpReady) return;

    bool homingSwitchPin = digitalRead(HOMING_SWITCH_PIN) == LOW;
    bool loadSwitchPin = digitalRead(LOAD_SWITCH_PIN) == LOW;


    if(heartbeatTimer.isReady()){
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(HEARTBEAT_PIN, blinkState);
        heartbeatTimer.reset();
    }

    if(state == STATE_PUNCH_RESET_INIT){
      if(programTimers[2].isReady()){
        state = STATE_PUNCH_RESET;
      }
    }

    if(state == STATE_PUNCH_RESET){
      /* Move stepper forward until homing limit switch has been reached */
      if(!homingSwitchPin){
        // Move the stepper forward
        stepper.setSpeed(-1 * STEPPER_SPEED); // Set a constant speed (steps per second)
        stepper.runSpeed(); // Run the motor at the set speed
        } else{
        //Stop the motor if no button is pressed
          stepper.setSpeed(0);
          state = STATE_PUNCH_LATCH;
          programTimers[3].reset();
        }
    }

    if(state == STATE_PUNCH_LATCH){
      if(programTimers[3].isReady()){
        servoMotor.write(SERVO_CLOSE_LATCH_ANGLE);
        myDFPlayer.playFolder(01, 004);
        state = STATE_PUNCH_LOAD;
        programTimers[4].reset();
      }

    }

    if(state == STATE_PUNCH_LOAD){     
      if(programTimers[4].isReady()){
          servoMotor.detach();
          if(!loadSwitchPin){
            // Move the stepper forward
            stepper.setSpeed(STEPPER_SPEED); // Set a constant speed (steps per second)
            stepper.runSpeed(); // Run the motor at the set speed
          }else{
            //Stop the motor if no button is pressed
            //Stepper is loaded
              stepper.setSpeed(0);
              state = STATE_IDLE;
          }
        }
    }

    if(state == STATE_IDLE){
      switchLED(true);
      if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            if(aaReal.z > 0 && aaReal.z > ACCELEROMETER_TRIGGER){ //May need to store values and wait //Edge case where it was reading a negative value, so checking to make sure it is greater than 0
              state = STATE_PUNCH_INIT; 
              programTimers[5].reset();
              if(DEBUG){
                // Serial.println(aaReal.z);
              }
            }
        #endif
      }
    }

    if(state == STATE_PUNCH_INIT){
      if(programTimers[5].isReady()){
          myDFPlayer.playFolder(01, 007);
          state = STATE_PUNCH_EXECUTE;
          programTimers[6].reset();
        }
    }

    if(state == STATE_PUNCH_EXECUTE){
      if(programTimers[6].isReady()){
          servoMotor.attach(SERVO_SIGNAL_PIN); //Assign the PWM pin to the servoMotor object
          servoMotor.write(SERVO_OPEN_LATCH_ANGLE);
          //servoMotor.detach();
          state = STATE_PUNCH__POST_EXECUTE;
          programTimers[7].reset();
        }
    }

    if(state == STATE_PUNCH__POST_EXECUTE){
      switchLED(false);
      if(programTimers[7].isReady()){
          myDFPlayer.playFolder(01, 002);
          state = STATE_PUNCH_RESET_INIT;
          programTimers[2].reset();
        }
    }
}

void switchLED(bool on){
  if(on){
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i % 1 == 0) {
        if(i >= 52){ //Turn off specific LEDs due to proximity to edge
          if(i <= 53){ //Turn off specific LEDs due to proximity to edge
            leds[i] = CRGB::Black;
          }else{
            leds[i] = CRGB::White;
          }
        }else{
          leds[i] = CRGB::White;
        }
      } else {
        leds[i] = CRGB::Black;
      }
    }

    // Show the updated LED colors
    FastLED.show();
  }else{
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }

    // Show the updated LED colors
    FastLED.show();
  }
}
