/*
  DB1 Robot Project - Motor Controller
  db1-motor-control-movePWM.ino
  Tests movePWM function
  Development for db1-motor-control

  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/

// Arduino Pin definitions

// Encoder output to Arduino Interrupt pin 2 - INT 0
#define ENC_IN 2

// Emergency Stop to Arduino Interrupt pin 3 - INT 1
#define EM_STOP 3

// MD10C PWM connected to pin 10
#define MD10C_PWM 10

// MD10C DIR connected to pin 12
#define MD10C_DIR 12

// I2C Address Select connected to pin 4
#define I2C_ADDR 4

// I2C Slave Address Selection

int i2c0 = 9;
int i2c1 = 10;

// Setup Robot Parameters

// Motor Parameters (change as required)
// Motor encoder output pulse per rotation
float count_rev = 374.32;

// Wheel Parameters
// Wheel diameter in millimeters, change if different
float wheel_diameter = 101.60;

// Operational Variables

// Pulse count from encoder
volatile long encoderValue = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measurement
int rpm = 0;

// Variable for PWM motor speed output
int motorPwm = 0;

// Variable for motor direction
int motorDir = 1;

// Variable for I2C address
int i2cAddr;

// Variable for Motor Status
// 0 = Busy
// 1 = Ready
// 8 = Stop
// 9 = Emergency Stop
int motorStatus = 0;

// motorAccel Function
// Accelerates motor between two speeds
// Required Low Speed and High Speed
// Optional Period of deceleration

void motorAccel(int mspeedlow, int mspeedhigh, int per = 100) {

  while (motorStatus < 2) {
  
  
  // Determine the increment
  long incr;
  if (mspeedhigh > mspeedlow){
  incr = per / (mspeedhigh - mspeedlow);
  } else {
  incr = 1;
  }
  
    for (int i = mspeedlow; i <= mspeedhigh; i++) {
      currentMillis = millis();
      analogWrite(MD10C_PWM, i);
      while (millis() < currentMillis + incr) {
        //wait for period in ms
      };
    }
  }

}


// motorDecel Function
// Decelerates motor between two speeds
// Required Low Speed and High Speed
// Optional Period of deceleration

void motorDecel(int mspeedlow, int mspeedhigh, int per = 5) {

  while (motorStatus < 2) {
    for (int i = mspeedhigh; i >= mspeedlow; i--) {
      currentMillis = millis();
      analogWrite(MD10C_PWM, i);
      while (millis() < currentMillis + per) {
        //wait for period in ms
      }
    }
  }
}

// movePWM Function (I2C Command)
// Moves motor at speed specified by PWM value
// Required Speed and Direction parameters
// Optional Run Time parameter - 1 ms to 60 seconds(0 = forever)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)
// Optional Deceleration parameter - 1 ms to 60 seconds(0 = none)

void movePWM(int spp, int dr, int tm = 60000, int acl = 0, int dcl = 0)
{
  while (motorStatus < 2) {

    // Set Status to Busy
    motorStatus = 0;

    // Keep PWM speed between 0 and 255
    if (spp > 255) spp = 255;
    if (spp < 0) spp = 0;

    // Keep Direction between 0 and 1
    if (dr > 1) dr = 1;
    if (dr < 0) dr = 0;

    // Keep run time between 0 and 1 minute
    if (tm > 60000) tm = 60000;
    if (tm < 0) tm = 0;

    // Keep acceleration between 0 and 1 minute
    if (acl > 60000) acl = 60000;
    if (acl < 0) acl = 0;

    // Keep deceleration between 0 and 1 minute
    if (dcl > 60000) dcl = 60000;
    if (dcl < 0) dcl = 0;

    // Set Motor Direction
    digitalWrite(MD10C_DIR, dr);

    // Check if there is Acceleration specified
    // If Yes then call accelerate function
    if (acl > 0) {
      motorAccel(0, spp, acl);
      // Update the motor speed variable
      motorPwm = spp;
    }

    // Check if Run Time specified
    // If Yes run for desired time and check for deceletation
    // If no then run forever
    if (tm > 0) {
      // Run for specified period
      currentMillis = millis();
      analogWrite(MD10C_PWM, spp);
      // Update the motor speed variable
      motorPwm = spp;
      while (millis() < currentMillis + tm) {
        //wait for time in ms
      }

      // Check for Deceleration
      if (dcl > 0) {
        // Call decelerate function
        motorDecel(0, spp, dcl);
        // Update the motor speed variable
        motorPwm = 0;
      } else {
        // Stop the motor
        analogWrite(MD10C_PWM, 0);
        // Update the motor speed variable
        motorPwm = 0;
      }

    } else {

      // Write speed to Motor Driver
      analogWrite(MD10C_PWM, spp);

    }

    // Set Status to Ready
    motorStatus = 1;

  }

}


void setup() {


  // Set the PWM Frequency
  setPwmFrequency(MD10C_PWM, 8);


  // Define Pins
  // Set Cytron Motor pins as Outputs
  pinMode(MD10C_PWM, OUTPUT);
  pinMode(MD10C_DIR, OUTPUT);
  // Set I2C Address Select pin as Input
  pinMode(I2C_ADDR, INPUT);
  // Set encoder as input with internal pullup
  pinMode(ENC_IN, INPUT_PULLUP);
  // Set emergency stop as input with internal pullup
  pinMode(EM_STOP, INPUT_PULLUP);

  //Determine I2C Slave Address
  i2cAddr = setI2C();

  // Set Motor Status to Ready
  motorStatus = 1;
  
}

void loop() {

	motorDir = 1;
	motorPwm = 200;

	// Write speed to Motor Driver
      analogWrite(MD10C_PWM, motorPwm);
    // Write direction to Motor Driver
      analogWrite(MD10C_DIR, motorDir);
      
    delay(2000);
    
    motorDir = 0;
    motorPwm = 200;

	// Write speed to Motor Driver
      analogWrite(MD10C_PWM, motorPwm);
    // Write direction to Motor Driver
      analogWrite(MD10C_DIR, motorDir);
      
    delay(2000);
    
    motorPwm = 0;
    
    motorAccel(0, 255, 200);
    
    delay(2000);
    
    motorDecel(0, 255, 200);
    
    delay(2000);
    

}
