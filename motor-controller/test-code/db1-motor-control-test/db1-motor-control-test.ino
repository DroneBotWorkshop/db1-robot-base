/*
  DB1 Robot Project - Motor Controller Test
  db1-motor-control-test.ino
  Controls single DC Gearmotor with Rotary Encoder
  Motor driven with Cytron MD10C Motor Driver

  Basic Emergency Stop Functionality Test
  Emergency Stop on INT1

  Version 0.43
  Updated 2020-04-30

  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/





// Robot-Specific Options
// Change as required

// Motor Parameters (change as required)
// Motor rotary encoder output pulses per rotation
float count_rev = 374.32;

// Wheel Parameters
// Wheel diameter in millimeters, change if different
float wheel_diameter = 101.60;

// I2C Address List
int i2cAddr0 = 9;		// I2C Address 0
int i2cAddr1 = 10;		// I2C Address 1
int i2cAddr2 = 11;		// I2C Address 2
int i2cAddr3 = 12;		// I2C Address 3

// PWM Frequency Divisors
int pwmDiv0 = 1;
int pwmDiv1 = 8;


// Pin number definitions
// Arduino Nano or ATMega328 design

// Rotary Encoder output to Arduino Interrupt pin 2 (ATMega328 pin 4) - INT 0
#define ENC_IN 2

// Emergency Stop to Arduino Interrupt pin 3 (ATMega328 pin 5) - INT 1
#define EM_STOP 3

// MD10C PWM connected to Arduino pin 11 (ATMega328 pin 17)
#define MD10C_PWM 11

// MD10C DIR connected to Arduino pin 12 (ATMega328 pin 18)
#define MD10C_DIR 12

// I2C Address Select connected to Arduino pins 4 & 5 (ATMega328 pins 6 & 11)
#define I2C_ADDR0 4
#define I2C_ADDR1 5

// PWM Frequency Select connected to Arduino pin 6 (ATMega328 pin 12)
#define PWM_FREQ 6

// Status LED Output connected to Arduino pin 13 (ATMega328 pin 19)
#define LED_STATUS 13

// Processor Status Output connected to Arduino pin 7 (ATMega328 pin 13)
#define PROC_STATUS 7

// Emergency Stop LED Output connected to Arduino pin 10 (ATMega328 pin 16)
#define LED_EM_STOP 10

// Emergency Stop Inputs (Analog Inputs used as Digital Inputs)
#define EM_STOP_0 8 	// Arduino pin 8 (ATMega328 pin 14)
#define EM_STOP_1 9 	// Arduino pin 9 (ATMega328 pin 15)		
#define EM_STOP_2 A0 	// Arduino pin A0 (ATMega328 pin 23)
#define EM_STOP_3 A1 	// Arduino pin A1 (ATMega328 pin 24)
#define EM_STOP_4 A2 	// Arduino pin A2 (ATMega328 pin 25)
#define EM_STOP_5 A3 	// Arduino pin A3 (ATMega328 pin 26)


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
volatile int motorPwm = 0;

// Variable for motor direction
int motorDir = 1;

// Variable for Motor Status
// 0 = Busy
// 1 = Ready
// 8 = Stop
// 9 = Emergency Stop
volatile int motorStatus = 0;

// Variable for I2C final address
int i2cAddr;


/*
	Function Status

	 CMtoSteps - Convert from centimeters to steps - DONE
	 clearStop - Clears Emergency Stop condition - FUNCTIONAL
	 getStatus - Returns current motor and controller status - NOT CODED
	 motorAccel - Accelerates motor between two speeds - DONE
	 motorDecel - Decelerates motor between two speeds - DONE
	 motorMove - Moves motor at desired speed - DONE
	 motorRPM - Reads encoder, returns speed in RPM - NOT CODED
	 moveDistance - Move robot platform a specified distance in CM - NOT CODED
	 movePWM - Moves motor at speed specified by PWM value - DONE
	 moveRPM - Moves motor at speed specified by RPM value - NOT CODED
	 readRPM - Reads motor speed in RPM - NOT CODED
	 setI2C - Select I2C address based upon state of two I2C Address Pins - DONE
	 setPWM - Select motor PWM Frequency based upon state of PWM FREQ Pin - DONE
	 setPwmFrequency - Sets PWM frequency using a divisor - DONE
	 stopMotor - Stops motor - DONE

*/


// Interrupt Service Routine
// Rotary Encoder input
// Interrupt 0

void isr_rotencode() {

}


// Interrupt Service Routine
// Emergency Stop input
// Interrupt 1

void isr_emstop() {
  // Set motor speed to zero
  motorPwm = 0;
  // Stop Motor by sending PWM signal low
  digitalWrite(MD10C_PWM, 0);

  // Set Status
  motorStatus = 9;

}


// CMtoSteps Function
// Convert from centimeters to steps

int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = (wheel_diameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / count_rev;  // CM per Step

  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)

  return result;  // End and return result

}


// clearStop Function (I2C Command)
// Clears Emergency Stop condition

void clearStop()
{
  // Reset Motor Speed variable
  motorPwm = 0;

  // Write values to Motor Driver
  analogWrite(MD10C_PWM, motorPwm);

  // Set Status to Ready
  motorStatus = 1;
}


// getStatus Function (I2C Command)
// Returns current motor and controller status

void getStatus()
{

}


// motorAccel Function
// Accelerates motor between two speeds
// Required Low Speed and High Speed
// Optional Period of acceleration

void motorAccel(int mspeedlow, int mspeedhigh, int per = 1000) {

  // Determine the increment
  long incr;
  if (mspeedhigh > mspeedlow) {
    incr = per / (mspeedhigh - mspeedlow);
  } else {
    incr = 1;
  }

  for (int i = mspeedlow; i <= mspeedhigh; i++) {
    currentMillis = millis();
    while (millis() < currentMillis + incr) {
      // Drive motor for period in ms
      motorMove(i);
    }
  }
}


// motorDecel Function
// Decelerates motor between two speeds
// Required Low Speed and High Speed
// Optional Period of deceleration

void motorDecel(int mspeedlow, int mspeedhigh, int per = 1000) {

  // Determine the increment
  long incr;
  if (mspeedhigh > mspeedlow) {
    incr = per / (mspeedhigh - mspeedlow);
  } else {
    incr = 1;
  }

  for (int i = mspeedhigh; i >= mspeedlow; i--) {
    currentMillis = millis();
    while (millis() < currentMillis + incr) {
      // Drive motor for period in ms
      motorMove(i);
    }
  }
}


// motorMove Function
// Moves motor at desired speed
// Replaces analogWrite function by checking status

void motorMove(int mtrspeed) {
  // Check motor status
  if (motorStatus == 9) {
    digitalWrite(MD10C_PWM, 0);
  } else {
    analogWrite(MD10C_PWM, mtrspeed);
  }

}


// motorRPM Function
// Reads encoder, returns speed in RPM

int motorRPM(int encpulse)
{

}


// moveDistance Function (I2C Command)
// Move robot platform a specified distance in CM
// Required Distance and Direction parameters
// Optional PWM Speed parameter - 0 to 255
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)

void moveDistance(int dis, int dr, int spp = 255, int acl = 0, int dcl = 0)
{

  // Set Status to Busy
  if (motorStatus == 9) {
    motorStatus = 9;
  } else {
    motorStatus = 0;
  }

  // Keep Direction between 0 and 1
  if (dr > 1) dr = 1;
  if (dr < 0) dr = 0;

  // Keep acceleration between 0 and 1 minute
  if (acl > 60000) acl = 60000;
  if (acl < 0) acl = 0;

  // Keep deceleration between 0 and 1 minute
  if (dcl > 60000) dcl = 60000;
  if (dcl < 0) dcl = 0;




  // Set Status to Ready
  if (motorStatus == 9) {
    motorStatus = 9;
  } else {
    motorStatus = 1;
  }
}


// movePWM Function (I2C Command)
// Moves motor at speed specified by PWM value
// Required Speed and Direction parameters
// Optional Run Time parameter in milliseconds - 1 ms to 60 seconds(0 = forever)
// Optional Acceleration parameter in milliseconds  - 1 ms to 60 seconds(0 = none)
// Optional Deceleration parameter in milliseconds  - 1 ms to 60 seconds(0 = none)

void movePWM(int spp, int dr, int tm = 60000, int acl = 0, int dcl = 0)
{

  // Set Status to Busy
  if (motorStatus == 9) {
    motorStatus = 9;
  } else {
    motorStatus = 0;
  }

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
    motorMove(spp);
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
    motorMove(spp);

  }

  // Set Status to Ready
  if (motorStatus == 9) {
    motorStatus = 9;
  } else {
    motorStatus = 1;
  }

}


// moveRPM Function (I2C Command)
// Moves motor at speed specified by RPM value
// Required Speed and Direction parameters
// Optional Run Time parameter - 1 ms to 60 seconds(0 = forever)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)

void moveRPM(int spr, int dr, int tm = 60000, int acl = 0, int dcl = 0)
{

  // Set Status to Busy
  //motorStatus = 0;

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



  // Set Status to Ready
  if (motorStatus == 9) {
    motorStatus = 9;
  } else {
    motorStatus = 1;
  }

}


// readRPM Function (I2C Command)
// Reads motor speed in RPM

void readRPM()
{

}


// setI2C Function
// Select I2C address based upon state of two I2C Address Pins
// Input four I2C Addresses
// Returns selected I2C Address

int setI2C(int addr0, int addr1, int addr2, int addr3) {

  int i2cpinstate0;  // State of I2C address select pin 0
  int i2cpinstate1;  // State of I2C address select pin 1
  int i2caddress;  // Final I2C address

  i2cpinstate0 = digitalRead(I2C_ADDR0); //Read I2C address select pin 0
  i2cpinstate1 = digitalRead(I2C_ADDR1); //Read I2C address select pin 1

  if (i2cpinstate0 == 0 & i2cpinstate1 == 0 ) {
    i2caddress = addr0;
  } else if (i2cpinstate0 == 1 & i2cpinstate1 == 0 ) {
    i2caddress = addr1;
  } else if (i2cpinstate0 == 0 & i2cpinstate1 == 1 ) {
    i2caddress = addr2;
  } else if (i2cpinstate0 == 1 & i2cpinstate1 == 1 ) {
    i2caddress = addr3;
  } else {
    i2caddress = addr0;
  }

  return i2caddress; //End and return I2C address
}


// setPWM Function
// Select motor PWM Frequency based upon state of PWM FREQ Pin
// Input two PWM Frequency Divisors
// Calls setPwmFrequency
// Sets frequency and returns selected PWM Divisor

int setPWM(int div0, int div1) {

  int pwmpinstate;  // State of PWM Frequency select pin
  int pwmdiv;			// Final divisor for PWM frequency

  pwmpinstate = digitalRead(PWM_FREQ); //Read I2C address select pin 0

  if (pwmpinstate == 0  ) {
    pwmdiv = div0;
  } else {
    pwmdiv = div1;
  }

  // Set the PWM frequency based upon the selected divisor
  setPwmFrequency(MD10C_PWM, pwmdiv);

  return pwmdiv; //End and return selected divisor

}


// setPwmFrequency Function
// Sets PWM frequency using a divisor

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


// stopMotor Function (I2C Command)
// Stops motor
// Optional Decelation parameter

void stopMotor(int dcl = 0)
{

  // Set Status to Busy
  //motorStatus = 0;

  // Get current motor PWM speed
  int spp = motorPwm;

  // Keep deceleration between 0 and 1 minute
  if (dcl > 60000) dcl = 60000;
  if (dcl < 0) dcl = 0;

  // Check for deceleration
  if (dcl > 0) {
    motorDecel(0, spp, dcl);
  }


  // Write values to motor driver to stop motor
  digitalWrite(MD10C_PWM, 0);

  // Reset Motor Speed variable
  motorPwm = 0;


  // Set Status to Stop
  if (motorStatus == 9) {
    motorStatus = 9;
  } else {
    motorStatus = 8;
  }
}





void setup() {

  // Setup Serial Monitor for debugging
  Serial.begin(9600);

  // Setup Pins as required

  // Set Cytron Motor pins as Outputs
  pinMode(MD10C_PWM, OUTPUT);
  pinMode(MD10C_DIR, OUTPUT);

  // Set encoder as an Input with internal pullup
  pinMode(ENC_IN, INPUT_PULLUP);

  // Set emergency stop as an Input with internal pullup
  pinMode(EM_STOP, INPUT_PULLUP);

  // Set Emergency Stop pins as Inputs with internal pullups
  pinMode(EM_STOP_0, INPUT_PULLUP);
  pinMode(EM_STOP_1, INPUT_PULLUP);
  pinMode(EM_STOP_2, INPUT_PULLUP);
  pinMode(EM_STOP_3, INPUT_PULLUP);
  pinMode(EM_STOP_4, INPUT_PULLUP);
  pinMode(EM_STOP_5, INPUT_PULLUP);

  // Set I2C Address Select pins as Inputs
  pinMode(I2C_ADDR0, INPUT);
  pinMode(I2C_ADDR1, INPUT);

  // Set PWM Address Select pin as Input
  pinMode(PWM_FREQ, INPUT);

  // Set Status and EM Stop LEDs as Outputs
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_EM_STOP, OUTPUT);

  // Set Processor Status as Output
  pinMode(PROC_STATUS, OUTPUT);


  // Set the PWM Frequency
  setPwmFrequency(MD10C_PWM, 8);
  //setPwmFrequency(MD10C_PWM, 1);

  //Determine I2C Slave Address
  //i2cAddr = setI2C(i2cAddr0,i2cAddr1,i2cAddr2,i2cAddr3);

  //Attach Emergency Stop Interrupt
  attachInterrupt(1, isr_emstop, FALLING);


  // Set Motor Status to Ready
  motorStatus = 1;


}

void loop() {


  movePWM(150, 0, 10000, 5000, 3000);
  delay(2000);
  movePWM(250, 1, 8000, 5000, 3000);
  delay(2000);


}
