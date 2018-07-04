//#define DEBUG

//The const variable declarations help to give meaning to the otherwise arbitrary
// pin assignments.  This abstraction also would help if we ever switched to a different
// Arduino board - we would just have to change the values in this single place, rather
// than everywhere in the entire program.

//Sensor array
const uint8_t SENSOR_0 = A4;    //Left sensor
const uint8_t SENSOR_1 = A1;
const uint8_t SENSOR_2 = A2;
const uint8_t SENSOR_3 = A3;
const uint8_t SENSOR_4 = A0;    //Right sensor

//Left motor
const uint8_t MOTOR_L_EN = 2;
const uint8_t MOTOR_L_A = 7;
const uint8_t MOTOR_L_B = 8;

//Right motor
const uint8_t MOTOR_R_EN = 9;
const uint8_t MOTOR_R_A = 5;
const uint8_t MOTOR_R_B = 6;

//LED
const uint8_t LED = 12;

//Button
const uint8_t BUTTON = 11;

//These are direction constants, used when calling doMoveSpin function.
const uint8_t DIRECTION_LEFT = 0;
const uint8_t DIRECTION_RIGHT = 1;

//These are global variables, which can be accessed from any function in 
// the entire program.  These are used to count time when creating software
// defined PWM waveforms.
uint8_t motorLeftPwmValue = 0x00;
uint8_t motorRightPwmValue = 0x00;
uint16_t ledControlValue = 0x00;

//This function is run once at the beginning of the program.
void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  //Configure left motor for output mode on the three input controls
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_L_A, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);

  //Configure right motor for output mode on the three input controls
  pinMode(MOTOR_R_EN, OUTPUT);
  pinMode(MOTOR_R_A, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);

  //Configure LED in output mode
  pinMode(LED, OUTPUT);

  //Configure button in input mode with internal pullup.  The pinMode call is not 
  // strictly needed as pins are input by default, but good practice nonetheless.
  // The internal pullup is required if you want to reliably read the button presses.
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);

  //Configure sensors in input mode.  Not strictly needed as pins are input 
  // by default, but good practice nonetheless.
  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
}

//This function is run repeatedly in the program.
void loop() {
  //The static keyword on a variable within a function means that the variable will retain its value
  // after exiting and re-executing the function.  This is an alternative to global variables when
  // you don't need to access the variable outside of this specific function.
  static uint8_t state = 0;
  static uint8_t buttonState = 0;
  
  //We need to update the motor PWM every loop.  This function handles timing 
  // and intervals for the two motors and the LED.
  updatePWM();

  //Reset the LED state every time through the loop.  This turns off the LED if the state changes.
  ledControlValue = LOW;
  
  //Read the button - this starts / stops the main program.
  if (digitalRead(BUTTON) == LOW){
      if (buttonState == 255){
        //Do nothing; this happens when we push and hold the button.
      }
      else if (buttonState < 100){
        buttonState++;
      }
      else {
        state ^= 0x01;        //Toggle state - if previously on, turn it off, and vice versa.
        buttonState = 255;    //Set the button to 'hold' mode.  We ignore this until it has been released.
      }
  }
  else {
    buttonState = 0;
  }

  if (state){
    //Use thresholds to construct a single 5 bit number constituting the state of 
    // the sensors. Sensor 0 is the LSB.
    uint8_t state = readSensors();
  
    //This set of if statements contain the core logic of the program.  Each conditional block 
    // checks for a particular state to determine what needs to be done at this time.
    if (state == 0x04 || state == 0x0E){
      //Only the single middle sensor sees the line, or the three middle sensors all see the line.  Stay straight.
      ledControlValue = HIGH;
      doMoveForward(0x0F, 0x0F);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Straight");
      #endif
    }
    else if (state == 0x06){
      //Only the middle sensor and the right-of-middle sensor see the line.  Veer very slightly right.
      ledControlValue = HIGH;
      doMoveForward(0x0F, 0x0E);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Very Slight Right");
      #endif
    }
    else if (state == 0x0C){
      //Only the middle sensor and the left-of-middle sensor see the line.  Veer very slightly left.
      ledControlValue = HIGH;
      doMoveForward(0x0E, 0x0F);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Very Slight Left");
      #endif
    }
    else if ((state & 0x01) == 0x00 && (state & 0x18) == 0x18){
      //The left two sensors are on the line, and the right-most sensor is not - slight left
      ledControlValue = HIGH;
      doMoveForward(0x08, 0x0F);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Slight Left");
      #endif
    }
    else if ((state & 0x03) == 0x03 && (state & 0x10) == 0x00){
      //The right two sensors are on the line, but the left-most sensor is not - slight right
      ledControlValue = HIGH;
      doMoveForward(0x0F, 0x08);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Slight Right");
      #endif
    }
    else if ((state & 0x01) == 0x00 && (state & 0x10) == 0x10){
      //The left sensor is on the line, but the right sensor is not - turn left hard
      ledControlValue = HIGH;
      doMoveForward(0x00, 0x0F);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Left");
      #endif
    }
    else if ((state & 0x01) == 0x01 && (state & 0x10) == 0x00){
      //The right sensor is on the line, but the left sensor is not - turn right hard
      ledControlValue = HIGH;
      doMoveForward(0x0F, 0x00);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Right");
      #endif
    }
    else if ((state & 0x11) == 0x11){
      //Both left and right sensors are on the line - presumably this means the line is
      // perpendicular to the sensor array.  Spin on the spot.
      ledControlValue = 256;
      doMoveSpin(DIRECTION_RIGHT, 0x08, 0x08);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Spin (Perpendicular)");
      #endif    
    }
    else if (state == 0x00){
      //We don't see any portion of the line.  Try moving in a spiral to find it.
      ledControlValue = 1024;
      doMoveForward(0x08, 0x07);
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Circle (See Nothing)");
      #endif
    }
    else {  //Unknown state.  This should not happen in real life when following a line.  If it does, we probably need to add more state checking.
      ledControlValue = 32;
      doStop();
      #ifdef DEBUG
        Serial.print(state, BIN);
        Serial.println(" - Stop (Unknown)");
      #endif
    }
  }
  else {
    ledControlValue = LOW;
    doStop();
  }
}

void doMoveForward(uint8_t leftMotorSpeed, uint8_t rightMotorSpeed){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = (leftMotorSpeed & 0x0F);
  motorRightPwmValue = (rightMotorSpeed & 0x0F);
}

void doMoveSpin(uint8_t spinDirection, uint8_t leftMotorSpeed, uint8_t rightMotorSpeed){
  if (spinDirection == DIRECTION_LEFT){
    digitalWrite(MOTOR_L_A, HIGH);
    digitalWrite(MOTOR_L_B, LOW);
    digitalWrite(MOTOR_R_A, LOW);
    digitalWrite(MOTOR_R_B, HIGH);
  }
  else if (spinDirection == DIRECTION_RIGHT){
    digitalWrite(MOTOR_L_A, LOW);
    digitalWrite(MOTOR_L_B, HIGH);
    digitalWrite(MOTOR_R_A, HIGH);
    digitalWrite(MOTOR_R_B, LOW);
  }
  else {
    doStop();
  }

  motorLeftPwmValue = (leftMotorSpeed & 0x0F);
  motorRightPwmValue = (rightMotorSpeed & 0x0F);
}

//Stop moving completely.
void doStop(){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, LOW);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, LOW);

  motorLeftPwmValue = 0x00;
  motorRightPwmValue = 0x00;
}

//Read the 5 raw values, decode them according to their thresholds, and encapsulate
// them into a single 5 bit unsigned integer.  Sensor 5 is the LSB.
uint8_t readSensors(){
  uint16_t rawValues[5];          //The raw, 10 bit values from analogRead  
  
  rawValues[0] = analogRead(SENSOR_0);
  rawValues[1] = analogRead(SENSOR_1);
  rawValues[2] = analogRead(SENSOR_2);
  rawValues[3] = analogRead(SENSOR_3);
  rawValues[4] = analogRead(SENSOR_4);

#ifdef DEBUG
//  Serial.print(rawValues[0]);
//  Serial.print("\t");
//  Serial.print(rawValues[1]);
//  Serial.print("\t");
//  Serial.print(rawValues[2]);
//  Serial.print("\t");
//  Serial.print(rawValues[3]);
//  Serial.print("\t");
//  Serial.println(rawValues[4]);
#endif
  
  uint8_t result = 0;
  
  if (rawValues[0] > 150){
    result |= 0x10;
  }
  if (rawValues[1] > 150){
    result |= 0x08;
  }
  if (rawValues[2] > 150){
    result |= 0x04;
  }
  if (rawValues[3] > 150){
    result |= 0x02;
  }
  if (rawValues[4] > 150){
    result |= 0x01;
  }

  return result;
}

void updatePWM(){
  //Get the 5 LSB of the millis value
  uint8_t millisLSB = (millis() & 0x0F);

#ifdef DEBUG
//  Serial.print(motorLeftPwmValue, HEX);
//  Serial.print("\t");
//  Serial.println(motorRightPwmValue, HEX);
#endif

  //The 5 LSB of the motor set values are compared with the 5 LSB of the millis value.  This is
  // a poor man's PWM approximation.  It will not give a proper PWM waveform, as it will sometimes
  // 'miss' a value, but it is close enough for rough motor control.
  digitalWrite(MOTOR_L_EN, (motorLeftPwmValue > millisLSB));
  digitalWrite(MOTOR_R_EN, (motorRightPwmValue > millisLSB));

  //The LED works a bit differently than the motors... if you set the value to 0 (LOW) or 1 (HIGH)
  // it is always on or always off.  Otherwise, we use the control value as the PWM period (in ms), 
  // and assume a 50% duty cycle.  For this to work, we always need to set the control value to be
  // a power of two (e.g. 32, 128, 256, 512, 1024, etc).
  uint16_t ledMillisLSB = (millis() & ledControlValue);
  if (ledControlValue == LOW){
    digitalWrite(LED, LOW);
  }
  else if (ledControlValue == HIGH){
    digitalWrite(LED, HIGH);
  }  
  else {
    digitalWrite(LED, (ledControlValue / 2) >= ledMillisLSB);
  }
}

