#define DEBUG

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

//LEDs
const uint8_t LED1 = 12;
const uint8_t LED2 = 13;

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
uint16_t led1ControlValue = 0x00;
uint16_t led2ControlValue = 0x00;

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

  //Configure LEDs in output mode
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

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
  static uint32_t lastTime = 0;

  //We need to update the motor PWM every loop.  This function handles timing 
  // and intervals for the two motors and the LED.
  updatePWM();

  //Reset the LED state every time through the loop.  This turns off the LED if the state changes.
  led1ControlValue = LOW;
  led2ControlValue = LOW;

  //Use thresholds to construct a single 5 bit number representing the state of all 5 sensors.
  uint8_t sensorState = readSensors();
  
  //Read and debounce the button - this allows us to start / stop the main code.
  if (digitalRead(BUTTON) == LOW){
      if (buttonState == 255){
        //Do nothing; this happens when we push and hold the button.
      }
      else if (buttonState < 8){
        buttonState++;
      }
      else {
        state ^= 0x01;        //Toggle state - if previously on, turn it off, and vice versa.
        buttonState = 255;    //Set the button to 'hold' mode.  We ignore this until it has been released.
        #ifdef DEBUG
          Serial.println("Button Press");
        #endif        
      }
  }
  else {
    buttonState = 0;
  }

  if (state){
    if (millis() - lastTime > 10){
      lastTime = millis();
      
      //This set of if statements contain the core logic of the program.  Each conditional block 
      // checks for a particular state to determine what needs to be done at this time.
      if (sensorState == 0x0C){
        //Only the middle sensor and the left-of-middle sensor see the line.  Slight left.
        led1ControlValue = HIGH;
        led2ControlValue = 2;
        doMoveForward(0x05, 0x06);
        #ifdef DEBUG
          Serial.println("L1");
        #endif
      }
      else if (sensorState == 0x06){
        //Only the middle sensor and the right-of-middle sensor see the line.  Slight right.
        led1ControlValue = HIGH;
        led2ControlValue = 2;
        doMoveForward(0x06, 0x05);
        #ifdef DEBUG
          Serial.println("R1");
        #endif
      }
      else if ((sensorState & 0x0E) != 0x00 && (sensorState & 0x11) == 0x00){
        //At least one of the three middle sensors see the line and the two outside ones do not.  Stay straight.
        led1ControlValue = HIGH;
        led2ControlValue = LOW;
        doMoveForward(0x07, 0x07);
        #ifdef DEBUG
          Serial.println("S");
        #endif
      }
      else if ((sensorState & 0x01) == 0x00 && (sensorState & 0x18) == 0x18){
        //The left two sensors are on the line, and the right-most sensor is not.  Left
        led1ControlValue = HIGH;
        led2ControlValue = 4;
        doMoveForward(0x02, 0x04);
        #ifdef DEBUG
          Serial.println("L2");
        #endif
      }
      else if ((sensorState & 0x03) == 0x03 && (sensorState & 0x10) == 0x00){
        //The right two sensors are on the line, but the left-most sensor is not.  Right
        led1ControlValue = HIGH;
        led2ControlValue = 4;
        doMoveForward(0x04, 0x02);
        #ifdef DEBUG
          Serial.println("R2");
        #endif
      }
      else if ((sensorState & 0x01) == 0x00 && (sensorState & 0x10) == 0x10){
        //The left sensor is on the line, but the right sensor is not - spin left hard.
        led1ControlValue = HIGH;
        led2ControlValue = 6;
        doMoveSpin(DIRECTION_LEFT, 0x03, 0x02);
        #ifdef DEBUG
          Serial.println("L3");
        #endif
      }
      else if ((sensorState & 0x01) == 0x01 && (sensorState & 0x10) == 0x00){
        //The right sensor is on the line, but the left sensor is not - turn right hard
        led1ControlValue = HIGH;
        led2ControlValue = 6;
        doMoveSpin(DIRECTION_RIGHT, 0x02, 0x03);
        #ifdef DEBUG
          Serial.println("R3");
        #endif
      }
      else if ((sensorState & 0x11) == 0x11){
        //Both left and right sensors are on the line.  Spin right to try to find the line again.
        led1ControlValue = LOW;
        led2ControlValue = HIGH;
        doMoveSpin(DIRECTION_RIGHT, 0x01, 0x03);
        #ifdef DEBUG
          Serial.println("TR");
        #endif
      }
      else if (sensorState == 0x00){
        //Neither left nor right sensors are on the line.  Turn left to try to find the line again.
        led1ControlValue = LOW;
        led2ControlValue = HIGH;
        doMoveSpin(DIRECTION_LEFT, 0x03, 0x01);
//        doStop();
        #ifdef DEBUG
          Serial.println("TL");
        #endif
      }
      else {  //Unknown sensor state.  This should never happen (the above conditions will catch all 32 possible states)
        led1ControlValue = 4;
        led2ControlValue = 4;
        doStop();
      }
    }
  }
  else {
    led1ControlValue = LOW;
    led2ControlValue = LOW;
    doStop();
  }
}

void doMoveForward(uint8_t leftMotorSpeed, uint8_t rightMotorSpeed){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = (leftMotorSpeed & 0x07);
  motorRightPwmValue = (rightMotorSpeed & 0x07);
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

  motorLeftPwmValue = (leftMotorSpeed & 0x07);
  motorRightPwmValue = (rightMotorSpeed & 0x07);
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

  uint8_t result = 0;
  
  if (rawValues[0] > 50){
    result |= 0x10;
  }
  if (rawValues[1] > 50){
    result |= 0x08;
  }
  if (rawValues[2] > 50){
    result |= 0x04;
  }
  if (rawValues[3] > 50){
    result |= 0x02;
  }
  if (rawValues[4] > 50){
    result |= 0x01;
  }

#ifdef DEBUG
//  Serial.print(rawValues[0]);
//  Serial.print("\t");
//  Serial.print(rawValues[1]);
//  Serial.print("\t");
//  Serial.print(rawValues[2]);
//  Serial.print("\t");
//  Serial.print(rawValues[3]);
//  Serial.print("\t");
//  Serial.print(rawValues[4]);
//  Serial.print("\t");
//  Serial.println(result, BIN);
#endif

  return result;
}

void updatePWM(){
  //Get the 3 LSB of the millis value
  uint8_t millisLSB = (millis() & 0x07);

  //The 3 LSB of the motor set values are compared with the 3 LSB of the millis value.  This is
  // a poor man's PWM approximation.  It will not give a proper PWM waveform, as it will sometimes
  // 'miss' a value, but it is close enough for rough motor control.
  digitalWrite(MOTOR_L_EN, (motorLeftPwmValue == 0 ? 0 : motorLeftPwmValue >= millisLSB));
  digitalWrite(MOTOR_R_EN, (motorRightPwmValue == 0 ? 0 : motorRightPwmValue >= millisLSB));

  //The LEDs are a bit different... we can set them to LOW, HIGH (digital values) or an analog value from 2 - 7 inclusive.
  digitalWrite(LED1, (led1ControlValue == LOW ? LOW : (led1ControlValue == HIGH ? HIGH : led1ControlValue >= millisLSB)));
  digitalWrite(LED2, (led2ControlValue == LOW ? LOW : (led2ControlValue == HIGH ? HIGH : led2ControlValue >= millisLSB)));
}

