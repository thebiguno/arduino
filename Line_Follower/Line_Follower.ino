//The const variable declarations help to give meaning to the otherwise arbitrary
// pin assignments.  This abstraction also would help if we ever switched to a different
// Arduino board - we would just have to change the values in this single place, rather
// than everywhere in the entire program.

//Sensor array
const int SENSOR_0 = A4;    //Left sensor
const int SENSOR_1 = A1;
const int SENSOR_2 = A2;
const int SENSOR_3 = A3;
const int SENSOR_4 = A0;    //Right sensor

//Left motor
const int MOTOR_L_EN = 0;
const int MOTOR_L_A = 7;
const int MOTOR_L_B = 8;

//Right motor
const int MOTOR_R_EN = 9;
const int MOTOR_R_A = 5;
const int MOTOR_R_B = 6;

//LED
const int LED = 12;

//Button
const int BUTTON = 11;

//These are global variables, which can be accessed from any function in 
// the entire program.  These are used to count time when creating software
// defined PWM waveforms.
uint8_t motorLeftPwmValue = 0;
uint8_t motorRightPwmValue = 0;
uint8_t ledPwmValue = 0;

//This function is run once at the beginning of the program.
void setup() {
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

  //Configure button in input mode.  Not strictly needed as pins are input 
  // by default, but good practice nonetheless.
  pinMode(BUTTON, INPUT);

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
  //We need to update the motor PWM every loop.  This function handles timing 
  // and intervals for the two motors and the LED.
  updatePWM();
  
  uint16_t rawValues[5];      //The raw, 10 bit values from analogRead
  
  //Read the sensors to find where the line is
  readSensors(rawValues);

  //Read the button - this starts the main program.
  if (readButton() == HIGH){
    
  }

  //Use thresholds to construct a single 5 bit number constituting the state of 
  // the sensors. Sensor 0 is the LSB.
  uint8_t state = decodeSensors(rawValues);

  //Conditional blocks to determine what to do given a particular state.
  
  if (state & 0x11 == 0x00 && state & 0x0E != 0x00){
    //Any of the three middle sensors see the line, and neither of the outside 
    // sensors do - stay straight
    doMoveStraight();
  }
  else if ((state & 0x01 != 0x00) && (state & 0x10 == 0x00)){
    //The left sensor is on the line, but the right sensor is not - turn left
    doMoveLeft();
  }
  else if ((state & 0x01 == 0x00) && (state & 0x10 != 0x00)){
    //The right sensor is on the line, but the left sensor is not - turn right
    doMoveRight();
  }
  else {
    doFlashLight();
  }
}

void doMoveStraight(){
  digitalWrite(MOTOR_L_A, HIGH);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, HIGH);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = 80;
  motorRightPwmValue = 80;
}

void doMoveLeft(){
  digitalWrite(MOTOR_L_A, HIGH);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, HIGH);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = 20;
  motorRightPwmValue = 80;
}

void doMoveRight(){
  digitalWrite(MOTOR_L_A, HIGH);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, HIGH);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = 80;
  motorRightPwmValue = 20;
}

void doStop(){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, LOW);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, LOW);

  motorLeftPwmValue = 0;
  motorRightPwmValue = 0;
}

void doFlashLight(){
  uint8_t millisLSB = (millis() & 0xFF);
  digitalWrite(LED, millisLSB > 128);
}

//Read analog values into the provided array of 5 unsigned 16 bit (uint16_t) values
void readSensors(uint16_t *rawValues){
  rawValues[0] = analogRead(SENSOR_0);
  rawValues[1] = analogRead(SENSOR_1);
  rawValues[2] = analogRead(SENSOR_2);
  rawValues[3] = analogRead(SENSOR_3);
  rawValues[4] = analogRead(SENSOR_4);
}

//Take the 5 raw values, decode them according to their thresholds, and encapsulate
// them into a single 5 bit unsigned integer.  Sensor 0 is the LSB.
uint8_t decodeSensors(uint16_t *rawValues){
  uint8_t result = 0;
  
  if (rawValues[0] > 100){
    result |= 0x01;
  }
  if (rawValues[1] > 100){
    result |= 0x02;
  }
  if (rawValues[2] > 100){
    result |= 0x04;
  }
  if (rawValues[3] > 100){
    result |= 0x08;
  }
  if (rawValues[4] > 100){
    result |= 0x10;
  }

  return result;
}

void updatePWM(){
  //Get the 8 LSB of the millis value
  uint8_t millisLSB = (millis() & 0xFF);

  digitalWrite(MOTOR_L_EN, (motorLeftPwmValue > millisLSB));
  digitalWrite(MOTOR_R_EN, (motorRightPwmValue > millisLSB));
  digitalWrite(LED, (ledPwmValue > millisLSB));
}

