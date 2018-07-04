//#define DEBUG

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
const int MOTOR_L_EN = 2;
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

  //Reset the LED state every time through the loop.  This turns off the LED if the state changes.
  ledControlValue = 0x00;
  
  //Read the button - this starts the main program.
//  if (readButton() == HIGH){
//    
//  }

  //Use thresholds to construct a single 5 bit number constituting the state of 
  // the sensors. Sensor 0 is the LSB.
  uint8_t state = readSensors();

  //Conditional blocks to determine what to do given a particular state.  
  if ((state & 0x11) == 0x00 && (state & 0x0E) != 0x00){
    //Any of the three middle sensors see the line, and neither of the outside 
    // sensors do - stay straight
    ledControlValue = 0x01;
    doMoveStraight();
#ifdef DEBUG
  Serial.print(state, BIN);
  Serial.println(" - Straight");
#endif
  }
  else if ((state & 0x01) == 0x00 && (state & 0x10) != 0x00){
    //The left sensor is on the line, but the right sensor is not - turn left
    ledControlValue = 0x01;
    doMoveLeft();
#ifdef DEBUG
  Serial.print(state, BIN);
  Serial.println(" - Left");
#endif
  }
  else if ((state & 0x01) != 0x00 && (state & 0x10) == 0x00){
    //The right sensor is on the line, but the left sensor is not - turn right
    ledControlValue = 0x01;
    doMoveRight();
#ifdef DEBUG
  Serial.print(state, BIN);
  Serial.println(" - Right");
#endif
  }
  else if ((state & 0x11) == 0x11){
    //Both left and right sensors are on the line - presumably this means the line is
    // perpendicular to the sensor array.  Spin on the spot.
    ledControlValue = 256;
    doSpin();
#ifdef DEBUG
  Serial.print(state, BIN);
  Serial.println(" - Spin (Perpendicular)");
#endif    
  }
  else if (state == 0x00){
    ledControlValue = 1024;
    doSpin();
#ifdef DEBUG
  Serial.print(state, BIN);
  Serial.println(" - Spin (Nothing)");
#endif
  }
  else {  //Unknown state - this should never happen, if it does we need to add more state checking.
    ledControlValue = 32;
    doStop();
#ifdef DEBUG
  Serial.print(state, BIN);
  Serial.println(" - Stop (Unknown)");
#endif
  }
}

void doMoveStraight(){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = 0x0F;
  motorRightPwmValue = 0x0F;
}

void doMoveLeft(){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = 0x00;
  motorRightPwmValue = 0x0F;
}

void doMoveRight(){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, LOW);
  digitalWrite(MOTOR_R_B, HIGH);

  motorLeftPwmValue = 0x0F;
  motorRightPwmValue = 0x00;
}

void doSpin(){
  digitalWrite(MOTOR_L_A, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_A, HIGH);
  digitalWrite(MOTOR_R_B, LOW);

  motorLeftPwmValue = 0x08;
  motorRightPwmValue = 0x08;  
}

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
  // 'miss' a value, but it is close enough for motor control.
  digitalWrite(MOTOR_L_EN, (motorLeftPwmValue > millisLSB));
  digitalWrite(MOTOR_R_EN, (motorRightPwmValue > millisLSB));

  //The LED works a bit differently than the motors... if you set the value to 0 (LOW) or 1 (HIGH)
  // it is always on or always off.  Otherwise, we use the control value as the PWM period (in ms), 
  // and assume a 50% duty cycle.  For this to work, we always need to set the control value to be
  // a power of two (e.g. 32, 128, 256, 512, 1024, etc).
  uint16_t ledMillisLSB = (millis() & ledControlValue);
  if (ledControlValue == 0){
    digitalWrite(LED, LOW);
  }
  else if (ledControlValue == 1){
    digitalWrite(LED, HIGH);
  }  
  else {
    digitalWrite(LED, (ledControlValue / 2) >= ledMillisLSB);
  }
}

