
#include <SoftwareSerial.h>
#include <Servo.h>
///
// bluetooth HC-06: attach arduino pin 10 to HC-06 TX (Ar10 = hc-06 TX). make voltage divider so that (HC-06 RX = 2/3*Ar9), so 3.3 V.   Vdd= 5V Gnd= gnd
// HBridge pololu md18a: ArGnd to GndLeft and VCC; batteries to Vin & GndRight; one motor per letter (A/B); Mode= 1 als de H-bridge aan moet staan.
//                        Enbl= hoe hard de motor moet draaien (pwm signaal); phase: richting;
//                      hier: mode= Ar4;  aenbl= Ar5; benbl= Ar6; aphase= Ar7; bphase= Ar8;
// Servo: 5V/ Gnd; pwm (geel) = 3

// Joystick  //TODO
//int buttonPin = 2;
//int xPin = A0;
//int yPin = A1;

SoftwareSerial BTSerial(10, 11); // RX, TX

#define NUM_SERVOS 3
Servo servos[NUM_SERVOS];
Servo& leftMotor = servos[0]; //alias to make code easier to read/understand
Servo& rightMotor = servos[1];
Servo& sharpServo = servos[2];
int servoPins[NUM_SERVOS] = {5, 6, 3};  // l, r, sharp
// find out centering of our servos, and dead zone
const int& leftMotorPin  = servoPins[0];  // middle = 95 degrees; dead= 92-98
const int& rightMotorPin = servoPins[1];  // middle = 92 degrees; dead= 90-94
const int& sharpServoPin = servoPins[2];

int receivedLed = 4; //flashes when something is received


// a command is composed of a command type (2 chars), and the command itself (up to 18 chars). (the command type is for knowing how to interpret the following command characters)
char stopChar = '$'; //this is used to mark the end of a command
const int maxCommandLength = 20; // the maximum length a command can have.
char command[maxCommandLength] = ""; // Stores response of bluetooth device

// possible commands
const int CONTROL_PIN_DIGITAL = 0; // format 00{pin}{on/off}$
const int CONTROL_PIN_ANALOG = 1;  // format 01{pin}{value(4chars)}$
const int CONTROL_SERVO = 2;       // format 02{servoName}{value(3chars)}$
const int CONTROL_2SERVOS = 3;     // format 03{servoName}{value(3chars){2ndServoName}{2ndValue(3chars)}$
const int FW = 4; //forward         // format 04$
const int RV = 5;                   // format 05$
const int ST = 6;                   // format 06$
const int TL = 7;                   // format 07$
const int TR = 8;                   // format 08$
const int RESET_SPEED = 9;          // format 09$
const int INCREASE_SPEED = 10;      // format 10$
const int REDUCE_SPEED = 11;        // format 11$
const int SET_SPEED = 12;           // format 12$

// global variables
int speedScalar = 50;    // can be changed using buttons, reset to 50 by pressing Reset button
int lastMoveCommand = ST; // used for updating speed of currently executing moveCommand (fw, rv, tl, tr) after changing speedScalar. Requires ugly speedHack() function

void setup()
{
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(sharpServoPin, OUTPUT);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90); //go to middle
  }
  st(); //stop motors
  pinMode(receivedLed, OUTPUT);
  BTSerial.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  // Read device output if available.
  int receivedChars = 0;
  while (! BTSerial.available() ) {
    ;
  }
  if (BTSerial.available()) {
    while (BTSerial.available() != stopChar && receivedChars < maxCommandLength) { // While there is more to be read, keep reading.
      command[receivedChars] = (char)BTSerial.read();
      receivedChars += 1;
      delay(1);
    }

    //blinkLedMany(receivedLed, 50, 3); // this causes too much delay/latency
    digitalWrite(receivedLed, 1); //turn led on, and turn it off at the end
    //Serial.println(command);

    // 1. Which type of command does the message contain?
    char commando[] = {command[0], command [1]};
    int commandCode = atoi(commando); // the code that denotes how the message must be interpreted
    //Serial.println(commandCode);

    // 2. Execute the correct command
    executeCommand(commandCode);
    Serial.println(lastMoveCommand);
    
    // 3. Send message back to acknowledge
    char received[] = "received";
    char returnMessage[30];
    sprintf(returnMessage, "%s%s", received, command);
    BTSerial.print(returnMessage); //show that you received the command
    memset(command, 0, maxCommandLength); // reset command; No repeats

   digitalWrite(receivedLed, 0); //turn led off
  } // end if
} // end loop

//------------------------------------------------------------------------------------------
//-------------------------------      FUNCTIONS      --------------------------------------
//------------------------------------------------------------------------------------------

void executeCommand(int commandCode){ // execute the command contained int the char[] 'command'
  switch (commandCode) { //look which function is called
    // ------------- General Control & Debugging -----------------
      case CONTROL_PIN_DIGITAL :
        controlPinDigital(); break;
      case CONTROL_PIN_ANALOG :
        controlPinAnalog(); break;
      case CONTROL_SERVO:
        controlServo(); break;
      case CONTROL_2SERVOS:
        control2Servos(); break;

      // ------------ Movement ------------
      case FW:    // forward
        fw(); 
        lastMoveCommand = FW;
        break;
      case RV:    // reverse
        rv(); 
        lastMoveCommand = RV;
        break;
      case ST:    // stop
        st();  
        lastMoveCommand = ST;
        break;
      case TL:    // turn left
        tl(); 
        lastMoveCommand = TL;
        break;
      case TR:    // turn right
        tr(); 
        lastMoveCommand = TR;
        break;
        
      // ---------- Speed management ---------------
      case RESET_SPEED:    // turn right
        resetSpeed(); break;
      case INCREASE_SPEED:    // turn right
        increaseSpeed(); break;
       case REDUCE_SPEED:    // turn right
        reduceSpeed(); break;
       case SET_SPEED:
        setSpeet();
      default: 2; //Serial.println("error in data received");
    }//close switch
}

//---------------- 00. Control Digital ---------------------
// format 00{pin}{on/off}$
// turn pins on or off (digital)
void controlPinDigital() {
  if ( (command[2] - '0') == 0 || (command[2] - '0') == 1) { // if it is a proper value
    digitalWrite((command[3] - '0'), (command[2] - '0'));
  }
}

//---------------- 01. Control Analog ---------------------
// format 01{pin}{value(4chars)}$
// write value to pin (analog)
void controlPinAnalog() {
  int pin = command[2];
  char commando[] = {command[3], command [4], command [5], command[6]};
  int value = atoi(commando);
  if ( value >= 0 && value <= 1023) { // if it is a proper value
    analogWrite(pin, value);
  }
}

//---------------- 02. Servo Control ---------------------
// format 02{servoName}{value(3chars)}$
//@effect   this method interprets the given command as a command to control a servo.
//          the first char of the command is the pin to which the servo is attached, the next 3 chars are the angle to which it should be set.
void controlServo() {
  int pin;
  switch (command[2]) {
    case 'l':  //left is first pin in array
      pin = servoPins[0]; break;
    case 'r': //right is second pin in array
      pin = servoPins[1]; break;
    default:
      pin = servoPins[2]; break; // otherwise control Sharp servo
  } // end case
  char commando[] = {command[3], command [4], command [5]};
  int angle = atoi(commando); // get the angle from the command

  //Serial.print("pin: "); //Serial.print(pin); //Serial.print("; angle: "); //Serial.println(angle);

  writeToServo(angle, pin);
}

//---------------- 03. Dual Servo Control ---------------------
// format 03{servoName}{value(3chars){2ndServoName}{2ndValue(3chars)}$
void control2Servos() {  //control 2 servos at the same time, using the CONTROL_2SERVOS command
  char firstServo = command[2];
  int pin = getPin(firstServo);
  char commando[] = {command[3], command [4], command [5]};
  int angle = atoi(commando); // get the angle from the command

  //Serial.print("the 1st servo is: "); //Serial.println(firstServo);
  //Serial.print("1st pin: "); //Serial.print(pin); //Serial.print("; 1st angle: "); //Serial.println(angle);

  writeToServo(angle, pin);

  // now the same thing for the second servo
  char secondServo = command[6];
  pin = getPin(secondServo);
  commando[0] = command[7]; commando[1] = command[8]; commando[2] = command[9];
  angle = atoi(commando); // get the angle from the command

  //Serial.print("the 2nd servo is: "); //Serial.println(secondServo);
  //Serial.print("2nd pin: "); //Serial.print(pin); //Serial.print("; 2nd angle: "); //Serial.println(angle);

  writeToServo(angle, pin);

}

//------ Servo help functions--------
//get the pin belonging to the name of the servo
int getPin(char servoName) {
  int pin;
  switch (servoName) {
    case 'l':  //left is first pin in array
      pin = servoPins[0]; break;
    case 'r': //right is second pin in array
      pin = servoPins[1]; break;
    default:
      pin = servoPins[2]; break; // otherwise control Sharp servo
  } // end case
  return pin;
}

//get servo belonging to the pin, and write angle to it
void writeToServo(int angle, int pin) {
  if  (isValidAngle(angle)) {
    for (int i; i < NUM_SERVOS; i++) {
      if (servoPins[i] == pin) {
        //Serial.print("the written angle is: ");//Serial.println(angle);
        servos[i].write(angle);
      }
    }
  }
  //else {//Serial.print("not a valid angle: "); //Serial.println(angle);}
}

//return   true if the given angle is between 0 and 180
boolean isValidAngle(int& servoAngle) {
  if (servoAngle >= 0 && servoAngle <= 180) {
    if (servoAngle > 176) { servoAngle = 175;} // fix weird thing when it started shaking when >176
    return true;
  }
  return false;
}


//----------------- Movement ----------------
int percentToLeftServoAngle(int percent) { // -100% = full reverse (angle 0) ; 100% = full forward (angle 180)
  percent = (int)percent * speedScalar / 100.0; // be able to change speed
  
  int leftAngle = (int)(95 + percent);   // + percent ipv + 90/100.0*percent because you don't have to work with floats (much faster), and there is hardly any difference in accuracy
  if (leftAngle > 180) {
    leftAngle = 180;  //make sure you don't go too far
  }
  if (leftAngle < 0) {
    leftAngle = 0;
  }
  return leftAngle;
}

int percentToRightServoAngle(int percent) {
  percent = (int)percent * speedScalar / 100.0;
  
  int rightAngle = (int)(92 + percent);
  if (rightAngle > 180) {
    rightAngle = 180;
  }
  if (rightAngle < 0) {
    rightAngle = 0;
  }
  return rightAngle;
}

//---------------- 04. Forward   ---------------------
// format 04$
void fw() {
  int percent = speedScalar;
  int angleLeft  = percentToLeftServoAngle(percent);
  int angleRight = percentToRightServoAngle(percent);
  writeToServo(angleLeft, leftMotorPin);
  writeToServo(angleRight, rightMotorPin);
}

//---------------- 05. Reverse  ---------------------
// format 05$
void rv() {
  int percent = speedScalar * -1;
  int angleLeft  = percentToLeftServoAngle(percent);
  int angleRight = percentToRightServoAngle(percent);
  writeToServo(angleLeft, leftMotorPin);
  writeToServo(angleRight, rightMotorPin);
}

//---------------- 06. STOP  ---------------------
// format 06$
void st() {
  int percent = 0;
  int angleLeft  = percentToLeftServoAngle(percent);
  int angleRight = percentToRightServoAngle(percent);
  writeToServo(angleLeft, leftMotorPin);
  writeToServo(angleRight, rightMotorPin);
}

//---------------- 07. Turn Left  ---------------------
// format 07$
void tl() {
  int percent = speedScalar;
  // only rightMotor goes forward
  int angleLeft  = percentToLeftServoAngle(percent * -1);
  int angleRight = percentToRightServoAngle(percent);
  writeToServo(angleLeft, leftMotorPin);
  writeToServo(angleRight, rightMotorPin);
}

//---------------- 08. Turn Right  ---------------------
// format 08$
void tr() {
  int percent = speedScalar;
  // only leftMotor goes forward
  int angleLeft  = percentToLeftServoAngle(percent);
  int angleRight = percentToRightServoAngle(percent * -1);
  writeToServo(angleLeft, leftMotorPin);
  writeToServo(angleRight, rightMotorPin);
}

//------------------ 09. ResetSpeed -------------------
void resetSpeed(){
  speedScalar = 50;
  executeCommand(lastMoveCommand);
}

//------------------ 10. IncreaseSpeed -------------------
void increaseSpeed(){
  speedScalar += 5;
  if (speedScalar > 100){
    speedScalar = 100;
  }
    executeCommand(lastMoveCommand);
}

//------------------ 11. ReduceSpeed -------------------
void reduceSpeed(){
  speedScalar -= 5;
  if (speedScalar < 0){
    speedScalar = 0;
  }
   executeCommand(lastMoveCommand);
}

//------------------ 12. setSpeed -------------------
void setSpeed(){  // get speed from command, set right value and update currently executing moveCommands
  char speedCommand[] = {command[2], command [3], command[4]};
  int speed = atoi(speedCommand); 
  if (speed < 0){ speed = 0;}
  if (speed > 100){ speed = 100;}
  speedScalar = speed;
  executeCommand(lastMoveCommand);
}


// ---------------- Led ------------
// @param  ledPin    the pin on which the led can be found
// @param  onOffTime the time the led has to ben on, and then off. (so this is half the time of the entice cycle)
void blinkLedOnce(int ledPin, int onOffTime) {
  digitalWrite(ledPin, 1);
  delay(onOffTime);
  digitalWrite(ledPin, 0);
  delay(onOffTime);
}

//@ effect   this method blinks the led on lepPin a number of nbTimes times, with the on-time == off-time= onOfftime. this onOfftime is thus half the time of one complete blink cycle.
void blinkLedMany(int ledPin, int onOffTime, int nbTimes) { // blink a led on the given ledPin many times with a given duration and number of times
  for (int i = 0; i < nbTimes; i++) {
    blinkLedOnce(ledPin, onOffTime);
  }
}

// -----------------Joystick ---------------
/*
  yValue => left= 1023, right = 0, center = 530, this needs some correction for non-center position
  xValue => bottom= 1023, up= 0, center = 510,  x is about OK
  SM     => pressed = 1, unpressed =0;
*/
//int buttonPin = 2;
//int xPin= A0;
//int yPin = A1;

//  int xValue = analogRead(xPin);
//  int yValue = analogRead(yPin);

//  int xServoAngle= mapAnalogToServo(xValue);
//  Serial.print(" x: "); Serial.print(xServoAngle);  Serial.print(" ");
//
//  int yServoAngle= mapYJoystickToServo(yValue); //special function because it needs to be corrected (non-centric value)
//  Serial.print(" y: ");Serial.println(yServoAngle);

//general function, used to map the x- joystick value to a servo angle
int mapAnalogToServo(int analogValue) {
  int servoAngle = map(analogValue, 0, 1020, 0, 180);
  servoAngle = constrain(servoAngle, 0, 180);
  return servoAngle;
}
//corrected function used to map the y- joystick value to a servo angle
int mapYJoystickToServo(int analogValue) {
  int servoAngle = map(analogValue, 35, 1023, 0, 180); //correct for the non-center place of the yJoystick
  servoAngle = constrain(servoAngle, 0, 180);
  return servoAngle;
}



