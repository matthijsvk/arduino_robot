
#include <SoftwareSerial.h>
#include <Servo.h>
///
// bluetooth HC-06: attach arduino pin 10 to HC-06 TX (Ar10 = hc-06 TX). make voltage divider so that (HC-06 RX = 2/3*Ar9), so 3.3 V.   Vdd= 5V Gnd= gnd
// HBridge pololu md18a: ArGnd to GndLeft and VCC; batteries to Vin & GndRight; one motor per letter (A/B); Mode= 1 als de H-bridge aan moet staan.
//                        Enbl= hoe hard de motor moet draaien (pwm signaal); phase: richting;
//                      hier: mode= Ar4;  aenbl= Ar5; benbl= Ar6; aphase= Ar7; bphase= Ar8;
// Servo: 5V/ Gnd; pwm (geel) = 3
// Joystick:
//
//
//
//
//
SoftwareSerial BTSerial(10, 11); // RX, TX

#define NUM_SERVOS 2
Servo servos[NUM_SERVOS];
Servo& leftMotor = servos[0]; //alias to make code easier to read/understand
Servo& rightMotor = servos[1];
int servoPins[NUM_SERVOS] = {5, 6};
const int& leftMotorPin  = servoPins[0];  //middle = 95 degrees; dead= 92-98
const int& rightMotorPin = servoPins[1];  // middle = 92 degrees; dead= 90-94


// a command is composed of a command type (2 chars), and the command itself (up to 18 chars). (the command type is for knowing how to interpret the following command characters)
char stopChar = '$'; //this is used to mark the end of a command
const int maxCommandLength = 20; // the maximum length a command can have.
char command[maxCommandLength]=""; // Stores response of bluetooth device
const int CONTROL_SERVO = 1;
const int CONTROL_LED = 2;

int receivedLed = 4; //flashes when something is received
int servoPin = 6; //a pin with PWM capabilities

// Joystick
int buttonPin = 2;
int xPin = A0;
int yPin = A1;

void setup()
{
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  for (int i=0; i<NUM_SERVOS; i++){
   servos[i].attach(servoPins[i]);
   servos[i].write(90); //go to middle
  }
  pinMode(receivedLed, OUTPUT);
  BTSerial.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  // Read device output if available.
  int receivedChars = 0;
  while (! BTSerial.available() ){;}
  if (BTSerial.available()) {
    while (BTSerial.available() != stopChar && receivedChars < maxCommandLength) { // While there is more to be read, keep reading.
      command[receivedChars] = (char)BTSerial.read();
      receivedChars += 1;
      blinkLedOnce(receivedLed, 10); // show you've received the character
    }
  }
  blinkLedMany(receivedLed,200,3); // show you've received the entire command
  Serial.println(command);

  char commando[]= {command[0],command [1]}; 
  int commandCode = atoi(commando); // the code that denotes how the message must be interpreted
  Serial.println(commandCode);
  
  switch (commandCode) { //look which function is called
    case (CONTROL_LED) : //command structure : / header (2 chars)/ onOff(1char) / pins(x chars)
      Serial.println("LED");
      controlPin(command); break;
    case (CONTROL_SERVO): //command structure : / header (2 chars)/ angle(3chars)/ pins(x chars)
      {
        Serial.println("SERVO");
        controlServo(command); break;
      }
    default: 2;//Serial.println("error in data received");
  }//close switch

  char received[]= "received";
  char returnMessage[30];
  sprintf(returnMessage,"%s%s",received,command);
  BTSerial.print(returnMessage); //show that you received the command
  memset(command,0,maxCommandLength); // reset command; No repeats

  delay(1);
} // end loop


// turn pins on or of
void controlPin(char* command) {
  if ( (command[2] - '0') == 0 || (command[2] - '0') == 1) { // if it is a proper value
//    for (int i = 3; i < command.length(); i++) {
      //if ((command[i] - '0') > 1 ) { // not pins 1 or 0, those are used for TX and RX
        digitalWrite((command[3] - '0'), (command[2] - '0'));
     // }
    //}
  }
}
//
//// control motor pins: first 2 chars are command, then 3 for value, then 1 for pin
//void controlMotor(String command) {
//  int analogValue = atoi((command.substring(3,3));//the value to be written to the pins (1023= 5V)
//      if (isValidServoPin(command.substring(6,1)){
//        analogWrite(atoi(command.substring(6,1)), analogValue);
//      }
//  }
//}

//@effect   this method interprets the given command as a command to control a servo.
//          the first char of the command is the pin to which the servo is attached, the next 3 chars are the angle to which it should be set.
void controlServo(char* command) {
  
  int pin = command[2] - '0'; // get the pin from the command. pin is the first character after the header
  char commando[]= {command[3],command [4],command [5]}; 
  int angle = atoi(commando); // get the angle from the command

  Serial.print("pin: ");Serial.print(pin);Serial.print("; angle: ");Serial.println(angle);
  
  if  (! isValidAngle(angle)) {
    return;
  }

  for (int i; i < NUM_SERVOS; i++) {
    if (servoPins[i] == pin) {
      Serial.print("the servo is: "); Serial.println(i);
      servos[i].write(angle);
      //sharpServo.write(angle);
    }
  }
}


//--------------------------------------------------------------------------


//@return   true if the given angle is between 0 and 180
boolean isValidAngle(int servoAngle) {
  if (servoAngle >= 0 && servoAngle <= 180) {
    return true;
  }
  return false;
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
  for (int i=0; i < nbTimes; i++) {
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



