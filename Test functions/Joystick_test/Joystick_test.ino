/*
  ReadJoystick
  
  yValue => left= 1023, right = 0, center = 530, this needs some correction for non-center position
  xValue => bottom= 1023, up= 0, center = 510,  x is about OK
  SM     => pressed = 1, unpressed =0;
 */
int buttonPin = 2;
int xPin= A0;
int yPin = A1;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);
  int buttonValue = digitalRead(buttonPin);
  
  // print out the values you read:
  Serial.print(xValue); Serial.print(" "); Serial.print(yValue); Serial.print(" "); Serial.print(buttonValue); Serial.print(" ");
  
  int xServoAngle= mapAnalogToServo(xValue);
  Serial.print(" x: "); Serial.print(xServoAngle);  Serial.print(" ");
  
  int yServoAngle= mapYJoystickToServo(yValue); //special function because it needs to be corrected (non-centric value)
  Serial.print(" y: ");Serial.println(yServoAngle);
  delay(5); // 20 Hz
}

//general function, used for the x
int mapAnalogToServo(int analogValue){
  int servoAngle= map(analogValue,0,1020,0,180);
  servoAngle= constrain(servoAngle,0,180);
  return servoAngle;
}

//corrected function for the y- value
int mapYJoystickToServo(int analogValue){
  int servoAngle= map(analogValue,35,1023,0,180); //correct for the non-center place of the yJoystick
  servoAngle= constrain(servoAngle,0,180);
  return servoAngle;
}
