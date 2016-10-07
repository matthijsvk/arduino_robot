#include <Servo.h>

float distance = 0.0;

int sizeOfLedArray= 6; // arrays need one more element at the end (null value)
int ledArray[] = {2,3,4,5,6};
int recoverTime = 40; //the SHARP rangefinder needs and 39 ms to respond

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
unsigned int scanAngle = 0;
int servoPin = 9;

unsigned int target_distance = 0; //distance away of target
unsigned int distance_thresh = 30; //acceptable target detection: 30 cm
unsigned int motorTimeOn = 10;

void setup() {
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object 
  
  for(int i = 0; i < sizeOfLedArray; i++){
    pinMode(ledArray[i], OUTPUT); // set the led pins as outputs
  }
  
  Serial.begin(9600); // initialize serial communication at 9600 bits per second:
  delay(50); // the SHARP rangefinder needs 44 ms to start up 
}


void loop() {
  // read the input on analog pin 0:
  
  
  {
	delayMicroseconds(2500);//so it doesnt change states too fast //2500 works
	
	scan();//move scanner one toward target
	
	//movement actions
	if (scanAngle > 120)//if target is too far on right
		{
		robotRight(motorTimeOn);//turn towards target
		scanAngle -= 2;//scanner turns left while robot turns right
		}
	else if (scanAngle < 60)//if target is too far on left
		{
		robotLeft(motorTimeOn);//turn towards target
		scanAngle += 2;//scanner turns right while robot turns left
		}
	else //centered on target
		{
		robotForward(motorTimeOn);//drive straight
		}
	}
}


//sense
void scan() {
	{/*psuedocode
	while object is detected
		scan left while object detected
	while object not detected
		scan right until object detected*/
	
	target_distance = convertAnalogToDistance(analog(A3));//check sensor
	
	if (target_distance < distance_thresh){//object detected
		if (scanAngle > 10) //overflow protection
			scanAngle -= 2;//scan left
                else scanAngle += 20;
        }
	else {//object not detected
		if (scanAngle < 150) //overflow protection
		      scanAngle += 2; //scan right
		else //if scanned all the way, this forces it to start over
		      scanAngle = -20;
	}
        servoTurn();
}


void convertAnalogToDistance(int sensorValue){
  delay(recoverTime);
  int sensorValue = analogRead(A0);  

  if (sensorValue > 307) {
    distance = (sensorValue - 800) / -61.0;
  }
  else if (sensorValue > 164) {
    distance = (sensorValue - 450) / -18.0;
  }
  else if (sensorValue > 80){
    distance = (sensorValue - 230) / -4.0;
  }
  else distance = 40; 
}

void servoTurn() {
  myservo.write(scanAngle);   // tell servo to go to position in variable 'pos' 
  delay(15);                   // waits 15ms for the servo to reach the position 
}
  
void robotRight(long time) {
  digitalWrite(leftMotorPin, HIGH);
  digitalWrite(rightMotorPin, LOW);
  delay(time);
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}

void robotLeft(long time) {
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, HIGH);
  delay(time);
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}

void robotForward(long time) {
  digitalWrite(leftMotorPin, HIGH);
  digitalWrite(rightMotorPin, HIGH);
  delay(time);
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}

void robotBackward(long time) {
  digitalWrite(leftMotorPin, HIGH,0);
  digitalWrite(rightMotorPin, HIGH,0);
  delay(time);
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}

void robotStop(long time) {
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
  delay(time);
}



  
