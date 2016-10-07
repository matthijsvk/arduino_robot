#include <Servo.h>
// This code uses a Sharp IR sensor mounted on a servo to track objects using 
// left-edge detection. The servo first sweeps left and right.
// If it detects some object in its range, it goes left until it doesn't detect the object anymore. 
// If the distance suddenly increases, the object has moved to the right, so turn the servo right.
// If the object moved to the left, you'll keep following it because you always keep trying to move left.
// This results in always tracking the left edge (seen from the robot) of an object.


int sizeOfLedArray= 5; // arrays need one more element at the end (null value)
int ledArray[] = {
  2,3,4,5,6};
int recoverTime = 40; //the SHARP rangefinder needs and 39 ms to respond

Servo myservo;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 
int scanAngle = 0;
int servo_angle_step= 5; // the step at whch the servo angle changes
int servoPin = 9;
int SharpPin = A0;

unsigned int target_distance = 0; //distance away of target
unsigned int distance_thresh = 40; //acceptable target detection: 30 cm

void setup() {
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object 
  myservo.write(0);
  for(int i = 0; i < sizeOfLedArray; i++){
    pinMode(ledArray[i], OUTPUT); // set the led pins as outputs
  }

  Serial.begin(9600); // initialize serial communication at 9600 bits per second:
  delay(1000); // the SHARP rangefinder needs 44 ms to start up 
}


void loop() {
  // read the input on analog pin 0:
  delay(10);//so it doesnt change states too fast 
  scan();//move scanner one toward target
}


/*psuedocode
   	while object is detected
   		scan left while object detected
   	while object not detected
   		scan right until object detected*/
void scan() {
  
  int target_distance = convertAnalogToDistance(analogRead(SharpPin));//check sensor
  if (target_distance < distance_thresh){//object detected, move left
    servo_angle_step= 5;
    if (scanAngle > 5) //overflow protection
      scanAngle -= servo_angle_step;   //scan left
    else scanAngle += servo_angle_step; //don't go further left, this damages the servo
  }
  else {//object not detected
    if (scanAngle < 175 && scanAngle > 5) //overflow protection
      scanAngle += servo_angle_step; //scan right
    else if (scanAngle <= 5){
      servo_angle_step = 5;
      scanAngle += 10;
    }
    else{ //if scanned all the way, this forces it to start over
        servo_angle_step = -5;
        scanAngle-= 10;
//      scanAngle -= 175;
    }
  }
  Serial.println(scanAngle);
  myservo.write(scanAngle);   // tell servo to go to position in variable 'pos' 
}


int convertAnalogToDistance(int sensorValue){

  delay(recoverTime);
  float distance;
  if (sensorValue > 307) {
    distance = (int)(sensorValue - 800) / -61.0;
  }
  else if (sensorValue > 164) {
    distance = (int)(sensorValue - 450) / -18.0;
  }
  else if (sensorValue > 80){
    distance = (int)(sensorValue - 230) / -4.0;
  }
  else distance = 40; 

  return distance;
}







