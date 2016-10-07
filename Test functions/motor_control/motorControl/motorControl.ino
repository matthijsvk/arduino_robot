// This program controls the motors of the robot, using a Pololu m15a H-bridge. http://www.pololu.com/product/2135
// there are functions to move forward, backward, turn and do the evade-then-attck-from-the-side special attack.

// pin numbers, can be changed if you want
const int rightEnablePin = 9; // enable of motor controller (should it brake or turn (and how fast,  speed can be controlled via PWM)?)
const int rightPhasePin = 8; // phase of motor controller (forward:0  or backward: 1)
const int leftEnablePin = 5; 
const int leftPhasePin = 4;
// MODE xENABLE xPHASE  (DC MOTOR)
// 1       0       -      Brake
// 1       1       1      Reverse
// 1       1       0      Forward

const int modePin = 2;                // mode of motor controller connected to digital pin 7 
const int maxRotationSpeed= 1023;     // the maximum rotation speed of the motor, in permilli of the voltage of the battery pack (I use 6 1.2V, so 7.2V)
const float distanceC= 1;             // turnC converts distances to milliseconds (how long to turn on the motor to move 10 cm?)
const float turnC= 1;                       // turnC converts degrees to milliseconds (how long to turn on the motor to turn 30 degrees?)

void setup() { 
  Serial.begin(9600);
  pinMode(modePin, OUTPUT); 
  digitalWrite(modePin, HIGH); // should always be HIGH to activate Phase/Enable mode
  pinMode(leftPhasePin, OUTPUT); 
  pinMode(leftEnablePin, OUTPUT);
  pinMode(rightPhasePin, OUTPUT); 
  pinMode(rightEnablePin, OUTPUT);
} 

void loop() {  //testing
 forward(1000);
 backward(1000);
 delay(2000);
} 


// ---------------------------- Functions --------------------------------

//--------  high-level move functions  -----------------

void forward() {
  turnOnMotors(1,70,1,70);
}

void backward(){
  turnOnMotors(0,100,0,100);
}

void forward(int distance){           //distance in cm;  diameter of wheels used is 21.5 cm. so 2*Pi*21.5= 6.8 cm/turn,   turnrate= 2 turns/sec  ==> distance per sec= 13.5 cm/sec
  int duration= distance* distanceC;  //  correct C to be determined: what is the forward robotspeed at max motorspeed?? 
  turnOnMotors(1,70,1,70);
  delay(duration);                    
  stop();
}

void backward(int distance){
  int duration= distance* C;          //  C to be determined: what is the forward robotspeed at max motorspeed??
  turnOnMotors(0,100,0,100);
  delay(duration);                        
  stop();
}

void turn(int degres){                //forward is 0 degrees; clockwise is >0; degres better not out of [-180,180] interval (except for showing off :D)
    if (degres >= 0)  {
    int duration= degres* turnC;      // turnC to be determined: what is the turnrate at zb 70% speed? at 100% speed? (i put 70% to prevent slipping)
                                      // with no load and on Arduino power: 2 turns/sec= 720 degres/sec
    turnOnMotors(1,70,0,70);          // phase 0 means forward, phase 1 means backward
    delay(duration);
  }
  else (degres < 0){
    int duration = (-degres)* turnC;
    turnOnMotors(0,70,1,70);
    delay(duration);
  }
  delay(50);                          //pause for a moment
}

void evadeBackward(){
  turn(-30);
  backward(10*distanceStep);
  attack();                           //TODO
}
    
// --------- low-level move (motorControl) functions ----------

//subFunctions to turn the wheels at a certain speed

//turn the motors on
void turnOnMotors(int leftDirection,int leftSpeedPercent,int rightDirection,int rightSpeedPercent) { 
 turnOnMotor(leftPhasePin , leftDirection , leftEnablePin , leftSpeedPercent);
 turnOnMotor(rightPhasePin, rightDirection, rightEnablePin, rightSpeedPercent);
}

//turn the motors off
void stop(){
 turnOffMotor(leftEnablePin);
 turnOffMotor(rightEnablePin);
 // Serial.println("turnedoff");
}

// subSubFunctions to control motors

void turnOnMotor(int phasePin,int turn_direction,int enablePin,int speedPercent){ 
  digitalWrite(phasePin, turn_direction); 
  //turn on gradually
  for(int fadeValue = 0 ; fadeValue <= speedPercent/100.0*maxRotationSpeed; fadeValue +=20) { 
   analogWrite(enablePin, fadeValue);   // sets the value (range from 0 to 255) 
   delay(5);                            // wait for 5 milliseconds to see the effect 
   }
} 

void turnOffMotor(int enablePin){ 
  //turn off
  digitalWrite(enablePin,LOW);
} 
