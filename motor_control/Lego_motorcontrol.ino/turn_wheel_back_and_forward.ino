// use H bridge to control lego motors

 const int BenablePin = 9; // enable of motor controller connected to digital pin 5 
const int BphasePin = 8; // phase of motor controller connected to digital pin 4 
// remember to connect the B wires the other way around!! it must be mirrored!  (so for A(=right motor): O1 to wire side, O2 to boobel side)

const int AenablePin = 5; // Wheel A is right wheel
const int AphasePin = 4;

const int modePin = 2; // mode of motor controller connected to digital pin 7 
const int maxRotationSpeed= 250;
 
void setup() { 
  Serial.begin(9600);
 pinMode(modePin, OUTPUT); 
 pinMode(AphasePin, OUTPUT); 
 pinMode(AenablePin, OUTPUT);
 pinMode(BphasePin, OUTPUT); 
 pinMode(BenablePin, OUTPUT);
} 

void loop() { 
 forward(100);
 backward(100);
 delay(2000);
} 


// Functions that make it move

void forward(int distance){ //distance in cm//  diameter of wheels used is 21.5 cm. so 2*Pi*21.5= 6.8 cm/turn,   turnrate= 2 turns/sec  ==> distance per sec= 13.5 cm/sec
  int C= 1/13.5;  // is this the right C?
  int duration= distance* C* 1000;  //  correct C to be determined: what is the forward robotspeed at max motorspeed?? 
  turnWheels(1,100,1,100,duration);
  delay(50); //pause for a moment
}

void backward(int distance){
  int C= 1;
  int duration= distance* C;  //  C to be determined: what is the forward robotspeed at max motorspeed??
  turnWheels(0,100,0,100,duration);
  delay(50); //pause for a moment
}

void turn(int degres){  //forward is 0 degrees; clockwise is >0; degres better not out of [-180,180] interval (except for showing off :D)
  int C= 1;
  if (degres>0)  {
    int duration= degres* C; // C to be determined: what is the turnrate at zb 70% speed? at 100% speed? (i put 70% to prevent slipping)
                              // with no load and on Arduino power: 2 turns/sec= 720 degres/sec
    turnWheels(0,70,1,70,duration);
  }
  else if (degres<0){
    degres= -degres;
    int duration = degres*C;
    turnWheels(0,70,1,70,duration);
  }
  delay(50); //pause for a moment
}
    
  


//subFunctions to turn the wheels at a certain speed and duration

void turnWheels(int Adirection,int Aspeed,int Bdirection,int Bspeed,unsigned long duration) { //takes 0,5s to power on and 0,5 to power off
 //turn the motors on
 turnOnMotor(AphasePin,Adirection,AenablePin,Aspeed);
 turnOnMotor(BphasePin,Bdirection,BenablePin,Bspeed);
 delay(100);
 
 //let them turn
 unsigned long startTime= millis();
 while (millis()< startTime+duration+100)  {  //+100 to give time to spin up properly
   delay(50); //wait
  }
  
 //turn the motors off
 turnOffMotor(AphasePin,Adirection,AenablePin);
 turnOffMotor(BphasePin,Bdirection,BenablePin);
 // Serial.println("turnedoff");
}

  
// subSubFunctions to control motors

void turnOnMotor(int phasePin,int turn_direction,int enablePin,int rotationPercent){ //takes 0,5 seconds
  digitalWrite(modePin, HIGH); 
  digitalWrite(phasePin, turn_direction); 
  delay(10);
  //turn on
  for(int fadeValue = 0 ; fadeValue <= rotationPercent/100.0*maxRotationSpeed; fadeValue +=5) { 
   analogWrite(enablePin, fadeValue); // sets the value (range from 0 to 255) 
   delay(5); // wait for 5 milliseconds to see the effect 
   }
} 

void turnOffMotor(int phasePin,int turn_direction,int enablePin){ //takes 0,5 seconds
  digitalWrite(modePin, HIGH); 
  digitalWrite(phasePin, 1-turn_direction);
  delay(30);
  //turn off
  digitalWrite(enablePin,LOW);
} 
