// This program uses an ultrasonic sensor to calculate the distance to an object.
// The ultrasonic sensor first sends a pulse, then receives it bouncing back and calcultates distance based on the time between sending and receiving.
// Leds are used to show this distance visually.
// 

int ledArray[] = {2,3,4,5,6,7};
int sizeOfLedArray= sizeof(ledArray);   // so you don't have to calculate it constantly
int UltrasonicOnPin = 11;               // the pin that turns on the ultrasonic sensor
int UltrasonicTrigPin = 9;              // the pin that sends the signal
int UltrasonicEchoPin = 8;              // the pin that receives the signal
int nbMeasurements = 4;                 // the nb of measurements to take the average of
int recoverTime = 10;                   // the rangefinder needs some time to recover

void setup() {
  
  Serial.begin(9600);                        
  for(int i = 0; i< sizeOfLedArray; i++){    // set the led pins as outputs
    pinMode(ledArray[i], OUTPUT);            
  }
  
  pinMode (UltrasonicOnPin, OUTPUT);         // als ik deze met de batterijen en een transistor aansluit, werkt het nauwelwijks of met gehalveerde waarden. via Vcc gaat het perfect...
  pinMode (UltrasonicTrigPin,OUTPUT);
  pinMode (UltrasonicEchoPin,OUTPUT);
}

void loop() {
  Serial.println("beaming...");
  digitalWrite(UltrasonicOnPin,HIGH);        // turn the ultrasonic sensor on
  float distance= 0.0;
  for (int i=0; i< nbMeasurements; i++){
    distance += getUltrasonicDistance(UltrasonicTrigPin, UltrasonicEchoPin);
  }
  distance = distance/(nbMeasurements/1.0);  //take the average (using floating point calculations)
  Serial.println(distance); 
  
  lightLeds(distance,ledArray);
  
  digitalWrite(UltrasonicOnPin, LOW);        //turn the ultrasonic sensor off
  delay(recoverTime);
}


//-----------------------------   Functions   ------------------------------------------------------------------------------------------

//-------------measuring the distance ------------------

void getUltrasonicDistance(int UltrasonicTrigPin, int UltrasonicEchoPin) { //get the average of several measurements to be more accurate.
  Serial.println("beaming...");
  digitalWrite(UltrasonicOnPin,HIGH);        // turn the ultrasonic sensor on
  float distance= 0.0;
  for (int i=0; i< nbMeasurements; i++){
    distance += getUltrasonicDistanceOnce(UltrasonicTrigPin, UltrasonicEchoPin);
  }
  distance = distance/(nbMeasurements/1.0);  //take the average (using floating point calculations)
  Serial.println(distance); 
  
  lightLeds(distance,ledArray);
  
  digitalWrite(UltrasonicOnPin, LOW);        //turn the ultrasonic sensor off
  delay(recoverTime);
}

long getUltrasonicDistanceOnce(int UltrasonicTrigPin, int UltrasonicEchoPin){
  delay(recoverTime);                           // Make sure the senor is in a proper state before measuring
  long duration, inches,cm;

  ping(UltrasonicTrigPin);                      // The PING))) is triggered by a HIGH pulse of several (10) microseconds.
  
  pinMode (UltrasonicEchoPin, INPUT);
  duration = pulseIn(UltrasonicEchoPin, HIGH);  // Get the time till the signal returns.

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm;
}

void ping(int TrigPin){                         // The PING))) is triggered by a HIGH pulse of several (10) microseconds. give a short LOW before to get a clean pulse.  
  pinMode(TrigPin, OUTPUT);
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter. return a float.ajhiiyi ube 
  // The ping travels out and back, so *0.5 . 
  return microseconds / 29.0 / 2;
}

//------------- lighting the leds ----------------------
void lightLeds(float distance, int[] ledArray){
  // map inversely to the number of leds on: all on means close by
  int distanceMapped= map(distance, 0,170, sizeOfLedArray-1, 0); 
  distanceMapped= constrain(distanceMapped,0,sizeOfLedArray-1);
  
  //Serial.println(distanceMapped);

  for(int i = 0; i< distanceMapped; i++){
    digitalWrite(ledArray[i], HIGH);
  }
  for(int i= distanceMapped; i< sizeOfLedArray; i++){
    digitalWrite(ledArray[i], LOW);
  }
}