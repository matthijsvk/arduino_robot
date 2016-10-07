
int ledPin= 3;
float distance = 0.0;
int sizeOfLedArray= 6; // arrays need one more element at the end (null value)
int ledArray[] = {2,3,4,5,6};
int recoverTime = 40; //the SHARP rangefinder needs and 39 ms to respond

void setup() {

  pinMode(ledPin,OUTPUT);
  Serial.begin(9600); // initialize serial communication at 9600 bits per second:
  for(int i = 0; i< sizeOfLedArray; i++){
    pinMode(ledArray[i], OUTPUT); // set the led pins as outputs
  }
  Serial.println(sizeOfLedArray);
  delay(50); // the SHARP rangefinder needs 44 ms to start up 
}

void loop() {
  // read the input on analog pin 0:
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

  Serial.print(sensorValue);
  Serial.print("; distance: ");
  Serial.print(distance);
  int distance_mapped= map(distance, 0,40, sizeOfLedArray- 1, 0); // map inversely to the number of leds on
  Serial.print("; mapped: ");
  Serial.print(distance_mapped);
  
  int onLeds[sizeOfLedArray];
  for(int i = 0; i< distance_mapped; i++){
    onLeds[i] = ledArray[i];
    digitalWrite(ledArray[i], HIGH);
  }
  for(int i= distance_mapped; i< sizeOfLedArray; i++){
    digitalWrite(ledArray[i], LOW);
  }

  Serial.print(" Leds: ");
  for(int i = 0; i< distance_mapped; i++){
    Serial.print(onLeds[i]);  
    Serial.print(" ,");
  }
  Serial.println();
}

