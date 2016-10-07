/*
  ReadAnalogVoltage from a Sharp infrared sensor (range 40cm)
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
*/
 int infinity= 100; //the distance value that represents no (reliable) object detection
int ledPin= 3;
float distance= 0.0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  pinMode(ledPin,OUTPUT);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  float sensorValue = analogRead(A0);  // 550 is 20 cm;  200 is 70cm
  delay(30);
  float sensorValue2= analogRead(A0);
  sensorValue= (sensorValue +sensorValue2)/2.0;
  
  if (sensorValue> 200){
    distance= (4 +(550- sensorValue)* (1.0/70.0));
  }
  else {
    distance= (10 + (200-sensorValue)* (1/7.0));
  }
  if (distance > 38.0){
    distance= 100; //very far and thus unreliable, set to a predefined value
  }
  Serial.println(sensorValue);
  Serial.print("the distance is ");
  Serial.print(distance);
  Serial.println(" cm");
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  analogWrite(ledPin, sensorValue/4.0);  
  delay(200);
}
