const int LED= 13;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  delay(100);
  char c[4]; //one bigger than the length of the chars to receive (terminating null)
  int index= 0;
  if (Serial.available()){
    while (Serial.available()) {
      c[index]= Serial.read();
      index++;
    }
      
  if (c[0] == '#') {
    int onOff= 0;
    
    if(c[2]=='H'){ // (1)
      onOff= 1;
    }
    else {onOff= 0; }
    
    pinMode(c[1]-'0', OUTPUT);
    digitalWrite(c[1]-'0',onOff); //write onOff to the right pin
    delay(1000);
  }
}
}

//to be aable to control 2 leds at once (replace (1) with this)

////if(c[2]=='H'){
//      onOff= 1;
//      }
//    else if(48<=c[2]<=57){ //if the 3rd char is an number (0-9), turn it on/off too
//      if(c[3]=='H'){
//        onOff= 1;
//      }
//      else {onOff=0;}
//      pinMode(c[2]-'0', OUTPUT);
//      digitalWrite(c[2]-'0',onOff); //write onOff to the right pin
//    }
//    else {onOff= 0; }
