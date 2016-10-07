//test_countdown.ino

int ledArray [] = {2, 3, 4, 5, 6};
int sizeOfLedArray = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for ( int i = 0; i < sizeOfLedArray; i++ ) { // set the led pins as outputs
        pinMode ( ledArray[i],    OUTPUT );
    }
}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(13, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);               // wait for a second
    digitalWrite(13, LOW);   // turn the LED off by making the voltage LOW
    delay(500);               // wait for a second

    countdown(5);
}

void countdown ( int seconds )
{
	if (seconds>=1){seconds = seconds - 1;}
	// time between led turnoffs
    float waitTime = seconds/sizeOfLedArray;

    // turn all leds on
    for ( int i = 0; i < sizeOfLedArray; i++ ) {
        digitalWrite ( ledArray[i], HIGH );
    }

    // turn leds off one by one
    for ( int i = sizeOfLedArray; i > 0; i-- ) {
        digitalWrite ( ledArray[i], LOW );
        delay ( 1000*waitTime );
    }

	delay (1000);
    //flicker 3 times (total 1 second)
    for ( int j = 0; j < 3; j++ ) { 
        for ( int i = 0; i < sizeOfLedArray; i++ ) {
            digitalWrite ( ledArray[i], HIGH );
        }
        delay ( 233 );
        for ( int i = 0; i < sizeOfLedArray; i++ ) {
            digitalWrite ( ledArray[i], LOW );
        }
        delay ( 100 );
    }
}

