// these programs use detection and move functions to make the robot fight properly

// Overview :   1. Sensor Functions
//              2. Motor  Functions
//              3. Setup & Main

//GENERAL
#include <Servo.h>

int ledArray [] = {2, 3, 4, 5};
int sizeOfLedArray = 4;

// CONSTANTS FOR DETECTION
// ULtrasonic
int ultrasonicTrigPin = 7;              // the pin that sends the signal
int ultrasonicEchoPin = 6;              // the pin that receives the signal
int ultrasonicNbMeasurements = 4;      // the nb of measurements to take the average of
int ultrasonicDistanceTreshold = 100;
// Sharp IR
int sharpServoPin = 9;
int sharpSensorPin = A0;
int sharpDistanceThreshold = 30;    //acceptable target detection: 30 cm
int scanAngle = 180;        // start at the left
int sharpAngleStep = 2; // the step at whch the servo angle changes
Servo sharpServo;               // create servo object to control a servo

//motors
Servo leftMotor;
int leftMotorServoPin = 11;
Servo rightMotor;
int rightMotorServoPin = 10;
int leftMotorValue = 0;
int rightMotorValue = 0;


//################################################################################################################################
//############################################  1. Sensor Functions   ###############################################################
//################################################################################################################################


//------------- Ultrasonic ------------------

int getUltrasonicDistance ( int ultrasonicTrigPin, int ultrasonicEchoPin ) //get the average of several measurements to be more accurate.
{
    Serial.println ( "beaming..." );
    float distance = 0.0;
    for ( int i = 0; i < ultrasonicNbMeasurements; i++ ) {
        distance += getUltrasonicDistanceOnce ( ultrasonicTrigPin, ultrasonicEchoPin );
    }
    distance = distance / ( ultrasonicNbMeasurements / 1.0 ); //take the average (using floating point calculations)
    Serial.println ( distance );
    // lightLedsDistance(distance,ledArray);
    // delay(recoverTime);
    return ( int ) distance;
}

int getUltrasonicDistanceOnce ( int ultrasonicTrigPin, int ultrasonicEchoPin )
{
    int ultrasonicRecoverTime = 10;        // the sensor needs some time to recover
    delay ( ultrasonicRecoverTime );                        // Make sure the senor is in a proper state before measuring
    int duration, inches, cm;
    ping ( ultrasonicTrigPin );                   // The PING))) is triggered by a HIGH pulse of several (10) microseconds.
    duration = pulseIn ( ultrasonicEchoPin, HIGH ); // Get the time till the signal returns.
    // convert the time into a distance
    cm = ( int ) ( microsecondsToCentimeters ( duration ) );
    return cm;
}

void ping ( int TrigPin )                       // The PING))) is triggered by a HIGH pulse of several (10) microseconds. give a short LOW before to get a clean pulse.
{
    digitalWrite ( TrigPin, LOW );
    delayMicroseconds ( 2 );
    digitalWrite ( TrigPin, HIGH );
    delayMicroseconds ( 10 );
    digitalWrite ( TrigPin, LOW );
}

float microsecondsToCentimeters ( int microseconds )
{
    // The speed of sound is 340 m/s or 29 microseconds per centimeter. return a float.ajhiiyi ube
    // The ping travels out and back, so *0.5 .
    return microseconds / 29.0 / 2;
}

//-------------------------------  SHARP IR  -------------------------------------------------

// turn sensor until object found. return 1 if target found. Also return the angle.
// if nothing found (sensor reaches 180 degrees), return 0 and the angle. Other functions see that nothing is detected.
void scan ( int results [] )
{
    results[0] = false;
    results[1] = 10 * sharpDistanceThreshold;
    if ( scanAngle < 5 || scanAngle > 175 ) { //reached the end, turn the other way around
        sharpAngleStep = -sharpAngleStep;
    }
    int targetDistance = convertAnalogToDistance ( analogRead ( sharpSensorPin ) ); //check sensor
    while ( targetDistance > sharpDistanceThreshold  && scanAngleOk() ) {       // no object detected, keep scanning till the other side
        scanAngle += sharpAngleStep;
        sharpServo.write ( scanAngle );
        targetDistance = convertAnalogToDistance ( analogRead ( sharpSensorPin ) ); //check sensor
        delay ( 2 );
    }
    if ( targetDistance < sharpDistanceThreshold ) { //enemy detected
        results[0] = true;
        results[1] = targetDistance;
    }
}

boolean scanAngleOk()
{
    if ( scanAngle > 5 && scanAngle < 175 ) {
        return true;
    } else {
        return false;
    }
}

int convertAnalogToDistance ( int sensorValue )                        // interpolation of sensor values to distances using the datasheet of the Sharp 40cm IR rangefinder.
{
    float distance;
    if ( sensorValue > 307 ) {
        distance = ( int ) ( sensorValue - 800 ) / -61.0;
    } else
        if ( sensorValue > 164 ) {
            distance = ( int ) ( sensorValue - 450 ) / -18.0;
        } else
            if ( sensorValue > 80 ) {
                distance = ( int ) ( sensorValue - 230 ) / -4.0;
            } else {
                distance = 40;
            }
    return distance;
}

//################################################################################################################################
//############################################  2. Motor Functions   ###############################################################
//################################################################################################################################

// --------- low-level move (motorControl) functions ----------

//turn the motors on
void turnOnMotors ( int leftDirection, int leftSpeedPercent, int rightDirection, int rightSpeedPercent )
{
    turnOnMotor ( leftDirection,  leftMotor , leftSpeedPercent );
    turnOnMotor ( rightDirection, rightMotor, rightSpeedPercent );
}

void turnOnMotor ( int motorDirection, Servo servo, int speedPercent ) // Pulse widths above the rest point => counterclock rotation, speed increasing as pulse width increases;                                                            // Pulse widths below the rest point => clockwise rotation, speed increasing as pulse width decreases.
{
    int offsetsign = -1;
    if ( motorDirection == 1 ) { //1 means forward; the servo's need command > 90 for that
        offsetsign = 1;
    }
    servo.write ( 90 + offsetsign * ( speedPercent * 85 / 100.0 ) ); // sets the value (range from 0 to 180). /100 because percent; *85 because the servo can only go in the interval [90 - 85; 90 + 85]
}

void turnOffMotor ( Servo servo )
{
    servo.write ( 90 );
}

//--------  high-level move functions  -----------------
void motorControl()
{
    int leftMotorDirection = 0;
    int rightMotorDirection = 0;
    if ( leftMotorValue > 0 ) { //1 means forward; the servo's need command > 90 for that
        leftMotorDirection = 1;
    } 
    if ( rightMotorValue > 0 ) { //1 means forward; the servo's need command > 90 for that
        int rightMotorDirection = 1;
    } 
    turnOnMotors ( leftMotorDirection, leftMotorValue, rightMotorDirection, rightMotorValue );
}

void stop()
{
    leftMotorValue = 0;
    rightMotorValue = 0;
    motorControl();
}

void attack()
{
    leftMotorValue = 100;
    rightMotorValue = 100;
    motorControl();
}

void forward()
{
    leftMotorValue = 50;
    rightMotorValue = 50;
    motorControl();
}

void backward()
{
    leftMotorValue = -100;
    rightMotorValue = -100;
    motorControl();
}

void evadeBackward()
{
    backward();
    turnMoving ( -30 );
    delay ( 1000 );
    attack();
}

void turnMoving ( int degres )     // forward is 0 degrees; clockwise is >0; degres better not out of [-180,180] interval (except for showing off :D)
{
    if ( degres >= 0 )  {      // turn right
        leftMotorValue  = 100;
        rightMotorValue = 100 - degres;  //turn sharper if degres is large.
        motorControl();
    } else {
        leftMotorValue  = 100 - degres;
        rightMotorValue = 100;
        motorControl();
    }                       //take a moment to let the motors work
}

void turnInPlace ( int degres )
{
    if ( degres >= 0 )  {      // turn right
        leftMotorValue  =   degres;
        rightMotorValue = - degres;  //turn sharper if degres is large.
        motorControl();
    } else {
        leftMotorValue  = - degres;
        rightMotorValue =   degres;
        motorControl();
    }
}

//------------------------------------  EXTRAS  -----------------------------------------
void countdown ( int seconds )
{
    if (seconds>=1){seconds = seconds - 1;}
    // time between led turnoffs
    float waitTime = (float)seconds/sizeOfLedArray;
    Serial.println(waitTime);

    // turn all leds on
    for ( int i = 0; i < sizeOfLedArray; i++ ) {
        digitalWrite ( ledArray[i], HIGH );
    }

    // turn leds off one by one
    for ( int i = sizeOfLedArray; i >= 0; i-- ) {
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


//----- lighting the leds, nb of leds lit is related to distance measured ------
void lightLedsDistance ( float distance, int  ledArray[] )
{
    // map inversely to the number of leds on: all on means close by
    int distanceMapped = map ( distance, 0, 170, sizeOfLedArray - 1, 0 );
    distanceMapped = constrain ( distanceMapped, 0, sizeOfLedArray - 1 );
    //Serial.println(distanceMapped);
    for ( int i = 0; i < distanceMapped; i++ ) {
        digitalWrite ( ledArray[i], HIGH );
    }
    for ( int i = distanceMapped; i < sizeOfLedArray; i++ ) {
        digitalWrite ( ledArray[i], LOW );
    }
}


//################################################################################################################################
//############################################  3. Setup &  Main   #################################################################
//################################################################################################################################

void setup()
{
    Serial.begin ( 9600 );
    pinMode ( ultrasonicEchoPin,  INPUT );
    pinMode ( sharpSensorPin,     INPUT );
    pinMode ( ultrasonicTrigPin,  OUTPUT );
    pinMode ( sharpServoPin,      OUTPUT );
    pinMode ( leftMotorServoPin,  OUTPUT );
    pinMode ( rightMotorServoPin, OUTPUT );
    for ( int i = 0; i < sizeOfLedArray; i++ ) { // set the led pins as outputs
        pinMode ( ledArray[i],    OUTPUT );
    }
    
    sharpServo.attach( sharpServoPin );
    leftMotor.attach ( leftMotorServoPin );
    rightMotor.attach( rightMotorServoPin );
    sharpServo.write ( scanAngle );
    Serial.begin(9600);
    //  delay(5000);
    //  countdown(sizeOfLedArray);
}

void loop()
{
    // first, look in front with the ultrasonic sensor.
    //if it detects an enemy, attack forward
    //else, scan with the Sharp.
    //      if it detects, turn and attack
    //      else, move forward and repeat the process
    //  int ultrasonicDistance = getUltrasonicDistance(ultrasonicTrigPin, ultrasonicEchoPin);
    //  if (ultrasonicDistance > ultrasonicDistanceTreshold){   // no object is found in front
    //    int results [] = scan();                              // scan with sharp to look around.
    //    boolean found= results[0];                            // 1 if sharp found anything, 0 else
    //    int scanAngle= results[1];                            // only use this if found == true
    //  int results[2];
    //  scan(results);
    //  boolean found= results[0];
    //  int scanAngle= results[1];
    //
    //  if (found){
    //    turn(90 - scanAngle);         // turn to face the enemy
    //    attack();
    //  }
    //  else {
    //    forward();                                        // move forward and start again
    //  }
    //  //  }
    //  //  else {// an object is found with the ultrasonic in front with the ultrasonic
    //  //    attack();
    //  //  }
    //  stop();
    stop();
    delay(2000);
    rightMotor.write ( 0 );
    leftMotor.write ( 180 );
    delay(2000);
}














