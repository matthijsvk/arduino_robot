/*******************************************************************************
 *                                  
 *                        (C) 2012 Dominick Vanthienen                         *
 *                    dominick dot vanthienen at gmail dot com                 *
 *                     http://code.google.com/p/sumo-robot/                    *
 *                                                                             *
 *       You may redistribute this software and/or modify it under either the  *
 *       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
 *       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
 *       discretion) of the Modified BSD License:                              *
 *       Redistribution and use in source and binary forms, with or without    *
 *       modification, are permitted provided that the following conditions    *
 *       are met:                                                              *
 *       1. Redistributions of source code must retain the above copyright     *
 *       notice, this list of conditions and the following disclaimer.         *
 *       2. Redistributions in binary form must reproduce the above copyright  *
 *       notice, this list of conditions and the following disclaimer in the   *
 *       documentation and/or other materials provided with the distribution.  *
 *       3. The name of the author may not be used to endorse or promote       *
 *       products derived from this software without specific prior written    *
 *       permission.                                                           *
 *       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
 *       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
 *       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
 *       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
 *       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
 *       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
 *       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
 *       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
 *       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
 *       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
 *       POSSIBILITY OF SUCH DAMAGE.                                           *
 *                                                                             *
 *******************************************************************************/

int enable1Pin = 5; // enable of motor controller connected to digital pin 5
int phase1Pin = 4; // phase of motor controller connected to digital pin 4
int enable2Pin = 3; // enable of motor controller connected to digital pin 3
int phase2Pin = 2; // phase of motor controller connected to digital pin 2
int modePin = 7; // mode of motor controller connected to digital pin 7
int ledPin = 13; // indicator LED 
int inPin = 8; // sensor input

int val = 0; 
int turn_direction = LOW;
int fadeValue = 0;
int increment = 1;
int up_flag = 1;
boolean still_on_white = false;
int debounce = 20; //number of val==high iterations before reset still_on_white boolean
int high_val_counter = 0;
int low_val_counter = 0;
int max_speed = 200; //value between 0 and 255

void setup() { 
  pinMode(ledPin, OUTPUT); 
  pinMode(inPin, INPUT); 
  pinMode(modePin, OUTPUT); 
  pinMode(phase1Pin, OUTPUT);
  pinMode(phase2Pin, OUTPUT);
  Serial.begin(9600);
} 

void loop()  { 
  Serial.println(fadeValue);
  //setup
  //put motor controller in phase/enable mode
  digitalWrite(modePin, HIGH);
  
  //check: if white line detected, stop (TODO: better with an interupt)
  val = digitalRead(inPin);
  if((val == LOW) && (low_val_counter<debounce) && (!still_on_white))
  {
   low_val_counter++;
   high_val_counter = 0; 
  }
  if(low_val_counter>=debounce){
    digitalWrite(ledPin, HIGH); // LED ON 
    still_on_white = true; //you're on the white!
    fadeValue = 0;
    high_val_counter = 0;
    low_val_counter = 0;
    analogWrite(enable1Pin, fadeValue); 
    analogWrite(enable2Pin, fadeValue);
    //move in other direction
    turn_direction = (!turn_direction);
    digitalWrite(phase1Pin, turn_direction);
    digitalWrite(phase2Pin, turn_direction);
  } else{    
    if((val == HIGH) && (high_val_counter<debounce))
    {
      high_val_counter++;
      low_val_counter = 0;  
    }
    if(high_val_counter>=debounce){
      still_on_white=false; //you're off the white
      digitalWrite(ledPin, LOW); // LED OFF
    }
    if((fadeValue>=max_speed) || (fadeValue>=255)) //max port value=255
    {
      up_flag = 0;
    }else{
      up_flag = 1;
    }
    fadeValue=fadeValue+up_flag*increment; //gently accelerate, till max speed
    analogWrite(enable1Pin, fadeValue); 
    analogWrite(enable2Pin, fadeValue);  
    delay(6); 
  }
}


