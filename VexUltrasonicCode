/*
Uses the Vex Range Finder in conjunction with 
Arduino Mega 2560
Pin 13 Triggers the Pulse (Yellow lead)
Pin 10 Recieves the Echo  (Orange lead)
*/

const int Trig_pin =  22;   // pin for triggering pulse
const int Echo_pin = 24;     // pin for recieving echo
long duration;


void setup() {
  Serial.begin(9600);
  Serial.println ("Starting");
  // initialize the pulse pin as output:
  pinMode(Trig_pin, OUTPUT);      
  // initialize the echo_pin pin as an input:
  pinMode(Echo_pin, INPUT);     
  
  //test code to be removed
}

void loop(){
  digitalWrite(Trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_pin, LOW);
  duration = pulseIn(Echo_pin,HIGH);
   
    
   
   Serial.println("Duration:  ");
   Serial.println(duration, DEC);

}
