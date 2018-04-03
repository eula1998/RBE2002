int PWM_pin = 22; 
int frequency = 10;
int delays;  

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_pin, OUTPUT); 
  delays = 1000/frequency / 2;
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(PWM_pin, HIGH); 
//  delay(delays); 
//  digitalWrite(PWM_pin, LOW); 
//  delay(delays);
    digitalWrite(PWM_pin, HIGH);
}
