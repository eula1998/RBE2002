// Flame sensor Code
// RBE 2002 Team 5
// Claire Sellen, Kayla Swiston, Eula Zhong

//lowest and highest sensor readings
const int flameSensorMin = 0;
const int flameSensorMax = 1024;    // subject to change

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);   // print sensor output 
}

void loop() {
  // put your main code here, to run repeatedly:
  int flameSensorReading = analogRead(A0);   // read sensor on pin A0

  Serial.println(flameSensorReading);     // print sensor reading to serial 
  delay(100);
  // map sensor range 0-3 ft
  //int range = map(flameSensorReading, flameSensorMin, flameSensorMax,  0, 3);
}

