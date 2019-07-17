/*
* Ultrasonic Sensor HC-SR04 and Arduino Tutorial
*
* by Dejan Nedelkovski,
* www.HowToMechatronics.com
*
*/
#define NUM_SONARS 4

// defines pins numbers
// pinos usados para ligar os sensores ultrassônicos
const int trigPin[4] = {6,7,8,9};
const int echoPin[4] = {10,11,12,13};
// defines variables
long duration[4]= {0,0,0,0};
int distance[4] = {0,0,0,0};

void setup() {
  int c = 0;
  for (; c < NUM_SONARS; c++) {
    pinMode(trigPin[c], OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin[c], INPUT); // Sets the echoPin as an Input
  }
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  int c = 0;
  
  for (; c < NUM_SONARS; c++) {
  // Clears the trigPin
    digitalWrite(trigPin[c], LOW);
    delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin[c], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[c], LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
    duration[c] = pulseIn(echoPin[c], HIGH);
  // Calculating the distance
    distance[c] = duration[c]*0.034/2.0;
  // Prints the distance on the Serial Monitor
    Serial.print("SONAR["); 
    Serial.print(c);  
    Serial.print("]: "); 
    Serial.println(distance[c]);
  }
  delay(1500);
}
