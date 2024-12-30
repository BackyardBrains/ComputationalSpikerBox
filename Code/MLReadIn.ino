const int speakerPin = 3;  // GPIO2 for speaker
#define INTERVAL 25
void setup() {
  pinMode(speakerPin, OUTPUT);  // Set GPIO2 as output
  Serial.begin(115200);
}

void loop() {
  static unsigned long lastInterval = 0;
  // Toggle the pin state randomly
//  digitalWrite(speakerPin, random(0, 2));  // Random HIGH or LOW
 // delayMicroseconds(random(50, 200));     // Random delay (50-200 microseconds)
  //Serial.print("Touch ";
  if(millis() > lastInterval + INTERVAL){
  lastInterval = millis();
  Serial.print(analogRead(10));
  Serial.print(", "); //Temp
  Serial.print(analogRead(11));
  Serial.print(", "); //Light
  Serial.println( analogRead(12) );
  }
  //Serial.print(", "); //Light
  //Comment out
  //int x = analogRead(13);
  //if(x > 300){
  //Serial.print(x); //Pull-up resistor?
  //}
  
  
}
