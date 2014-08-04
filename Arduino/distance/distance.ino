

#define ECHO_PIN 8
#define TRIG_PIN 7
#define BUZZ_PIN 9



const int ledPin = 13;

void setup()
{
   Serial.begin(9600);
   pinMode(ECHO_PIN, INPUT);
   pinMode(TRIG_PIN, OUTPUT);
   pinMode(BUZZ_PIN, OUTPUT);
   digitalWrite(BUZZ_PIN, LOW);
}

void loop() 
{
   float dist = measureDistance();
   if (dist < 70) {
     digitalWrite(BUZZ_PIN, HIGH);
   } else {
     digitalWrite(BUZZ_PIN, LOW);
   }
   delay(100);
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  return ((float)duration / 58.2);
}
