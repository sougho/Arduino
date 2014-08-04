int mydelay = 1000;
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  
  byte buf[4];
  int len;
  
  if ( Serial.available()) {
    Serial.readBytes((char*)buf, 4);
    Serial.write(buf, 4);
  }
  
  blink(1000);
}

void blink(int p_delay) {
  digitalWrite(ledPin, HIGH);
  delay(p_delay);
  digitalWrite(ledPin, LOW);
  delay(2*p_delay);
}
