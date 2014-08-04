#include <LiquidCrystal.h>
const int numRows = 2;
const int numColumns = 16;

LiquidCrystal LCD(12, 11, 5,4,3,2);

void setup () {
  LCD.begin(numColumns, numRows);
  LCD.print("Hello World");
}

void loop() {
  LCD.setCursor(0,1);
  LCD.print(millis()/1000);
}
