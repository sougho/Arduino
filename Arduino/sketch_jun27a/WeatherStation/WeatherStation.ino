#include <dht11.h>

#include <MsTimer2.h>
#include <LiquidCrystal.h>
#include <Wire.h> 
#include <EEPROM.h>

#define BMPADDR  0x77
#define MAGIC 0x55
#define DHT11PIN 13

dht11 DHT11;


/* LCD Consts */

const int numRows = 2;
const int numColumns = 16;
LiquidCrystal LCD(12, 11, 5,4,3,6);

/* BMP Calibration Consts */

int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;
long b5; 

const unsigned  char OSS = 0;

/* Variables */

float temp;        // The current temp
long pressure;     // The current Pressure
float exp_press = 1013.0; // Expected pressure at the set altitude
const float sea_press = 1013.25; // Atm pressure at sea level
float altitude;   //Altitude 
float humidity;
float dht_temp;

// For Trending

float press_mov_avg = 0; // Mov Av of Pressure
float alpha = 0.9;       // For expo MA


//  Variables for state management 
// See comment for timer handlers

boolean to_read = true;
boolean to_display = true;
short display_refresh_rate = 10; // 1/2 seconds 
short data_refresh_rate = 2; // 1/2 seconds
int basic_clock = 500;
boolean first_time = true;

// Hardware PIN definitions

const int modeSwitchPin = 10;
const int counterPin = 9;

// Switch Thresholds

const int debounce_interval = 20; // ms
const int long_pressed_thr = 1000; //ms


//////////////////////////////////////////////////////////////////
//                  State Management                            //
//////////////////////////////////////////////////////////////////

typedef void (*HANDLER_FN_TYPE)();

// States 

enum states {
  DISPLAYING_PRESSURE,
  DISPLAYING_HEIGHT,
  SETUP_MODE_BLINK,
  SETUP_MODE_DIGIT,
  RESET_HEIGTH,
  NUM_STATES
};

// Inputs 

enum inputs {
  BUTTON_1_SHORT,
  BUTTON_1_LONG,
  BUTTON_2_SHORT,
  BUTTON_2_LONG,
  NUM_INPUTS
};

// One Cell for the states table 

struct state_entry {
  int next_state;  // The next state for this cell
  HANDLER_FN_TYPE action; // THe action handler for this cell
};
int current_state = DISPLAYING_PRESSURE; // Initial state

// The state table
// For STATE = i and INPUT = j this 
// Table gives the next state and the action to perform for this transition

struct state_entry state_entry_table[NUM_STATES][NUM_INPUTS];

// Handlers for display functinos corresponding to states

HANDLER_FN_TYPE state_display_fn_array[NUM_STATES];

boolean showing_pressure = true;
int decimal_place_index = 0;
int digit_to_blink = -1;

boolean rel_or_abs_hgt = 1; // Absolute by default

char curr_alt_setting[5] = {'0', '0', '0', '0'};
int curr_alt;

float curr_alt_offset;

void change_state(int this_input) {
  HANDLER_FN_TYPE f = state_entry_table[current_state][this_input].action;
  if (f != 0) f();
  if (state_entry_table[current_state][this_input].next_state != -1)
    current_state = state_entry_table[current_state][this_input].next_state;
}

// POPULATE State Table

void populate_state_table() {
  
  // Initialize all state entries to no - change 
  // And all handler functions to no - op
  
  for (int i = 0; i < NUM_STATES; i++) 
    for (int j = 0; j < NUM_INPUTS; j++) {
      state_entry_table[i][j].next_state = -1;
      state_entry_table[i][j].action = 0;
    }
    
  // Initializing all display functions to default display 
  
  for (int i = 0; i < NUM_STATES; i++) {
    state_display_fn_array[i] = display_prod_info;
  }
    
  // State table entries
  
  state_entry_table[DISPLAYING_PRESSURE][BUTTON_1_SHORT].next_state = DISPLAYING_HEIGHT;
  state_entry_table[DISPLAYING_PRESSURE][BUTTON_1_SHORT].action = toggleDisplayType;
  
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_1_SHORT].next_state = DISPLAYING_PRESSURE;
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_1_SHORT].action = toggleDisplayType;
  
  state_entry_table[DISPLAYING_PRESSURE][BUTTON_1_LONG].next_state = SETUP_MODE_BLINK;
  state_entry_table[DISPLAYING_PRESSURE][BUTTON_1_LONG].action = handle_setup;
  
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_1_LONG].next_state = SETUP_MODE_BLINK;
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_1_LONG].action = handle_setup;
  
  state_entry_table[SETUP_MODE_BLINK][BUTTON_1_LONG].next_state = DISPLAYING_PRESSURE;
  state_entry_table[SETUP_MODE_BLINK][BUTTON_1_LONG].action = set_disp_to_press;
 
  state_entry_table[SETUP_MODE_BLINK][BUTTON_1_SHORT].next_state = SETUP_MODE_DIGIT;
  state_entry_table[SETUP_MODE_BLINK][BUTTON_1_SHORT].action = set_digit_to_blink;
 
  state_entry_table[SETUP_MODE_DIGIT][BUTTON_1_SHORT].next_state = SETUP_MODE_DIGIT;
  state_entry_table[SETUP_MODE_DIGIT][BUTTON_1_SHORT].action = select_digit_to_blink;
 
  state_entry_table[SETUP_MODE_DIGIT][BUTTON_1_LONG].next_state = DISPLAYING_PRESSURE;
  state_entry_table[SETUP_MODE_DIGIT][BUTTON_1_LONG].action = set_disp_to_press;
  
  state_entry_table[SETUP_MODE_DIGIT][BUTTON_2_SHORT].next_state = SETUP_MODE_DIGIT;
  state_entry_table[SETUP_MODE_DIGIT][BUTTON_2_SHORT].action = increment_digit;
  
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_2_SHORT].next_state = DISPLAYING_HEIGHT;
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_2_SHORT].action = swtch_rel_abs_alt;
  
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_2_LONG].next_state = DISPLAYING_HEIGHT;
  state_entry_table[DISPLAYING_HEIGHT][BUTTON_2_LONG].action = set_alt_offset;
  
  // Display Task Array
  state_display_fn_array[DISPLAYING_PRESSURE] = display_pressure_state;
  state_display_fn_array[DISPLAYING_HEIGHT] = display_altitude_state;
  state_display_fn_array[SETUP_MODE_BLINK] = display_all_blink_digits;
  state_display_fn_array[SETUP_MODE_DIGIT] = blink_individual_digit;
  
}

////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
//                Action Handlers                                     ///
/////////////////////////////////////////////////////////////////////////

void toggleDisplayType() {
  showing_pressure = !showing_pressure;
  to_display = true;
}

void handle_setup (void) {
  display_refresh_rate = 1;
  digit_to_blink = 0;
}

void set_disp_to_press () {
  display_refresh_rate = 10;
  display_pressure_state();
  
  write_alt_setting_to_eeprom();
  
  char t [5];
  for (int i = 0; i < 4; i++) {
    t[i] = curr_alt_setting[i];
  }
  t[4] = 0;
  curr_alt = atoi(t);
  exp_press = get_expected_pressure();
}

void select_digit_to_blink() {
  digit_to_blink ++;
  if (digit_to_blink > 3) 
    digit_to_blink = 0;
}

void set_digit_to_blink() {
    digit_to_blink = 0;
}

void increment_digit () {
   increment_char(digit_to_blink);
}

void swtch_rel_abs_alt() {
  rel_or_abs_hgt = !rel_or_abs_hgt;
  display_altitude_state();
}

void set_alt_offset() {
  LCD.clear();
  LCD.print("Setting Altitude Offset");
  delay(100);
  LCD.clear();
  curr_alt_offset = get_altitude(press_mov_avg, temp);
  display_altitude_state();
}

///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
//           State Display Funcitons                             //
///////////////////////////////////////////////////////////////////

void display_pressure_state(void) {
  exp_press = get_expected_pressure();
  LCD.clear();
  print_pressure(press_mov_avg);
  print_temparature(temp);
  print_relative_humidity(humidity);
}

void display_altitude_state(void) {
  LCD.clear();
  if (rel_or_abs_hgt) {
    print_altitude(get_altitude(press_mov_avg, temp), "ABS");
  } else {
    print_altitude(get_altitude(press_mov_avg, temp) - curr_alt_offset, "REL");
  }
  print_temparature(temp);
  print_relative_humidity(humidity);
}

void display_all_blink_digits() {
  
  static boolean show = true;
  LCD.clear();
 
  LCD.print("Set Altitude: ");
  if (show == true) {
    LCD.setCursor(0,1);
    LCD.print(" ");
    LCD.print(curr_alt_setting);
  }
  show = !show;
}

void blink_individual_digit() {
  LCD.clear();
  LCD.print("Set Altitude:");
  static boolean show = true;
  
  LCD.setCursor(1, 1);
  LCD.print(curr_alt_setting);
  if (!show) {
   LCD.setCursor(1 + digit_to_blink, 1);
   LCD.print(" ");
  }
  show = !show;
}

void display_prod_info () {
  LCD.clear();
  LCD.print("Weather Station Version 1.0");
}
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
//           Main Programs                                       //
///////////////////////////////////////////////////////////////////

void setup () {
  
  LCD.begin(numColumns, numRows);
  populate_state_table();
  set_pin_modes();
  Serial.begin(9600);
  Wire.begin();
  read_calibration_values();
  MsTimer2::set(basic_clock, timer_handler);
  MsTimer2::start();
  read_alt_setting_from_eeprom();
  
}



void loop() {
  
  process_button_inputs();
  
  read_sensor_data();
  
  call_display_function();
  
 
}


//////////////////////////////////////////////////////////////////////////
//              Key Handlers                                            //
//////////////////////////////////////////////////////////////////////////


int mode_key_pressed() {
  static int last_state = 1;
  static long last_timestamp;
  
  if (digitalRead(modeSwitchPin) != last_state) {
    last_state = (1 - last_state);

    if (last_state == 1) {
      return (millis() - last_timestamp);
    }
    else {
      last_timestamp = millis();
      return 0;
    }
  }
} 
  
int counter_key_pressed() {
  static int last_state = 1;
  static long last_timestamp;

  if (digitalRead(counterPin) != last_state) {
    last_state = (1 - last_state);

    if (last_state == 1) {
      return (millis() - last_timestamp);
    }
    else {
      last_timestamp = millis();
      return 0;
    }
  } 
} 

//////////////////////////////////////////////////////////////////////////
//                      Timer Handlers                                  //
//////////////////////////////////////////////////////////////////////////

void timer_handler(void) {

  static short display_tick = 0;
  static short data_tick = 0;
  
  display_tick ++; 
  if (display_tick >= display_refresh_rate) {
    to_display = true;
    display_tick = 0;
  }
  
  data_tick ++; 
  if (data_tick >= data_refresh_rate) {
    to_read = true;
    data_tick = 0;
  }
}

//////////////////////////////////////////////////////////////////////////
//                Display Utilities                                     //
//////////////////////////////////////////////////////////////////////////

void print_pressure(float pressure) {
  LCD.setCursor(0,0);
  LCD.print("P:");
  LCD.print(pressure, 1);
  LCD.print("hp");
  
  if ((pressure - exp_press) < - 2.5) 
    LCD.print("<<");
  else if (((pressure - exp_press) >= -2.5 ) && ((pressure - exp_press) <= -1.5 ))
    LCD.print("<");
  else if ( ((pressure - exp_press) > -1.5 ) && ((pressure - exp_press) < 1.5)) 
    LCD.print("=");
  else if (((pressure - exp_press) >= 1.5 ) && ((pressure - exp_press) <= 2.5 ))
  LCD.print(">");
  else if ((pressure - exp_press > 2.5))
    LCD.print(">>");
}

void print_altitude(float altitude, char *str) {
  LCD.setCursor(0,0);
  LCD.print("H:");
  LCD.print(altitude, 1);
  LCD.print("m ");
  LCD.print(str);
}

void print_temparature(float temp){
  LCD.setCursor(0,1);
  LCD.print("T:");
  LCD.print(temp,  1);
  LCD.print("c");
}

void print_relative_humidity(float humid) {
  LCD.setCursor(8,1);
  LCD.print("RH:" );
  LCD.print(humid,1);
  LCD.print("%");
}

//////////////////////////////////////////////////////////////////////////
//                    Sensor Functions                                  //
//////////////////////////////////////////////////////////////////////////

long get_pressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

short get_temp(unsigned int ut)
{

  unsigned int ut1 = 123;

  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}


unsigned long read_uncomp_pressure()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMPADDR);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMPADDR);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMPADDR, 3);

  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

unsigned int read_uncomp_temp()
{
  unsigned int ut;
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMPADDR);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = read_int_from_bmp(0xF6);
  return ut;
}

char read_char (unsigned char address) {

  Wire.beginTransmission(BMPADDR);

  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMPADDR,1);
  while(!Wire.available());
  return Wire.read();
}

int read_int_from_bmp(unsigned char address) {

  unsigned char msb, lsb;

  Wire.beginTransmission(BMPADDR);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMPADDR,2);
  while(Wire.available() < 2);

  msb = Wire.read();
  lsb = Wire.read();

  return(int)((msb << 8 ) | lsb); 
}

void read_calibration_values () {

  ac1 = read_int_from_bmp(0xAA);
  ac2 = read_int_from_bmp(0xAC);
  ac3 = read_int_from_bmp(0xAE);
  ac4 = read_int_from_bmp(0xB0);
  ac5 = read_int_from_bmp(0xB2);
  ac6 = read_int_from_bmp(0xB4);
  b1 = read_int_from_bmp(0xB6);
  b2 = read_int_from_bmp(0xB8);
  mb = read_int_from_bmp(0xBA);
  mc = read_int_from_bmp(0xBC);
  md = read_int_from_bmp(0xBE);
}

float get_relative_humidity() {
  return 99;
}

float get_altitude(float press, float temp) {
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

//////////////////////////////////////////////////////////////////////////////
//                      End Sensor Functions                                //
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//                      Utility Functions                                   //
//////////////////////////////////////////////////////////////////////////////

float get_expected_pressure() {
  float p = sea_press * pow((1 - (float)curr_alt/44330), 5.25);
  return p;
}

void print_data_to_serial(float pressure, float temp, float humidity) {
    
      Serial.print(pressure);    
      Serial.print(",");
      Serial.print(temp);
      Serial.print(",");
      Serial.print(humidity);
      Serial.print(",");
      Serial.print(dht_temp);
      Serial.print(",");
      Serial.println(exp_press);
}

void write_alt_setting_to_eeprom() {
  EEPROM.write(0, MAGIC);
  for (int i = 0; i < 4; i++) {
     EEPROM.write(i+1, curr_alt_setting[i]);
  }
}

void read_alt_setting_from_eeprom() {
  int magic = EEPROM.read(0);
  if (magic != MAGIC ) 
    return;
  for (int i = 0; i < 4; i++) {
     curr_alt_setting[i] = EEPROM.read(i+1);
  }
  curr_alt = convert_height_from_str();
}

void process_button_inputs() {
  
  int  mode_key_press_time = mode_key_pressed();
  
  if (mode_key_press_time > long_pressed_thr) {
    change_state(BUTTON_1_LONG);
  }
  else if (mode_key_press_time > debounce_interval) {
    change_state(BUTTON_1_SHORT);
  } 
  
  int  counter_kp_time = counter_key_pressed();
  
  if (counter_kp_time > long_pressed_thr) {
    change_state(BUTTON_2_LONG);
  }
  else if (counter_kp_time > debounce_interval) {
    change_state(BUTTON_2_SHORT);
  }   
}

void read_sensor_data() {
  
   //Serial.print("abcdef");
   if (to_read || first_time ){
    
    temp = (float)get_temp(read_uncomp_temp())/10;
    float pressure = (float)get_pressure(read_uncomp_pressure())/100;

    to_read = false;
    first_time = false;

    if (press_mov_avg == 0) {
      press_mov_avg = pressure;
    } 
    else {
      press_mov_avg = alpha * pressure + (1-alpha) * press_mov_avg;
    }
     
    int status = DHT11.read(DHT11PIN);
    if (status == DHTLIB_OK) {
      humidity = DHT11.humidity;
      dht_temp = DHT11.temperature;
    }  else {
      Serial.print("Some problem with dht11");
      Serial.print(status);
      Serial.print("\n");
    }
    
    //Serial.print("test");
    print_data_to_serial(pressure, temp, humidity);
  }
}

void call_display_function() {
  if (first_time || to_display) {
      HANDLER_FN_TYPE f = state_display_fn_array[current_state];
      if (f != 0)
        f();
      to_display = false;
    } 
}

void set_pin_modes() {
  pinMode(modeSwitchPin, INPUT);
  digitalWrite(modeSwitchPin, HIGH);
  pinMode(counterPin, INPUT);
  digitalWrite(counterPin, HIGH);
}

void increment_char(int index) {
  char digit = curr_alt_setting[index];
  digit ++; 
  if (digit > '9') {
    digit = '0';
  }
  curr_alt_setting[index] = digit;
}

int convert_height_from_str() {
  
  char t [5];
  for (int i = 0; i < 4; i++) {
    t[i] = curr_alt_setting[i];
  }
  t[4] = 0;
  return( atoi(t));
}
//////////////////////////////////////////////////////////////////////////////




