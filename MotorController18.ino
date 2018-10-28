/*
 * ============== ChemE Car Motor Controller v1.1 ==============
 * Author: Tanner Bobak, University of Colorado Boulder
 *
 * Notes: 
 * - Photoresistor resistance increases in darker conditions 
 *   (about 700ohm lit, 1Mohm dark), so voltage read will drop.
 */

#include <Servo.h>

// Digital Pin Configuration
const int dp_spect = 0;  // Spectrophotometer light
const int dp_batok = 1;  // Battery OK light
const int dp_pemok = 2;  // Fuel Cell OK light
const int dp_swton = 4;  // Read on switch
const int dp_regon = 8;  // DC/DC conv. on
const int dp_serv1 = 9;  // Motor 1
const int dp_serv2 = 10; // Motor 2
const int dp_servs = 11; // Stirring servo

// Analog Pin Configuration
const int ap_rebat = 0;  // Battery voltage (VIN) divider
const int ap_repem = 1;  // PEM voltage divider
const int ap_photo = 2;  // Photoresistor

// Voltage divider 
const float div_const = 4.7/(10+4.7);

// "OK" voltages for battery and PEM
const float vbat_ok = 5.5;
const float vpem_ok = 7;

// Servo objects
Servo servo1;
Servo servo2;
Servo stirvo;

// Spectrophotometer configuration variables
const float vdrop_frac = 0.8; // Deactivate
const int stir_speed = 9; // 0-9, 0 is off, 9 is max speed.
int base_light = 1023; // Baseline light level, used as reference
int setup_samples = 10; // Samples to take for baseline light value.

// Timers (NO EFFECT ON STOPPING, DEBUG PURPOSES ONLY)
unsigned long run_time;       // Tracks time since start of run.
unsigned long update_time;    // Slows update rate of some messages.
const int update_rate = 5000; // Update rate in ms

// Debug level, prints a lot of output regarding state
// 0 = no output to serial
// 1 = light levels
// 2 = voltages (bat, PEM)
// 3 = timing data (run time)
// 4 = all
const int debug_level = 3;

// Move car or not
bool run_car = false;

// Has the iodine clock changed color?
bool color_change = false;

// Prints debug message
void debug(int level, String msg) {
  if(debug_level == level || debug_level == 4)
    Serial.println("[DEBUG " + String(level) + "] " + msg);
}

// Convert millis to seconds
float getSeconds(unsigned long m) {
  return m/1000.0;
}

float analogToV(int v) {
  return ((float(v)/1023)*5.0)/div_const;
}

// Attach the motors that move the car and run
void attachAndWrite() {
    // Turn on converter.
    digitalWrite(dp_regon, HIGH);
  
    if(!servo1.attached())
      servo1.attach(dp_serv1);
    if(!servo2.attached())
      servo2.attach(dp_serv2);

    // Adjust these for steering
    servo1.write(180);
    servo2.write(0);
}

// Detach the motors that move the car and stop
void detachAndWrite() {
    servo1.write(90);
    servo2.write(90);

    servo1.detach();
    servo2.detach();

    // Turn off converter.
    digitalWrite(dp_regon, LOW);
}

void setup() {
  if(debug_level > 0) Serial.begin(9600);
  debug(4, "===== RESET =====");

  // Pin Modes
  pinMode(dp_spect, OUTPUT);
  pinMode(dp_batok, OUTPUT);
  pinMode(dp_pemok, OUTPUT);
  pinMode(dp_swton, INPUT);
  pinMode(dp_regon, OUTPUT);
  pinMode(dp_serv1, OUTPUT);
  pinMode(dp_serv2, OUTPUT);
  pinMode(dp_servs, OUTPUT);

  // Turn off the DC/DC converter
  digitalWrite(dp_regon, LOW);

  // Turn on the spectrophotometer
  digitalWrite(dp_spect, HIGH);

  // Set up the spectrophotometer
  debug(4, "Setting up spectrophotometer...");
  int sum = 0;
  for(int i = 0; i < setup_samples; i++) {
    int value = analogRead(ap_photo);
    sum += value;
    debug(4, "Read " + String(value) + " for light level.");
    delay(50); // Temporal spacing
  }

  base_light = sum/setup_samples;
  debug(1, "Baseline light level: " + String(base_light));

  // Start stirring
  debug(4, "Starting stirplate...");
  stirvo.attach(dp_servs);
  stirvo.write((stir_speed-9)*-10);

  // Flash lights to let us know its ready to go.
  debug(4, "Spectrophotometer ready.");
  for(int i = 0; i < 4; i++) {
    digitalWrite(dp_batok, LOW);
    digitalWrite(dp_pemok, LOW);
    delay(150);
    digitalWrite(dp_batok, HIGH);
    digitalWrite(dp_pemok, HIGH);
    delay(350);
  }

  delay(500);

  // Show state of voltages
  debug(4, "Reading voltages...");
  float vbat = analogToV(analogRead(ap_rebat));
  float vpem = analogToV(analogRead(ap_repem));

  debug(4, "Battery volatage: " + String(vbat) + "V");
  debug(4, "PEM volatage: " + String(vpem) + "V");
  
  if(vbat < vbat_ok) {
    digitalWrite(dp_batok, LOW);
    debug(2, "Low battery voltage: " + String(vbat) + "V");
  }

  if(vpem < vpem_ok) {
    digitalWrite(dp_pemok, LOW);
    debug(2, "Low fuel cell voltage: " + String(vpem) + "V");
  }

}

void loop() {
  // Start switch handling
  if(!run_car && !color_change && digitalRead(dp_swton) == HIGH) {
    run_car = true;
    attachAndWrite();
    run_time = millis();
    debug(3, "Starting run at T=" + String(getSeconds(run_time)) + "s");
  } else if(!color_change && digitalRead(dp_swton) == LOW) {
    run_time = 0;
    if(run_car) debug(3, "Run Aborted");
    run_car = false;
    detachAndWrite();
  }

  int light_value = analogRead(ap_photo);
  if(millis() - update_time > update_rate) {
    update_time = millis();
    debug(1, "Light level: " + String(light_value) + " -~- " + 
              int(vdrop_frac*base_light));
  }

  if(run_car && light_value < int(vdrop_frac*base_light)) {
    run_car = false;
    color_change = true;
    detachAndWrite();
    unsigned long cur_time = millis();
    debug(3, "Ending run at T=" + String(getSeconds(cur_time)) + 
             "s. Run Time is " + String(getSeconds(cur_time-run_time)) +
             "s.");
    run_time = 0;
  }
}
