
#ifndef BATTERY_H
#define BATTERY_H

/********************************************
   Includes
 *******************************************/

/********************************************
   Defines
 *******************************************/
#define batteryMonitorPin A9
#define BATTERY_READ_SAMPLES 20
#define BATTERY_MONITOR_REFRESH_MS 1000
#define A 0.0165
#define B 0.118
#define MIN_VOLTAGE_FOR_MISSION 10.5

/********************************************
   Variables
 *******************************************/
float batteryLevel;
boolean battryPrintFlag = true;
unsigned long batteryPrevMillis;

/********************************************
   Local Functions
 *******************************************/

/********************************************
   API Functions
 *******************************************/
/*
   Read battery level in volts
*/
float battery_getVoltage() {
  unsigned long sum = 0;
  for (int i = 0; i < BATTERY_READ_SAMPLES; i++) {
    int sensorVal = analogRead(batteryMonitorPin);
//    Serial.println(sensorVal); //for debug
    sum += sensorVal;
    delay(1); //for ADC
  }
  //  Serial.println("sum = " + String(sum));
  //  Serial.println(sum / BATTERY_READ_SAMPLES);
  //  Serial.println((float)BATTERY_VOLTAGE_COEF);
  return (float(sum) / BATTERY_READ_SAMPLES) * A + B;
}

#endif
