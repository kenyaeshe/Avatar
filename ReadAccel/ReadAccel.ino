#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
sensors_event_t event;    // create a new event for initialization
float ZdirAcc = 0.0;    // z direction acceleration

IntervalTimer SerialTimer;
const int freqSerial = 300;

void SerialPrint () {
  Serial.println(ZdirAcc);
}

void setup(void) 
{
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_2_G);
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);

  // set the timer intervals
  SerialTimer.begin(SerialPrint, 1000000 / freqSerial);

  Serial.begin(1000000);
}

void loop(void) 
{
  accel.begin();
  
  // create new event
  sensors_event_t event;
  accel.getEvent(&event);
  
  //  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");Serial.println("m/s^2 ");
//    Serial.print("+++++++++++++++++++++Z: "); Serial.print(event.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");
  
  ZdirAcc = event.acceleration.z;
}
