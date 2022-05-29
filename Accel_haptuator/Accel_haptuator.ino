#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


IntervalTimer SerialTimer;
IntervalTimer myTimer;
const int freqSerial = 3000;
const int freqAnalog = 30000;
int Frequency=300;
int flag=0;
float D_cycle=0.0;
float alpha = 0.9;
float AccelNew, AccelOld = 0.0;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
sensors_event_t event;    // create a new event for initialization
float ZdirAcc = 0.0;    // z direction acceleration

void SerialPrint () 
{
  AccelNew = AccelOld * alpha + ZdirAcc * (1 - alpha);
  D_cycle=(ZdirAcc - AccelNew)/9.8 * 7;
  if(D_cycle < 0.0) {
    D_cycle = -D_cycle;
  }
  if(D_cycle >= 1.0){
    D_cycle = 0.99;
  }
  Serial.print("D_cycle:");
  Serial.println(D_cycle);
  analogWriteFrequency(11, freqAnalog);
  analogWrite(11, int(255*D_cycle));
  AccelOld = AccelNew;
}

void setup()
{
  pinMode(11, OUTPUT);
  analogWriteFrequency(11, freqAnalog);
  analogWrite(11, int(255*D_cycle));

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

void loop()
{
//    accel.begin();
  
  // create new event
  sensors_event_t event;
  accel.getEvent(&event);
  
  //  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");Serial.println("m/s^2 ");
  //  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");
  
  ZdirAcc = event.acceleration.z;
}
