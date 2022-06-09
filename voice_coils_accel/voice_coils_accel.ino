#include <SPI.h>  // include the SPI library:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
sensors_event_t event;    // create a new event for initialization
float ZdirAcc = 0.0;    // z direction acceleration

const int slaveSelectPin = 3;

uint16_t config_byte = 0b0011 << 12;

IntervalTimer mytimer;
const int freqSerial = 100;

int i = 0;


void setup() {

    /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1) {
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
      delay(500);

      if(accel.begin()) {break;}
    }
  }
  
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_2_G);
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);

  Serial.begin(1000000);
//
//  int ticks_per_second = 1000000/dt;
//  n_ticks = period*ticks_per_second;
  // set the slaveSelectPin as an output:
  pinMode (slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin(); 

//  for (i = 0; i < n_ticks; i++) {
//    levels[i] = (uint16_t) ((pow(2,11)-1)*( 1+sin( 2*PI*((float) i) / ((float) n_ticks) )));
//  }
//
//  i = 0;
  Serial.begin(9600);

  mytimer.begin(set_level, freqSerial);
}

void set_level() {

  Serial.println(ZdirAcc);

  uint16_t level = 2048 + ((((ZdirAcc) / 50.0)*2048.0));
//  if (i++ > 1000) {level = 0;}
//
//  else if (i++ > 2000) {level = 4095; i = 0;}
//
//  else {level = 0;}

  uint8_t byte1 = (level & 0xFF); 
  uint8_t byte2 = ((level | config_byte)) >> 8;
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(byte2);
  SPI.transfer(byte1);
  digitalWrite(slaveSelectPin, HIGH);
  
  
//  digitalWrite(slaveSelectPin, LOW);
//  SPI.transfer(byte2);
//  SPI.transfer(byte1);
//  digitalWrite(slaveSelectPin, HIGH);
//  Serial.println(voltage);
}

void loop() {
//  accel.begin();
  
  // create new event
  sensors_event_t event;
  accel.getEvent(&event);
  
  //  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");Serial.println("m/s^2 ");
//    Serial.print("+++++++++++++++++++++Z: "); Serial.print(event.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");
  
  ZdirAcc = event.acceleration.z-9.79;
}
