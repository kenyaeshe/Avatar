#include <SPI.h>  // include the SPI library:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
sensors_event_t event;    // create a new event for initialization
float ZdirAcc = 0.0;    // z direction acceleration

const int slaveSelectPin = 3;

uint16_t config_byte = 0b0011 << 12;  // sets control bits for the DAC

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

      // blinks user LED while no accelerometer is detected
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

  // set the slaveSelectPin as an output:
  pinMode (slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin(); 
  Serial.begin(9600);

  mytimer.begin(set_level, freqSerial);  // run the haptic loop on an interrupt based timer
}

void set_level() {

  Serial.println(ZdirAcc);

  uint16_t level = 2048 + ((((ZdirAcc) / 50.0)*2048.0));  // calculate DAC output value such that max negative accel is 0V, max positive accel is 5V,]
                                                          // and no accel is 2.5V

  // Prepare two-byte SPI transmission with data and control bits
  uint8_t byte1 = (level & 0xFF); 
  uint8_t byte2 = ((level | config_byte)) >> 8;
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(byte2);
  SPI.transfer(byte1);
  digitalWrite(slaveSelectPin, HIGH);
  
}

void loop() {

  // read encoder on a loop
    
  sensors_event_t event;
  accel.getEvent(&event);
  
  ZdirAcc = event.acceleration.z-9.79;
}
