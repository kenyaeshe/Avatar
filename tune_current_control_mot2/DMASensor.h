#ifndef DMASensor_h
#define DMASensor_h

#include <SimpleFOC.h>

class DMASensor: public Sensor{
 public:
 
    DMASensor();

    DMASensor(int _bit_precision);
    
    void init(int _bit_precision);

    float getSensorAngle();

    uint16_t* addr();

    uint16_t getRawAngle();

  private:

   volatile uint16_t value;
   uint16_t mask;
   int counts;
   
    
};

#endif
