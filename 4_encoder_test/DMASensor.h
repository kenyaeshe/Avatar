#ifndef DMASensor_h
#define DMASensor_h

#include <SimpleFOC.h>

class DMASensor: public Sensor{
 public:
 
    DMASensor();

    DMASensor(int _bit_precision);
    
    void init(int _bit_precision);

    float getSensorAngle();

    volatile uint16_t* getAddr();

    void setAddr(volatile uint16_t *addr);

    uint16_t getRawAngle();

    void clearRotations();

    uint16_t checkParity();

//  private:

   volatile uint16_t value;
   volatile uint16_t* value_ptr;
   uint16_t mask;
   int counts;
   
    
};

#endif
