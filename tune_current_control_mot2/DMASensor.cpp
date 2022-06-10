#include "DMASensor.h"

DMASensor::DMASensor(){
    ;
}

DMASensor::DMASensor(int _bit_precision) {
  init(_bit_precision);
}

void DMASensor::init(int _bit_precision){

    mask = ~0;
    
    if ((_bit_precision < 16) && (_bit_precision > 0)) {
      mask = mask >> (16 - _bit_precision);
      counts = 1 << _bit_precision;
    } else {
      counts = 1 << 16;
    }
    
    
}

uint16_t* DMASensor::addr() {
  return &value;
}

float DMASensor::getSensorAngle(){
    return 2*PI*((float) (value & mask)) / counts;
}

uint16_t DMASensor::getRawAngle() {
  return value;
}
