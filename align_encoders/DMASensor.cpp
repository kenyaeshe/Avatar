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

    value_ptr = &value;

}

volatile uint16_t* DMASensor::getAddr() {
  return value_ptr;
}

void DMASensor::setAddr(volatile uint16_t *addr) {
  value_ptr = addr;
}

float DMASensor::getSensorAngle(){
    return 2*PI*((float) ((*value_ptr) & mask)) / counts;
}

uint16_t DMASensor::getRawAngle() {
  return *value_ptr;
}

void DMASensor::clearRotations() {
  full_rotations = 0;
}

uint16_t DMASensor::checkParity() {
  uint16_t val = ((*value_ptr) & mask);
  val = val ^ (val << 8);
  val = val ^ (val << 4);
  val = ~(val ^ (val << 2));
  return ((val & ~mask)) == ((*value_ptr) & ~mask);
}
