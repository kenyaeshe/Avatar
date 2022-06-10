#include "DMASensor.h"

DMASensor::DMASensor(){
    ;
}

DMASensor::DMASensor(int _bit_precision, float _tc) {
  init(_bit_precision, _tc);
}

void DMASensor::init(int _bit_precision, float _tc){

    mask = ~0;

    // configures data mask based on sensor precision
    if ((_bit_precision < 16) && (_bit_precision > 0)) {
      mask = mask >> (16 - _bit_precision);
      counts = 1 << _bit_precision;
    } else {
      counts = 1 << 16;
    }

    // assigns
    value_ptr = &value;

    tc = _tc;

}

volatile uint16_t* DMASensor::getAddr() {
  return value_ptr;
}

void DMASensor::setAddr(volatile uint16_t *addr) {
  value_ptr = addr;
}

// returns float value of position, filtered by a 1st order filter
float DMASensor::getSensorAngle(){
    float cur_pos = 2*PI*((float) ((*value_ptr) & mask)) / counts;
    float dt = time_since_read;
    time_since_read = 0;
    filt_pos = ((tc * filt_pos) / (tc + dt)) + ((dt * cur_pos) / (tc + dt));
    return filt_pos;
}

// returns 14-bit value of position
uint16_t DMASensor::getRawAngle() {
  return *value_ptr;
}

// clears count of encoder rotations
void DMASensor::clearRotations() {
  full_rotations = 0;
}

// checks encoder message for data corruption
uint16_t DMASensor::checkParity() {
  uint16_t val = ((*value_ptr) & mask);
  val = val ^ (val << 8);
  val = val ^ (val << 4);
  val = ~(val ^ (val << 2));
  return ((val & ~mask)) == ((*value_ptr) & ~mask);
}
