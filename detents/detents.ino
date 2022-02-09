#include <SimpleFOC.h>

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);  // Instantiates magnetic position sensing
// MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, A0, A2);  // Instantiates on-board current sensing

float target_current = 1;

BLDCMotor motor = BLDCMotor(1, 3.65);  
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

void setup() {

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24; 
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // set torque mode:
  // TorqueControlType::dc_current
  // TorqueControlType::voltage
  // TorqueControlType::foc_current
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.sensor_direction = Direction::CCW;
  // set motion control loop to be used
  

  // foc currnet control parameters (Arduino UNO/Mega)
  //  motor.PID_current_q.P = 5;
  //  motor.PID_current_q.I= 300;
  //  motor.PID_current_d.P= 5;T1
  
  //  motor.PID_current_d.I = 300;
  //  motor.LPF_current_q.Tf = 0.01f;
  //  motor.LPF_current_d.Tf = 0.01f;
  // foc currnet control parameters (stm/esp/due/teensy)
   motor.PID_current_q.P = 1;
   motor.PID_current_q.I= 0.5;
   motor.PID_current_d.P= 1;
   motor.PID_current_d.I = 0.5;
   motor.LPF_current_q.Tf = 0.02f; // 1ms default
   motor.LPF_current_d.Tf = 0.02f; // 1ms default

   motor.current_limit = 1;
   motor.voltage_limit = 5.4; // Volts. Set such that the motor, with terminal resistance 3.65 Ohms, will never overdraw at stall.

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_VOLT_Q | _MON_VOLT_D;
 // motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  
}

void loop() {
  // Also stolen code
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or torque (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
 
  motor.move(target_current );
  float current_magnitude = current_sense.getDCCurrent();
  PhaseCurrent_s  currents = current_sense.getPhaseCurrents();
//
//    Serial.print(currents.a*1000); // milli Amps
//    Serial.print("\t");
//    Serial.print(currents.b*1000); // milli Amps
//    Serial.print("\t");
//    Serial.print((0-currents.a-currents.b)*1000); // milli Amps
//    Serial.print("\t");
//   Serial.println(current_magnitude*1000); // milli Amps

  motor.monitor();
}
