#include <SimpleFOC.h>
#include "imxrt.h"
#include "DMAnChannel.h"
#include "DMASensor.h"
#include "DMAUART.h"
#include "Avatar_pinmap.h"

DMASensor sensor;  // Instantiates magnetic position sensing
DMAUART serial4;
DMAnChannel enc_send, enc_rec;
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, MOT1_CS_A, MOT1_CS_B);  // Instantiates on-board current sensing


// Init loop-to-loop storage variables
float last_time = 0;
float this_vel;
float Tau = 1000;  // Velocity filter time constant in microseconds
float dt;
bool disabled = false;
int decimate = 0;
float com_current = 0;
uint8_t enc_command = 0x54;
int ctr = 0;
float rec_curs[10000];
float com_curs[10000];
float vels[10000];
bool done_flag = false;

BLDCMotor motor = BLDCMotor(1, 3.65);  // Inits the motor. The first argument is the number of pole pairs physically in the motor; the EC-i 539482 has 4, the EC-max 272768 has 1. The second argument is motor terminal resistance
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT1_PWM_A, MOT1_PWM_B, MOT1_PWM_C, MOT1_ENABLE);  // Inits the PWM and enable channels for the motor. Arguments are pin numbers on the FOC shield. Do not change.
float last_vel = 0;

void setup() {

  ////// Most of this is stock code taken from SimpleFOC tutorials


  // init commutation encoder
  sensor.init(14);
  serial4.init(SERIAL1_CH, enc_uart);
  enc_send = DMAnChannel(0, (uint16_t*) &enc_command, (uint16_t*) &(serial4.ctrl->DATA), serial4.dma_src_tx, enc_send_tcd);
  enc_send.initPIT(2400);
  enc_rec = DMAnChannel(5, (uint16_t*) &(serial4.ctrl->DATA), (uint16_t*) sensor.addr(), serial4.dma_src_rx, enc_receive_tcd);
   
  // current sense init hardware
  current_sense.gain_a*= -1;    // VERY IMPORTANT LINE OF CODE
  current_sense.init();
  delay(1000);
  current_sense.skip_align = true;
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // driver config
  // power supply voltage [V]
  driver.pwm_frequency = 50000;
  driver.voltage_power_supply = 12; 
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // set torque mode:
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  motor.sensor_direction = Direction::CCW;      //Sets motor direction with respect to the sensor direction. Must be set for EC-max; autodetection will fail.
//  motor.zero_electric_angle = 2.91;
 
  // foc currnet control parameters (stm/esp/due/teensy)
   motor.PID_current_q.P = 3;
   motor.PID_current_q.I = 5;
   motor.PID_current_q.D = .00;
   motor.PID_current_q.limit = 0.5;
   motor.PID_current_d.limit = 0.5;
   motor.PID_current_d.P = 3;
   motor.PID_current_d.I = 5;
   motor.PID_current_q.D = .00;
   motor.LPF_current_q.Tf = 0.001f; // 1ms default
   motor.LPF_current_d.Tf = 0.001f; // 1ms default

   motor.voltage_limit = 5; // Volts. For a conservative setting that prevents damage to the motor, set to 5.4V. This prevents the nominal current from being exceeded at stall; however this conservative setting
   // will not meet the 10N spec. increasing this value gives better torque performance, but do not apply large forces for a long time, you can cook the motor.
   motor.current_limit = 1;

  // use monitoring with serial
  Serial.begin(115200);
  motor.useMonitoring(Serial);

  // Initialize
  motor.init();
  // align sensor and start FOC
  // Motor will twitch a few times
  motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
//
//  analogWriteFrequency(MOT1_PWM_A, 100000);
//  analogWriteFrequency(MOT1_PWM_B, 100000);
//  analogWriteFrequency(MOT1_PWM_C, 100000);
  _delay(1000);

  motor.enable();


  // Cycles through each motor winding, setting 10V across each in turn, and printing the resulting current
  // This should cause predictable currents through each winding

  // Check A winding
  driver.setPwm(10, 0, 0);
  Serial.println("A B ref");
  for (int i = 0; i < 100; i++) {
    PhaseCurrent_s cur = current_sense.getPhaseCurrents();
    Serial.print(cur.a);
    Serial.print(" ");
    Serial.print(cur.b);
    Serial.print(" ");
    Serial.println(10/3.65);
    delay(10);
  }

  // Check B winding
  driver.setPwm(0, 10, 0);
  for (int i = 0; i < 100; i++) {
    PhaseCurrent_s cur = current_sense.getPhaseCurrents();
    Serial.print(cur.a);
    Serial.print(" ");
    Serial.print(cur.b);
    Serial.print(" ");
    Serial.println(10/3.65);
    delay(10);
  }

  // Check C winding
  driver.setPwm(0, 0, 10);
  for (int i = 0; i < 100; i++) {
    PhaseCurrent_s cur = current_sense.getPhaseCurrents();
    Serial.print(cur.a);
    Serial.print(" ");
    Serial.print(cur.b);
    Serial.print(" ");
    Serial.println(10/3.65);
    delay(10);
  }
  driver.setPwm(0,0,0);
  motor.disable();
  while(true){;}        ///////////////// COMMENT OUT THIS LINE TO CHECK SQUARE WAVE TRACKING PERFORMANCE OF CURRENT CONTROLLERS

  
}

//Check Square wave tracking
void loop() {
  
  if (ctr < 9999) {

    // Cycle through square wave current command
    if ((ctr % 100) < 25) {com_current = 0;} else if ((ctr % 100) < 50) {com_current = 1;} else if ((ctr % 100) < 75) {com_current = 0;} else {com_current = -1;}
    ctr++;
    DQCurrent_s curs = current_sense.getFOCCurrents(motor.electricalAngle());
    PhaseCurrent_s pcurs = current_sense.getPhaseCurrents();
    float dccur = current_sense.getDCCurrent(motor.electricalAngle());
    
    com_curs[ctr] = com_current;
    rec_curs[ctr] = curs.q;
    vels[ctr] = curs.d;
    motor.move(com_current);    // Apply calculated current
    motor.loopFOC();    // Run FOC algo.

    
  } else if (ctr < 10000) {

    // prints the results of the previous test
    motor.disable();
    motor.loopFOC();
    Serial.println("Target Actual Vel");
    for (int i = 0; i < 9999; i+=1) {
      Serial.print(com_curs[i]);
      Serial.print(" ");
      Serial.print(rec_curs[i]);
      Serial.print(" ");
      Serial.println(vels[i]);
    }
    while (true) { ; }
  }
}
