
#include <SimpleFOC.h>
#include "DMASensor.h"
#include "DMAnChannel.h"
#include "DMAUART.h"
#include "Avatar_pinmap.h"

#define LABELPRINT(label, value) Serial.print(label); Serial.print(": "); Serial.println(value);
#define PRINT(msg) Serial.println(msg);

float com_current;

// Declare FOC, encoder, and DMA variables
DMASensor enc_hcom, enc_hpos, enc_vcom, enc_vpos, enc_hlink, enc_vlink;
DMAUART uart_hcom, uart_hpos, uart_vcom, uart_vpos, uart_link;
DMAnChannel dma_hcom_s, dma_hcom_r, dma_hpos_s, dma_hpos_r, dma_vcom_s, dma_vcom_r, dma_vpos_s, dma_vpos_r, dma_link_s, dma_link_r;

InlineCurrentSense cs_h = InlineCurrentSense(0.01, 50.0, HMOT_CS_A, HMOT_CS_B);
InlineCurrentSense cs_v = InlineCurrentSense(0.01, 50.0, VMOT_CS_A, VMOT_CS_B);

BLDCMotor motor_h = BLDCMotor(1, 3.65);  // Inits the motor. The first argument is the number of pole pairs physically in the motor; the EC-i 539482 has 4, the EC-max 272768 has 1. The second argument is motor terminal resistance
BLDCDriver3PWM driver_h = BLDCDriver3PWM(HMOT_PWM_A, HMOT_PWM_B, HMOT_PWM_C, HMOT_ENABLE);  // Inits the PWM and enable channels for the motor. Arguments are pin numbers on the FOC shield. Do not change.
BLDCMotor motor_v = BLDCMotor(1, 3.65);  // Inits the motor. The first argument is the number of pole pairs physically in the motor; the EC-i 539482 has 4, the EC-max 272768 has 1. The second argument is motor terminal resistance
BLDCDriver3PWM driver_v = BLDCDriver3PWM(VMOT_PWM_A, VMOT_PWM_B, VMOT_PWM_C, VMOT_ENABLE);  // Inits the PWM and enable channels for the motor. Arguments are pin numbers on the FOC shield. Do not change.

uint8_t encoder_command = 0x54;
int i;

void setup() {

  // Setup Encoders and DMA Channels
  Serial.begin(9600);
  enc_hcom.init(14);
  enc_hpos.init(14);
  enc_vcom.init(14);
  enc_vpos.init(14);

  uart_hcom.init(HCOM_CH, enc_uart);
  uart_hpos.init(HPOS_CH, enc_uart);
  uart_vcom.init(VCOM_CH, enc_uart);
  uart_vpos.init(VPOS_CH, enc_uart);
  
  dma_hcom_s.init(0, &encoder_command, &(uart_hcom.ctrl->DATA), uart_hcom.dma_src_tx, enc_trig_tcd);
  dma_hpos_s.init(1, &encoder_command, &(uart_hpos.ctrl->DATA), uart_hpos.dma_src_tx, enc_trig_tcd);
  dma_vcom_s.init(2, &encoder_command, &(uart_vcom.ctrl->DATA), uart_vcom.dma_src_tx, enc_trig_tcd);
  dma_vpos_s.init(3, &encoder_command, &(uart_vpos.ctrl->DATA), uart_vpos.dma_src_tx, enc_trig_tcd);

  dma_hcom_r.init(4, &(uart_hcom.ctrl->DATA), enc_hcom.getAddr(), uart_hcom.dma_src_rx, enc_receive_tcd);
  dma_hpos_r.init(5, &(uart_hpos.ctrl->DATA), enc_hpos.getAddr(), uart_hpos.dma_src_rx, enc_receive_tcd);
  dma_vcom_r.init(6, &(uart_vcom.ctrl->DATA), enc_vcom.getAddr(), uart_vcom.dma_src_rx, enc_receive_tcd);
  dma_vpos_r.init(7, &(uart_vpos.ctrl->DATA), enc_vpos.getAddr(), uart_vpos.dma_src_rx, enc_receive_tcd);

  dma_hcom_s.initPIT(2400);
  dma_hpos_s.initPIT(2400);
  dma_vcom_s.initPIT(2400);
  dma_vpos_s.initPIT(2400);

  // Setup Motor and Driver

  // Setup Horizontal Motor and Driver

  motor_h.linkCurrentSense(&cs_h);
  motor_h.linkDriver(&driver_h);

  cs_h.gain_a *= -1;
  cs_h.skip_align = true;
  cs_h.init();
  driver_h.pwm_frequency = 50000;
  driver_h.voltage_power_supply = 24;
  driver_h.init(); 

  motor_h.linkSensor(&enc_hcom);

  // set torque mode:
  motor_h.torque_controller = TorqueControlType::foc_current;
  motor_h.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor_h.controller = MotionControlType::torque;
  motor_h.sensor_direction = Direction::CCW;      //Sets motor direction with respect to the sensor direction. Must be set for EC-max; autodetection will fail.
//  motor.zero_electric_angle = 2.91;

// foc currnet control parameters (stm/esp/due/teensy)
   motor_h.PID_current_q.P = 3;
   motor_h.PID_current_q.I = 5;
   motor_h.PID_current_q.D = .00;
   motor_h.PID_current_q.limit = 0.5;
   motor_h.PID_current_d.limit = 0.5;
   motor_h.PID_current_d.P = 3;
   motor_h.PID_current_d.I = 5;
   motor_h.PID_current_q.D = .00;
   motor_h.LPF_current_q.Tf = 0.01f; // 1ms default
   motor_h.LPF_current_d.Tf = 0.01f; // 1ms default

   motor_h.voltage_limit = 10;
   motor_h.current_limit = 10;

   Serial.begin(115200);
   motor_h.useMonitoring(Serial);

//   motor_h.init();
//   motor_h.initFOC();

   PRINT("HMOT READY");
   
   // Setup Vertical Motor and Driver

  motor_v.linkCurrentSense(&cs_v);
  motor_v.linkDriver(&driver_v);

  cs_v.gain_a * -1;
  cs_v.skip_align = true;
  cs_v.init();
  driver_v.pwm_frequency = 50000;
  driver_v.voltage_power_supply = 24;
  driver_v.init(); 
  
  motor_v.linkSensor(&enc_vcom);

  // set torque mode:
  motor_v.torque_controller = TorqueControlType::foc_current;
  motor_v.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor_v.controller = MotionControlType::torque;
  motor_v.sensor_direction = Direction::CCW;      //Sets motor direction with respect to the sensor direction. Must be set for EC-max; autodetection will fail.
//  motor.zero_electric_angle = 2.91;

// foc currnet control parameters (stm/esp/due/teensy)
   motor_v.PID_current_q.P = 3;
   motor_v.PID_current_q.I = 5;
   motor_v.PID_current_q.D = .00;
   motor_v.PID_current_q.limit = 0.5;
   motor_v.PID_current_d.limit = 0.5;
   motor_v.PID_current_d.P = 3;
   motor_v.PID_current_d.I = 5;
   motor_v.PID_current_q.D = .00;
   motor_v.LPF_current_q.Tf = 0.01f; // 1ms default
   motor_v.LPF_current_d.Tf = 0.01f; // 1ms default

   motor_v.voltage_limit = 10;
   motor_v.current_limit = 10;
   
   motor_v.useMonitoring(Serial);

//   motor_v.init();
//   motor_v.initFOC();

   PRINT("VMOT READY");
  
  delay(3000);


  // Read encoder position
  enc_hpos.update();
  enc_hcom.update();
  enc_vcom.update();
  enc_vpos.update();

  // Print encoder position
  Serial.println();
  Serial.print(enc_hcom.getMechanicalAngle());
  Serial.print(" ");
  Serial.print(enc_hpos.getMechanicalAngle());
  Serial.print(" ");
  Serial.print(enc_vcom.getMechanicalAngle());
  Serial.print(" ");
  Serial.println(enc_hpos.getMechanicalAngle());
  Serial.println();


  // Prompts to make sure encoders aren't zeroed unintentionally
  while (Serial.available()) {Serial.read();}  
  Serial.println("PRESS ENTER TO ZERO");
  while (!Serial.available()) {;}
  
  while (Serial.available()) {Serial.read();}
  
  Serial.println("ARE YOU SURE? (PRESS ENTER TO INDICATE YES)");
  while (!Serial.available()) {;}
  
  delay(500);
  
  while (Serial.available()) {Serial.read();}

  Serial.println("ZEROING. CHECK THAT MOTORS CAN SPIN FREELY. \r\nPRESS ANY KEY TO CANCEL.");

  for (int ii = 5; ii > 0; ii--) {
    Serial.println(ii);
    delay(1000);
    if (Serial.available()) {
      Serial.println("ZERO OPERATION CANCELLED. RESET TEENSY OR FLASH NEW CODE.");
      while (1) {;} 
    }
  }

  // Disable dma to make sure zeroing sequence isn't interrupted
  dma_hpos_s.disable();
  dma_vpos_s.disable();

  delay(100);

  Serial.print("Zeroing VPOS and HPOS...");

  // Send encoder zero command to POS encoders
  uart_hpos.ctrl->DATA = 0x56;
  uart_hpos.ctrl->DATA = 0x5E;

  uart_vpos.ctrl->DATA = 0x56;
  uart_vpos.ctrl->DATA = 0x5E;

  Serial.println("DONE");

  delay(100);

  // Restart DMA sequence
  dma_hpos_s.enable();
  dma_vpos_s.enable();

  dma_hcom_s.disable();
  dma_vcom_s.disable();

  delay(100);

  Serial.print("Zeroing VCOM and HCOM...");

  // Enable motors
  motor_h.enable();
  motor_v.enable();

  // Snap motors to their zero electrical angle
  driver_h.setPwm(10, 0, 0);
  driver_v.setPwm(10, 0, 0);

  delay(700);

  // Send zero command to commutation encoders
  uart_hcom.ctrl->DATA = 0x56;
  uart_hcom.ctrl->DATA = 0x5E;

  uart_vcom.ctrl->DATA = 0x56;
  uart_vcom.ctrl->DATA = 0x5E;

  delay(100);

  driver_h.setPwm(0, 0, 0);
  driver_v.setPwm(0, 0, 0);

//  motor_h.disable();
//  motor_v.disable();

  dma_hcom_s.enable();
  dma_vcom_s.enable();



  delay(100);

  Serial.println("DONE");

  // update encoders

  enc_hpos.update();
  enc_hcom.update();
  enc_vcom.update();
  enc_vpos.update();
  Serial.println();
  Serial.print(enc_hcom.getMechanicalAngle());
  Serial.print(" ");
  Serial.print(enc_hpos.getMechanicalAngle());
  Serial.print(" ");
  Serial.print(enc_vcom.getMechanicalAngle());
  Serial.print(" ");
  Serial.print(enc_hpos.getMechanicalAngle());


  // Init FOC
  motor_h.initFOC(0, Direction::CCW);
  motor_v.initFOC(0, Direction::CCW);

  delay(10);

  com_current = 5;

}

void loop() {

  // If zeroing has been successful, motor will spin smoothly
  motor_v.move(com_current);
  motor_v.loopFOC();
  motor_h.move(com_current);
  motor_h.loopFOC();

  enc_hpos.update();
  enc_hcom.update();
  enc_vcom.update();
  enc_vpos.update();

  if ((i++ % 1000) == 0) {
    if ((!enc_hcom.checkParity())) {uart_hcom.flushRX();}
    if ((!enc_hpos.checkParity())) {uart_hpos.flushRX();}
    if ((!enc_vcom.checkParity())) {uart_hcom.flushRX();}
    if ((!enc_vpos.checkParity())) {uart_hpos.flushRX();}
    Serial.println();
    Serial.print(enc_hcom.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(enc_hpos.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(enc_vcom.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(enc_vpos.getMechanicalAngle());
  }
  
}
