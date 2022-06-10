
#include <SimpleFOC.h>
#include "DMASensor.h"
#include "DMAnChannel.h"
#include "DMAUART.h"
#include "Avatar_pinmap.h"
#include "Avatar_params.h"

#define LABELPRINT(label, value) Serial.print(label); Serial.print(": "); Serial.println(value);
#define PRINT(msg) Serial.println(msg);

DMASensor enc_hcom, enc_hpos, enc_vcom, enc_vpos, enc_hlink, enc_vlink;
DMAUART uart_hcom, uart_hpos, uart_vcom, uart_vpos, uart_link, uart_state;
DMAnChannel dma_hcom_s, dma_hcom_r, dma_hpos_s, dma_hpos_r, dma_vcom_s, dma_vcom_r, dma_vpos_s, dma_vpos_r, dma_link_s, dma_link_r, dma_state_s, dma_state_r;
DMAnChannel dma_hcom_stamp, dma_hpos_stamp, dma_vcom_stamp, dma_vpos_stamp, dma_link_stamp, dma_state_stamp;

uint32_t hcom_stamp, hpos_stamp, vcom_stamp, vpos_stamp, link_stamp, state_stamp;

uint32_t foo = 0xFFFFFFFF;

// Initiate send and recieve buffers for the interdevice communication
volatile uint16_t rec_link[3];
uint16_t snd_link[3];

// Define global state machine
enum GlobalState : uint8_t {
  STANDBY = 0,
  READY = 1,
  ACTIVE = 2,
  ERR = 3
};

// Define state machine for each axis
enum AxisState : uint8_t {
  DAMPING = 0,
  LOW_STIFF = 1,
  HIGH_STIFF = 2,
  DISABLED = 3
};
  
// Declare and init state machines
enum AxisState v_state = DAMPING;
enum AxisState h_state = DAMPING;
enum GlobalState self_state = STANDBY;
volatile enum GlobalState link_state = READY;
enum GlobalState last_link_state = STANDBY;

// Define and set the command sent to the encoders to trigger a position reading
const uint8_t encoder_command = 0x54;


// Declare and init counters
int i = 0;
int j = 0;
int k = 0;

// Init current sense modules
InlineCurrentSense cs_h = InlineCurrentSense(0.01, 50.0, HMOT_CS_A, HMOT_CS_B);
InlineCurrentSense cs_v = InlineCurrentSense(0.01, 50.0, VMOT_CS_A, VMOT_CS_B);


// Init FOC motors and driver s
BLDCMotor motor_h = BLDCMotor(1, 3.65);  // Inits the motor. The first argument is the number of pole pairs physically in the motor; the EC-i 539482 has 4, the EC-max 272768 has 1. The second argument is motor terminal resistance
BLDCDriver3PWM driver_h = BLDCDriver3PWM(HMOT_PWM_A, HMOT_PWM_B, HMOT_PWM_C, HMOT_ENABLE);  // Inits the PWM and enable channels for the motor. Arguments are pin numbers on the FOC shield. Do not change.
BLDCMotor motor_v = BLDCMotor(1, 3.65);  // Inits the motor. The first argument is the number of pole pairs physically in the motor; the EC-i 539482 has 4, the EC-max 272768 has 1. The second argument is motor terminal resistance
BLDCDriver3PWM driver_v = BLDCDriver3PWM(VMOT_PWM_A, VMOT_PWM_B, VMOT_PWM_C, VMOT_ENABLE);  // Inits the PWM and enable channels for the motor. Arguments are pin numbers on the FOC shield. Do not change.

void setup() {

  // Configure GPT1 for encoder reading timestamping
    CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON); // Enable clock to GPT1 module
  GPT1_CR = 0; // Disable for configuration
  GPT1_PR = 2400-1; // Prescale 24 MHz clock by 3 => 8 MHz
  GPT1_CR = GPT_CR_EN /* Enable timer */
    | GPT_CR_CLKSRC(1) /* 24 MHz peripheral clock as clock source */
    | GPT_CR_FRR /* Free-Run, do not reset */;


  // Setup the index value at the first position of the send and recieve buffers
  rec_link[0] = 255;
  snd_link[0] = 255;

  // Setup Encoders and MCU Linking

  // Init encoder objects
  enc_hcom.init(14, POS_TC);
  enc_hpos.init(14, POS_TC);
  enc_vcom.init(14, POS_TC);
  enc_vpos.init(14, POS_TC);
  enc_hlink.init(14, POS_TC);
  enc_vlink.init(14, POS_TC);

  // Link encoder value variables to the appropriate register in the DMA UART receive buffers
  enc_hpos.setAddr(snd_link+1);
  enc_vpos.setAddr(snd_link+2);
  enc_hlink.setAddr(rec_link+1);
  enc_vlink.setAddr(rec_link+2);

  // Init UART channels
  uart_hcom.init(HCOM_CH, enc_uart);
  uart_hpos.init(HPOS_CH, enc_uart);
  uart_vcom.init(VCOM_CH, enc_uart);
  uart_vpos.init(VPOS_CH, enc_uart);
  uart_link.init(LINK_CH, link_uart);
  uart_state.init(STATE_CH, state_uart);

  // Init DMA channels that send the encoder read command over each respective encoder UART channel
  dma_hcom_s.init(0, &encoder_command, &(uart_hcom.ctrl->DATA), uart_hcom.dma_src_tx, enc_trig_tcd);
  dma_hpos_s.init(1, &encoder_command, &(uart_hpos.ctrl->DATA), uart_hpos.dma_src_tx, enc_trig_tcd);
  dma_vcom_s.init(2, &encoder_command, &(uart_vcom.ctrl->DATA), uart_vcom.dma_src_tx, enc_trig_tcd);
  dma_vpos_s.init(3, &encoder_command, &(uart_vpos.ctrl->DATA), uart_vpos.dma_src_tx, enc_trig_tcd);

  // Init DMA channels that transfer the encoder responses out of the UART recieve buffer
  dma_hcom_r.init(4, &(uart_hcom.ctrl->DATA), enc_hcom.getAddr(), uart_hcom.dma_src_rx, enc_receive_tcd);
  dma_hpos_r.init(5, &(uart_hpos.ctrl->DATA), enc_hpos.getAddr(), uart_hpos.dma_src_rx, enc_receive_tcd);
  dma_vcom_r.init(6, &(uart_vcom.ctrl->DATA), enc_vcom.getAddr(), uart_vcom.dma_src_rx, enc_receive_tcd);
  dma_vpos_r.init(7, &(uart_vpos.ctrl->DATA), enc_vpos.getAddr(), uart_vpos.dma_src_rx, enc_receive_tcd);

  // Init DMA channels that handle sending/recieving position data over the device link bus
  dma_link_s.init(8, snd_link, &(uart_link.ctrl->DATA), 0, link_send_tcd);
  dma_link_r.init(9, &(uart_link.ctrl->DATA), rec_link, uart_link.dma_src_rx, link_receive_tcd);

  // Init DMA channels that handle sending/recieving machine state over the device state bus
  dma_state_s.init(10, &self_state, &(uart_state.ctrl->DATA), 0, state_tcd);
  dma_state_r.init(11, &(uart_state.ctrl->DATA), &link_state, uart_state.dma_src_rx, state_tcd);
  
  // trigger the DMA channel that sends position over the device link uart to trigger whenever the robot recieves a new
  // value from the HPOS encoder
  dma_hpos_r.linkCh(dma_link_s.ch);

  // trigger the DMA channel that sends the device state over the device state uart to trigger whenever the device requests
  // a new value from the HCOM encoder
  dma_hcom_s.linkCh(dma_state_s.ch);

  // Trigger the DMA channels that request new values from the encoder to trigger off of a 10kHz timer
  dma_hcom_s.initPIT(2400);
  dma_hpos_s.initPIT(2400);
  dma_vcom_s.initPIT(2400);
  dma_vpos_s.initPIT(2400);

  // Setup Horizontal Motor and Driver
  motor_h.linkCurrentSense(&cs_h);
  motor_h.linkDriver(&driver_h);

  cs_h.gain_a *= -1;   // invert current sense A's polarity. Necessary do to idiosyncracies in the driver boards
  cs_h.skip_align = true;  // prevent the FOC board from overriding user settings
  cs_h.init(); 
  driver_h.pwm_frequency = 50000;
  driver_h.voltage_power_supply = 12;
  driver_h.init(); 

  
  motor_h.linkSensor(&enc_hcom);

  // set torque mode:
  motor_h.torque_controller = TorqueControlType::foc_current;
  motor_h.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor_h.controller = MotionControlType::torque;
  motor_h.sensor_direction = Direction::CCW;      //Sets motor direction with respect to the sensor direction. Must be set for EC-max; autodetection will fail.
  motor_h.zero_electric_angle = 0;     // since encoder is zeroed at the zero electrical angle, an angle zero search can be skipped.

// foc current control parameters
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

   motor_h.voltage_limit = 5;
   motor_h.current_limit = 1.5;

   Serial.begin(115200);
   motor_h.useMonitoring(Serial);

   motor_h.init();
   motor_h.initFOC();

   PRINT("HMOT READY");
   
   // Setup Vertical Motor and Driver

  motor_v.linkCurrentSense(&cs_v);
  motor_v.linkDriver(&driver_v);


  cs_v.gain_a *= -1;
  cs_v.skip_align = true;
  cs_v.init();
  driver_v.pwm_frequency = 50000;
  driver_v.voltage_power_supply = 12;
  driver_v.init(); 

  
  motor_v.linkSensor(&enc_vcom);

  // set torque mode:
  motor_v.torque_controller = TorqueControlType::foc_current;
  motor_v.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor_v.controller = MotionControlType::torque;
  motor_v.sensor_direction = Direction::CCW;      //Sets motor direction with respect to the sensor direction. Must be set for EC-max; autodetection will fail.
  motor_v.zero_electric_angle = 0;

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

   motor_v.voltage_limit = 5;
   motor_v.current_limit = 1.5;

   Serial.begin(115200);
   motor_v.useMonitoring(Serial);

   motor_v.init();
   motor_v.initFOC();

   PRINT("VMOT READY");
}

void loop() {

  // Read any new encoder information, if available; update velocity measurements
  enc_hcom.update();
  enc_hpos.update();
  enc_vcom.update();
  enc_vpos.update();
  enc_hlink.update();
  enc_vlink.update();

  // calculate each axis' displacement with respect to the other robot
  float h_diff = enc_hlink.getMechanicalAngle()-enc_hpos.getMechanicalAngle();
  float v_diff = enc_vlink.getMechanicalAngle()-enc_vpos.getMechanicalAngle();

  // Update self state based on self conditions
  switch (self_state) {

    case ACTIVE :   // trigger standby state if axis displacement is too high, or any axis in the system exceeds velocity limits
      if (
        (abs(h_diff) > H_LINK_DISABLE_DISTANCE) ||
        (abs(v_diff) > V_LINK_DISABLE_DISTANCE) || 
        (abs(enc_hpos.getVelocity()) > VELOCITY_LIMIT) ||
        (abs(enc_vpos.getVelocity()) > VELOCITY_LIMIT) ||
        (abs(enc_hlink.getVelocity()) > VELOCITY_LIMIT) ||
        (abs(enc_vlink.getVelocity()) > VELOCITY_LIMIT)
      ) {
        self_state = STANDBY; 
        v_state = DAMPING; 
        h_state = DAMPING;
      }
      break;

      // TODO encoder and state timeouts trigger error/standby states
      
     case STANDBY : 
        if (                 // set horz axis to low stiffness if velocity+displacement criteria are met
          (abs(h_diff) < H_LINK_ENABLE_DISTANCE) ||
          (abs(enc_hpos.getVelocity()) < VELOCITY_LIMIT) ||
          (abs(enc_hlink.getVelocity()) < VELOCITY_LIMIT)
        ) {
          h_state = LOW_STIFF;
        }


        if (                // set vert axis to low stiffness if velocity+displacement criteria are met
          (abs(v_diff) < V_LINK_ENABLE_DISTANCE) ||
          (abs(enc_vpos.getVelocity()) < VELOCITY_LIMIT) ||
          (abs(enc_vlink.getVelocity()) < VELOCITY_LIMIT)
        ) {
          v_state = LOW_STIFF;
        }

        // set global state to READY if both axes are in low stiffness state
        if ((v_state == LOW_STIFF) && (h_state == LOW_STIFF)) {
          self_state = READY;
        }


      case READY :
        // drop back to standby mode if velocity/displacement critera are not met for horz axis
        if (
          (abs(h_diff) > H_LINK_DISABLE_DISTANCE) ||
          (abs(enc_hpos.getVelocity()) > VELOCITY_LIMIT) ||
          (abs(enc_hlink.getVelocity()) > VELOCITY_LIMIT)
        ) {
          h_state = DAMPING;
          self_state = STANDBY;
        }

        // drop back to standby mode if velocity/displacement critera are not met for vert axis
        if (
          (abs(v_diff) > V_LINK_DISABLE_DISTANCE) ||
          (abs(enc_vpos.getVelocity()) > VELOCITY_LIMIT) ||
          (abs(enc_vlink.getVelocity()) > VELOCITY_LIMIT)
        ) {
          v_state = DAMPING;
          self_state = STANDBY;
        }

  }

  // Update self state based on link state
  
  switch (link_state) {


    // ERR state not implemented
    case ERR :
      self_state = STANDBY;
      h_state = DAMPING;
      v_state = DAMPING;
      break;

    // drop to STANDBY mode if other robot also drops to STANDBY mode
    case STANDBY :
      if (last_link_state != STANDBY) {
        self_state = STANDBY;
        h_state = DAMPING;
        v_state = DAMPING;
      }
      break;

    // move to active mode if both robots are in READY mode (or if the other robot has already moved to ACTIVE mode)
    case READY :
      if (self_state == READY) {
        self_state = ACTIVE;
        h_state = HIGH_STIFF;
        v_state = HIGH_STIFF;
      }
      break;

    case ACTIVE :
      if (self_state == READY) {
        self_state = ACTIVE;
        h_state = HIGH_STIFF;
        v_state = HIGH_STIFF;
      }
      break;

//    default :
//      self_state = STANDBY;
//      h_state = DAMPING;
//      v_state = DAMPING;
  }
  last_link_state = link_state;

    
  float h_com_current, v_com_current;

  // calculate motor current based on state

  switch (h_state) {
    
    case LOW_STIFF :
      h_com_current = H_LOW_STIFFNESS * h_diff;
      break;

    case HIGH_STIFF :
      h_com_current = H_HIGH_STIFFNESS * h_diff;
      break;
      
    default :
      h_com_current = H_STANDBY_DAMPING * enc_hpos.getVelocity();
  }

    switch (v_state) {
    
    case LOW_STIFF :
      v_com_current = V_LOW_STIFFNESS * v_diff;
      break;

    case HIGH_STIFF :
      v_com_current = V_HIGH_STIFFNESS * v_diff;
      break;
      
    default :
      v_com_current = V_STANDBY_DAMPING * enc_vpos.getVelocity();
  }

  // Update motor current based on state

  if (i++ > 1000) {
    i = 0;
    PRINT();
    Serial.print("SELF STATE: ");
    switch (self_state) {
      case STANDBY: 
        PRINT("STANDBY");
        break;
      case READY: 
        PRINT("READY");
        break;
      case ACTIVE: 
        PRINT("ACTIVE");
        break;
      case ERR: 
        PRINT("ERROR");
        break;
    }
    Serial.print("LINK STATE: ");
    switch (link_state) {
      case STANDBY: 
        PRINT("STANDBY");
        break;
      case READY: 
        PRINT("READY");
        break;
      case ACTIVE: 
        PRINT("ACTIVE");
        break;
      case ERR: 
        PRINT("ERROR");
        break;
      default :
        PRINT(link_state);
        
    }

    Serial.print("V STATE: ");
    switch (v_state) {
      case DAMPING: 
        PRINT("DAMPING");
        break;
      case READY: 
        PRINT("LOW_STIFF");
        break;
      case HIGH_STIFF: 
        PRINT("HIGH_STIFF");
        break;
      case DISABLED: 
        PRINT("DISABLED");
        break;
    }

    
    Serial.print("H STATE: ");
    switch (h_state) {
      case DAMPING: 
        PRINT("DAMPING");
        break;
      case READY: 
        PRINT("LOW_STIFF");
        break;
      case HIGH_STIFF: 
        PRINT("HIGH_STIFF");
        break;
      case DISABLED: 
        PRINT("DISABLED");
        break;
    }

    uint32_t cur_stamp = GPT1_CNT;
    LABELPRINT("VDIFF", v_diff);
    LABELPRINT("HDIFF", h_diff);
    LABELPRINT("HCOM", enc_hcom.getMechanicalAngle());
    LABELPRINT("VCOM", enc_vcom.getMechanicalAngle());
    LABELPRINT("HPOS", enc_hpos.getMechanicalAngle());
    LABELPRINT("VPOS", enc_vpos.getMechanicalAngle());
    LABELPRINT("HLNK", enc_hlink.getMechanicalAngle());
    LABELPRINT("VLNK", enc_vlink.getMechanicalAngle());
    LABELPRINT("PARITY FAILURES", j);

    if (!enc_vlink.checkParity() || !enc_vlink.checkParity() || rec_link[0] != 255) {uart_link.flushRX(); j++;}
    if (!enc_hcom.checkParity()) {uart_hcom.flushRX(); j++;}
    if (!enc_vcom.checkParity()) {uart_vcom.flushRX(); j++;}
    if (!enc_hpos.checkParity()) {uart_hpos.flushRX(); j++;}
    if (!enc_vpos.checkParity()) {uart_vpos.flushRX(); j++;}
  }

  motor_h.move(h_com_current);
  motor_v.move(v_com_current);
//  motor_h.move(0);
//  motor_v.move(0);
//  Serial.println("Here");
  motor_h.loopFOC();
  motor_v.loopFOC();
}
