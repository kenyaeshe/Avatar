#include "DMAUART.h"
#include "imxrt.h"
#include <Arduino.h>

// This class contains data abstractions for easily configuring the registers of a UART peripheral for operation with DAM channels.

// It is NOT advised to use this class and standard SerialX.read() etc calls at the same time.

// stores UART register values to configure the encoder UART channels. see reference manual for more info.
LPUART_t enc_uart = {
  
      .GLOBAL = 0x0,
      .PINCFG = 0x0,
      .BAUD = 0xBA00001,
      .STAT = 0xC00000,
      .CTRL = 0x300000,
      .DATA = 0x1000,
      .MATCH = 0x0,
      .MODIR = 0x6,
      .FIFO = 0xC10099,
      .WATER = 0x10000
};

// stores UART register values to configure the position link UART channel. see reference manual for more info.
LPUART_t link_uart = {
  
      .GLOBAL = 0x0, // LPUART1_GLOBAL
      .PINCFG = 0x0,
      .BAUD = 0xBA00001,
      .STAT = 0xC00000,
      .CTRL = 0x300000,
      .DATA = 0x1000,
      .MATCH = 0x0,
      .MODIR = 0x6,
      .FIFO = 0xC10099,
      .WATER = 0x20000 //& (0b11 << 10)
};

// stores UART register values to configure the robot state UART channel. see reference manual for more info.
LPUART_t state_uart = {
  
      .GLOBAL = 0x0, // LPUART1_GLOBAL
      .PINCFG = 0x0,
      .BAUD = 0xBA00001,
      .STAT = 0xC00000,
      .CTRL = 0x300000,
      .DATA = 0x1000,
      .MATCH = 0x0,
      .MODIR = 0x6,
      .FIFO = 0xC10099,
      .WATER = 0x10000 //& (0b11 << 10)
};

// Data to select RX pins for each UART channel. See reference manual for more info.
pin_info_t DMAUART::rx_pins[8] = {
  {25, 2, nullptr,                         0},   //LPUART1 Serial6
  {15, 2, &IOMUXC_LPUART2_RX_SELECT_INPUT, 1},   //LPUART2 Serial3
  {16, 2, &IOMUXC_LPUART3_RX_SELECT_INPUT, 0},   //LPUART3 Serial4
  {7 , 2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2},   //LPUART4 Serial2
  {34, 1, &IOMUXC_LPUART5_RX_SELECT_INPUT, 1},   //LPUART5 Serial8
  {0 , 2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 1},   //LPUART6 Serial1
  {28, 2, &IOMUXC_LPUART7_RX_SELECT_INPUT, 1},   //LPUART7 Serial7
  {21, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 1}    //LPUART8 Serial5
};

// Data to select TX pins for each UART channel. See reference manual for more info.
pin_info_t DMAUART::tx_pins[8] = {
  {24, 2, nullptr,                         0},
  {14, 2, &IOMUXC_LPUART2_TX_SELECT_INPUT, 1},
  {17, 2, &IOMUXC_LPUART3_TX_SELECT_INPUT, 0},
  {8 , 2, &IOMUXC_LPUART4_TX_SELECT_INPUT, 2},
  {35, 1, &IOMUXC_LPUART5_TX_SELECT_INPUT, 1},
  {1 , 2, &IOMUXC_LPUART6_TX_SELECT_INPUT, 1},
  {29, 2, &IOMUXC_LPUART7_TX_SELECT_INPUT, 1}, 
  {20, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 1}
};

// Data to select TE pins for each UART channel. See reference manual for more info. Only available on Serial1, Serial8, Serial3 and Serial4
pin_info_t DMAUART::rts_pins[8] = {
  {0 , 0, nullptr,                         0},
  {18, 2, &IOMUXC_LPUART2_TX_SELECT_INPUT, 0},
  {41, 2, &IOMUXC_LPUART3_TX_SELECT_INPUT, 0},
  {0 , 0, nullptr,                         0},
  {49, 2, &IOMUXC_LPUART5_TX_SELECT_INPUT, 0},
  {54, 2, &IOMUXC_LPUART2_TX_SELECT_INPUT, 0},
  {0 , 0, nullptr,                         0},
  {0 , 0, nullptr,                         0},
};

// Transmitter empty DMA trigger for each channel. See reference manual for more info.
const uint32_t DMAUART::dma_src_txs[8] = {
  DMAMUX_SOURCE_LPUART1_TX,
  DMAMUX_SOURCE_LPUART2_TX,
  DMAMUX_SOURCE_LPUART3_TX,
  DMAMUX_SOURCE_LPUART4_TX,
  DMAMUX_SOURCE_LPUART5_TX,
  DMAMUX_SOURCE_LPUART6_TX,
  DMAMUX_SOURCE_LPUART7_TX,
  DMAMUX_SOURCE_LPUART8_TX
};

// Reciever full DMA trigger for each channel. See reference manual for more info.
const uint32_t DMAUART::dma_src_rxs[8] = {
  DMAMUX_SOURCE_LPUART1_RX,
  DMAMUX_SOURCE_LPUART2_RX,
  DMAMUX_SOURCE_LPUART3_RX,
  DMAMUX_SOURCE_LPUART4_RX,
  DMAMUX_SOURCE_LPUART5_RX,
  DMAMUX_SOURCE_LPUART6_RX,
  DMAMUX_SOURCE_LPUART7_RX,
  DMAMUX_SOURCE_LPUART8_RX
};

// Clock config values. See reference manual for more info.
volatile uint32_t* DMAUART::ccm_registers[8] = {
  &CCM_CCGR5,
  &CCM_CCGR0,
  &CCM_CCGR0,
  &CCM_CCGR1,
  &CCM_CCGR3,
  &CCM_CCGR3,
  &CCM_CCGR5,
  &CCM_CCGR6
};

// Clock config values. See reference manual for more info.
const uint32_t DMAUART::ccm_values[8] = {
  CCM_CCGR5_LPUART1(CCM_CCGR_ON),
  CCM_CCGR0_LPUART2(CCM_CCGR_ON),
  CCM_CCGR0_LPUART3(CCM_CCGR_ON),
  CCM_CCGR1_LPUART4(CCM_CCGR_ON),
  CCM_CCGR3_LPUART5(CCM_CCGR_ON),
  CCM_CCGR3_LPUART6(CCM_CCGR_ON),
  CCM_CCGR5_LPUART7(CCM_CCGR_ON),
  CCM_CCGR6_LPUART8(CCM_CCGR_ON)
};

DMAUART::DMAUART() {
  ;
}

DMAUART::DMAUART(int _ch, LPUART_t _config) {
  init(_ch, _config);
}

// Initializes UART channel
void DMAUART::init(int _ch, LPUART_t _config) {

  if ((_ch < 1) || (_ch > 8)) { return; }  // make sure channel index is in range 1-8 inclusive

  // selects RX/TX/RTS pins for the uart channel
  tx_pin_info = tx_pins[_ch-1];
  rx_pin_info = rx_pins[_ch-1];
  rts_pin_info = rts_pins[_ch-1];

  // selects DMA sources for the uart channel
  dma_src_tx = dma_src_txs[_ch-1];
  dma_src_rx = dma_src_rxs[_ch-1];

  // sets RX pin for the uart channel
  *(portControlRegister(rx_pin_info.pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
  *(portConfigRegister(rx_pin_info.pin)) = rx_pin_info.mux_val;
  if (rx_pin_info.select_input_register) *(rx_pin_info.select_input_register) = rx_pin_info.select_val;

  // sets TX pin for the uart channel
  *(portControlRegister(tx_pin_info.pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
  *(portConfigRegister(tx_pin_info.pin)) = tx_pin_info.mux_val;
  if (tx_pin_info.select_input_register) *(tx_pin_info.select_input_register) = tx_pin_info.select_val;

  // sets RTS pin for the uart channel
  if (rts_pin_info.select_input_register) *(portConfigRegister(rts_pin_info.pin)) = rts_pin_info.mux_val;
  
  *(ccm_registers[_ch-1]) |= ccm_values[_ch-1]; // enable lpuart clock

  ctrl = (LPUART_t *) (0x40184008 + (0x4000 * (_ch - 1)));  // set control structure pointer to port control memory

  *ctrl = _config;  // set control memory from given config control structure
 
  ctrl->CTRL |= 0b11 << 18; // enable lpuart port
  
}

// returns pointer to the uart DATA register
uint8_t* DMAUART::addr() {
  return (uint8_t*) &(ctrl->DATA);
}

// Reads from uart DATA register (reads new transmissions if there are any)
uint8_t DMAUART::read() {
  return (uint8_t) ((ctrl->DATA) & 0xFF);
}

// Writes to uart DATA register (sends new transmission)
void DMAUART::write(uint8_t send_byte) {
  ctrl->DATA |= send_byte;
}

// Checks if a new transmission has been recieved
int DMAUART::available() {
  return (int) (ctrl->DATA & (0b111 << 24));
}

// returns DMA transmit source number
uint32_t DMAUART::src_tx() {
  return dma_src_tx;
}

// returns DMA recieve source number
uint32_t DMAUART::src_rx() {
  return dma_src_rx;
}
// Clears any recieved data
void DMAUART::flushRX() {
  ctrl->CTRL &= ~(0b11<<18);
  while (ctrl->CTRL & (0b11<<18)) {;}
  ctrl->FIFO |= (1<<14);
  ctrl->CTRL |= (0b11<<18);
}
