#ifndef DMAUART_h
#define DMAUART_h

#define SERIAL1_CH 6  // Map Serialx to LPUARTy
#define SERIAL2_CH 4
#define SERIAL3_CH 2
#define SERIAL4_CH 3
#define SERIAL5_CH 8
#define SERIAL6_CH 1
#define SERIAL7_CH 7
#define SERIAL8_CH 5

#include "imxrt.h"

typedef struct __attribute__((packed, aligned(4))) LPUART_t {

      volatile uint32_t GLOBAL;
      volatile uint32_t PINCFG;
      volatile uint32_t BAUD;
      volatile uint32_t STAT;
      volatile uint32_t CTRL;
      volatile uint32_t DATA;
      volatile uint32_t MATCH;
      volatile uint32_t MODIR;
      volatile uint32_t FIFO;
      volatile uint32_t WATER;
      
} LPUART_t;

typedef struct {
  uint8_t pin;
  uint32_t mux_val;
  volatile uint32_t *select_input_register;
  uint32_t select_val;
} pin_info_t;

extern LPUART_t enc_uart, link_uart;

class DMAUART {

  public:
  
    DMAUART();

    DMAUART(int _ch, LPUART_t _config);

    void init(int _ch, LPUART_t _config);

    uint8_t read();

    void write(uint8_t send_byte);

    LPUART_t* ctrl;

    uint8_t* addr();

    int available();

    uint32_t src_tx();
    uint32_t src_rx();

    void flushRX();

    uint32_t dma_src_tx;
    uint32_t dma_src_rx;

  private:
  
    uint16_t ch;

    pin_info_t rx_pin_info;
    pin_info_t tx_pin_info;
    pin_info_t rts_pin_info;
    
    static pin_info_t tx_pins[8];
    static pin_info_t rx_pins[8];
    static pin_info_t rts_pins[8];

    static volatile uint32_t* ccm_registers[8];
    static const uint32_t ccm_values[8];

    static const uint32_t dma_src_txs[8];
    static const uint32_t dma_src_rxs[8];
     
};
    
#endif
