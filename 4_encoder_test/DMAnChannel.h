#ifndef DMAnChannel_h
#define DMAnChannel_h

#include "imxrt.h"

//typedef struct __attribute__((packed, aligned(4))) TCD_t {
//      volatile const void * volatile SADDR;
//      int16_t SOFF;
//      union { uint16_t ATTR;
//        struct { uint8_t ATTR_DST; uint8_t ATTR_SRC; }; };
//      union { uint32_t NBYTES; uint32_t NBYTES_MLNO;
//        uint32_t NBYTES_MLOFFNO; uint32_t NBYTES_MLOFFYES; };
//      int32_t SLAST;
//      volatile void * volatile DADDR;
//      int16_t DOFF;
//      union { volatile uint16_t CITER;
//        volatile uint16_t CITER_ELINKYES; volatile uint16_t CITER_ELINKNO; };
//      int32_t DLASTSGA;
//      volatile uint16_t CSR;
//      union { volatile uint16_t BITER;
//        volatile uint16_t BITER_ELINKYES; volatile uint16_t BITER_ELINKNO; };
//        
//} TCD_t; 

typedef struct __attribute__((packed, aligned(4))) TCD_t {
  
      volatile const void * volatile SADDR;
      int16_t SOFF;
      uint16_t ATTR;
      uint32_t NBYTES;
      int32_t SLAST;
      volatile void * volatile DADDR;
      int16_t DOFF;
      volatile uint16_t CITER;
      int32_t DLASTSGA;
      volatile uint16_t CSR;
      volatile uint16_t BITER;
        
} TCD_t; 

extern TCD_t enc_receive_tcd, link_send_tcd, link_receive_tcd, enc_trig_tcd;

class DMAnChannel {
  public:

    DMAnChannel();

    DMAnChannel(int _ch, void* _src, void* _dest, int _mux_src, TCD_t& _config);

    void init(int _ch, void* _src, void* _dest, int _mux_src, TCD_t& _config);

    void enable();  // untested
 
    void disable(); // untested

    void trigger(); // untested

    void initPIT(uint32_t ldval);

    void linkCh(uint16_t link_ch);

    TCD_t *TCD;

    uint32_t *mux_config;

//  private:
  
    uint16_t ch;
    uint32_t *pit_ldval;
    uint32_t *pit_tctrl;
  
};

#endif
