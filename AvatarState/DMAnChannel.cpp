#include "DMAnChannel.h"
TCD_t enc_trig_tcd = {           // DMA configuration for triggering encoder
  .SADDR = 0, // Source address
  .SOFF = 0, // Adjustment to the source address made after every minor loop iteration
  .ATTR = (0b000000 << 11) | (0b000 << 8) | (0b000000 << 3) | (0b000), // SMOD | SSIZE | DMOD | DSIZE
  .NBYTES = 1,  // Set number of bytes to transfer per minor loop iteration
  .SLAST = 0, // Source address adjustment after major loop completion
  .DADDR = 0,  // Destination address
  .DOFF = 0,
  .CITER = (0b0 << 15) | 1, // CITER and BITER must be the same when the TCD memory is loaded by software. CITER will decrement with each minor loop
  .DLASTSGA = 0, 
  .CSR = (0b00 << 14) | (0b00000 << 8) | (0b0 << 5) | (0b0 << 4) | (0b0 << 3) | (0b0 << 2), // BWC | MAJORLINKCH | ESG | DREQ | INTHALF | INTMAJOR
  .BITER = (0b0 << 15) | 1 // Channel linking disabled | single service request (NO major looping)
};

TCD_t enc_receive_tcd = {        // DMA configuration for recieving encoder data
  .SADDR = 0, // Source address
  .SOFF = 0, // Adjustment to the source address made after every minor loop iteration
  .ATTR = (0b000000 << 11) | (0b000 << 8) | (0b000000 << 3) | (0b000), // SMOD | SSIZE | DMOD | DSIZE
  .NBYTES = 2,  // Set number of bytes to transfer per minor loop iteration
  .SLAST = 0, // Source address adjustment after major loop completion
  .DADDR = 0,  // Destination address
  .DOFF = 1,
  .CITER = (0b0 << 15) | 1, // CITER and BITER must be the same when the TCD memory is loaded by software. CITER will decrement with each minor loop
  .DLASTSGA = -2,
  .CSR = (0b00 << 14) | (0b00000 << 8) | (0b0 << 5) | (0b0 << 4) | (0b0 << 3) | (0b0 << 2), // BWC | MAJORLINKCH | ESG | DREQ | INTHALF | INTMAJOR
  .BITER = (0b0 << 15) | 1 // Channel linking disabled | single service request (NO major looping)
};

TCD_t link_receive_tcd = {        // DMA configuration for recieving position data over the LINK channel
  .SADDR = 0, // Source address
  .SOFF = 0, // Adjustment to the source address made after every minor loop iteration
  .ATTR = (0b000000 << 11) | (0b000 << 8) | (0b000000 << 3) | (0b000), // SMOD | SSIZE | DMOD | DSIZE
  .NBYTES = 2,  // Set number of bytes to transfer per minor loop iteration
  .SLAST = 0, // Source address adjustment after major loop completion
  .DADDR = 0,  // Destination address
  .DOFF = 1,
  .CITER = (0b0 << 15) | 3, // CITER and BITER must be the same when the TCD memory is loaded by software. CITER will decrement with each minor loop
  .DLASTSGA = -6,
  .CSR = (0b00 << 14) | (0b00000 << 8) | (0b0 << 5) | (0b0 << 4) | (0b0 << 3) | (0b0 << 2), // BWC | MAJORLINKCH | ESG | DREQ | INTHALF | INTMAJOR
  .BITER = (0b0 << 15) | 3 // Channel linking disabled | single service request (NO major looping)
};

TCD_t link_send_tcd = {        // DMA configuration for sending position data over the LINK channel
  .SADDR = 0, // Source address
  .SOFF = 1, // Adjustment to the source address made after every minor loop iteration
  .ATTR = (0b000000 << 11) | (0b000 << 8) | (0b000000 << 3) | (0b000), // SMOD | SSIZE | DMOD | DSIZE
  .NBYTES = 2,  // Set number of bytes to transfer per minor loop iteration
  .SLAST = -6, // Source address adjustment after major loop completion
  .DADDR = 0,  // Destination address
  .DOFF = 0,
  .CITER = (0b0 << 15) | 3, // CITER and BITER must be the same when the TCD memory is loaded by software. CITER will decrement with each minor loop
  .DLASTSGA = 0,
  .CSR = (0b00 << 14) | (0b00000 << 8) | (0b0 << 5) | (0b0 << 4) | (0b0 << 3) | (0b0 << 2), // BWC | MAJORLINKCH | ESG | DREQ | INTHALF | INTMAJOR
  .BITER = (0b0 << 15) | 3 // Channel linking disabled | single service request (NO major looping)
};

TCD_t state_tcd = {        // DMA configuration for sending/recieving state data over the STATE channel
  .SADDR = 0, // Source address
  .SOFF = 0, // Adjustment to the source address made after every minor loop iteration
  .ATTR = (0b000000 << 11) | (0b000 << 8) | (0b000000 << 3) | (0b000), // SMOD | SSIZE | DMOD | DSIZE
  .NBYTES = 1,  // Set number of bytes to transfer per minor loop iteration
  .SLAST = 0, // Source address adjustment after major loop completion
  .DADDR = 0,  // Destination address
  .DOFF = 0,
  .CITER = (0b0 << 15) | 1, // CITER and BITER must be the same when the TCD memory is loaded by software. CITER will decrement with each minor loop
  .DLASTSGA = 0,
  .CSR = (0b00 << 14) | (0b00000 << 8) | (0b0 << 5) | (0b0 << 4) | (0b0 << 3) | (0b0 << 2), // BWC | MAJORLINKCH | ESG | DREQ | INTHALF | INTMAJOR
  .BITER = (0b0 << 15) | 1 // Channel linking disabled | single service request (NO major looping)
};

TCD_t stamp_tcd = {        // DMA configuration for recieving encoder data
  .SADDR = 0, // Source address
  .SOFF = 1, // Adjustment to the source address made after every minor loop iteration
  .ATTR = (0b00000 << 11) | (0b000 << 8) | (0b00000 << 3) | (0b000), // SMOD | SSIZE | DMOD | DSIZE
  .NBYTES = 4,  // Set number of bytes to transfer per minor loop iteration
  .SLAST = -4, // Source address adjustment after major loop completion
  .DADDR = 0,  // Destination address
  .DOFF = 1,
  .CITER = (0b0 << 15) | 1, // CITER and BITER must be the same when the TCD memory is loaded by software. CITER will decrement with each minor loop
  .DLASTSGA = -4,
  .CSR = (0b00 << 14) | (0b00000 << 8) | (0b0 << 5) | (0b0 << 4) | (0b0 << 3) | (0b0 << 2), // BWC | MAJORLINKCH | ESG | DREQ | INTHALF | INTMAJOR
  .BITER = (0b0 << 15) | 1 // Channel linking disabled | single service request (NO major looping)
};


DMAnChannel::DMAnChannel() {
  ;
}

DMAnChannel::DMAnChannel(int _ch, void* _src, void* _dest, int _mux_src, TCD_t& _config){
  init(_ch, _src, _dest, _mux_src, _config);
}

void DMAnChannel::init(int _ch, void* _src, void* _dest, int _mux_src, TCD_t& _config){

  if ((_ch < 0) || (31 < _ch)) { return; }  // make sure channel number is in range 0-31 inclusive
  
  ch = _ch;
  
  CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);   // enables DMA clock if not already enabled
  DMA_CR = DMA_CR_GRP1PRI | DMA_CR_EMLM | DMA_CR_EDBG;   // configure overall DMA channel
  
  DMA_CERQ = _ch;  // Disable channel
  DMA_CERR = _ch;  // Clear channel error state
  DMA_CEEI = _ch;  // Disable channel interrupt
  DMA_CINT = _ch;  // Clear channel interrupt flag

  __enable_irq();    // Not sure what this does. but without it the code doesn't work
  
  TCD = (TCD_t *)(0x400E9000 + (_ch * 32));  // Set pointer to register corresponding to correct
  mux_config = (uint32_t *)(0x400EC000 + (_ch * 0x4));  // Set mux config pointer to correct channel (RM pg 85)
  
  if (_ch < 4) {
    pit_ldval = (uint32_t *) (0x40084100 + (_ch * 0x10));     // set PIT config pointers (RM pg 3042) if the DMA channel has an associated PIT
    pit_tctrl = (uint32_t *) (0x40084108 + (_ch * 0x10));
  }
  
  *TCD = _config;    // configure TCD memory with preset
  TCD->SADDR = _src; // set source address
  TCD->DADDR = _dest; // set destination address
  
  *mux_config = (uint32_t) ((0b1 << 31) | (_mux_src));  // configure request muxing

  DMA_SERQ = _ch;  // enable channel
  
}

// initializes Periodice Interrupt Timer to periodically trigger a DMA transfer
void DMAnChannel::initPIT(uint32_t ldval) {
  
  if (ch < 4) {

    DMA_CERQ = ch;
    PIT_MCR = 0x00;
    *pit_ldval = ldval;
    *pit_tctrl = 1;

    *mux_config |= (0b1 << 30);
    DMA_SERQ = ch;
    
  }
    
}

// Sets DMA to trigger a transfer on a different channel whenever this channel completes a transfer
void DMAnChannel::linkCh(uint16_t link_ch) {

  if ((link_ch < 0) || (link_ch > 31)) {return;}   // channel range guard

  TCD->CSR &= ~(0b111111 << 7);                   // clear any previous channel linking and DONE flag
  TCD->CSR |= (link_ch & 0b11111) << 8;           // set new channel to link to
  TCD->CSR |= 0b1 << 5;                          // enable channel linking
}


void DMAnChannel::enable() {
  DMA_SERQ = ch;
}

void DMAnChannel::disable() {
  DMA_CERQ = ch;
}

// Manually triggers the DMA transfer from software (useful for debuging)
void DMAnChannel::trigger() {
  DMA_SSRT = ch;
}

 
