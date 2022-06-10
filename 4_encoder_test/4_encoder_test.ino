
#include <SimpleFOC.h>
#include "DMASensor.h"
#include "DMAnChannel.h"
#include "DMAUART.h"
#include "Avatar_pinmap.h"

// Declare comm objects
DMASensor encoder1, encoder2, encoder3, encoder4, encoder5, encoder6, par_test;
DMAUART uart1, uart2, uart3, uart4, uart56;
DMAnChannel dma1s, dma1r, dma2s, dma2r, dma3s, dma3r, dma4s, dma4r, dma56s, dma56r;

// Init LINK buffers
uint16_t rec_link[2];
uint16_t snd_link[2];

// set encoder trigger command
const uint8_t encoder_command = 0x54;

uint32_t sink;

// init counters
int i = 0;
int j = 0;  // good sommunications reciepts
int k = 0;  // link parity misses
int l = 0;  // encoder parity misses

void setup() {

  // init encoders and DAM channels
  encoder1.init(14);
  encoder2.init(14);
  encoder3.init(14);
  encoder4.init(14);
  encoder5.init(14);
  encoder6.init(14);

  encoder3.setAddr(snd_link);
  encoder4.setAddr(snd_link+1);
  encoder5.setAddr(rec_link);
  encoder6.setAddr(rec_link+1);

  uart1.init(SERIAL8_CH, enc_uart);
  uart2.init(SERIAL3_CH, enc_uart);
  uart3.init(SERIAL4_CH, enc_uart);
  uart4.init(SERIAL1_CH, enc_uart);
  uart56.init(SERIAL6_CH, link_uart);

  dma1s.init(0, &encoder_command, &(uart1.ctrl->DATA), uart1.dma_src_tx, enc_trig_tcd);
  dma2s.init(1, &encoder_command, &(uart2.ctrl->DATA), uart2.dma_src_tx, enc_trig_tcd);
  dma3s.init(2, &encoder_command, &(uart3.ctrl->DATA), uart3.dma_src_tx, enc_trig_tcd);
  dma4s.init(3, &encoder_command, &(uart4.ctrl->DATA), uart4.dma_src_tx, enc_trig_tcd);
  
  dma1r.init(4, &(uart1.ctrl->DATA), encoder1.getAddr(), uart1.dma_src_rx, enc_receive_tcd);
  dma2r.init(5, &(uart2.ctrl->DATA), encoder2.getAddr(), uart2.dma_src_rx, enc_receive_tcd);
  dma3r.init(6, &(uart3.ctrl->DATA), encoder3.getAddr(), uart3.dma_src_rx, enc_receive_tcd);
  dma4r.init(7, &(uart4.ctrl->DATA), encoder4.getAddr(), uart4.dma_src_rx, enc_receive_tcd);

  dma56s.init(8, snd_link, &(uart56.ctrl->DATA), 0, link_send_tcd);
  dma56r.init(9, &(uart56.ctrl->DATA), rec_link, uart56.dma_src_rx, link_receive_tcd);

  dma4r.linkCh(dma56s.ch);

  // init 
  dma1s.initPIT(2400);
  dma2s.initPIT(2400);
  dma3s.initPIT(2400);
  dma4s.initPIT(2400);

  // clear link data
  uart56.ctrl->DATA;
  uart56.ctrl->DATA;
  uart56.ctrl->DATA;
  uart56.ctrl->DATA;

//  uart56.flushRX();
  
}

void loop() {
  // update encoders
  encoder1.update();
  encoder2.update();
  encoder3.update();
  encoder4.update();
  encoder5.update();
  encoder6.update();

  if ((i++ % 1000) == 0) {

    // check for data corruption
    if ((!encoder1.checkParity())) {uart1.flushRX(); l++;} else {j++;}
    if ((!encoder2.checkParity())) {uart2.flushRX(); l++;} else {j++;}
    if ((!encoder3.checkParity())) {uart3.flushRX(); l++;} else {j++;}
    if ((!encoder4.checkParity())) {uart4.flushRX(); l++;} else {j++;}
    if (!encoder5.checkParity()) {uart56.flushRX(); k++;} else {j++;}

    // print encoder values
    Serial.print(encoder1.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(encoder2.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(encoder3.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(encoder4.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(encoder5.getMechanicalAngle());
    Serial.print(" ");
    Serial.print(encoder6.getMechanicalAngle());
    Serial.print(" ");
    Serial.print( k);
    Serial.print(" ");
    Serial.print( l);
    Serial.print(" ");
    Serial.println(j);
//    Serial.println(i++);
//    Serial.println(j);
//    Serial.println(k);
  }

//  uart1.ctrl->DATA = 0x54;
//
//  Serial.println(uart1.ctrl->DATA);
  
}
