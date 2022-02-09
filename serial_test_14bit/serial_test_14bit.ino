
HardwareSerial Serial1(PB7, PB6);  // Serial Connection to other MCU (Rx, Tx)\

uint8_t bytes_buffer[64];

long lsb_mask = 0b01111111;           // Mask to extract least significant 7 bits
long msb_mask = 0b0011111110000000;   // Mask to extract most significant 7 bits of 14 bit number
long msb_parity = 0b10000000;         // mask to enforce parity on most significant byte 
long deparity = lsb_mask;

uint8_t msb;
uint8_t lsb;

long _send;
long _send_check;
long _recieve;
int num_new_bytes;
int num_bytes_read;

void setup() {

  Serial.begin(115200);   // Init Serial connection to PC COM port
  Serial1.begin(115200);  // Serial connection to other MCU

  Serial.println("Setup Complete.");
  Serial.println("Very Complete.");


}



void loop() {

  if (Serial.available() > 0) {
    
    Serial.println("Reading new long from PC...");
    
    _send = Serial.parseInt();   // Read number from PC
    Serial.println(_send);       // Echo back for error checking
    
    msb = ((_send & msb_mask) >> 7) | msb_parity;  // Construct a byte of the form 0b1xxxxxxx where x are the most significant 7 bits of the 14 bit number
    lsb = _send & lsb_mask;                        // Construct a byte of the form 0b0yyyyyyy where y are the least significant 7 bits of the 14 bit number
    Serial1.write(msb);                          // Send bytes to the other MCU
    Serial1.flush();
    Serial1.write(lsb);
    Serial1.flush();
    _send_check = ((msb & deparity) << 7) | lsb;   // Reconstruct the 14 bit number from bytes, and echo back for error checking
    Serial.println(_send_check);
  }

  num_new_bytes = Serial1.available();
  if (num_new_bytes >= 3) {                      // Any three consecutive bytes are garaunteed to include one complete 14 bit number
    
    Serial.println("Reading new long from other MCU!");
    num_bytes_read = Serial1.readBytes(bytes_buffer, num_new_bytes);  // read all values from the serial buffer into the program buffer
    
    if (bytes_buffer[num_bytes_read-2] >> 7) {        // Check the second most recently recieved byte; if this is an msb, the two most recent bytes are reconsructed into a 14 bit number, else the 2nd and 3rd most recent bytes are used.
      msb = bytes_buffer[num_bytes_read-2];
      lsb = bytes_buffer[num_bytes_read-1];
    } else {
      msb = bytes_buffer[num_bytes_read-3];
      lsb = bytes_buffer[num_bytes_read-2];
    }
    _recieve = ((msb & deparity) << 7) | lsb;

    Serial.println(_recieve);   // Send recieved 14 bit number to the PC
  }
    
}
