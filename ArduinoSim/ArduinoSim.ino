/*
  Mega multple serial test
 
 Receives from the main serial port, sends to the others. 
 Receives from serial port 1, sends to the main serial (Serial 0).
 
 This example works only on the Arduino Mega
 
 The circuit: 
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:
 
 created 30 Dec. 2008
 modified 20 May 2012
 by Tom Igoe & Jed Roach
 
 This example code is in the public domain.
 
 */

#define BT_P_HEADER     0xBB
#define BT_P_TAIL       0xEE

#define BT_ACK_HEADER   0xBA
#define BT_ACK_TAIL     0xAE

byte bt_p[128];
int bt_p_index = 0;
int bt_p_size = 0;

byte inByte ;

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
}

void loop() {
  bt_p_index = 0;

  while(1){
    if (Serial.available()) {
      inByte = Serial.read();
      if (inByte == BT_P_HEADER){
        //       Serial.print("HEAD\r\n");
        bt_p[bt_p_index++] = inByte;
        break;
      } 
    }
  }

  while (1){
    if (Serial.available()) {
      inByte = Serial.read();
      bt_p[bt_p_index++] = inByte;
      if (inByte == BT_P_TAIL){
        //        Serial.print("TAIL\r\n");
        break;
      } 
    }
  }

  while (1){
    if (Serial.available()) {
      inByte = Serial.read(); 
      bt_p[bt_p_index++] = inByte;
      bt_p_size = bt_p_index;
      break;
    }
  }

  bt_p[0] = BT_ACK_HEADER;
  // bt_p[1] id
  // bt_p[2] payload size
  bt_p[2] = bt_p[2] + 2;
  bt_p[bt_p_size] = BT_ACK_TAIL;

  Serial.write(bt_p, bt_p_size+1); 

}



