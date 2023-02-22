/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#define OUT_PIN 3
#define IN_PIN 19
#define INT_PIN 21


byte inputbuffer[4];

bool stopsend = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(IN_PIN, INPUT_PULLUP);
  pinMode(INT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial1.begin(14000, SERIAL_8N2); //14300 ska nog va rätt
  attachInterrupt(digitalPinToInterrupt(INT_PIN), readData, FALLING);
  Serial.println("start");
}

byte calculateChecksum(byte* data, int length) {
  byte checksum = 255;
  for (int i = 1; i < length; i++) {
    checksum -= data[i];
  }
  return checksum;
}

void addData(byte aByte) {
  inputbuffer[0] = inputbuffer[1];
  inputbuffer[1] = inputbuffer[2];
  inputbuffer[2] = inputbuffer[3];
  inputbuffer[3] = ~aByte;
  byte check = calculateChecksum(inputbuffer, 3);
  if (check == inputbuffer[3]) {
    //valid packet 
    if (inputbuffer[0] == 0x03) {
      if (inputbuffer[1] == 0xD3) {
        Serial.print("valid packet ");
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[0], HEX);
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[1], HEX); 
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[2], HEX); 
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[3], HEX); 
        Serial.print(" Bilnr:");
        Serial.println((byte)inputbuffer[2]); 
        sendResponse(inputbuffer[2]);
        return;
      }
    }
    Serial.print("valid packet ");
    Serial.print(" 0x");
    Serial.print((byte)inputbuffer[0], HEX);
    Serial.print(" 0x");
    Serial.print((byte)inputbuffer[1], HEX); 
    Serial.print(" 0x");
    Serial.print((byte)inputbuffer[2], HEX); 
    Serial.print(" 0x");
    Serial.println((byte)inputbuffer[3], HEX);  
  }
  else {
    if (inputbuffer[1] == 0xD3 && inputbuffer[2] == 0x36) {
        Serial.print("trasigt paket packet ");
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[0], HEX);
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[1], HEX); 
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[2], HEX); 
        Serial.print(" 0x");
        Serial.print((byte)inputbuffer[3], HEX); 
        Serial.print(" check 0x");
        Serial.print((byte)check, HEX); 
        Serial.print(" Bilnr:");
        Serial.println((byte)inputbuffer[2]); 
        //sendResponse(inputbuffer[2]);
        return;
      }
  }
}

void sendResponse(byte nr) {
  byte response[]={0x03, 0x53, 0x36, 0x76};
  response[2] = nr;
  response[3] = calculateChecksum(response, 3);
  sendData(response, 4);
  Serial.print("send response");
  Serial.print(nr);
  Serial.print(" 0x");
  Serial.print((byte)response[0], HEX);
  Serial.print(" 0x");
  Serial.print((byte)response[1], HEX); 
  Serial.print(" 0x");
  Serial.print((byte)response[2], HEX); 
  Serial.print(" 0x");
  Serial.println((byte)response[3], HEX); 
}

void printBin(byte aByte) { 
  for (int8_t aBit = 7; aBit >= 0; aBit--) {
    Serial.write(bitRead(aByte, aBit) ? '1' : '0'); 
  } 
  Serial.println();
}

byte banan = 0;
bool flipp;
// the loop function runs over and over again forever
void loop() {

  if (Serial.available()) {
    readNumber();
  }
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(6);                       // wait for a second
  //byte cmd[]={0xFC,0x35,0x55,0x74};
  byte cmd[]={0x03, 0xCA, 0xAA, 0x8B};
  
  byte response[]={0x03, 0x53, 0x36, 0x76};
  //Serial.println(calculateCrc8(cmd, 3), HEX);
  unsigned char data[] = {0x03, 0x53, 0x36}; // fäörväntad checksumma 0xDC
  int len = 3;
  unsigned char crc = 0x00;
  unsigned char checksum;
//  for (unsigned char poly = 0; poly<255; poly++) {
//    checksum = calculateChecksum(data, len);
//    Serial.print("Checksum: 0x");
//      Serial.println(checksum, HEX); //prints checksum in hex format
//    if (checksum == 0x74) {
//      
//      Serial.print("poly: 0x");
//      Serial.println(poly, HEX); //prints checksum in hex format
//    }
//  
//  }
  unsigned char test = 0xa5;
  //Serial.print("0b");
  //printBin((byte)~test); 
//  Serial.print("0x");
//  Serial.println((byte)~test, HEX); 
//  Serial.print("byte:");
//  Serial.println((byte)~test); 
//  Serial.print("char:");
//  Serial.println((char)~test); 
  //sendData(cmd, 4);
//  checksum = calculateChecksum(data, 3);
//    Serial.print("Checksum: 0x");
//      Serial.println(checksum, HEX); //prints checksum in hex format
  
  if (stopsend == 1) {
    //sendData(response, 4);
    //Serial.println("response");
  }else {
    sendLoopPing();
  }
  //banan ++;
  
  //if (Serial1.available()){
    //char inByte = Serial1.read();
    //Serial.println(inByte, HEX);
  //}
  bool lys = false;
  while (Serial1.available()) {
    byte inByte = Serial1.read();
    addData(inByte);
    //Serial.println(inByte, HEX);
    flipp = !flipp;
    lys = true;
  }
  Serial1.write(cmd, 4);
  digitalWrite(LED_BUILTIN, lys);
}

void sendLoopPing() {
  // Datat som tidtagningsloopen skickar hela tiden för att fråga efter nummer
  byte cmd[]={0x03, 0xCA, 0xAA, 0x8B}; //03
  sendData(cmd, 4);
}

int readNumber() {
  int num = 0;
  boolean reading = false;

  char c = Serial.read();
  
  if (c == 'w') {
    delay(200);
    reading = true;
  }
  if (c == 's') {
    stopsend = !stopsend;
    Serial.println("stopsend");
    return 0;
  }
  while (Serial.available()) {
    c = Serial.read();

    if (reading && isDigit(c)) {
      num = num * 10 + (c - '0');
    } else if (reading && !isDigit(c)) {
      Serial.print("save number to transponder: ");
      Serial.println(num);
      sendSaveNrCommand(num);
      break;
    } 
  }
  //saveNumber(num)
  
  return num;
}

#define PULSE_DELAY 27
#define PULSE_DELAY_OFF 119
#define PULSE_DELAY_ON 12

void delay1us(int us) {
  __asm__("nop\n\t");
  __asm__("nop\n\t");
}
void delaySmall(int counts) {
  for (int i=0;i< counts;i++) {
    __asm__("nop\n\t");
  }
  
}
void bitPulse(bool bits) {
  
  digitalWrite(OUT_PIN, bits);
  //delayMicroseconds(12);
  for (int i=0;i< PULSE_DELAY_ON;i++) {
    __asm__("nop\n\t");
  }
  __asm__("nop\n\t");
  __asm__("nop\n\t");
  __asm__("nop\n\t");
  __asm__("nop\n\t");
  __asm__("nop\n\t");

  digitalWrite(OUT_PIN, HIGH);
  //delayMicroseconds(PULSE_DELAY);
  delaySmall(PULSE_DELAY_OFF);
}
void sendData(byte data[], int len) {
  //byte cmd[]={0xFC,0x35,0x55,0x74};
  cli();                            // Disable interrupts

  byte value;
  for (int x = 0;x<len;x++) {
    value = data[x];
    // Start bit
    bitPulse(0);
    for(int i = 0, mask = 1; i < 8; i++, mask = mask << 1)  {
      if (value & mask)  {
          // bit is on
          bitPulse(0);
      }
      else {
          // bit is off
          bitPulse(1);
      }
    }
    // Stop bit
    bitPulse(1);
    bitPulse(1);
  }
  sei();                            // Enable interrupts
  //digitalWrite(LED_BUILTIN, HIGH);
}

void sendSaveNrCommand(byte nr) {
  byte response[]={0x03, 0xE2, 0x01, 0x00};
  response[2] = nr;
  response[3] = calculateChecksum(response, 3);
  sendData(response, 4);
}

#define PULSE_READ_DELAY 77
bool readflip;
byte bitReads() {
  bool out = 1;
  //digitalWrite(TEST_PIN, HIGH);
  PORTB |= (1 << PB4);     // set pin 3 of Port B high
  //digitalRead(IN_PIN)
  for (int i=0;i< 60;i++) {
    out = ( ((PIND & (1 << PIND0) ) >> PIND0) && out);
    //out = digitalRead(INT_PIN) && out;
  }
  
  //digitalWrite(TEST_PIN, LOW);
  PORTB &= ~(1 << PB4);    // set pin 3 of Port B low
  //delaySmall(PULSE_DELAY_SMALL-1);
  //readflip = !readflip;
  
  digitalWrite(LED_BUILTIN, LOW);// !out);
  delaySmall(PULSE_READ_DELAY);
  return out;
}

#define START_DELAY 130

void readData() {
  if (((PIND & (1 << PIND0) ) >> PIND0) == 0) {
  PORTB |= (1 << PB4);
  cli();
  
  byte newdata;
 // while (((PIND & (1 << PIND0) ) >> PIND0) == 0) {
   // wait for pin to go low again to sync the read to the end of the data trigger 
  //}

  PORTB &= ~(1 << PB4);    // set pin 3 of Port B low
  digitalWrite(LED_BUILTIN, HIGH);

  byte value;

    delaySmall(START_DELAY);
    for (int i=0; i<8; i++)  {
      if (bitReads() != 0)  {
          // bit is on
          bitSet(newdata, i);
      }
      else {
          // bit is off
          bitClear(newdata, i);
      }
    }

  addData(newdata);
  //digitalWrite(TEST_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delaySmall(PULSE_READ_DELAY);
  }
  sei();
//  Serial.println("read interupt");
}
