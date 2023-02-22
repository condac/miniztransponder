#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h> 
//#include <EEPROM.h>
//#include <SoftwareSerial.h>
#include "config.h"




byte inputbuffer[4];

byte mynumber = 0;

long silent;

byte startup = 0;

ISR (INT0_vect)        // Interrupt service routine
{
  if (((PINB & (1 << PINB2) ) >> PINB2) == 0) {
    PORTB |= (1 << PB4);     // set pin 3 of Port B high
    readData();
  }

}

// Timer/Counter0 Compare Match A interrupt handler 
ISR (TIMER0_COMPA_vect) {
   PORTB ^= 1 << PINB3;        // Invert pin PB3
}

void loadNumber() {
  //EEPROM.get(1, mynumber);
  mynumber = eeprom_read_byte( (uint8_t*) 1 );
}
void saveNumber(byte number) {
  //EEPROM.write(1, number);
  eeprom_write_byte( (uint8_t*) 1, number );
}
// the setup function runs once when you press reset or power the board
void setup() {
  wdt_enable(WDTO_2S); // options: WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S
  wdt_reset();
   cli(); //disable interrupts - this is a timed sequence.
  // CLKPR = (1<<CLKPCE); // Prescaler enable
  // CLKPR = (1<<CLKPS0); // Clock division factor 2 (0001)
#ifdef SPEED_1MHZ
    // 1mhz
  CLKPR = _BV(CLKPCE); // 0x80
  CLKPR = _BV(CLKPS2); // 0x03
#endif

#ifdef SPEED_2MHZ
//  // 2mhz
  CLKPR = _BV(CLKPCE); // 0x80
  CLKPR = _BV(CLKPS0 | CLKPS1); // 0x03
#endif

#ifdef SPEED_4MHZ
//    // 4mhz
  CLKPR = _BV(CLKPCE); // 0x80
  CLKPR = _BV(CLKPS1); // 0x03
#endif

#ifdef SPEED_8MHZ
//      // 8mhz
  CLKPR = _BV(CLKPCE); // 0x80
  CLKPR = _BV(CLKPS0); // 0x03
#endif

#ifdef SPEED_16MHZ
//        // 16mhz
  CLKPR = _BV(CLKPCE); // 0x80
  CLKPR = 0; // 0x03
#endif
  pinMode(3,OUTPUT);          // Set PB4 to output
  TCNT0 = 0;                  // Count up from 0
  TCCR0A = 2 << WGM00;        // CTC mode
  TCCR0B = (1<<CS00);     // Set prescaler to /1 (1uS at 1Mhz)
  
  GTCCR |= 1 << PSR0;         // Reset prescaler
  OCR0A = 49;                 // 49 + 1 = 50 microseconds (10KHz)
  TIFR = 1 << OCF0A;          // Clear output compare interrupt flag
  TIMSK |= 1 << OCIE0A;       // Enable output compare interrupt
  sei(); //enable interrupts
  // initialize digital pin LED_PIN as an output.
  pinMode(LED_PIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(IN_PIN, INPUT_PULLUP);

  pinMode(OUT_PIN, OUTPUT);
  pinMode(TEST_PIN, OUTPUT);
  loadNumber();
  //Serial.begin(14200);
  sei();     //enabling global interrupt
  GIMSK |= (1 << INT0);   // enabling the INT0 (external interrupt)
  MCUCR |= (1 << ISC01);  // Configuring as falling edge
  //  attachInterrupt(INTERRUPT_PIN,readData,FALLING);
  //   cli();                            // Disable interrupts during setup
  //  PCMSK |= (1 << INTERRUPT_PIN);    // Enable interrupt handler (ISR) for our chosen interrupt pin (PCINT1/PB1/pin 6)
  //  GIMSK |= (1 << PCIE);             // Enable PCINT interrupt in the general interrupt mask
  //  pinMode(IN_PIN, INPUT_PULLUP);   // Set our interrupt pin as input with a pullup to keep it stable
  //  sei();                            //last line of setup - enable interrupts after setup
  startup = 0;
  digitalWrite(LED_PIN, HIGH);
}

bool lys = 1;
long blinktimer;
long starttimer;
bool startRead = false;
// the loop function runs over and over again forever
void loop() {
  wdt_reset();
  //bool out = digitalRead(IN_PIN);
  if (startup < 10) {
    if (starttimer < millis()) {
      starttimer = millis() + 150;
      lys = !lys;
      startup++;
    }
    digitalWrite(LED_PIN, lys);
    
  }
  
  if (silent > millis()) {
    if (blinktimer < millis()) {
      blinktimer = millis() + 150;
      lys = !lys;
    }
    digitalWrite(LED_PIN, lys);
  }else {
    if (blinktimer < millis()) {
      blinktimer = millis() + 2000;
      startup = 0;
      //lys = !lys;
    }
    //digitalWrite(LED_PIN, lys);
  }
  // digitalWrite(LED_PIN, out);
//
//  if (digitalRead(IN_PIN) == 0) {
//
//    //cli();
//    //readData();
//    //sei();
//  }
  sei();
  //sendMyNumber();

  //delay(6);
}
void loop2() {
  // TEST LOOP
  //  if (blinktimer<millis()) {
  //    blinktimer = millis() + 1000;
  //    //lys = !lys;
  //  }
  //digitalWrite(LED_PIN, lys);   // turn the LED on (HIGH is the voltage level)
  //delay(6);                       // wait for a second
  //digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(6);                       // wait for a second
  //digitalWrite(LED_PIN, HIGH);
  sendLoopPing();
  //digitalWrite(LED_PIN, LOW);
  if (digitalRead(IN_PIN) == 0) {
    //lys = !lys;
  }
}

//ISR(INT0_vect)
//{
//  if (digitalRead(IN_PIN) == 0) {
//    startRead = true;
//    cli();
//  }
//
//  //digitalWrite(LED_PIN, lys);
//
//  //digitalWrite(TEST_PIN, LOW);
//}
//void myinter() {
//  lys = true;
//  digitalWrite(LED_PIN, lys);
//}

long millis2() {
  return 0;
}
void sendLoopPing() {
  // Datat som tidtagningsloopen skickar hela tiden för att fråga efter nummer
  byte cmd[] = {0xff, 0xCA, 0xAA, 0x8B};
  sendData(cmd, 4);
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
  parseData();
}

void parseData() {
  byte checksum = calculateChecksum(inputbuffer, 3);
  if (inputbuffer[1] == 0xCA && inputbuffer[2] == 0xAA) {
    // Bågens ping att den vill ha bilens nummer, oavsett checksumma
    digitalWrite(LED_PIN, HIGH);
    if (silent > millis()) {
      //silent = millis() +1000;
      digitalWrite(LED_PIN, HIGH);
    } else {
      //digitalWrite(LED_PIN, HIGH);
      delay(1);
      sendMyNumber();
    }
    digitalWrite(LED_PIN, LOW);

  }
  if (checksum == inputbuffer[3]) {

    if (inputbuffer[1] == 0xE2) {
      // Spara nytt nummer
      mynumber = inputbuffer[2];
      saveNumber(mynumber);
      sendSaveNrOk();
    }
    if (inputbuffer[1] == 0x53 && inputbuffer[2] == mynumber) {
      // Bågen svarar att den sett bilen, sluta skicka mitt nummer

      //      byte banan = mynumber;
      //      mynumber = 2;
      //      sendMyNumber();
      //      mynumber = banan;
      digitalWrite(LED_PIN, HIGH);
      silent = millis() + 2000;
    }
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void sendMyNumber() {
  byte cmd[] = {0x03, 0xD3, mynumber, 0x00};
  cmd[3] = calculateChecksum(cmd, 3);
  sendData(cmd, 4);
}

void sendSaveNrOk() {
  byte cmd[] = {0x03, 0x8B, mynumber, 0x00};
  cmd[3] = calculateChecksum(cmd, 3);
  sendData(cmd, 4);
}

#define PULSE_DELAY 27
#define PULSE_DELAY_SMALL 55


void delaySmall(int counts) {
  for (int i = 0; i < counts; i++) {
    __asm__("nop\n\t");
  }
}
//void bitPulse(bool bits) {
//
//  digitalWrite(OUT_PIN, bits);
//  //delayMicroseconds(PULSE_DELAY);
//  delaySmall(PULSE_DELAY_SMALL);
//  digitalWrite(OUT_PIN, HIGH);
//  //delayMicroseconds(PULSE_DELAY);
//  delaySmall(PULSE_DELAY_SMALL);
//}
#define PULSE_DELAY_OFF 114
#define PULSE_DELAY_ON 14

void bitPulse(bool bits) {

  digitalWrite(OUT_PIN, !bits);
  //delayMicroseconds(12);

  delaySmall(PULSE_DELAY_ON);

  __asm__("nop\n\t");
  __asm__("nop\n\t");
  __asm__("nop\n\t");
  __asm__("nop\n\t");
  __asm__("nop\n\t");

  digitalWrite(OUT_PIN, !HIGH);
  //delayMicroseconds(PULSE_DELAY);
  delaySmall(PULSE_DELAY_OFF);
}

void sendData(byte data[], int len) {
  //byte cmd[]={0xFC,0x35,0x55,0x74};
  cli();

  byte value;
  for (int x = 0; x < len; x++) {
    value = data[x];
    // Start bit
    bitPulse(0);
    for (int i = 0, mask = 1; i < 8; i++, mask = mask << 1)  {
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
  sei();
  //digitalWrite(LED_PIN, HIGH);
}
#define PULSE_READ_DELAY 83
bool readflip;
byte bitReads() {
  bool out = 1;
  //digitalWrite(TEST_PIN, HIGH);
  PORTB |= (1 << PB4);     // set pin 3 of Port B high
  //digitalRead(IN_PIN)
  for (int i = 0; i < 60; i++) {
    out = ( ((PINB & (1 << PINB2) ) >> PINB2) && out);
  }

  //digitalWrite(TEST_PIN, LOW);
  PORTB &= ~(1 << PB4);    // set pin 3 of Port B low
  //delaySmall(PULSE_DELAY_SMALL-1);
  //readflip = !readflip;

  digitalWrite(LED_PIN, LOW);// !out);
  delaySmall(PULSE_READ_DELAY);
  return out;
}

#define START_DELAY 100
void readData() {
  cli();
  byte newdata;
  while (((PINB & (1 << PINB2) ) >> PINB2) == 0) {

  }

  PORTB &= ~(1 << PB4);    // set pin 3 of Port B low
  digitalWrite(LED_PIN, LOW);

  byte value;
  //for (int x = 0;x<3;x++) {
  //value = data[x];
  // Start bit
  //bitReads();
  delaySmall(START_DELAY);
  for (int i = 0; i < 8; i++)  {
    if (bitReads() != 0)  {
      // bit is on
      bitSet(newdata, i);
    }
    else {
      // bit is off
      bitClear(newdata, i);
    }
  }
  // Stop bit
  //    bitReads();
  //    bitReads();
  //    readflip = 1;
  //    digitalWrite(TEST_PIN, readflip);


  //}
  addData(newdata);
  digitalWrite(TEST_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  delaySmall(PULSE_READ_DELAY);
  //delaySmall(PULSE_READ_DELAY);
  //  if (inputbuffer[1] == 0xCA) {
  //    digitalWrite(LED_PIN, HIGH);
  //  } else {
  //    digitalWrite(LED_PIN, LOW);
  //  }
  //sendData(inputbuffer, 4);

  sei();
  //digitalWrite(LED_PIN, LOW);
}
