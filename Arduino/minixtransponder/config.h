  // boards url http://digistump.com/package_digistump_index.json,http://drazzy.com/package_drazzy.com_index.json
  //avrdude -U lfuse:w:0x62:m  -U hfuse:w:0xC5:m

  //  avrdude -p t25 -P /dev/ttyUSB3 -c avrisp -b 19200 -U lfuse:w:0x62:m -U hfuse:w:0xC5:m
#define SPEED_16MHZ
//#define SPEED_8MHZ
//#define SPEED_4MHZ
//#define SPEED_2MHZ
//#define SPEED_1MHZ

//#define CPU_DIGISTUMP85
//#define CPU_MEGA
#define CPU_TINY25


#ifdef CPU_DIGISTUMP85
  #define LED_PIN 1
  #define OUT_PIN 0
  #define IN_PIN 2
  #define INTERRUPT_PIN PCINT2
  #define TEST_PIN 4
#endif

#ifdef CPU_TINY25
  #define LED_PIN 1
  #define OUT_PIN 0
  #define IN_PIN 2
  #define INTERRUPT_PIN PCINT2
  #define TEST_PIN 4
#endif
