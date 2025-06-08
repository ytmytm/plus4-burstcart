// IDE: ATtiny25/45/85, internal 1MHz
// 1MHz is the default speed, no need to switch fuses to 8MHz
// compile to HEX (Sketch->Export compiled binary) and burn using TL866-II

// pinout
//                                /RESET 1  U  8 5V
// -- selector (floating or GND)  A3/PB3 2     7 A1/2/PB2
//                                A2/PB4 3     6 PB1 PWM
//                                   GND 4     5 PB0 PWM -- output PWM 50Hz

/* -D CKDIV8
board = attiny85
board_build.f_cpu = 1000000L
board_fuses.lfuse = 0x62
board_fuses.hfuse = 0xdf
board_fuses.efuse = 0xff
*/

const uint8_t PWM_pin = 0; // PB0, pin 5
const uint8_t mode_pin = 3; // PB3, pin 2 - jumper to GND for NTSC mode, keep floating (internal pullup) for PAL
uint8_t half_duty = 10;    // 10ms for PAL, 8ms for NTSC = 20ms/16ms

// use PAL (50Hz) clock, otherwise we would have to maintain two ROM versions

// Define OCR0A values and prescaler based on CKDIV8
#if F_CPU == 1000000L
  // 1MHz clock settings
  #define PRESCALER_BITS ((1 << CS01) | (1 << CS00))  // Prescaler = 64
  #define OCR0A_50HZ     155  // 50Hz: 1MHz / (2*64*(155+1)) ≈ 50.08Hz
  #define OCR0A_60HZ     129  // 60Hz: 1MHz / (2*64*(129+1)) ≈ 60.1Hz
#else
  // 8MHz clock settings
  #define PRESCALER_BITS ((1 << CS02) | (1 << CS00))  // Prescaler = 1024
  #define OCR0A_50HZ     77   // 50Hz: 8MHz / (2*1024*(77+1)) ≈ 50.08Hz
  #define OCR0A_60HZ     64   // 60Hz: 8MHz / (2*1024*(64+1)) ≈ 60.14Hz
#endif

void setup() {
  pinMode(PWM_pin, OUTPUT);
  digitalWrite(PWM_pin, LOW);
  pinMode(mode_pin, INPUT_PULLUP);
  delay(200);
  if (digitalRead(mode_pin) == LOW) { // is it tied to GND? yes - use software clock
    while (true) {
      delay(half_duty); // 10ms - half of 20ms duty cycle
      digitalWrite(PWM_pin, HIGH);
      delay(half_duty); // 10ms - half of 20ms duty cycle
      digitalWrite(PWM_pin, LOW);
    }
  }
  // otherwise use h/w clock
    DDRB |= (1 << PB0);    // PB0 (OC0A) as output

    // Configure Timer0 for CTC mode (toggle OC0A on compare match)
    TCCR0A = (1 << COM0A0) | (1 << WGM01);  // Toggle OC0A, CTC mode
    TCCR0B = PRESCALER_BITS;  // Set prescaler (64 or 1024)
    OCR0A = OCR0A_50HZ;

    while (1) { }
}

void loop() {
  // could be done using PWM and clock prescalers, but that is good enough for me
}
