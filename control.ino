
#include <EnableInterrupt.h>
#define n  4

#define ch1  0
#define ch2  1
#define ch3  2
#define ch4  3

#define a0  A0
#define a1  A1
#define a2  A2
#define a3  A3

// Mapping interval constants
#define min 900
#define max 2000

uint16_t positions[n];
double v1;
uint32_t time[n];
volatile uint16_t pulse[n]; //volatile for use in interrupt

void read() {
  noInterrupts();
  memcpy(positions, (const void *)pulse, sizeof(pulse)); 
  interrupts();
}

void read_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) { // if no read
    time[channel] = micros(); //set equal to time
  } else {
    uint16_t dt = (uint16_t)(micros() - time[channel]); //else set to difference in time since last read
    pulse[channel] = dt;
  }
}

// functions for 
void read_ch1() { read_input(ch1, a0); }
void read_ch2() { read_input(ch2, a1); }
void read_ch3() { read_input(ch3, a2); }
void read_ch4() { read_input(ch4, a3); }

void setup() {
  Serial.begin(57600);

  //Set pins for reading
  pinMode(a0, INPUT);
  pinMode(a1, INPUT);
  pinMode(a2, INPUT);
  pinMode(a3, INPUT);

  //interrupt for each channel
  enableInterrupt(a0, read_ch1, CHANGE);
  enableInterrupt(a1, read_ch2, CHANGE);
  enableInterrupt(a2, read_ch3, CHANGE);
  enableInterrupt(a3, read_ch4, CHANGE);
}

void loop() {
  read();
  v1 = map(positions[ch1], 904, 2024, 0, 50);
  //TODO: correct mapping
  //TODO: write to file with new enable interrupt pin

  delay(200);
}
