#include <Arduino.h>
#include <U8g2lib.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setup() {
  // put your setup code here, to run once:


  // Timer setup
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

uint8_t readCols() {
  uint8_t result = 0;
  
  // read column 0
  if (digitalRead(C0_PIN) == HIGH) {
    result |= 0b0001;
  }
  
  // read column 1
  if (digitalRead(C1_PIN) == HIGH) {
    result |= 0b0010;
  }
  
  // read column 2
  if (digitalRead(C2_PIN) == HIGH) {
    result |= 0b0100;
  }
  
  // read column 3
  if (digitalRead(C3_PIN) == HIGH) {
    result |= 0b1000;
  }
  
  return result;
}


/*
uint8_t readCols(){
  uint8_t result = 0;
  
  // set row select addresses low and enable row select
  digitalWrite(RA0_PIN, LOW);
  digitalWrite(RA1_PIN, LOW);
  digitalWrite(RA2_PIN, LOW);
  digitalWrite(REN_PIN, HIGH);
  
  // read column 0
  if (digitalRead(C0_PIN) == HIGH) {
    result |= 0b0001;
  }
  
  // read column 1
  if (digitalRead(C1_PIN) == HIGH) {
    result |= 0b0010;
  }
  
  // read column 2
  if (digitalRead(C2_PIN) == HIGH) {
    result |= 0b0100;
  }
  
  // read column 3
  if (digitalRead(C3_PIN) == HIGH) {
    result |= 0b1000;
  }
  
  // disable row select
  digitalWrite(REN_PIN, LOW);
  
  return result;
}
*/

void setRow(uint8_t rowIdx) {
  // disable row select
  digitalWrite(REN_PIN, LOW);
  
  // set row select addresses low
  digitalWrite(RA0_PIN, (rowIdx & 0x01));
  digitalWrite(RA1_PIN, (rowIdx & 0x02));
  digitalWrite(RA2_PIN, (rowIdx & 0x04));
  
  // enable row select
  digitalWrite(REN_PIN, HIGH);
}


const float baseFreq = 440.0; // Frequency of A4
const float baseNote = 9; // Index of A4 in the notes array
const float semitoneRatio = pow(2, 1/12.0); // Factor between adjacent semitones

const uint32_t stepSizes[] = {
  (uint32_t)(pow(semitoneRatio, -9) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -8) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -7) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -6) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -5) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -4) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -3) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -2) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, -1) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, 1) * baseFreq * (1ULL << 32) / 22000),
  (uint32_t)(pow(semitoneRatio, 2) * baseFreq * (1ULL << 32) / 22000),
};

static uint32_t phaseAcc = 0; // declare phase accumulator as static local variable

void sampleISR() {
  phaseAcc += currentStepSize; // update phase accumulator
  
  int32_t Vout = (phaseAcc >> 24) - 128; // convert phase accumulator to sawtooth waveform
  
  analogWrite(OUTR_PIN, Vout + 128); // write voltage to output pin
}



volatile uint32_t currentStepSize = 0;

void loop() {
  static uint32_t next = millis();
  static uint8_t keyArray[7];

  while (millis() < next);  // Wait for next interval

  next += interval;

  // Scan the keys
  for (int i = 0; i < 3; i++) {
    setRow(i);
    delayMicroseconds(3);  // Wait for column lines to settle
    keyArray[i] = readCols();
  }

  // Look up the corresponding step size for the pressed key
  for (int i = 0; i < 12; i++) {
    if (keyArray[0] & (1 << i)) {
      currentStepSize = stepSizes[i];
    }
  }

  //Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.setCursor(2, 10);
  for (int i = 0; i < 12; i++) {
    if (keyArray[0] & (1 << i)) {
      u8g2.print(notes[i]);
      break;
    }
  }
  u8g2.sendBuffer();          // transfer internal memory to the display

  //Toggle LED
  digitalToggle(LED_BUILTIN);
}

