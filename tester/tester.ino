/*
 * Controller Tester
 * (c) 2018 Ed Salisbury <ed.salisbury@gmail.com>
 * Some code courtesy of Jon Thysell <thysell@gmail.com> (SEGA)
 */

typedef unsigned int uint;

const int DPAD_UP       = 0x01;
const int DPAD_DOWN     = 0x02;
const int DPAD_LEFT     = 0x04;
const int DPAD_RIGHT    = 0x08;
const int BUTTON_A      = 0x10;
const int BUTTON_B      = 0x20;
const int BUTTON_C      = 0x40;
const int BUTTON_X      = 0x80;
const int BUTTON_Y      = 0x100;
const int BUTTON_Z      = 0x200;
const int BUTTON_L      = 0x400;
const int BUTTON_R      = 0x800;
const int BUTTON_SELECT = 0x1000;
const int BUTTON_START  = 0x2000;

const int NES_PINS[]   = {9, 8, 10}; // Strobe, Clock, Data
const int SNES_PINS[]  = {A2, A3, A1};

const int NES_A      = 0x01;
const int NES_B      = 0x02;
const int NES_SELECT = 0x04;
const int NES_START  = 0x08;
const int NES_UP     = 0x10;
const int NES_DOWN   = 0x20;
const int NES_LEFT   = 0x40;
const int NES_RIGHT  = 0x80;

const int SNES_B      = 0x001;
const int SNES_Y      = 0x002;
const int SNES_SELECT = 0x004;
const int SNES_START  = 0x008;
const int SNES_UP     = 0x010;
const int SNES_DOWN   = 0x020;
const int SNES_LEFT   = 0x040;
const int SNES_RIGHT  = 0x080;
const int SNES_A      = 0x100;
const int SNES_X      = 0x200;
const int SNES_L      = 0x400;
const int SNES_R      = 0x800;

int playerState;
int segaState;

// Controller Button Flags
const int SEGA_ON = 1;
const int SEGA_UP = 2;
const int SEGA_DOWN = 4;
const int SEGA_LEFT = 8;
const int SEGA_RIGHT = 16;
const int SEGA_START = 32;
const int SEGA_A = 64;
const int SEGA_B = 128;
const int SEGA_C = 256;
const int SEGA_X = 512;
const int SEGA_Y = 1024;
const int SEGA_Z = 2048;

const int SEGA_SELECT = A5;
 
typedef struct
{
  int pin;
  int lowFlag;
  int highFlag;
  int pulse3Flag;
} input;
 
// Controller DB9 Pin to Button Flag Mappings
// First column is the controller index, second column
// is the Arduino pin that the controller's DB9 pin is
// attached to
input inputMap[] = {
  { 2,  SEGA_UP,    SEGA_UP,     SEGA_Z}, // P0 DB9 Pin 1
  { 3,  SEGA_DOWN,  SEGA_DOWN,   SEGA_Y}, // P0 DB9 Pin 2
  { 4,  SEGA_ON,    SEGA_LEFT,   SEGA_X}, // P0 DB9 Pin 3
  { 5,  SEGA_ON,    SEGA_RIGHT,  0}, // P0 DB9 Pin 4
  { 6,  SEGA_A,     SEGA_B,      0}, // P0 DB9 Pin 6
  { 7,  SEGA_START, SEGA_C,      0}, // P0 DB9 Pin 9
};
 
// Default to three-button mode until six-button connects
boolean sixButtonMode = false;
 
void setup()
{
  // Setup input pins
  for (uint i = 0; i < sizeof(inputMap) / sizeof(input); i++)
  {
    pinMode(inputMap[i].pin, INPUT);
    digitalWrite(inputMap[i].pin, HIGH);
  }

  pinMode(SEGA_SELECT, OUTPUT);
  digitalWrite(SEGA_SELECT, HIGH);

  // Set NES/SNES data pins as input
  pinMode(NES_PINS[2], INPUT);
  pinMode(SNES_PINS[2], INPUT);

  pinMode(NES_PINS[0], OUTPUT);
  pinMode(SNES_PINS[1], OUTPUT);
  pinMode(SNES_PINS[0], OUTPUT);
  pinMode(SNES_PINS[1], OUTPUT);
  
  Serial.begin(9600);
}
 
void loop()
{
  playerState = 0;
  ReadSega();
  ReadNES();
  ReadSNES();
  Output();
  delay(10);
}

void Output()
{
  char output[] = "              ";
  int oldState = 0;
  if ((playerState & DPAD_UP) == DPAD_UP) output[0] = 'U';
  if ((playerState & DPAD_DOWN) == DPAD_DOWN) output[1] = 'D';
  if ((playerState & DPAD_LEFT) == DPAD_LEFT)  output[2] = 'L';
  if ((playerState & DPAD_RIGHT) == DPAD_RIGHT)  output[3] = 'R';
  if ((playerState & BUTTON_A) == BUTTON_A)  output[4] = 'A';
  if ((playerState & BUTTON_B) == BUTTON_B)  output[5] = 'B';
  if ((playerState & BUTTON_C) == BUTTON_C)  output[6] = 'C';
  if ((playerState & BUTTON_X) == BUTTON_X)  output[7] = 'X';
  if ((playerState & BUTTON_Y) == BUTTON_Y)  output[8] = 'Y';
  if ((playerState & BUTTON_Z) == BUTTON_Z)  output[9] = 'Z';
  if ((playerState & BUTTON_L) == BUTTON_L)  output[10] = 'L';
  if ((playerState & BUTTON_R) == BUTTON_R)  output[11] = 'R';
  if ((playerState & BUTTON_SELECT) == BUTTON_SELECT)  output[12] = 'S';
  if ((playerState & BUTTON_START) == BUTTON_START)  output[13] = 'S';
  
  if (playerState > 0 && playerState != oldState)
  {
    Serial.println(output);
    oldState = playerState;
  }
}


void ReadSega()
{
  segaState = 0;
  
  if (sixButtonMode)
  {
    ReadSega6Buttons();
  }
  else
  {
    ReadSega3Buttons();
  }

  if ((segaState & SEGA_UP) == SEGA_UP)         playerState |= DPAD_UP;
  if ((segaState & SEGA_DOWN) == SEGA_DOWN)     playerState |= DPAD_DOWN;
  if ((segaState & SEGA_LEFT) == SEGA_LEFT)     playerState |= DPAD_LEFT;
  if ((segaState & SEGA_RIGHT) == SEGA_RIGHT)   playerState |= DPAD_RIGHT; 
  if ((segaState & SEGA_A) == SEGA_A)           playerState |= BUTTON_A; 
  if ((segaState & SEGA_B) == SEGA_B)           playerState |= BUTTON_B; 
  if ((segaState & SEGA_C) == SEGA_C)           playerState |= BUTTON_C; 
  if ((segaState & SEGA_X) == SEGA_X)           playerState |= BUTTON_X; 
  if ((segaState & SEGA_Y) == SEGA_Y)           playerState |= BUTTON_Y; 
  if ((segaState & SEGA_Z) == SEGA_Z)           playerState |= BUTTON_Z; 
  if ((segaState & SEGA_START) == SEGA_START)   playerState |= BUTTON_START; 
}
 
void ReadSega3Buttons()
{
  // Set SELECT LOW and read lowFlag
  digitalWrite(SEGA_SELECT, LOW);
 
  delayMicroseconds(20);
 
  for (uint i = 0; i < sizeof(inputMap) / sizeof(input); i++)
  {
    if (digitalRead(inputMap[i].pin) == LOW)
    {
      segaState |= inputMap[i].lowFlag;
    }
  }
 
  // Set SELECT HIGH and read highFlag
  digitalWrite(SEGA_SELECT, HIGH);
 
  delayMicroseconds(20);
 
  for (uint i = 0; i < sizeof(inputMap) / sizeof(input); i++)
  {
    if (digitalRead(inputMap[i].pin) == LOW)
    {
      segaState |= inputMap[i].highFlag;
    }
  }
 
  // When a six-button first connects, it'll spam UP and DOWN,
  // which signals the game to switch to 6-button polling
  if (segaState == (SEGA_ON | SEGA_UP | SEGA_DOWN))
  {
    sixButtonMode = true;
  }
  // When a controller disconnects, revert to three-button polling
  else if ((segaState & SEGA_ON) == 0)
  {
    sixButtonMode = false;
  }
  delayMicroseconds(20);
}
 
void ReadSega6Buttons()
{
  // Poll for three-button states twice
  ReadSega3Buttons();
  ReadSega3Buttons();
 
  // After two three-button polls, pulse the SELECT line
  // so the six-button reports the higher button states
  digitalWrite(SEGA_SELECT, LOW);
  delayMicroseconds(20);
  digitalWrite(SEGA_SELECT, HIGH);
 
  for(uint i = 0; i < sizeof(inputMap) / sizeof(input); i++)
  {
    if (digitalRead(inputMap[i].pin) == LOW)
    {
      segaState |= inputMap[i].pulse3Flag;
    }
  }
 
  delayMicroseconds(1000);
}

void ReadNES()
{
  int state = 0;
  
  Strobe(NES_PINS[0]);
  for (int i = 0 ; i < 8 ; ++i)
  {
    state |= ShiftIn(NES_PINS[2], NES_PINS[1]) << i;
  }
  state = ~state;

  if ((state & NES_UP) == NES_UP && (state & NES_DOWN) == NES_DOWN)
  {
    state = 0;
  }
  
  if ((state & NES_UP) == NES_UP)         playerState |= DPAD_UP;
  if ((state & NES_DOWN) == NES_DOWN)     playerState |= DPAD_DOWN;
  if ((state & NES_LEFT) == NES_LEFT)     playerState |= DPAD_LEFT;
  if ((state & NES_RIGHT) == NES_RIGHT)   playerState |= DPAD_RIGHT; 
  if ((state & NES_A) == NES_A)           playerState |= BUTTON_A; 
  if ((state & NES_B) == NES_B)           playerState |= BUTTON_B; 
  if ((state & NES_SELECT) == NES_SELECT) playerState |= BUTTON_SELECT; 
  if ((state & NES_START) == NES_START)   playerState |= BUTTON_START; 
}

void ReadSNES()
{
  int state = 0;
  
  Strobe(SNES_PINS[0]);
  for (int i = 0 ; i < 16 ; ++i)
  {
    state |= ShiftIn(SNES_PINS[2], SNES_PINS[1]) << i;
  }
  state = ~state;
  
  if ((state & SNES_UP) == SNES_UP && (state & SNES_DOWN) == SNES_DOWN)
  {
    state = 0;
  }

  if ((state & SNES_UP) == SNES_UP)         playerState |= DPAD_UP;
  if ((state & SNES_DOWN) == SNES_DOWN)     playerState |= DPAD_DOWN;
  if ((state & SNES_LEFT) == SNES_LEFT)     playerState |= DPAD_LEFT;
  if ((state & SNES_RIGHT) == SNES_RIGHT)   playerState |= DPAD_RIGHT; 
  if ((state & SNES_A) == SNES_A)           playerState |= BUTTON_A; 
  if ((state & SNES_B) == SNES_B)           playerState |= BUTTON_B; 
  if ((state & SNES_X) == SNES_X)           playerState |= BUTTON_X; 
  if ((state & SNES_Y) == SNES_Y)           playerState |= BUTTON_Y; 
  if ((state & SNES_L) == SNES_L)           playerState |= BUTTON_L; 
  if ((state & SNES_R) == SNES_R)           playerState |= BUTTON_R; 
  if ((state & SNES_SELECT) == SNES_SELECT) playerState |= BUTTON_SELECT; 
  if ((state & SNES_START) == SNES_START)   playerState |= BUTTON_START; 
}

void Strobe(int pin)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(12);
  digitalWrite(pin, LOW);
}

byte ShiftIn(int dataPin, int clockPin)
{
  byte ret = digitalRead(dataPin);
  delayMicroseconds(12);
  digitalWrite(clockPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(clockPin, LOW);
  return ret;
}
