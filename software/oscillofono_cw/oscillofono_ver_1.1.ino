/*
  Morse Code Oscillophone
  Author: Emanuele Rossi – IK1APW
  Hardware: Arduino Nano
  Status: Stable

  Pure sinusoidal CW generator with real-time Morse decoder
*/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ================= PIN =================
const byte PIN_KEY     = 2;   // tasto verso GND
const byte PIN_ENC_A   = 3;   // encoder CLK
const byte PIN_ENC_B   = 4;   // encoder DT
const byte PIN_ENC_SW  = 5;   // encoder SW
const byte PIN_PWM     = 9;   // OC1A (Timer1) -> filtro RC -> ampli

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// LCD refresh controllato
volatile bool lcdDirty = true;
unsigned long lastLcdMs = 0;
const unsigned long LCD_PERIOD_MS = 90;

// ================= PARAMETRI =================
volatile int freqHz = 700;
volatile int wpmSet = 20;

const int FREQ_MIN = 300;
const int FREQ_MAX = 1200;
const int WPM_MIN  = 5;
const int WPM_MAX  = 45;

// ================= AUDIO DDS =================
static const uint32_t FS = 15625UL;  // 16MHz/1024
int8_t sine256[256];

volatile uint32_t phaseAcc = 0;
volatile uint32_t phaseInc = 0;

// envelope anti-click
volatile uint8_t amp = 0;
const uint8_t AMP_MAX  = 220;
const uint8_t ATT_STEP = 8;
const uint8_t REL_STEP = 6;

volatile bool keyState = false;

// ================= UI / MODE =================
enum Mode { MODE_FREQ, MODE_WPM };
Mode mode = MODE_FREQ;

// ================= ENCODER (POLLING stile “nostro”) =================
int lastCLK = HIGH;
unsigned long lastEncStepUs = 0;
const unsigned long ENC_STEP_US = 1200; // anti-rimbalzo step: prova 800..2000 se vuoi

// Pulsante encoder: short press = cambia MODE, long press (>2s) = clear decoder
bool swStable = HIGH;
bool swLastStable = HIGH;
bool swPressing = false;
unsigned long swPressStartMs = 0;
bool longDone = false;

const unsigned long SW_DEBOUNCE_MS = 30;
const unsigned long LONG_PRESS_MS  = 2000;

// ================= DECODER CW =================
static const unsigned int KEY_DEBOUNCE_MS = 12;

char cwBuf[10];
byte cwLen = 0;

char textLine[17];     // riga 2: 16 char + '\0'
byte textLen = 0;

unsigned long lastEdgeMs = 0;
bool keyNow = false;
bool keyPrev = false;

// ================= UTILS =================
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint32_t calcPhaseInc(int fHz) {
  return (uint32_t)((((uint64_t)fHz) << 32) / FS);
}

int ditMs() {
  int w = (int)wpmSet;
  if (w < 1) w = 1;
  return 1200 / w;
}

void resetCwBuf() {
  cwLen = 0;
  cwBuf[0] = '\0';
}

void clearDecoderLine() {
  textLen = 0;
  textLine[0] = '\0';
  resetCwBuf();
  lcdDirty = true;
}

void pushTextChar(char c) {
  if (textLen < 16) {
    textLine[textLen++] = c;
    textLine[textLen] = '\0';
  } else {
    for (byte i = 0; i < 15; i++) textLine[i] = textLine[i + 1];
    textLine[15] = c;
    textLine[16] = '\0';
  }
  lcdDirty = true;
}

char decodeMorse(const char* s) {
  if (!strcmp(s, ".-")) return 'A';
  if (!strcmp(s, "-...")) return 'B';
  if (!strcmp(s, "-.-.")) return 'C';
  if (!strcmp(s, "-..")) return 'D';
  if (!strcmp(s, ".")) return 'E';
  if (!strcmp(s, "..-.")) return 'F';
  if (!strcmp(s, "--.")) return 'G';
  if (!strcmp(s, "....")) return 'H';
  if (!strcmp(s, "..")) return 'I';
  if (!strcmp(s, ".---")) return 'J';
  if (!strcmp(s, "-.-")) return 'K';
  if (!strcmp(s, ".-..")) return 'L';
  if (!strcmp(s, "--")) return 'M';
  if (!strcmp(s, "-.")) return 'N';
  if (!strcmp(s, "---")) return 'O';
  if (!strcmp(s, ".--.")) return 'P';
  if (!strcmp(s, "--.-")) return 'Q';
  if (!strcmp(s, ".-.")) return 'R';
  if (!strcmp(s, "...")) return 'S';
  if (!strcmp(s, "-")) return 'T';
  if (!strcmp(s, "..-")) return 'U';
  if (!strcmp(s, "...-")) return 'V';
  if (!strcmp(s, ".--")) return 'W';
  if (!strcmp(s, "-..-")) return 'X';
  if (!strcmp(s, "-.--")) return 'Y';
  if (!strcmp(s, "--..")) return 'Z';

  if (!strcmp(s, "-----")) return '0';
  if (!strcmp(s, ".----")) return '1';
  if (!strcmp(s, "..---")) return '2';
  if (!strcmp(s, "...--")) return '3';
  if (!strcmp(s, "....-")) return '4';
  if (!strcmp(s, ".....")) return '5';
  if (!strcmp(s, "-....")) return '6';
  if (!strcmp(s, "--...")) return '7';
  if (!strcmp(s, "---..")) return '8';
  if (!strcmp(s, "----.")) return '9';

  if (!strcmp(s, ".-.-.-")) return '.';
  if (!strcmp(s, "--..--")) return ',';
  if (!strcmp(s, "..--..")) return '?';
  if (!strcmp(s, "-....-")) return '-';
  if (!strcmp(s, "-..-.")) return '/';
  if (!strcmp(s, ".--.-.")) return '@';

  return '#';
}

void finalizeLetter() {
  if (cwLen == 0) return;
  cwBuf[cwLen] = '\0';
  pushTextChar(decodeMorse(cwBuf));
  resetCwBuf();
}

// ================= LCD UPDATE =================
void updateDisplay(bool force = false) {
  unsigned long now = millis();

  if (!force) {
    if (!lcdDirty) return;
    if (now - lastLcdMs < LCD_PERIOD_MS) return;
  }

  lastLcdMs = now;
  lcdDirty = false;

  // Riga 1: ultimo carattere = modalità (F/W)
  char temp[17];
  snprintf(temp, sizeof(temp), "F:%04dHz W:%02d", (int)freqHz, (int)wpmSet);

  char l1[17];
  for (int i = 0; i < 16; i++) l1[i] = ' ';
  for (int i = 0; i < 15 && temp[i] != '\0'; i++) l1[i] = temp[i];
  l1[15] = (mode == MODE_FREQ) ? 'F' : 'W';
  l1[16] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(l1);

  // Riga 2: SOLO decoder
  char l2[17];
  for (byte i = 0; i < 16; i++) l2[i] = (i < textLen) ? textLine[i] : ' ';
  l2[16] = '\0';

  lcd.setCursor(0, 1);
  lcd.print(l2);
}

// ================= TIMER SETUP =================
void setupTimer1PWM_62k5() {
  pinMode(PIN_PWM, OUTPUT);
  TCCR1A = 0; TCCR1B = 0;
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  OCR1A = 128;
}

void setupTimer2SampleISR_15625() {
  TCCR2A = 0; TCCR2B = 0;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // /1024
  OCR2A = 0;
  TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
  if (keyState) {
    if (amp < AMP_MAX) {
      uint16_t a = amp + ATT_STEP;
      amp = (a > AMP_MAX) ? AMP_MAX : (uint8_t)a;
    }
  } else {
    if (amp > 0) {
      int16_t a = (int16_t)amp - (int16_t)REL_STEP;
      amp = (a < 0) ? 0 : (uint8_t)a;
    }
  }

  phaseAcc += phaseInc;
  uint8_t idx = (uint8_t)(phaseAcc >> 24);
  int8_t s = sine256[idx];

  int16_t y = (int16_t)s * (int16_t)amp;
  y /= 255;
  OCR1A = (uint8_t)(128 + y);
}

// ================= KEY + DECODER =================
void handleKeyAndDecode() {
  keyNow = (digitalRead(PIN_KEY) == LOW);
  unsigned long now = millis();

  if (keyNow != keyPrev) {
    unsigned long dt = now - lastEdgeMs;
    if (dt < KEY_DEBOUNCE_MS) return;

    lastEdgeMs = now;
    int d = ditMs();

    if (keyPrev == true && keyNow == false) {
      unsigned long mark = dt;
      if (mark < (unsigned long)(2 * d)) {
        if (cwLen < sizeof(cwBuf) - 1) cwBuf[cwLen++] = '.';
      } else {
        if (cwLen < sizeof(cwBuf) - 1) cwBuf[cwLen++] = '-';
      }
      lcdDirty = true;

    } else if (keyPrev == false && keyNow == true) {
      unsigned long space = dt;
      if (space >= (unsigned long)(7 * d)) {
        finalizeLetter();
        if (textLen == 0 || textLine[textLen - 1] != ' ') pushTextChar(' ');
      } else if (space >= (unsigned long)(3 * d)) {
        finalizeLetter();
      }
    }

    keyPrev = keyNow;
    keyState = keyNow;
  }

  if (!keyNow && cwLen > 0) {
    unsigned long idle = now - lastEdgeMs;
    if (idle > (unsigned long)(3 * ditMs())) {
      finalizeLetter();
      lastEdgeMs = now;
    }
  }
}

// ================= ENCODER (POLLING) =================
void handleEncoderPolling() {
  int currentCLK = digitalRead(PIN_ENC_A);

  // Evento SOLO sul fronte di discesa
  if (currentCLK != lastCLK && currentCLK == LOW) {

    unsigned long nowUs = micros();
    if (nowUs - lastEncStepUs >= ENC_STEP_US) {
      lastEncStepUs = nowUs;

      // stessa logica del tuo codice: direzione da DT rispetto a CLK
      int det = (digitalRead(PIN_ENC_B) != currentCLK) ? +1 : -1;

      if (mode == MODE_FREQ) {
        int f = clampInt((int)freqHz + det * 10, FREQ_MIN, FREQ_MAX);
        if (f != freqHz) {
          freqHz = f;
          phaseInc = calcPhaseInc(f);
          lcdDirty = true;
        }
      } else {
        int w = clampInt((int)wpmSet + det, WPM_MIN, WPM_MAX);
        if (w != wpmSet) {
          wpmSet = w;
          lcdDirty = true;
        }
      }
    }
  }

  lastCLK = currentCLK;
}

// ================= BUTTON (non bloccante, short/long) =================
void handleEncoderButton() {
  bool sw = (digitalRead(PIN_ENC_SW) == LOW);
  unsigned long now = millis();

  static bool swRawPrev = false;
  static unsigned long swRawChangeMs = 0;

  if (sw != swRawPrev) {
    swRawPrev = sw;
    swRawChangeMs = now;
  }

  if (now - swRawChangeMs > SW_DEBOUNCE_MS) {
    swStable = sw;

    if (swStable && !swLastStable) {
      swPressing = true;
      swPressStartMs = now;
      longDone = false;
    }

    if (swPressing && swStable && !longDone) {
      if (now - swPressStartMs >= LONG_PRESS_MS) {
        clearDecoderLine();
        longDone = true;
      }
    }

    if (!swStable && swLastStable) {
      if (swPressing) {
        if (!longDone) {
          mode = (mode == MODE_FREQ) ? MODE_WPM : MODE_FREQ;
          lcdDirty = true;
        }
        swPressing = false;
      }
    }

    swLastStable = swStable;
  }
}

// ================= SETUP / LOOP =================
void setup() {
  pinMode(PIN_KEY, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  clearDecoderLine();

  // sinusoide
  for (int i = 0; i < 256; i++) {
    float ang = 2.0f * 3.14159265f * (float)i / 256.0f;
    int v = (int)lroundf(127.0f * sinf(ang));
    sine256[i] = (int8_t)v;
  }

  phaseInc = calcPhaseInc(freqHz);

  // init encoder
  lastCLK = digitalRead(PIN_ENC_A);
  lastEncStepUs = micros();

  // PWM + ISR audio
  noInterrupts();
  setupTimer1PWM_62k5();
  setupTimer2SampleISR_15625();
  interrupts();

  lastEdgeMs = millis();
  updateDisplay(true);
}

void loop() {
  handleKeyAndDecode();
  handleEncoderPolling();
  handleEncoderButton();
  updateDisplay();
}
