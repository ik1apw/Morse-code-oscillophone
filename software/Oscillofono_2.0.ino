#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <EEPROM.h>
#include <string.h>

// =====================================================
//                     PINOUT
// =====================================================
const byte PIN_KEY        = 2;   // tasto verticale verso GND
const byte PIN_ENC_A      = 3;   // encoder CLK
const byte PIN_ENC_B      = 4;   // encoder DT
const byte PIN_ENC_SW     = 5;   // encoder SW
const byte PIN_PADDLE_DIT = 6;   // paddle DIT verso GND (comune a massa)
const byte PIN_PADDLE_DAH = 7;   // paddle DAH verso GND (comune a massa)
const byte PIN_MODE_BTN   = 8;   // pulsante MODE verso GND
const byte PIN_PWM        = 9;   // OC1A (Timer1) -> filtro RC -> ampli

// =====================================================
//                     TIPI (IN ALTO!)
// =====================================================
enum ModeApp : uint8_t { MODE_MANUAL = 0, MODE_PADDLE = 1 };
enum KeyerState : uint8_t { K_IDLE, K_MARK, K_GAP };
enum ElemType   : uint8_t { E_DIT, E_DAH };
enum ManualEncMode : uint8_t { MAN_FREQ, MAN_WPM };

// =====================================================
//                     LCD
// =====================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
//                  PARAMETRI UTENTE
// =====================================================
volatile int freqHz = 700;
volatile int wpmSet = 20;

const int FREQ_MIN = 300;
const int FREQ_MAX = 1200;
const int WPM_MIN  = 5;
const int WPM_MAX  = 45;

// =====================================================
//                    AUDIO DDS
// =====================================================
static const uint32_t FS = 15625UL;   // Timer2 tick rate (16MHz/1024)
int8_t sine256[256];

volatile uint32_t phaseAcc = 0;
volatile uint32_t phaseInc = 0;

// inviluppo anti-click
volatile uint8_t amp = 0;
const uint8_t AMP_MAX  = 220;
const uint8_t ATT_STEP = 8;
const uint8_t REL_STEP = 6;

volatile bool keyState = false; // ON/OFF nota (ISR fa rampa)

// =====================================================
//                    EEPROM MODE
// =====================================================
const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_MODE  = 1;
const byte EEPROM_MAGIC = 0xA7;

// =====================================================
//                     STATO UI
// =====================================================
ModeApp appMode = MODE_MANUAL;

ManualEncMode manualEncMode = MAN_FREQ;

// Encoder SW (click + long press)
bool encLastSw = HIGH;
unsigned long encSwDebMs = 0;
unsigned long encPressStartMs = 0;
bool encLongDone = false;

// MODE button (D8)
bool modeBtnLast = HIGH;
unsigned long modePressStartMs = 0;
bool modeLongDone = false;

// =====================================================
//                DECODER CW (MANUAL)
// =====================================================
static const unsigned int DEBOUNCE_MS = 12;

char cwBuf[10];
byte cwLen = 0;

char textLine[17];   // 16 char + \0
byte textLen = 0;

bool keyNow = false;
bool keyPrev = false;
unsigned long lastEdgeMs = 0;

// =====================================================
//                 PADDLE KEYER (NON-BLOCKING)
// =====================================================
KeyerState kst = K_IDLE;
ElemType currentElem = E_DIT;
unsigned long kNextMs = 0;
bool lastWasDit = true;
const uint8_t PADDLE_DEBOUNCE_MS = 10;

// =====================================================
//                     UTILS
// =====================================================
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint32_t calcPhaseInc(int fHz) {
  return (uint32_t)((((uint64_t)fHz) << 32) / FS);
}

uint16_t ditMs() {
  int w = (int)wpmSet;
  if (w < 1) w = 1;
  return (uint16_t)(1200 / w);
}

bool readDit() { return digitalRead(PIN_PADDLE_DIT) == LOW; }
bool readDah() { return digitalRead(PIN_PADDLE_DAH) == LOW; }

// =====================================================
//                     DISPLAY
// =====================================================
void showLine2Padded(const char* s) {
  char buf[17];
  size_t n = strlen(s);
  for (byte i = 0; i < 16; i++) buf[i] = (i < n) ? s[i] : ' ';
  buf[16] = '\0';
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void updateLine1() {
  lcd.setCursor(0, 0);
  char l1[17];
  snprintf(l1, sizeof(l1), "Hz:%04d W:%02d", (int)freqHz, (int)wpmSet);
  lcd.print(l1);
}

void updateDisplayManualDecoded() {
  updateLine1();
  char l2[17];
  for (byte i = 0; i < 16; i++) l2[i] = (i < textLen) ? textLine[i] : ' ';
  l2[16] = '\0';
  lcd.setCursor(0, 1);
  lcd.print(l2);
}

void updateDisplayPaddle() {
  updateLine1();
  char l2[17];
  snprintf(l2, sizeof(l2), "PADDLE  WPM=%02d", (int)wpmSet);
  showLine2Padded(l2);
}

// =====================================================
//                 CLEAR DECODE (LONG PRESS ENCODER)
// =====================================================
void resetCwBuf() {
  cwLen = 0;
  cwBuf[0] = '\0';
}

void clearDecodedLine() {
  textLen = 0;
  textLine[0] = '\0';
  resetCwBuf();
  updateDisplayManualDecoded();
}

// =====================================================
//                     EEPROM
// =====================================================
void loadModeFromEEPROM() {
  byte m = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (m == EEPROM_MAGIC) {
    byte md = EEPROM.read(EEPROM_ADDR_MODE);
    appMode = (md == 1) ? MODE_PADDLE : MODE_MANUAL;
  } else {
    appMode = MODE_MANUAL;
  }
}

void saveModeToEEPROM() {
  EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.update(EEPROM_ADDR_MODE, (appMode == MODE_PADDLE) ? 1 : 0);
}

// =====================================================
//                TIMER SETUP (AUDIO)
// =====================================================
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

// =====================================================
//                MODE BUTTON (D8)
// =====================================================
void handleModeButton() {
  bool b = digitalRead(PIN_MODE_BTN);
  unsigned long now = millis();

  if (b != modeBtnLast) {
    modeBtnLast = b;

    if (b == LOW) {
      modePressStartMs = now;
      modeLongDone = false;
    } else {
      unsigned long pressDur = now - modePressStartMs;

      if (!modeLongDone && pressDur > 30 && pressDur < 800) {
        appMode = (appMode == MODE_MANUAL) ? MODE_PADDLE : MODE_MANUAL;

        // reset stati
        keyState = false;
        kst = K_IDLE;
        resetCwBuf();
        keyPrev = false;
        lastEdgeMs = now;

        if (appMode == MODE_PADDLE) updateDisplayPaddle();
        else updateDisplayManualDecoded();
      }
    }
  }

  if (b == LOW && !modeLongDone) {
    if ((now - modePressStartMs) > 800) {
      saveModeToEEPROM();
      modeLongDone = true;
      showLine2Padded("MODE SAVED");
    }
  }
}

// =====================================================
//                     ENCODER
//  Versione ROBUSTA: 1 detent = 1 delta
// =====================================================
int readEncoderDelta() {
  static int lastA = HIGH;
  int a = digitalRead(PIN_ENC_A);
  int delta = 0;

  // conteggio solo sul fronte di discesa di A
  if (lastA == HIGH && a == LOW) {
    int b = digitalRead(PIN_ENC_B);

    // Se il verso ti è invertito, scambia + e - qui sotto
    delta = (b == HIGH) ? +1 : -1;
  }

  lastA = a;
  return delta;
}

void handleEncoder() {
  int delta = readEncoderDelta();

  if (delta != 0) {
    if (appMode == MODE_PADDLE) {
      // PRIORITÀ: in PADDLE SOLO WPM (1 scatto = 1 WPM)
      wpmSet = clampInt(wpmSet + delta, WPM_MIN, WPM_MAX);
      updateDisplayPaddle();
    } else {
      // MANUAL: 10 Hz per scatto oppure 1 WPM per scatto
      if (manualEncMode == MAN_FREQ) {
        freqHz = clampInt(freqHz + (delta * 10), FREQ_MIN, FREQ_MAX);
      } else {
        wpmSet = clampInt(wpmSet + delta, WPM_MIN, WPM_MAX);
      }
      updateDisplayManualDecoded();
    }
  }

  // --- click / long-press encoder: SOLO in MANUAL ---
  bool sw = digitalRead(PIN_ENC_SW);
  unsigned long now = millis();

  if (appMode == MODE_MANUAL) {
    if (sw != encLastSw) {
      encSwDebMs = now;
      encLastSw = sw;

      if (sw == LOW) {
        encPressStartMs = now;
        encLongDone = false;
      } else {
        unsigned long pressDur = now - encPressStartMs;

        // click breve -> cambia FREQ/WPM
        if (!encLongDone && pressDur > 30 && pressDur < 3000) {
          manualEncMode = (manualEncMode == MAN_FREQ) ? MAN_WPM : MAN_FREQ;
          updateDisplayManualDecoded();
        }
      }
    }

    // long press > 3s -> clear riga 2
    if (sw == LOW && !encLongDone) {
      if ((now - encPressStartMs) >= 3000) {
        encLongDone = true;
        clearDecodedLine();
      }
    }
  }
}

// =====================================================
//             DECODER: morse table
// =====================================================
void pushTextChar(char c) {
  if (textLen < 16) {
    textLine[textLen++] = c;
    textLine[textLen] = '\0';
  } else {
    for (byte i = 0; i < 15; i++) textLine[i] = textLine[i + 1];
    textLine[15] = c;
    textLine[16] = '\0';
  }
}

char decodeMorse(const char* s) {
  // lettere
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

  // numeri
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

  // punteggiatura base
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

// =====================================================
//           MANUAL: key + decode (non-blocking)
// =====================================================
void handleManualKeyAndDecode() {
  if (appMode != MODE_MANUAL) return;

  unsigned long now = millis();
  keyNow = (digitalRead(PIN_KEY) == LOW);

  // audio gating + decode edges
  if (keyNow != keyPrev) {
    unsigned long dt = now - lastEdgeMs;
    if (dt < DEBOUNCE_MS) return;

    lastEdgeMs = now;
    uint16_t d = ditMs();

    if (keyPrev == true && keyNow == false) {
      // KEY UP: MARK
      unsigned long mark = dt;
      if (mark < (unsigned long)(2 * d)) {
        if (cwLen < sizeof(cwBuf) - 1) cwBuf[cwLen++] = '.';
      } else {
        if (cwLen < sizeof(cwBuf) - 1) cwBuf[cwLen++] = '-';
      }
      keyState = false;

    } else if (keyPrev == false && keyNow == true) {
      // KEY DOWN: SPACE
      unsigned long space = dt;

      if (space >= (unsigned long)(7 * d)) {
        finalizeLetter();
        if (textLen == 0 || textLine[textLen - 1] != ' ') pushTextChar(' ');
      } else if (space >= (unsigned long)(3 * d)) {
        finalizeLetter();
      }
      keyState = true;
    }

    keyPrev = keyNow;
    updateDisplayManualDecoded();
  }

  // timeout lettera se smetti
  if (!keyNow && cwLen > 0) {
    unsigned long idle = now - lastEdgeMs;
    if (idle > (unsigned long)(3 * ditMs())) {
      finalizeLetter();
      updateDisplayManualDecoded();
      lastEdgeMs = now;
    }
  }

  // aggiorna frequenza
  phaseInc = calcPhaseInc(freqHz);
}

// =====================================================
//           PADDLE KEYER (non-blocking)
// =====================================================
void chooseNextElement(bool dit, bool dah) {
  if (dit && dah) currentElem = lastWasDit ? E_DAH : E_DIT;
  else if (dit)   currentElem = E_DIT;
  else            currentElem = E_DAH;
}

void startElement(ElemType e) {
  uint16_t d = ditMs();
  unsigned long now = millis();

  keyState = true;
  kst = K_MARK;

  if (e == E_DIT) {
    kNextMs = now + d;
    lastWasDit = true;
  } else {
    kNextMs = now + (uint16_t)(3 * d);
    lastWasDit = false;
  }
}

void startGap() {
  uint16_t d = ditMs();
  unsigned long now = millis();
  keyState = false;
  kst = K_GAP;
  kNextMs = now + d;
}

void handlePaddleKeyer() {
  if (appMode != MODE_PADDLE) return;

  phaseInc = calcPhaseInc(freqHz);

  unsigned long now = millis();

  static unsigned long lastReadMs = 0;
  static bool dit = false, dah = false;

  if (now - lastReadMs >= PADDLE_DEBOUNCE_MS) {
    dit = readDit();
    dah = readDah();
    lastReadMs = now;
  }

  switch (kst) {
    case K_IDLE:
      keyState = false;
      if (dit || dah) {
        chooseNextElement(dit, dah);
        startElement(currentElem);
      }
      break;

    case K_MARK:
      if ((long)(now - kNextMs) >= 0) startGap();
      break;

    case K_GAP:
      if ((long)(now - kNextMs) >= 0) {
        if (dit || dah) {
          chooseNextElement(dit, dah);
          startElement(currentElem);
        } else {
          kst = K_IDLE;
          keyState = false;
        }
      }
      break;
  }
}

// =====================================================
//                     SETUP / LOOP
// =====================================================
void setup() {
  pinMode(PIN_KEY, INPUT_PULLUP);
  pinMode(PIN_PADDLE_DIT, INPUT_PULLUP);
  pinMode(PIN_PADDLE_DAH, INPUT_PULLUP);
  pinMode(PIN_MODE_BTN, INPUT_PULLUP);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  // init decoder buffers
  textLen = 0;
  textLine[0] = '\0';
  resetCwBuf();

  // sinusoide
  for (int i = 0; i < 256; i++) {
    float ang = 2.0f * 3.14159265f * (float)i / 256.0f;
    int v = (int)lroundf(127.0f * sinf(ang));
    sine256[i] = (int8_t)v;
  }

  loadModeFromEEPROM();

  phaseInc = calcPhaseInc(freqHz);

  noInterrupts();
  setupTimer1PWM_62k5();
  setupTimer2SampleISR_15625();
  interrupts();

  unsigned long now = millis();
  lastEdgeMs = now;
  keyPrev = false;

  if (appMode == MODE_PADDLE) updateDisplayPaddle();
  else updateDisplayManualDecoded();
}

void loop() {
  handleModeButton();
  handleEncoder();

  if (appMode == MODE_MANUAL) {
    handleManualKeyAndDecode();
  } else {
    handlePaddleKeyer();
  }
}
