# Morse Code Oscillophone – Versione 2.0

**Autore:** Emanuele Rossi – IK1APW  
**Piattaforma:** Arduino Nano / Arduino UNO  
**Ambito:** Radioamatoriale – Telegrafia CW – Didattica  

Oscillofono è un oscillatore CW a **uscita sinusoidale** progettato come
strumento **didattico** per l’apprendimento e l’allenamento della
telegrafia Morse.

La **Versione 2.0** introduce la modalità **PADDLE (keyer automatico)**,
mantenendo piena compatibilità con la modalità **MANUAL (tasto verticale)**.

---

## ✨ Caratteristiche principali

### 🔹 Modalità MANUAL (tasto verticale)
- Generazione CW con **onda sinusoidale reale (DDS)**
- Inviluppo di attacco/rilascio per eliminare i *click*
- **Decodifica Morse in tempo reale** su LCD (riga 2)
- Supporto lettere, numeri e punteggiatura base
- **Cancellazione del testo decodificato** con pressione lunga del pulsante encoder (> 3 s)

### 🔹 Modalità PADDLE (keyer automatico)
- Supporto paddle **3 fili con comune a massa**
- Keyer automatico **non-blocking**
- Supporto *squeeze* (alternanza DIT / DAH)
- Encoder con **priorità assoluta sulla velocità**
  - 1 scatto = 1 WPM
- Ideale per allenamento alla trasmissione CW

### 🔹 Interfaccia utente
- LCD 16×2 con interfaccia I2C
- Encoder rotativo con pulsante
- Pulsante dedicato MODE per selezione MANUAL / PADDLE
  - pressione breve → cambia modalità
  - pressione lunga → salva modalità in EEPROM

---

## 🎛️ Controlli

### Encoder
- **Rotazione**
  - MANUAL →  
    - FREQ: 10 Hz per scatto  
    - WPM: 1 WPM per scatto
  - PADDLE →  
    - WPM: 1 WPM per scatto
- **Click (MANUAL)** → commuta FREQ / WPM
- **Pressione lunga (MANUAL)** → cancella testo decodificato (riga 2)

### Pulsante MODE
- Click → MANUAL ↔ PADDLE
- Pressione lunga → salva modalità in EEPROM

---

## 🧩 Hardware richiesto

- Arduino Nano o Arduino UNO
- Display LCD 16×2 I2C (indirizzo tipico 0x27)
- Encoder rotativo con pulsante
- Tasto verticale CW
- Paddle CW (3 fili, comune a massa)
- Filtro RC + amplificatore audio + altoparlante

---

## 🔌 Assegnazione pin (default)

| Funzione           | Pin Arduino |
|--------------------|-------------|
| Tasto verticale    | D2          |
| Encoder CLK        | D3          |
| Encoder DT         | D4          |
| Encoder SW         | D5          |
| Paddle DIT         | D6          |
| Paddle DAH         | D7          |
| Pulsante MODE      | D8          |
| Audio PWM          | D9          |
| LCD I2C SDA        | A4          |
| LCD I2C SCL        | A5          |

(Tutti gli ingressi usano `INPUT_PULLUP`)

---

## 🧠 Filosofia del progetto

Oscillofono non è solo un oscillatore CW, ma uno **strumento didattico**:

- aiuta a **imparare il ritmo corretto**
- consente l’allenamento sia in **ricezione** che in **trasmissione**
- è pensato per **corsi di telegrafia**, autocostruzione e sperimentazione

---

## 📦 Struttura del repository


## License
This project is shared for **educational and amateur radio use**.

## License
This project is released under the MIT License.

## 📡 73
**Emanuele Rossi – IK1APW**
