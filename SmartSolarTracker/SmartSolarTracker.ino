/*
 * ============================================================================
 *   SMART SOLAR TRACKER - S AUTOMATICKÝM SKENOM NAJVÄČŠIEHO SVETLA
 * ============================================================================
 *   Hardware:
 *     - Mikrokontrolér: Arduino UNO R4 WiFi
 *     - Horizontálne servo: Pin D9 (rozsah 30° až 120°, stred 75°)
 *     - Vertikálne servo:   Pin D8 (rozsah 90° až 180°, stred 135°)
 *     - 4x Fotorezistor (LDR) s tieniacim krížom:
 *         A0 = VPRAVO HORE (Top-Right)
 *         A1 = VPRAVO DOLE (Bottom-Right)
 *         A2 = VĽAVO HORE  (Top-Left)
 *         A3 = VĽAVO DOLE  (Bottom-Left)
 *
 *   NOVÁ FUNKCIA - Úvodný sken a kalibrácia (Startup Sun-Seeking):
 *     - Po zapnutí / reštarte tracker automaticky prejde celý rozsah:
 *         1. Prejde horizontálnu os (30° -> 120°) a zmeria celkovú intenzitu svetla.
 *         2. Natočí sa na uhol s najvyšším nameraným svetlom.
 *         3. Prejde vertikálnu os (90° -> 180°) a zmeria celkovú intenzitu svetla.
 *         4. Nastaví vertikálne servo na polohu s maximom svetla.
 *     - Po dokončení skenu plynule prejde do kontinuálneho sledovania slnka.
 * ============================================================================
 */

#include <Servo.h>

// ============================================================================
// 1. HARDVÉROVÁ KONFIGURÁCIA A PINOUT
// ============================================================================
const uint8_t PIN_SERVO_H = 9;   // Horizontálne servo (azimut)
const uint8_t PIN_SERVO_V = 8;   // Vertikálne servo (elevácia)

const uint8_t PIN_LDR_TR  = A0;  // Fotorezistor: Vpravo hore
const uint8_t PIN_LDR_BR  = A1;  // Fotorezistor: Vpravo dole
const uint8_t PIN_LDR_TL  = A2;  // Fotorezistor: Vľavo hore
const uint8_t PIN_LDR_BL  = A3;  // Fotorezistor: Vľavo dole

// ============================================================================
// 2. ÚVODNÝ SKEN A KALIBRÁCIA PO ŠTARTE
// ============================================================================
// true = po zapnutí preskenuje celý priestor a nastaví sa na najsvetlejšie miesto
const bool ENABLE_STARTUP_SCAN = true;

// Rýchlosť skenovania: oneskorenie na každý krok (ms) a veľkosť kroku (stupne)
const int SCAN_STEP_DEG = 2;   // krok o 2 stupne
const int SCAN_DELAY_MS = 25;  // 25 ms na krok (celý sken trvá cca 3 sekundy)

// ============================================================================
// 3. SMEROVÉ PREPÍNAČE (LADENIE ORIENTÁCIE)
// ============================================================================
// Horizontálny smer serva D9:
// false = viac svetla vpravo -> uhol klesá k 30° (reverznuté vľavo/vpravo)
// true  = viac svetla vpravo -> uhol rastie k 120°
const bool REVERSE_H_SERVO = true;

// Vertikálny smer serva D8:
// false = viac svetla hore posúva k 90°, viac svetla dole k 180° (TERAZ INVERTOVANÉ)
// true  = viac svetla hore posúva k 180°, viac svetla dole k 90°
const bool REVERSE_V_SERVO = false;

// Prehodenie senzorov vľavo <-> vpravo:
// false = štandardné zapojenie (A0/A1 vpravo, A2/A3 vľavo)
// true  = zrkadlové zapojenie
const bool SWAP_LR_SENSORS = false;

// Prehodenie senzorov hore <-> dole:
// false = štandardné zapojenie (A0/A2 hore, A1/A3 dole)
// true  = zrkadlové zapojenie (A1/A3 hore, A0/A2 dole)
const bool SWAP_TB_SENSORS = false;

// ============================================================================
// 4. BEZPEČNOSTNÉ LIMITY SERV
// ============================================================================
const int LIMIT_H_MIN = 30;   // 30° mechanický doraz
const int LIMIT_H_MAX = 120;  // 120° mechanický doraz
const int START_H_POS = 75;   // Počiatočný horizontálny stred

const int LIMIT_V_MIN = 90;   // 90° mechanický doraz
const int LIMIT_V_MAX = 180;  // 180° mechanický doraz
const int START_V_POS = 135;  // Počiatočný vertikálny stred

// ============================================================================
// 5. SOFTVÉROVÁ KALIBRÁCIA JEDNOTLIVÝCH LDR
// ============================================================================
const float A0_OFFSET = 0.0f;
const float A1_OFFSET = 0.0f;
const float A2_OFFSET = 0.0f;
const float A3_OFFSET = 0.0f;

const float A0_GAIN   = 1.0f;
const float A1_GAIN   = 1.0f;
const float A2_GAIN   = 1.0f;
const float A3_GAIN   = 1.0f;

// ============================================================================
// 6. REGULAČNÉ PARAMETRE, HYSTERÉZIA A DEAD ZONE
// ============================================================================
const float FILTER_ALPHA = 0.22f;      // Koeficient EMA filtra
const float DEAD_ZONE    = 0.028f;     // 2.8 % mŕtva zóna
const float HYSTERESIS   = 0.012f;     // 1.2 % hysterézia proti kmitaniu
const float MIN_LIGHT_THRESHOLD = 15.0f; // Nočný prah

// Dynamické prahy rýchlosti:
const float THRESHOLD_MEDIUM = 0.080f; // 8 %
const float THRESHOLD_LARGE  = 0.180f; // 18 %

const int STEP_FINE   = 1; // 1° krok
const int STEP_MEDIUM = 2; // 2° krok
const int STEP_FAST   = 3; // 3° krok

// ============================================================================
// 7. ČASOVANIE ÚLOH (NEBLOKUJÚCE millis())
// ============================================================================
const unsigned long INTERVAL_SENSORS_MS   = 12;  // Vzorkovanie LDR (~83 Hz)
const unsigned long INTERVAL_TRACKING_MS  = 35;  // Vyhodnotenie serv (~28 Hz)
const unsigned long INTERVAL_TELEMETRY_MS = 300; // Výpis do Serial Monitora (~300 ms)

// ============================================================================
// GLOBÁLNE PREMENNÉ
// ============================================================================
Servo servoH; // D9
Servo servoV; // D8

int currentPosH = START_H_POS;
int currentPosV = START_V_POS;

// Filtrované hodnoty senzorov
float filteredA0 = 500.0f;
float filteredA1 = 500.0f;
float filteredA2 = 500.0f;
float filteredA3 = 500.0f;

// Agregované intenzity strán
float valRight  = 0.0f;
float valLeft   = 0.0f;
float valTop    = 0.0f;
float valBottom = 0.0f;

// Normalizované chyby [-1.0f až 1.0f]
float hError = 0.0f;
float vError = 0.0f;

// Stav aktivity v mŕtvej zóne (pre hysteréziu)
bool inDeadZoneH = true;
bool inDeadZoneV = true;

// Reťazce stavu pre telemetriu
char statusHStr[14] = "CENTER";
char statusVStr[14] = "CENTER";

// Časové pečiatky
unsigned long lastSensorsTime   = 0;
unsigned long lastTrackingTime  = 0;
unsigned long lastTelemetryTime = 0;

// ============================================================================
// POMOCNÉ FUNKCIE PRE MERANIE
// ============================================================================

/**
 * 4-násobný oversampling na odstránenie šumu ADC.
 */
float readAnalogOversampled(uint8_t pin) {
  int sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }
  return (float)sum * 0.25f;
}

/**
 * Vráti celkový súčet svetla zo všetkých 4 senzorov (pre hľadanie maxima).
 */
float readTotalLightInstant() {
  float r0 = (readAnalogOversampled(PIN_LDR_TR) + A0_OFFSET) * A0_GAIN;
  float r1 = (readAnalogOversampled(PIN_LDR_BR) + A1_OFFSET) * A1_GAIN;
  float r2 = (readAnalogOversampled(PIN_LDR_TL) + A2_OFFSET) * A2_GAIN;
  float r3 = (readAnalogOversampled(PIN_LDR_BL) + A3_OFFSET) * A3_GAIN;
  return r0 + r1 + r2 + r3;
}

/**
 * Kontinuálne vzorkovanie, kalibrácia a exponenciálne filtrovanie signálu.
 */
void updateSensorReadings() {
  float rawA0 = readAnalogOversampled(PIN_LDR_TR);
  float rawA1 = readAnalogOversampled(PIN_LDR_BR);
  float rawA2 = readAnalogOversampled(PIN_LDR_TL);
  float rawA3 = readAnalogOversampled(PIN_LDR_BL);

  float calA0 = (rawA0 + A0_OFFSET) * A0_GAIN;
  float calA1 = (rawA1 + A1_OFFSET) * A1_GAIN;
  float calA2 = (rawA2 + A2_OFFSET) * A2_GAIN;
  float calA3 = (rawA3 + A3_OFFSET) * A3_GAIN;

  if (calA0 < 0.0f) calA0 = 0.0f;
  if (calA1 < 0.0f) calA1 = 0.0f;
  if (calA2 < 0.0f) calA2 = 0.0f;
  if (calA3 < 0.0f) calA3 = 0.0f;

  filteredA0 = (FILTER_ALPHA * calA0) + ((1.0f - FILTER_ALPHA) * filteredA0);
  filteredA1 = (FILTER_ALPHA * calA1) + ((1.0f - FILTER_ALPHA) * filteredA1);
  filteredA2 = (FILTER_ALPHA * calA2) + ((1.0f - FILTER_ALPHA) * filteredA2);
  filteredA3 = (FILTER_ALPHA * calA3) + ((1.0f - FILTER_ALPHA) * filteredA3);

  if (SWAP_LR_SENSORS) {
    valRight = filteredA2 + filteredA3;
    valLeft  = filteredA0 + filteredA1;
  } else {
    valRight = filteredA0 + filteredA1;
    valLeft  = filteredA2 + filteredA3;
  }

  if (SWAP_TB_SENSORS) {
    valTop    = filteredA1 + filteredA3;
    valBottom = filteredA0 + filteredA2;
  } else {
    valTop    = filteredA0 + filteredA2;
    valBottom = filteredA1 + filteredA3;
  }
}

/**
 * Vypočíta normalizovanú odchýlku osvetlenia.
 */
bool calculateNormalizedErrors() {
  float sumH = valRight + valLeft;
  float sumV = valTop + valBottom;

  if (sumH < MIN_LIGHT_THRESHOLD || sumV < MIN_LIGHT_THRESHOLD) {
    hError = 0.0f;
    vError = 0.0f;
    return false;
  }

  hError = (valRight - valLeft) / sumH;
  vError = (valTop - valBottom) / sumV;

  return true;
}

/**
 * Dynamický výpočet kroku podľa veľkosti chyby.
 */
inline int calculateStep(float absErr) {
  if (absErr >= THRESHOLD_LARGE) {
    return STEP_FAST;   // 3°
  } else if (absErr >= THRESHOLD_MEDIUM) {
    return STEP_MEDIUM; // 2°
  } else {
    return STEP_FINE;   // 1°
  }
}

// ============================================================================
// ÚVODNÁ KALIBRÁCIA A SKEN NAJVÄČŠIEHO SVETLA (SUN-SEEKING)
// ============================================================================
void performStartupScan() {
  Serial.println(F("\n================================================="));
  Serial.println(F(">>> STARTUP KALIBRÁCIA: HĽADANIE MAXIMA SVETLA"));
  Serial.println(F("================================================="));

  // 1. Nastavenie vertikálnej osi na bezpečný stred pre horizontálny sken
  servoV.write(START_V_POS);
  delay(300);

  // --------------------------------------------------------------------------
  // FÁZA 1: Horizontálny sken (D9 od 30° do 120°)
  // --------------------------------------------------------------------------
  Serial.print(F("[1/2] Skenujem horizontálnu os ("));
  Serial.print(LIMIT_H_MIN); Serial.print(F("° -> "));
  Serial.print(LIMIT_H_MAX); Serial.println(F("°)..."));

  float maxLightH = -1.0f;
  int bestAngleH  = START_H_POS;

  servoH.write(LIMIT_H_MIN);
  delay(350); // Čas na dojazd do začiatočného uhla

  for (int pos = LIMIT_H_MIN; pos <= LIMIT_H_MAX; pos += SCAN_STEP_DEG) {
    servoH.write(pos);
    delay(SCAN_DELAY_MS);

    float light = readTotalLightInstant();
    if (light > maxLightH) {
      maxLightH  = light;
      bestAngleH = pos;
    }
  }

  Serial.print(F(">>> Nájdené maximum horizontálne: "));
  Serial.print(bestAngleH);
  Serial.print(F("° (Intenzita: "));
  Serial.print((int)maxLightH);
  Serial.println(F(")"));

  // Plynulý nájazd na nájdené horizontálne maximum
  servoH.write(bestAngleH);
  currentPosH = bestAngleH;
  delay(350);

  // --------------------------------------------------------------------------
  // FÁZA 2: Vertikálny sken (D8 od 90° do 180°) pri najlepšom azimute
  // --------------------------------------------------------------------------
  Serial.print(F("[2/2] Skenujem vertikálnu os ("));
  Serial.print(LIMIT_V_MIN); Serial.print(F("° -> "));
  Serial.print(LIMIT_V_MAX); Serial.println(F("°)..."));

  float maxLightV = -1.0f;
  int bestAngleV  = START_V_POS;

  servoV.write(LIMIT_V_MIN);
  delay(350);

  for (int pos = LIMIT_V_MIN; pos <= LIMIT_V_MAX; pos += SCAN_STEP_DEG) {
    servoV.write(pos);
    delay(SCAN_DELAY_MS);

    float light = readTotalLightInstant();
    if (light > maxLightV) {
      maxLightV  = light;
      bestAngleV = pos;
    }
  }

  Serial.print(F(">>> Nájdené maximum vertikálne: "));
  Serial.print(bestAngleV);
  Serial.print(F("° (Intenzita: "));
  Serial.print((int)maxLightV);
  Serial.println(F(")"));

  // Nastavenie na nájdené vertikálne maximum
  servoV.write(bestAngleV);
  currentPosV = bestAngleV;
  delay(400);

  Serial.println(F("================================================="));
  Serial.print(F(">>> KALIBRÁCIA DOKONČENÁ: D9="));
  Serial.print(bestAngleH);
  Serial.print(F("°, D8="));
  Serial.print(bestAngleV);
  Serial.println(F("°"));
  Serial.println(F(">>> PRECHÁDZAM NA KONTINUÁLNE JEMNÉ SLEDOVANIE"));
  Serial.println(F("=================================================\n"));
}

// ============================================================================
// MANUÁLNY OVERRIDE REŽIM (OVLÁDANIE Z WEBOVEJ APLIKÁCIE)
// ============================================================================
bool isManualMode = false;
String inputCmdBuffer = "";

void executeManualCommand(const String& cmd) {
  if (cmd == "MANUAL:ON") {
    isManualMode = true;
    strcpy(statusHStr, "MANUAL");
    strcpy(statusVStr, "MANUAL");
  } else if (cmd == "MANUAL:OFF") {
    isManualMode = false;
  } else if (cmd == "CMD:SCAN" || cmd == "SCAN") {
    performStartupScan();
  } else if (isManualMode) {
    const int MANUAL_STEP = 3; // Krok o 3 stupne na jedno stlačenie klávesu

    if (cmd == "MOVE:UP") {
      // HORE:
      if (REVERSE_V_SERVO) currentPosV += MANUAL_STEP;
      else currentPosV -= MANUAL_STEP;
      currentPosV = constrain(currentPosV, LIMIT_V_MIN, LIMIT_V_MAX);
      servoV.write(currentPosV);
    } else if (cmd == "MOVE:DOWN") {
      // DOLE:
      if (REVERSE_V_SERVO) currentPosV -= MANUAL_STEP;
      else currentPosV += MANUAL_STEP;
      currentPosV = constrain(currentPosV, LIMIT_V_MIN, LIMIT_V_MAX);
      servoV.write(currentPosV);
    } else if (cmd == "MOVE:LEFT") {
      // VĽAVO:
      if (REVERSE_H_SERVO) currentPosH -= MANUAL_STEP;
      else currentPosH += MANUAL_STEP;
      currentPosH = constrain(currentPosH, LIMIT_H_MIN, LIMIT_H_MAX);
      servoH.write(currentPosH);
    } else if (cmd == "MOVE:RIGHT") {
      // VPRAVO:
      if (REVERSE_H_SERVO) currentPosH += MANUAL_STEP;
      else currentPosH -= MANUAL_STEP;
      currentPosH = constrain(currentPosH, LIMIT_H_MIN, LIMIT_H_MAX);
      servoH.write(currentPosH);
    } else if (cmd == "MOVE:CENTER") {
      currentPosH = START_H_POS;
      currentPosV = START_V_POS;
      servoH.write(currentPosH);
      servoV.write(currentPosV);
    }
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputCmdBuffer.length() > 0) {
        inputCmdBuffer.trim();
        executeManualCommand(inputCmdBuffer);
        inputCmdBuffer = "";
      }
    } else {
      inputCmdBuffer += c;
      if (inputCmdBuffer.length() > 32) {
        inputCmdBuffer = "";
      }
    }
  }
}

// ============================================================================
// REGULAČNÝ CYKLUS TRACKINGU (KONTINUÁLNY)
// ============================================================================
void processTracking() {
  // Ak je zapnutý manuálny režim z aplikácie, automatický tracking stojí
  if (isManualMode) {
    strcpy(statusHStr, "MANUAL");
    strcpy(statusVStr, "MANUAL");
    return;
  }

  bool lightSufficient = calculateNormalizedErrors();

  if (!lightSufficient) {
    strcpy(statusHStr, "LOW LIGHT");
    strcpy(statusVStr, "LOW LIGHT");
    return;
  }

  // --------------------------------------------------------------------------
  // 1. HORIZONTÁLNA OS (D9)
  // --------------------------------------------------------------------------
  float absH = fabsf(hError);
  float thresholdH = inDeadZoneH ? (DEAD_ZONE + HYSTERESIS) : DEAD_ZONE;

  if (absH <= thresholdH) {
    inDeadZoneH = true;
    strcpy(statusHStr, "CENTER");
  } else {
    inDeadZoneH = false;
    int step = calculateStep(absH);

    if (hError > 0.0f) {
      // Viac svetla VPRAVO
      strcpy(statusHStr, "RIGHT");
      if (REVERSE_H_SERVO) {
        currentPosH += step;
      } else {
        currentPosH -= step;
      }
    } else {
      // Viac svetla VĽAVO
      strcpy(statusHStr, "LEFT");
      if (REVERSE_H_SERVO) {
        currentPosH -= step;
      } else {
        currentPosH += step;
      }
    }

    currentPosH = constrain(currentPosH, LIMIT_H_MIN, LIMIT_H_MAX);
    servoH.write(currentPosH);
  }

  // --------------------------------------------------------------------------
  // 2. VERTIKÁLNA OS (D8)
  // --------------------------------------------------------------------------
  float absV = fabsf(vError);
  float thresholdV = inDeadZoneV ? (DEAD_ZONE + HYSTERESIS) : DEAD_ZONE;

  if (absV <= thresholdV) {
    inDeadZoneV = true;
    strcpy(statusVStr, "CENTER");
  } else {
    inDeadZoneV = false;
    int step = calculateStep(absV);

    if (vError > 0.0f) {
      // Viac svetla HORE
      strcpy(statusVStr, "UP");
      if (REVERSE_V_SERVO) {
        currentPosV += step;
      } else {
        currentPosV -= step;
      }
    } else {
      // Viac svetla DOLE
      strcpy(statusVStr, "DOWN");
      if (REVERSE_V_SERVO) {
        currentPosV -= step;
      } else {
        currentPosV += step;
      }
    }

    currentPosV = constrain(currentPosV, LIMIT_V_MIN, LIMIT_V_MAX);
    servoV.write(currentPosV);
  }
}

// Režim výstupu sériovej linky:
// true  = čisté JSON API pre Web Serial HTML aplikáciu ({"a0":..., "d9":...})
// false = pôvodný textový formát pre Serial Monitor v Arduino IDE
const bool OUTPUT_JSON_API = true;

// ============================================================================
// VÝPIS TELEMETRIE DO SERIAL MONITORA / SERIAL API
// ============================================================================
void printTelemetry() {
  // Zostavenie reťazca celkového stavu
  char combinedStatus[24];
  if (strcmp(statusHStr, "CENTER") == 0 && strcmp(statusVStr, "CENTER") == 0) {
    strcpy(combinedStatus, "CENTER");
  } else {
    combinedStatus[0] = '\0';
    bool hasH = false;
    if (strcmp(statusHStr, "CENTER") != 0) {
      strcat(combinedStatus, statusHStr);
      hasH = true;
    }
    if (strcmp(statusVStr, "CENTER") != 0) {
      if (hasH) strcat(combinedStatus, " / ");
      strcat(combinedStatus, statusVStr);
    }
  }

  if (OUTPUT_JSON_API) {
    // Čisté jedno-riadkové JSON API pre Web Serial HTML aplikáciu
    Serial.print(F("{\"a0\":"));       Serial.print((int)filteredA0);
    Serial.print(F(",\"a1\":"));      Serial.print((int)filteredA1);
    Serial.print(F(",\"a2\":"));      Serial.print((int)filteredA2);
    Serial.print(F(",\"a3\":"));      Serial.print((int)filteredA3);
    Serial.print(F(",\"right\":"));   Serial.print((int)valRight);
    Serial.print(F(",\"left\":"));    Serial.print((int)valLeft);
    Serial.print(F(",\"top\":"));     Serial.print((int)valTop);
    Serial.print(F(",\"bottom\":"));  Serial.print((int)valBottom);
    Serial.print(F(",\"h_err\":"));   Serial.print(hError, 4);
    Serial.print(F(",\"v_err\":"));   Serial.print(vError, 4);
    Serial.print(F(",\"d9\":"));      Serial.print(currentPosH);
    Serial.print(F(",\"d8\":"));      Serial.print(currentPosV);
    Serial.print(F(",\"status_h\":\"")); Serial.print(statusHStr);
    Serial.print(F("\",\"status_v\":\"")); Serial.print(statusVStr);
    Serial.print(F("\",\"status\":\"")); Serial.print(combinedStatus);
    Serial.print(F("\",\"manual\":"));   Serial.print(isManualMode ? F("true") : F("false"));
    Serial.println(F("\"}"));
  } else {
    // Pôvodný textový formát pre štandardný Serial Monitor
    Serial.print(F("A0: ")); Serial.println((int)filteredA0);
    Serial.print(F("A1: ")); Serial.println((int)filteredA1);
    Serial.print(F("A2: ")); Serial.println((int)filteredA2);
    Serial.print(F("A3: ")); Serial.println((int)filteredA3);
    Serial.println();

    Serial.print(F("RIGHT: "));  Serial.println((int)valRight);
    Serial.print(F("LEFT: "));   Serial.println((int)valLeft);
    Serial.print(F("TOP: "));    Serial.println((int)valTop);
    Serial.print(F("BOTTOM: ")); Serial.println((int)valBottom);
    Serial.println();

    Serial.print(F("H_ERROR: ")); Serial.println(hError, 4);
    Serial.print(F("V_ERROR: ")); Serial.println(vError, 4);
    Serial.println();

    Serial.print(F("D9: ")); Serial.print(currentPosH); Serial.println(F("°"));
    Serial.print(F("D8: ")); Serial.print(currentPosV); Serial.println(F("°"));
    Serial.println();

    Serial.print(F("STATUS: "));
    Serial.println(combinedStatus);

    Serial.println(F("----------------------------------------"));
  }
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LDR_TR, INPUT);
  pinMode(PIN_LDR_BR, INPUT);
  pinMode(PIN_LDR_TL, INPUT);
  pinMode(PIN_LDR_BL, INPUT);

  // Inicializácia servomotorov
  servoH.attach(PIN_SERVO_H);
  servoV.attach(PIN_SERVO_V);

  // Ak je povolený úvodný sken, preskenuje oblohu a nájde najsvetlejšie miesto
  if (ENABLE_STARTUP_SCAN) {
    performStartupScan();
  } else {
    currentPosH = START_H_POS;
    currentPosV = START_V_POS;
    servoH.write(currentPosH);
    servoV.write(currentPosV);
    delay(200);
  }

  // Inicializácia filtrov z aktuálnej pozície
  filteredA0 = (readAnalogOversampled(PIN_LDR_TR) + A0_OFFSET) * A0_GAIN;
  filteredA1 = (readAnalogOversampled(PIN_LDR_BR) + A1_OFFSET) * A1_GAIN;
  filteredA2 = (readAnalogOversampled(PIN_LDR_TL) + A2_OFFSET) * A2_GAIN;
  filteredA3 = (readAnalogOversampled(PIN_LDR_BL) + A3_OFFSET) * A3_GAIN;

  unsigned long now = millis();
  lastSensorsTime   = now;
  lastTrackingTime  = now;
  lastTelemetryTime = now;
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================
void loop() {
  // Spracovanie príkazov z Web Serial aplikácie (manuálne ovládanie)
  handleSerialCommands();

  unsigned long currentMillis = millis();

  // 1. Čítanie a filtrovanie senzorov každých 12 ms
  if (currentMillis - lastSensorsTime >= INTERVAL_SENSORS_MS) {
    lastSensorsTime = currentMillis;
    updateSensorReadings();
  }

  // 2. Regulačný výpočet a pohyb serv každých 35 ms
  if (currentMillis - lastTrackingTime >= INTERVAL_TRACKING_MS) {
    lastTrackingTime = currentMillis;
    processTracking();
  }

  // 3. Telemetria každých 300 ms
  if (currentMillis - lastTelemetryTime >= INTERVAL_TELEMETRY_MS) {
    lastTelemetryTime = currentMillis;
    printTelemetry();
  }
}
