/*
  AURORA - PCB TX: Tes Overall PCB + BATT Monitoring + Sync PWM Dimming + LoRa E220
  =================================================================================
  Board   : ESP32-WROOM-32D / 32E -> "ESP32 Dev Module"
  Core    : esp32 by Espressif 3.x (diuji untuk 3.3.11)
  Serial  : 115200 baud, line ending "Newline"
  Library : TIDAK perlu library tambahan

  Pembaruan Fitur:
    1. Pembacaan Power Monitoring Baterai (INA219 BATT - U11) beserta Daya (Watt).
       - Mendukung auto-discovery maupun penguncian alamat I2C manual.
       - Mampu mengukur arus dua arah (charging / discharging) pada jalur baterai.
    2. Sinkronisasi penuh PWM LED HPL (IO18 - MOSFET Q3) & LED Internal (IO32 - D2):
       - Duty cycle LED Internal selalu 100% selaras dengan LED HPL di semua mode.
    3. Kontrol Dimming PWM via Serial Monitor:
       - Ketik angka 0..100 : Set duty cycle manual (0% - 100%)
       - Ketik + atau -     : Naikkan / turunkan duty cycle bertahap (+/- 5%)
       - Ketik d            : Mode Auto-Sweep Dimming Test (fade in/out 0% <-> 100%)
       - Ketik a            : Mode Simulasi Adaptive Dimming (Standby 20% <-> Motion 100% <-> Cooldown 50% <-> Off 0%)
       - Ketik t            : Mode Toggle ON/OFF (0% <-> 100%) tiap 3 detik

  Pemetaan pin:
    IO21 SDA / IO22 SCL  : bus I2C
    IO32 LED internal    : D2 (Selaras dengan PWM_DIMMING)
    IO18 PWM_DIMMING     : Gate MOSFET LED HPL (Q3 pada skematik)
    IO23 BUZZ            : MOSFET Q6 -> Buzzer
    IO33 DATA_NTC5       : ADC1
    IO34 OUT_PIR         : Input-only
    IO16 RX1 <- TXD LoRa | IO17 TX1 -> RXD LoRa
    IO25 M0 | IO26 M1 | IO27 AUX  (E220)
    IO13 EXT_GPIO        : SHORT KE GND -> dikunci INPUT, JANGAN OUTPUT!
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================= Pin =================
const int PIN_SDA     = 21;
const int PIN_SCL     = 22;
const int PIN_LED_INT = 32;
const int PIN_PWM_HPL = 18;
const int PIN_BUZZ    = 23;
const int PIN_NTC5    = 33;
const int PIN_PIR     = 34;
const int PIN_SHORTED = 13;

const int PIN_LORA_RX  = 16;   // ESP RX  <- TXD modul
const int PIN_LORA_TX  = 17;   // ESP TX  -> RXD modul
const int PIN_LORA_M0  = 25;
const int PIN_LORA_M1  = 26;
const int PIN_LORA_AUX = 27;

// ================= Konfigurasi Alamat I2C INA219 =================
// Berdasarkan skematik:
// - U12 (Main SCC) : A0=GND, A1=VS  -> 0x44
// - U9  (PV)       : A0=GND, A1=GND -> 0x40 (default)
// - U11 (BATT)     : A0=VS,  A1=GND -> 0x41 (atau sesuai jumper A0/A1 pada PCB)
const uint8_t ADDR_INA_MAIN_DEFAULT = 0x44;
const uint8_t ADDR_INA_PV_DEFAULT   = 0x40;
const uint8_t ADDR_INA_BATT_DEFAULT = 0x41;

int addrInaMain = -1, addrInaPV = -1, addrInaBatt = -1;
int addrADS = -1, addrLM75 = -1;
const uint8_t ADDR_BH1750[2] = {0x23, 0x5C};
bool hasBH[2] = {false, false};

// ================= Parameter Hardware =================
const float SHUNT_MAIN_OHM = 0.020f;   // R27 (Main SCC) = 20 mOhm
const float SHUNT_PV_OHM   = 0.020f;   // R18 (PV)       = 20 mOhm
const float SHUNT_BATT_OHM = 0.020f;   // R24 (BATT)     = 20 mOhm
const float VCC_NTC        = 3.30f;
const float R_PULLUP       = 10000.0f;
const float NTC_R25        = 10000.0f; // Sesuaikan datasheet NTC
const float NTC_BETA       = 3950.0f;  // Sesuaikan datasheet NTC

// ================= PWM / Timing =================
const int PWM_FREQ = 5000;
const int PWM_RES  = 10;               // Resolusi 10-bit (0..1023)
const int PWM_MAX  = (1 << PWM_RES) - 1;

const uint32_t READ_INTERVAL_MS   = 1000;
const uint32_t TOGGLE_PERIOD_MS   = 3000;
const uint32_t SWEEP_STEP_MS      = 50;   // Kecepatan transisi mode sweep dimming
const uint32_t LORA_INTERVAL_MS   = 5000;
const uint32_t ACK_TIMEOUT_MS     = 3000;

// ================= Mode & State Kontrol Dimming =================
enum DimMode {
  MODE_MANUAL_PWM = 0, // Kontrol manual 0..100% dari Serial Monitor
  MODE_TOGGLE     = 1, // Toggle 0% <-> 100% setiap 3 detik
  MODE_SWEEP      = 2, // Tes naik-turun (fade) otomatis 0% <-> 100%
  MODE_ADAPTIVE   = 3  // Tes simulasi adaptive dimming (20% -> 100% -> 50% -> 0%)
};

DimMode  currentDimMode = MODE_MANUAL_PWM;
int      hplPercent     = 0;               // 0..100 %
uint32_t currentPwmRaw  = 0;               // 0..1023
bool     gateOn         = false;
int      sweepDir       = 1;               // +1 naik, -1 turun
int      adaptStage     = 0;
int      adaptTarget    = 20;

uint32_t lastToggle = 0, lastSweep = 0, lastAdaptState = 0, lastRead = 0, lastLoRa = 0;
volatile uint32_t pirRiseCount = 0;

struct Readings {
  float t    = 0.0f;                                        // Waktu berjalan (detik)
  float duty = 0.0f;                                        // Duty cycle (%)
  bool  mainOk = false; float mainV = 0, mainA = 0, mainW = 0;
  bool  pvOk   = false; float pvV   = 0, pvA   = 0, pvW   = 0;
  bool  battOk = false; float battV = 0, battA = 0, battW = 0, battAsq = 0;
  bool  lmOk   = false; float tBoard = NAN;
  float ntcV[5] = {NAN, NAN, NAN, NAN, NAN};
  float ntcT[5] = {NAN, NAN, NAN, NAN, NAN};
  bool  bhOk[2] = {false, false}; float lux[2] = {NAN, NAN};// lux[0]=lux_amb, lux[1]=lux_lamp
  int   pir   = 0;
  int   fault = 0;                                          // Placeholder fault: 0
  const char *label = "NORMAL";                             // Placeholder label: NORMAL
} data;

// LoRa statistik
HardwareSerial &LoRa = Serial2;
uint32_t txSeq = 0, txCount = 0, ackCount = 0;
uint32_t waitingAckSeq = 0, txTime = 0;
bool waitingAck = false;
String loraRxBuf;

// =====================================================================
//                              I2C Helper
// =====================================================================
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val >> 8);
  Wire.write(val & 0xFF);
  return Wire.endTransmission() == 0;
}

bool i2cWriteByte(uint8_t addr, uint8_t b) {
  Wire.beginTransmission(addr);
  Wire.write(b);
  return Wire.endTransmission() == 0;
}

bool i2cRead16(uint8_t addr, uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  out = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

// =====================================================================
//                     Scan & Identifikasi Device I2C
// =====================================================================
void identifyDevices() {
  addrInaMain = addrInaPV = addrInaBatt = addrADS = addrLM75 = -1;

  uint8_t detectedIna[8];
  int inaCount = 0;

  for (uint8_t a = 0x40; a <= 0x4F; a++) {
    if (!i2cPresent(a)) continue;
    uint16_t r0 = 0, r3 = 0;

    // Cek apakah device adalah INA219 (Reg 0x00 == 0x399F)
    if (i2cRead16(a, 0x00, r0) && r0 == 0x399F) {
      if (inaCount < 8) detectedIna[inaCount++] = a;
      continue;
    }

    // Cek ADS1115 atau LM75B dari Reg 0x03
    if (!i2cRead16(a, 0x03, r3)) continue;
    if (r3 == 0x7FFF && addrADS < 0)                  addrADS = a;
    else if ((r3 & 0xFF80) == 0x5000 && addrLM75 < 0) addrLM75 = a;
  }

  // 1. Pasangkan INA219 yang cocok dengan alamat default terlebih dahulu
  for (int i = 0; i < inaCount; i++) {
    uint8_t a = detectedIna[i];
    if (a == ADDR_INA_MAIN_DEFAULT && addrInaMain < 0)      { addrInaMain = a; detectedIna[i] = 0; }
    else if (a == ADDR_INA_PV_DEFAULT && addrInaPV < 0)     { addrInaPV   = a; detectedIna[i] = 0; }
    else if (a == ADDR_INA_BATT_DEFAULT && addrInaBatt < 0) { addrInaBatt = a; detectedIna[i] = 0; }
  }

  // 2. Jika ada INA219 dengan konfigurasi jumper alamat berbeda, masukkan ke slot yang masih kosong
  for (int i = 0; i < inaCount; i++) {
    uint8_t a = detectedIna[i];
    if (a == 0) continue;
    if (addrInaMain < 0)      addrInaMain = a;
    else if (addrInaPV < 0)   addrInaPV   = a;
    else if (addrInaBatt < 0) addrInaBatt = a;
  }

  // Fallback jika register config INA219 Main sudah berubah dari 0x399F
  if (addrInaMain < 0 && i2cPresent(ADDR_INA_MAIN_DEFAULT)) addrInaMain = ADDR_INA_MAIN_DEFAULT;

  // Bangunkan LM75B dari shutdown
  if (addrLM75 >= 0) {
    Wire.beginTransmission(addrLM75);
    Wire.write(0x01); Wire.write(0x00);
    Wire.endTransmission();
  }

  // BH1750: power on + continuous high-res
  for (int i = 0; i < 2; i++) {
    hasBH[i] = i2cPresent(ADDR_BH1750[i]);
    if (hasBH[i]) {
      i2cWriteByte(ADDR_BH1750[i], 0x01);
      i2cWriteByte(ADDR_BH1750[i], 0x10);
    }
  }

  auto pr = [](const char *name, int a) {
    if (a >= 0) Serial.printf("  %-12s -> 0x%02X\n", name, a);
    else        Serial.printf("  %-12s -> TIDAK TERDETEKSI\n", name);
  };
  Serial.println(F("  Identifikasi Perangkat I2C:"));
  pr("INA219 Main", addrInaMain);
  pr("INA219 PV",   addrInaPV);
  pr("INA219 BATT", addrInaBatt);
  pr("ADS1115",     addrADS);
  pr("LM75B",       addrLM75);
  pr("BH1750 #1",   hasBH[0] ? ADDR_BH1750[0] : -1);
  pr("BH1750 #2",   hasBH[1] ? ADDR_BH1750[1] : -1);
}

void scanI2C() {
  Serial.println(F("\n--- Scan I2C ---"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.printf("  Ditemukan 0x%02X\n", a); n++; }
  }
  if (n == 0) Serial.println(F("  Tidak ada device! Cek pull-up SDA/SCL & 3.3V."));
  identifyDevices();
  Serial.println(F("----------------"));
}

// =====================================================================
//                              Sensor
// =====================================================================
bool readINA219(uint8_t addr, float shuntOhm, float &busV, float &currentA, float &powerW) {
  uint16_t rawShunt, rawBus;
  if (!i2cRead16(addr, 0x01, rawShunt)) return false;
  if (!i2cRead16(addr, 0x02, rawBus))   return false;

  // Reg 0x01: Shunt voltage signed 16-bit, 1 LSB = 10 uV (0.01 mV)
  float shuntmV = (int16_t)rawShunt * 0.01f;
  // Reg 0x02: Bus voltage bit[15:3], 1 LSB = 4 mV (0.004 V)
  busV     = (rawBus >> 3) * 0.004f;
  currentA = (shuntmV / 1000.0f) / shuntOhm;
  powerW   = busV * currentA;
  return true;
}

bool readADS1115(uint8_t ch, float &volt) {
  if (addrADS < 0) return false;
  uint16_t cfg = 0x8000 | (0x4000 | ((uint16_t)ch << 12)) | 0x0200 | 0x0100 | 0x0080 | 0x0003;
  if (!i2cWrite16(addrADS, 0x01, cfg)) return false;
  uint32_t t0 = millis();
  uint16_t st;
  do {
    delay(2);
    if (!i2cRead16(addrADS, 0x01, st)) return false;
    if (millis() - t0 > 50) return false;
  } while (!(st & 0x8000));
  uint16_t raw;
  if (!i2cRead16(addrADS, 0x00, raw)) return false;
  volt = (int16_t)raw * (4.096f / 32768.0f);
  return true;
}

bool readLM75B(float &tempC) {
  if (addrLM75 < 0) return false;
  uint16_t raw;
  if (!i2cRead16(addrLM75, 0x00, raw)) return false;
  tempC = ((int16_t)raw >> 5) * 0.125f;
  return true;
}

bool readBH1750(int i, float &lux) {
  if (!hasBH[i]) return false;
  if (Wire.requestFrom((int)ADDR_BH1750[i], 2) != 2) return false;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  lux = raw / 1.2f;
  return true;
}

float ntcTempC(float v) {
  if (isnan(v) || v <= 0.05f || v >= 3.10f) return NAN;   // short / open
  float r = R_PULLUP * v / (VCC_NTC - v);
  float invT = 1.0f / 298.15f + log(r / NTC_R25) / NTC_BETA;
  return 1.0f / invT - 273.15f;
}

void IRAM_ATTR pirISR() { pirRiseCount++; }

void readAll() {
  data.t    = millis() / 1000.0f;
  data.duty = (float)hplPercent;

  // 1. Power Monitoring (Main SCC, PV, dan Battery)
  data.mainOk  = (addrInaMain >= 0) && readINA219(addrInaMain, SHUNT_MAIN_OHM, data.mainV, data.mainA, data.mainW);
  data.pvOk    = (addrInaPV   >= 0) && readINA219(addrInaPV,   SHUNT_PV_OHM,   data.pvV,   data.pvA,   data.pvW);
  data.battOk  = (addrInaBatt >= 0) && readINA219(addrInaBatt, SHUNT_BATT_OHM, data.battV, data.battA, data.battW);
  data.battAsq = data.battOk ? (data.battA * data.battA) : NAN;

  // 2. Sensor Suhu Board LM75B
  float t;
  data.lmOk = readLM75B(t);
  data.tBoard = data.lmOk ? t : NAN;

  // 3. Array NTC 1..4 (via ADS1115) & NTC 5 (via ADC Internal IO33)
  for (int ch = 0; ch < 4; ch++) {
    float v;
    data.ntcV[ch] = readADS1115(ch, v) ? v : NAN;
    data.ntcT[ch] = ntcTempC(data.ntcV[ch]);
  }
  data.ntcV[4] = analogReadMilliVolts(PIN_NTC5) / 1000.0f;
  data.ntcT[4] = ntcTempC(data.ntcV[4]);

  // 4. Sensor Cahaya BH1750 x2
  for (int i = 0; i < 2; i++) {
    float l;
    data.bhOk[i] = readBH1750(i, l);
    data.lux[i]  = data.bhOk[i] ? l : NAN;
  }

  // 5. Sensor Gerak PIR & Placeholder Fault/Label
  data.pir   = digitalRead(PIN_PIR) ? 1 : 0;
  data.fault = 0;
  data.label = "NORMAL";
}

void addField(String &s, bool ok, float v, int dec) {
  s += ',';
  if (!ok || isnan(v)) s += "NA";
  else s += String(v, dec);
}

// Susun baris CSV:
// t,duty,V_main,I_main,P_main,V_pv,I_pv,P_pv,V_bat,I_bat,I_bat_sq,NTC1,NTC2,NTC3,NTC4,NTC5,T_LM75,lux_amb,lux_lamp,PIR,fault,label
String buildCsvRow() {
  String row = String(data.t, 2);
  addField(row, true,        data.duty,    1); // duty
  addField(row, data.mainOk, data.mainV,   2); // V_main
  addField(row, data.mainOk, data.mainA,   3); // I_main
  addField(row, data.mainOk, data.mainW,   2); // P_main
  addField(row, data.pvOk,   data.pvV,     2); // V_pv
  addField(row, data.pvOk,   data.pvA,     3); // I_pv
  addField(row, data.pvOk,   data.pvW,     2); // P_pv
  addField(row, data.battOk, data.battV,   2); // V_bat
  addField(row, data.battOk, data.battA,   3); // I_bat
  addField(row, data.battOk, data.battAsq, 4); // I_bat_sq
  for (int i = 0; i < 5; i++) {
    addField(row, !isnan(data.ntcT[i]), data.ntcT[i], 1); // NTC1..NTC5
  }
  addField(row, data.lmOk,    data.tBoard, 2); // T_LM75
  addField(row, data.bhOk[0], data.lux[0], 1); // lux_amb
  addField(row, data.bhOk[1], data.lux[1], 1); // lux_lamp
  row += "," + String(data.pir);               // PIR
  row += "," + String(data.fault);             // fault
  row += "," + String(data.label);             // label
  return row;
}

void printFloatField(const char *key, bool ok, float val, int dec) {
  Serial.printf("%-12s : ", key);
  if (!ok || isnan(val)) Serial.println(F("NA"));
  else                   Serial.println(val, dec);
}

void printAll() {
  Serial.println(F("\n================ DATA TELEMETRI AURORA ================"));
  printFloatField("t",        true,        data.t,       2);
  printFloatField("duty",     true,        data.duty,    1);
  printFloatField("V_main",   data.mainOk, data.mainV,   2);
  printFloatField("I_main",   data.mainOk, data.mainA,   3);
  printFloatField("P_main",   data.mainOk, data.mainW,   2);
  printFloatField("V_pv",     data.pvOk,   data.pvV,     2);
  printFloatField("I_pv",     data.pvOk,   data.pvA,     3);
  printFloatField("P_pv",     data.pvOk,   data.pvW,     2);
  printFloatField("V_bat",    data.battOk, data.battV,   2);
  printFloatField("I_bat",    data.battOk, data.battA,   3);
  printFloatField("I_bat_sq", data.battOk, data.battAsq, 4);
  printFloatField("NTC1",     !isnan(data.ntcT[0]), data.ntcT[0], 1);
  printFloatField("NTC2",     !isnan(data.ntcT[1]), data.ntcT[1], 1);
  printFloatField("NTC3",     !isnan(data.ntcT[2]), data.ntcT[2], 1);
  printFloatField("NTC4",     !isnan(data.ntcT[3]), data.ntcT[3], 1);
  printFloatField("NTC5",     !isnan(data.ntcT[4]), data.ntcT[4], 1);
  printFloatField("T_LM75",   data.lmOk,    data.tBoard, 2);
  printFloatField("lux_amb",  data.bhOk[0], data.lux[0], 1);
  printFloatField("lux_lamp", data.bhOk[1], data.lux[1], 1);
  Serial.printf("%-12s : %d\n", "PIR",   data.pir);
  Serial.printf("%-12s : %d\n", "fault", data.fault);
  Serial.printf("%-12s : %s\n", "label", data.label);
  Serial.println(F("-------------------------------------------------------"));
}

// =====================================================================
//         Aktuator: Sinkronisasi Penuh LED HPL (Q3) & LED Internal
// =====================================================================
void applySyncedPWM(int pct) {
  hplPercent    = constrain(pct, 0, 100);
  currentPwmRaw = ((uint32_t)hplPercent * PWM_MAX) / 100;

  // Tulis nilai PWM yang sama ke MOSFET LED HPL (IO18) dan LED Internal (IO32)
  ledcWrite(PIN_PWM_HPL, currentPwmRaw);
  ledcWrite(PIN_LED_INT, currentPwmRaw);
}

void setManualPWM(int pct) {
  currentDimMode = MODE_MANUAL_PWM;
  applySyncedPWM(pct);
  Serial.printf(">> [PWM MANUAL] Duty Cycle = %d%% (Raw PWM: %lu / %d) -> LED HPL & LED Internal Selaras\n",
                hplPercent, (unsigned long)currentPwmRaw, PWM_MAX);
}

void setGateToggleState(bool on) {
  gateOn = on;
  applySyncedPWM(on ? 100 : 0);
  Serial.printf(">> [%lu ms] [TOGGLE] MOSFET LED HPL & LED Internal: %s (Duty = %d%%)\n",
                millis(), on ? "ON" : "OFF", hplPercent);
}

void startToggleMode() {
  currentDimMode = MODE_TOGGLE;
  lastToggle = millis();
  Serial.println(F(">> Mode TOGGLE Aktif: MOSFET LED HPL & LED Internal ON/OFF selaras tiap 3 detik"));
  setGateToggleState(false);
}

void startSweepMode() {
  currentDimMode = MODE_SWEEP;
  sweepDir = 1;
  lastSweep = millis();
  Serial.println(F(">> Mode AUTO-SWEEP Dimming Aktif: Duty cycle naik-turun otomatis 0% <-> 100%"));
}

void startAdaptiveMode() {
  currentDimMode = MODE_ADAPTIVE;
  adaptStage     = 0;
  adaptTarget    = 20;
  lastAdaptState = millis();
  lastSweep      = millis();
  applySyncedPWM(20);
  Serial.println(F(">> Mode ADAPTIVE DIMMING SIMULASI Aktif: Standby (20%) -> Motion (100%) -> Cooldown (50%) -> Off (0%)"));
}

void beep(int times, int onMs = 100, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_BUZZ, HIGH); delay(onMs);
    digitalWrite(PIN_BUZZ, LOW);  if (i < times - 1) delay(offMs);
  }
}

// =====================================================================
//                              LoRa E220
// =====================================================================
bool waitAux(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint32_t highSince = 0; bool high = false;
  while (millis() - t0 < timeoutMs) {
    if (digitalRead(PIN_LORA_AUX) == HIGH) {
      if (!high) { high = true; highSince = millis(); }
      if (millis() - highSince >= 3) return true;
    } else high = false;
    delay(1);
  }
  return false;
}

void loraSetMode(bool m0, bool m1) {
  waitAux(1000);
  digitalWrite(PIN_LORA_M0, m0);
  digitalWrite(PIN_LORA_M1, m1);
  delay(20);
  waitAux(1000);
  delay(20);
}

void loraReadConfig() {
  Serial.println(F("\n--- Konfigurasi LoRa E220 ---"));
  loraSetMode(1, 1);                  // mode konfigurasi
  while (LoRa.available()) LoRa.read();
  const uint8_t cmd[3] = {0xC1, 0x00, 0x08};
  LoRa.write(cmd, 3);

  uint8_t r[11]; int n = 0;
  uint32_t t0 = millis();
  while (n < 11 && millis() - t0 < 500) if (LoRa.available()) r[n++] = LoRa.read();
  loraSetMode(0, 0);                  // kembali ke mode normal

  if (n < 11 || r[0] != 0xC1) {
    Serial.printf("  GAGAL (terima %d byte). Cek wiring M0/M1/AUX/TX/RX & catu modul.\n", n);
    return;
  }
  uint8_t reg0 = r[5], reg1 = r[6], ch = r[7], reg3 = r[8];
  const char *baud[] = {"1200","2400","4800","9600","19200","38400","57600","115200"};
  const char *air[]  = {"2.4k","2.4k","2.4k","4.8k","9.6k","19.2k","38.4k","62.5k"};
  Serial.printf("  Alamat     : 0x%02X%02X\n", r[3], r[4]);
  Serial.printf("  UART       : %s bps\n", baud[(reg0 >> 5) & 7]);
  Serial.printf("  Air rate   : %s bps\n", air[reg0 & 7]);
  Serial.printf("  Channel    : %d (E220-900: %.3f MHz | E220-400: %.3f MHz)\n",
                ch, 850.125 + ch, 410.125 + ch);
  Serial.printf("  Sub-packet : %d byte\n", 200 >> ((reg1 >> 6) & 3));
  Serial.printf("  Transmisi  : %s\n", (reg3 & 0x40) ? "FIXED (harus transparent!)" : "transparent");
  Serial.println(F("  -> Pastikan Channel & Air rate SAMA dengan receiver."));
}

const uint8_t LORA_CHANNEL = 23;   // 873.125 MHz
void loraWriteConfig(uint8_t reg1 = 0x03) {   // 0x03 = 10 dBm, 0x00 = 22 dBm
  Serial.printf("\n--- Konfigurasi standar AURORA (C2, CH23, 2.4k, %s) ---\n",
                reg1 == 0x03 ? "10 dBm" : "22 dBm");
  loraSetMode(1, 1);
  while (LoRa.available()) LoRa.read();
  const uint8_t cmd[11] = {0xC2, 0x00, 0x08,   // C2 = config RAM, diulang tiap boot
                           0x00, 0x00,         // ADDH, ADDL
                           0x62,               // REG0: 9600, 8N1, 2.4k
                           reg1,               // REG1: 200B, daya TX
                           LORA_CHANNEL,       // REG2: channel
                           0x03,               // REG3: transparent, LBT off
                           0x00, 0x00};        // CRYPT_H, CRYPT_L
  LoRa.write(cmd, 11);
  uint8_t r[11]; int n = 0;
  uint32_t t0 = millis();
  while (n < 11 && millis() - t0 < 500) if (LoRa.available()) r[n++] = LoRa.read();
  loraSetMode(0, 0);
  if (n >= 3 && r[0] == 0xC1) Serial.println(F("  CONFIG_OK (RAM, otomatis diulang setiap reset)."));
  else Serial.printf("  GAGAL (terima %d byte)\n", n);
  loraReadConfig();
}

uint8_t xorChecksum(const String &s) {
  uint8_t cs = 0;
  for (size_t i = 0; i < s.length(); i++) cs ^= (uint8_t)s[i];
  return cs;
}

// Format Paket LoRa:
// AUR,seq,t,duty,V_main,I_main,P_main,V_pv,I_pv,P_pv,V_bat,I_bat,I_bat_sq,NTC1,NTC2,NTC3,NTC4,NTC5,T_LM75,lux_amb,lux_lamp,PIR,fault,label*CS
void loraSend() {
  txSeq++;
  String p = "AUR," + String(txSeq) + "," + buildCsvRow();

  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X", xorChecksum(p));
  p += tail;

  if (!waitAux(500)) { Serial.println(F(">> LoRa: AUX LOW (modul sibuk), paket dibatalkan")); return; }
  LoRa.print(p);
  LoRa.print('\n');
  LoRa.flush();

  uint32_t tA = millis(); bool wentLow = false;
  while (millis() - tA < 100) if (digitalRead(PIN_LORA_AUX) == LOW) { wentLow = true; break; }
  uint32_t tLow = millis();
  if (wentLow) waitAux(3000);
  if (wentLow) Serial.printf("   AUX LOW %lu ms (modul memancarkan)\n", millis() - tLow);
  else         Serial.println(F("   !! AUX tidak pernah LOW -> modul TIDAK memancarkan"));

  txCount++;
  waitingAck = true; waitingAckSeq = txSeq; txTime = millis();
  Serial.printf(">> LoRa TX #%lu (%d byte): %s\n", (unsigned long)txSeq, p.length() + 1, p.c_str());
  if (p.length() + 1 > 200) Serial.println(F("   !! Paket > 200 byte, bisa terpotong !!"));
}

void loraPoll() {
  while (LoRa.available()) {
    char c = LoRa.read();
    if (c == '\n') {
      loraRxBuf.trim();
      if (loraRxBuf.startsWith("ACK,")) {
        uint32_t seq = loraRxBuf.substring(4).toInt();
        ackCount++;
        if (waitingAck && seq == waitingAckSeq) {
          Serial.printf("<< LoRa ACK #%lu diterima, RTT %lu ms\n",
                        (unsigned long)seq, millis() - txTime);
          waitingAck = false;
        } else {
          Serial.printf("<< LoRa ACK #%lu (terlambat)\n", (unsigned long)seq);
        }
      } else if (loraRxBuf.length()) {
        Serial.printf("<< LoRa RX: %s\n", loraRxBuf.c_str());
      }
      loraRxBuf = "";
    } else if (loraRxBuf.length() < 220) {
      loraRxBuf += c;
    }
  }
  if (waitingAck && millis() - txTime > ACK_TIMEOUT_MS) {
    Serial.printf("!! LoRa: ACK #%lu tidak diterima (timeout)\n", (unsigned long)waitingAckSeq);
    waitingAck = false;
  }
}

// =====================================================================
//                         Kontrol Serial Monitor
// =====================================================================
void printHelp() {
  Serial.println(F("\n--- PERINTAH SERIAL MONITOR ---"));
  Serial.println(F("  0..100 : Set Duty Cycle PWM Manual (0% s/d 100%) [LED HPL & LED Internal Selaras]"));
  Serial.println(F("  + / -  : Naikkan / turunkan Duty Cycle sebesar 5%"));
  Serial.println(F("  d      : Mode Tes Auto-Sweep Dimming (Fade 0% <-> 100% otomatis)"));
  Serial.println(F("  a      : Mode Simulasi Adaptive Dimming (20% -> 100% -> 50% -> 0%)"));
  Serial.println(F("  t      : Mode Toggle ON/OFF (0% <-> 100%) tiap 3 detik"));
  Serial.println(F("  s      : Scan & identifikasi ulang bus I2C (Main, PV, BATT, ADS, LM75, BH1750)"));
  Serial.println(F("  b      : Tes Buzzer (3x beep)"));
  Serial.println(F("  l      : Paksa kirim paket data ke LoRa sekarang"));
  Serial.println(F("  c / w  : Baca / Tulis ulang konfigurasi LoRa E220"));
  Serial.println(F("  h      : Tampilkan bantuan ini\n"));
}

bool isNumericString(const String &s) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

void handleSerial() {
  if (!Serial.available()) return;
  String in = Serial.readStringUntil('\n');
  in.trim();
  if (in.length() == 0) return;

  if      (in.equalsIgnoreCase("s")) scanI2C();
  else if (in.equalsIgnoreCase("t")) startToggleMode();
  else if (in.equalsIgnoreCase("d")) startSweepMode();
  else if (in.equalsIgnoreCase("a")) startAdaptiveMode();
  else if (in == "+")                setManualPWM(hplPercent + 5);
  else if (in == "-")                setManualPWM(hplPercent - 5);
  else if (in.equalsIgnoreCase("b")) { Serial.println(F(">> Buzzer test")); beep(3, 200, 150); }
  else if (in.equalsIgnoreCase("l")) { readAll(); loraSend(); }
  else if (in.equalsIgnoreCase("c")) loraReadConfig();
  else if (in.equalsIgnoreCase("w")) loraWriteConfig(0x03);
  else if (in.equalsIgnoreCase("h")) printHelp();
  else if (isNumericString(in))      setManualPWM(in.toInt());
  else {
    Serial.printf("!! Perintah '%s' tidak dikenali.\n", in.c_str());
    printHelp();
  }
}

// =====================================================================
//                              Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== AURORA Overall PCB Test + BATT Monitoring + Sync Dimming ==="));
  Serial.printf("Core %d.%d.%d | Chip rev %d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH, ESP.getChipRevision());

  // IO13 mengalami short ke GND saat penyolderan -> WAJIB dikunci sebagai INPUT!
  pinMode(PIN_SHORTED, INPUT);

  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  pinMode(PIN_PIR, INPUT);            // IO34: input-only, tidak punya pull-up internal
  attachInterrupt(digitalPinToInterrupt(PIN_PIR), pirISR, RISING);

  // Inisialisasi PWM 5 kHz, 10-bit pada LED Internal (IO32) & MOSFET LED HPL (IO18)
  ledcAttach(PIN_LED_INT, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWM_HPL, PWM_FREQ, PWM_RES);
  applySyncedPWM(0);                  // Mulai dalam kondisi 0% (OFF) yang selaras

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_NTC5, ADC_11db);

  // Inisialisasi LoRa E220
  pinMode(PIN_LORA_M0, OUTPUT);
  pinMode(PIN_LORA_M1, OUTPUT);
  pinMode(PIN_LORA_AUX, INPUT_PULLUP);
  LoRa.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
  loraSetMode(0, 0);
  loraWriteConfig(0x03);

  // Inisialisasi I2C & deteksi semua sensor (termasuk INA219 BATT)
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  scanI2C();
  printHelp();

  beep(2);                            // Tanda proses boot selesai
  startToggleMode();                  // Mulai dengan mode toggle (ketik 0..100 untuk manual)
  lastLoRa = millis();
}

void loop() {
  handleSerial();
  loraPoll();

  // Eksekusi Mode Kontrol Dimming (Selalu menyelaraskan IO18 & IO32 via applySyncedPWM)
  if (currentDimMode == MODE_TOGGLE) {
    if (millis() - lastToggle >= TOGGLE_PERIOD_MS) {
      lastToggle = millis();
      setGateToggleState(!gateOn);
    }
  } else if (currentDimMode == MODE_SWEEP) {
    if (millis() - lastSweep >= SWEEP_STEP_MS) {
      lastSweep = millis();
      int nextPct = hplPercent + sweepDir;
      if (nextPct >= 100) { nextPct = 100; sweepDir = -1; }
      if (nextPct <= 0)   { nextPct = 0;   sweepDir = 1;  }
      applySyncedPWM(nextPct);
    }
  } else if (currentDimMode == MODE_ADAPTIVE) {
    const int targets[4]      = {20, 100, 50, 0};
    const uint32_t holds[4]   = {4000, 5000, 4000, 3000};
    const char *stageNames[4] = {
      "Standby Malam (20%)",
      "Gerakan Terdeteksi - Full Bright (100%)",
      "Cooldown / Redup Bertahap (50%)",
      "Siang / Mati (0%)"
    };
    if (millis() - lastAdaptState >= holds[adaptStage]) {
      lastAdaptState = millis();
      adaptStage = (adaptStage + 1) % 4;
      adaptTarget = targets[adaptStage];
      Serial.printf(">> [ADAPTIVE SIM] Fase %d: %s\n", adaptStage + 1, stageNames[adaptStage]);
    }
    if (millis() - lastSweep >= 25) {
      lastSweep = millis();
      if (hplPercent < adaptTarget)      applySyncedPWM(hplPercent + 1);
      else if (hplPercent > adaptTarget) applySyncedPWM(hplPercent - 1);
    }
  }

  // Pembacaan periodik seluruh sensor & tampilkan ke Serial Monitor
  if (millis() - lastRead >= READ_INTERVAL_MS) {
    lastRead = millis();
    readAll();
    printAll();
  }

  // Pengiriman telemetri LoRa periodik
  if (millis() - lastLoRa >= LORA_INTERVAL_MS) {
    lastLoRa = millis();
    loraSend();
  }
}