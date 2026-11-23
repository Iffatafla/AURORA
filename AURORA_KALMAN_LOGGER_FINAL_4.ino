/*
  AURORA - PCB TX: Kalman Filter Anomaly Detector (L1-L4) + Pure CSV Logger + LoRa E220
  =====================================================================================
  Board   : ESP32-WROOM-32D / 32E -> "ESP32 Dev Module"
  Core    : esp32 by Espressif 3.x (diuji untuk 3.3.11)
  Serial  : 115200 baud, line ending "Newline"
  Library : TIDAK perlu library tambahan

  Fitur Tambahan pada Versi Ini:
    1. Arsitektur Deteksi Anomali 4-Lapis (L1 - L4) On-Device berbasis Kalman Filter + NIS:
       - [L1] Validitas Data   : Deteksi I2C gagal, NaN, stuck value, & luar rentang fisik.
       - [L2] Batas Keselamatan: Deteksi cepat (1-3 sampel) Overvoltage, Low Batt, Overheat.
       - [L3] Kalman + NIS     : Estimasi state & Normalized Innovation Squared per subsistem.
       - [L4] Klasifikasi      : Keputusan 15 Anomaly Code berdasarkan konteks (duty, siang/malam, float).
    2. Mode Cetak CSV Murni (Pure CSV Logging Mode):
       - Aktif otomatis untuk kemudahan logging berjam-jam (Panel Surya + Baterai + Beban).
       - Ketika mode CSV murni aktif, pesan debug LoRa/PWM disembunyikan agar file log bersih 100%.
       - Ketik 'v' di Serial Monitor untuk berpindah antara Mode CSV Murni <-> Mode Tabel Telemetri.
       - Ketik 'header' untuk mencetak ulang baris judul kolom CSV.
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
const uint8_t ADDR_INA_MAIN_DEFAULT = 0x44;
const uint8_t ADDR_INA_PV_DEFAULT   = 0x40;
const uint8_t ADDR_INA_BATT_DEFAULT = 0x41;

int addrInaMain = -1, addrInaPV = -1, addrInaBatt = -1;
int addrADS = -1, addrLM75 = -1;
// Pemetaan BH1750 sesuai hardware:
// - Index 0 (lux_amb  / Lingkungan)   : ADDR = VCC -> 0x5C
// - Index 1 (lux_lamp / Lampu LED HPL): ADDR = GND -> 0x23
const uint8_t ADDR_BH1750[2] = {0x5C, 0x23};
const uint8_t BH_AMB_MTREG   = 31;                                  // sensor lingkungan (lihat identifyDevices)
const float   LUX_AMB_MAX    = 65535.0f / 1.2f * 69.0f / BH_AMB_MTREG; // ~121.557 lux (batas jenuh)
bool hasBH[2] = {false, false};

// ================= Parameter Hardware =================
const float SHUNT_MAIN_OHM = 0.020f;   // R27 (Main SCC) = 20 mOhm
const float SHUNT_PV_OHM   = 0.020f;   // R18 (PV)       = 20 mOhm
const float SHUNT_BATT_OHM = 0.020f;   // R24 (BATT)     = 20 mOhm
const float VCC_NTC        = 3.30f;
const float R_PULLUP       = 10000.0f;
const float NTC_R25        = 10000.0f;
const float NTC_BETA       = 3950.0f;

// ================= PWM / Timing =================
const int PWM_FREQ = 5000;
const int PWM_RES  = 10;               // Resolusi 10-bit (0..1023)
const int PWM_MAX  = (1 << PWM_RES) - 1;

const uint32_t READ_INTERVAL_MS   = 1000;
const uint32_t TOGGLE_PERIOD_MS   = 3000;
const uint32_t SWEEP_STEP_MS      = 50;
const uint32_t LORA_INTERVAL_MS   = 5000;
const uint32_t ACK_TIMEOUT_MS     = 3000;

// Langkah sesi logging otomatis (sesiD, sesiE, ...). Didefinisikan DI SINI (atas file) karena
// Arduino IDE membuat prototipe fungsi otomatis di bagian atas; tipe yang dipakai sebagai parameter
// fungsi harus sudah dikenal sebelum prototipe itu.
struct SeqStep { int16_t duty; uint16_t sec; int8_t mark; const char *say; };

// ================= Mode & State Kontrol Dimming =================
enum DimMode {
  MODE_MANUAL_PWM   = 0, // Manual 0..100% (dengan Soft-Ramp)
  MODE_TOGGLE       = 1, // Toggle 0% <-> 100% tiap 3 detik
  MODE_SWEEP        = 2, // Sweep 0% <-> 100%
  MODE_ADAPTIVE_SIM = 3, // Simulasi Adaptive Timer (10% -> 40% -> 20% -> 0%)
  MODE_ADAPTIVE_LUX = 4, // Adaptive TANPA PIR (murni berdasarkan BH1750 Ambient lux_amb)
  MODE_ADAPTIVE_PIR = 5  // Adaptive DENGAN PIR (berdasarkan BH1750 Ambient + gerakan PIR)
};

DimMode  currentDimMode = MODE_MANUAL_PWM;
int      hplPercent     = 0;               // Nilai PWM aktual saat ini (0..100 %)
int      targetPercent  = 0;               // Target PWM yang dituju (Soft-Ramp)
uint32_t currentPwmRaw  = 0;
bool     gateOn         = false;
int      sweepDir       = 1;
int      adaptStage     = 0;
int      adaptTarget    = 20;

// Variabel state untuk Adaptive Dimming dengan PIR (MODE_ADAPTIVE_PIR)
uint32_t lastPirMotionMs = 0;
uint32_t lastSeenPirCount = 0;
bool     pirMotionActive = false;
// Batas duty maksimum demi keamanan lampu/driver (CC driver U8 tidak membatasi arus).
// SEMUA mode & sesi dibatasi ke nilai ini. Catatan: PWM hanya menurunkan daya RATA-RATA;
// arus puncak tetap dibatasi oleh setelan CV U8.
const int DUTY_MAX = 40;   // uji 1 Nov: >40% lampu mati (proteksi trip), 40% ~27 W P_main
// Duty standby (mode PIR, tanpa gerakan, malam).
const int standbyDuty = 10; // ~7 W P_main (uji 1 Nov)
const uint32_t PIR_HOLD_FULL_MS = 8000;    // Tahan 100% selama 8 detik sejak gerakan terakhir
const uint32_t PIR_COOLDOWN_MS  = 4000;    // Turun ke 20% selama 4 detik sebelum kembali Standby

// Kecepatan Soft-Ramp untuk mencegah inrush current / error saat lompat jauh (misal 0 -> 100)
// Naik/turun 2% setiap 40 ms (~2 detik dari 0% ke 100%, setara naik per 5% tiap 100 ms)
const uint32_t RAMP_INTERVAL_MS = 80;   // 2% tiap 80 ms: 25->100% ~3 s, 0->100% ~4 s. BERLAKU UNTUK SEMUA SESI & MODE
const int      RAMP_STEP_PCT    = 2;
uint32_t lastRampStep = 0;

// Mode output Serial: true = Hanya baris CSV murni (cocok untuk logging), false = Tabel telemetri
bool csvOnlyMode = true;

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
  int   fault = 0;                                          // Kode anomali UTAMA (prioritas tertinggi, 0..14)
  const char *label = "NONE";                               // Label kode utama
  uint16_t faultMask = 0;                                   // SEMUA anomali aktif: bit k = kode k (0 = normal)
} data;

// LoRa statistik
HardwareSerial &LoRa = Serial2;
uint32_t txSeq = 0, txCount = 0, ackCount = 0;
uint32_t waitingAckSeq = 0, txTime = 0;
bool waitingAck = false;
String loraRxBuf;
uint32_t consecutiveLoraTimeout = 0;
bool     loraTxFlag = false;   // 1 pada baris CSV berikutnya setelah LoRa memancar (untuk pipeline Python)

// =====================================================================
//        ARSITEKTUR DETEKSI ANOMALI 4-LAPIS (L1 - L4) ON-DEVICE
// =====================================================================

enum AnomalyCode {
  NONE                       = 0,  // 15. Normal (tidak ada anomali)
  PV_LOW_GENERATION          = 1,  // 1.  Cahaya terang tapi daya/arus PV rendah
  PV_SENSOR_FAULT            = 2,  // 2.  INA219 PV gagal baca / NaN / stuck / luar rentang fisik
  PV_ABNORMAL_VOLTAGE        = 3,  // 3.  Inovasi/NIS tegangan PV melonjak/anjlok tidak wajar
  BATTERY_LOW                = 4,  // 4.  Tegangan baterai di bawah batas aman (< V_BAT_LOW)
  BATTERY_ABNORMAL           = 5,  // 5.  NIS tegangan/arus baterai tinggi (drop/spike mendadak)
  BATTERY_OVERVOLTAGE        = 6,  // 6.  Tegangan baterai melebihi batas aman (> V_BAT_MAX)
  LAMP_FAULT                 = 7,  // 7.  Duty > 0 & daya masuk normal, tapi lux_lamp gelap / lampu mati
  LOAD_FAULT                 = 8,  // 8.  Duty > 0 tapi daya/arus beban (P_main) tidak sesuai model
  LOAD_LEAKAGE               = 9,  // 9.  Duty == 0 (mati) tapi masih ada arus/daya beban mengalir
  LIGHT_SENSOR_FAULT         = 10, // 10. Sensor BH1750 (amb/lamp) gagal baca / NaN / stuck
  THERMAL_ABNORMAL           = 11, // 11. Kenaikan suhu tidak wajar (NIS tinggi)
  THERMAL_OVERLIMIT          = 12, // 12. Suhu melewati batas keselamatan (> T_MAX_LIMIT)
  COMMUNICATION_FAULT        = 13, // 13. LoRa gagal transmisi / ACK timeout beruntun
  COMMUNICATION_DATA_INVALID = 14  // 14. Data sensor utama korup / gagal validasi paket
};

const char* getAnomalyLabel(AnomalyCode code) {
  switch (code) {
    case PV_LOW_GENERATION:          return "PV_LOW_GENERATION";
    case PV_SENSOR_FAULT:            return "PV_SENSOR_FAULT";
    case PV_ABNORMAL_VOLTAGE:        return "PV_ABNORMAL_VOLTAGE";
    case BATTERY_LOW:                return "BATTERY_LOW";
    case BATTERY_ABNORMAL:           return "BATTERY_ABNORMAL";
    case BATTERY_OVERVOLTAGE:        return "BATTERY_OVERVOLTAGE";
    case LAMP_FAULT:                 return "LAMP_FAULT";
    case LOAD_FAULT:                 return "LOAD_FAULT";
    case LOAD_LEAKAGE:               return "LOAD_LEAKAGE";
    case LIGHT_SENSOR_FAULT:         return "LIGHT_SENSOR_FAULT";
    case THERMAL_ABNORMAL:           return "THERMAL_ABNORMAL";
    case THERMAL_OVERLIMIT:          return "THERMAL_OVERLIMIT";
    case COMMUNICATION_FAULT:        return "COMMUNICATION_FAULT";
    case COMMUNICATION_DATA_INVALID: return "COMMUNICATION_DATA_INVALID";
    default:                         return "NONE";
  }
}

// ---------------- [L3] Struktur Kalman Filter 1D + NIS ----------------
struct KalmanNIS {
  float x;          // Estimasi state (x_hat)
  float P;          // Error covariance
  float Q;          // Process noise variance
  float R;          // Measurement noise variance
  float nu;         // Innovation / residual: nu = z - x_pred (bertanda + / -)
  float S;          // Innovation covariance: S = P_pred + R
  float nis;        // Normalized Innovation Squared: NIS = (nu * nu) / S
  bool  initialized;

  float update(float z, float u_expected = NAN) {
    if (!initialized) {
      x = isnan(u_expected) ? z : u_expected;
      P = R;
      nu = 0.0f; S = R; nis = 0.0f;
      initialized = true;
      return x;
    }

    // 1. Predict
    float x_pred = isnan(u_expected) ? x : (0.7f * x + 0.3f * u_expected);
    float P_pred = P + Q;

    // 2. Innovation & NIS
    nu  = z - x_pred;
    S   = P_pred + R;
    nis = (nu * nu) / S;

    // 3. Update (Soft-gating jika terjadi spike ekstrem agar state tidak rusak)
    float K = P_pred / S;
    if (nis > 25.0f) K *= 0.2f;
    x = x_pred + K * nu;
    P = (1.0f - K) * P_pred;
    return x;
  }

  void resetTo(float z) {
    if (!isnan(z)) {
      x = z; P = R; nu = 0.0f; S = R; nis = 0.0f; initialized = true;
    }
  }
};

// ---------------- [L1] Detektor Sensor Stuck (Nilai Membeku) ----------------
struct StuckDetector {
  float lastVal = NAN;
  int   sameCount = 0;

  bool checkStuck(float val, float eps, int maxRepeat) {
    if (isnan(val)) return false;
    if (!isnan(lastVal) && fabs(val - lastVal) <= eps) {
      sameCount++;
    } else {
      sameCount = 0;
      lastVal = val;
    }
    return (sameCount >= maxRepeat);
  }
};

// Instansiasi Kalman Filter per Subsistem (SUDAH DIKALIBRASI dari 740 sampel data Baseline PCB AURORA)
// Format: {x, P, Q (process noise), R (measurement noise), nu, S, nis, initialized}
KalmanNIS kfPvV      = {0, 1, 0.005f, 0.020f, 0, 1, 0, false}; // V_pv
KalmanNIS kfPvP      = {0, 1, 0.050f, 0.200f, 0, 1, 0, false}; // P_pv
KalmanNIS kfBattV    = {0, 1, 0.0002f,0.002f, 0, 1, 0, false}; // V_bat (sigma_noise ~ 0.005V)
KalmanNIS kfBattI    = {0, 1, 0.001f, 0.005f, 0, 1, 0, false}; // I_bat (termasuk pulsa LoRa TX +38mA tiap 5s)
KalmanNIS kfMainP    = {0, 1, 0.050f, 0.250f, 0, 1, 0, false}; // P_main (idle 1.47W, pulsa LoRa 1.93W)
KalmanNIS kfLuxLamp  = {0, 1, 1.000f, 5.000f, 0, 1, 0, false}; // Lux lampu BH1750 (0x23)
KalmanNIS kfTempBat  = {0, 1, 0.002f, 0.020f, 0, 1, 0, false}; // NTC #1 Baterai (sigma_step ~ 0.04 C)
KalmanNIS kfTempNtc3 = {0, 1, 0.002f, 0.020f, 0, 1, 0, false}; // NTC #3 Heatsink (sigma_step ~ 0.04 C)
KalmanNIS kfTempLm75 = {0, 1, 0.003f, 0.030f, 0, 1, 0, false}; // LM75B Board (resolusi 0.125 C)

StuckDetector stuckPvV, stuckBattV, stuckLuxAmb;

// ---------------- [L2] Safety Limits & Hysteresis (LFP 4S2P 12Ah & Panel 50W) ----------------
constexpr float VBAT_OV_ON   = 14.8f, VBAT_OV_OFF  = 14.6f;
constexpr float VBAT_LOW_ON  = 12.6f, VBAT_LOW_OFF = 13.0f; // LFP 4S: ~<10% SOC. JANGAN diturunkan untuk membungkam alarm
constexpr float TBAT_MAX_ON  = 45.0f, TBAT_MAX_OFF = 42.0f;
constexpr float TCTRL_MAX_ON = 70.0f, TCTRL_MAX_OFF = 65.0f;
constexpr float VPV_MAX_ON   = 22.9f, VPV_MAX_OFF  = 22.4f;
constexpr float IPV_MAX      = 3.7f;
constexpr int   N_PERSIST    = 3;     // Persistensi tegangan & PV = 3 sampel; suhu = 1 sampel

// Konfigurasi Sensor Suhu yang Dipakai: NTC #1 (indeks 0 = Baterai) & NTC #3 (indeks 2 = Heatsink/SCC)
constexpr int   NTC_BATT_INDEX = 0;   // NTC #1 -> TBAT (45 C)
constexpr int   NTC_CTRL_INDEX = 2;   // NTC #3 -> TCTRL (70 C)

// Counter persistensi & status latch histeresis untuk [L2]
int  cntBattOV = 0, cntBattLow = 0, cntPvOver = 0;
bool latchBattOV = false, latchBattLow = false, latchPvOver = false;
bool latchTbatOver = false, latchTctrlOver = false;

int   rampSettlingSamples = 0; // Masa tenang 2 detik selama/setelah transisi dimming
float prevDutyKF = -1.0f;

// Parameter [L3] & [L4] (Dikalibrasi dari data Baseline CSV & Spesifikasi Lampu 30W)
const float NIS_THRESHOLD    = 9.0f;   // Ambang NIS = 9.0 (setara 3 sigma / Z = 3.0)
const float P_MAIN_IDLE_W    = 1.55f;  // Daya idle board (ESP32+Sensor+LoRa) saat Duty = 0% (~1.47W - 1.93W)
const float NOMINAL_LAMP_W   = 60.0f;  // Daya lampu ekstrapolasi ke duty 100% dari uji 1 Nov (10%=7 W, 40%=27 W P_main).
                                        // Hanya prior kasar; diganti parameter hasil identifikasi Kalman
const float NOMINAL_LAMP_LUX = 150.0f; // Ekspektasi lux lampu pada Duty 100% (Lux)

// =====================================================================
//      SIMULASI FAULT INJECTION VIA SERIAL (TANPA MERUSAK HARDWARE)
// =====================================================================
int      injectedFault     = 0;     // 0 = Normal (Sensor Asli), 1..14 = Injeksi Anomali 1..14
uint32_t faultStepCount    = 0;     // Counter sampel sejak fault diaktifkan
bool     autoFaultMode     = false; // Mode urutan otomatis f1 -> f14
uint32_t lastAutoFaultMs   = 0;
int      autoFaultStage    = 0;     // Genap = Normal (8 detik), Ganjil = Fault (10 detik)

void clearAnomalyLatchesAndResetKF() {
  cntBattOV = cntBattLow = cntPvOver = 0;
  latchBattOV = latchBattLow = latchPvOver = false;
  latchTbatOver = latchTctrlOver = false;
  consecutiveLoraTimeout = 0;
  // Izinkan Kalman Filter menyesuaikan kembali ke nilai sensor asli pada pembacaan berikutnya
  kfPvV.initialized      = false;
  kfPvP.initialized      = false;
  kfBattV.initialized    = false;
  kfBattI.initialized    = false;
  kfMainP.initialized    = false;
  kfLuxLamp.initialized  = false;
  kfTempBat.initialized  = false;
  kfTempNtc3.initialized = false;
  kfTempLm75.initialized = false;
}

void setFaultInjection(int code, bool fromAuto = false) {
  if (!fromAuto) autoFaultMode = false;
  injectedFault  = constrain(code, 0, 14);
  faultStepCount = 0;
  if (injectedFault == 0) {
    clearAnomalyLatchesAndResetKF();
    if (!csvOnlyMode) Serial.println(F(">> [FAULT INJECTION OFF] Kembali ke pembacaan sensor murni (NONE)"));
  } else {
    clearAnomalyLatchesAndResetKF();
    if (!csvOnlyMode) {
      Serial.printf(">> [FAULT INJECTION AKTIF] Kode #%d -> %s\n",
                    injectedFault, getAnomalyLabel((AnomalyCode)injectedFault));
    }
  }
}

void startAutoFaultTest() {
  autoFaultMode   = true;
  autoFaultStage  = 1; // Langsung mulai dari Fault #1 (PV_LOW_GENERATION)
  lastAutoFaultMs = millis();
  setFaultInjection(1, true);
  if (!csvOnlyMode) {
    Serial.println(F(">> [AUTO FAULT TEST AKTIF] Menguji kode f1 s/d f14 bergantian otomatis (10s Fault / 6s Normal)"));
  }
}

// Memodifikasi data sensor di memori sesaat sebelum masuk ke Lapisan L1-L4
void applyFaultInjection() {
  if (autoFaultMode) {
    uint32_t holdMs = (injectedFault > 0) ? 10000 : 6000; // 10 detik Fault, 6 detik Normal (jeda pemulihan)
    if (millis() - lastAutoFaultMs >= holdMs) {
      lastAutoFaultMs = millis();
      autoFaultStage++;
      if (autoFaultStage > 28) {
        // Selesai seluruh 14 anomali -> kembali ke Normal permanen
        autoFaultMode = false;
        setFaultInjection(0, true);
      } else if (autoFaultStage % 2 == 0) {
        setFaultInjection(0, true); // Jeda Normal di antara fault
      } else {
        int nextFault = (autoFaultStage + 1) / 2;
        setFaultInjection(nextFault, true);
      }
    }
  }

  if (injectedFault == 0) return;
  faultStepCount++;

  switch (injectedFault) {
    case PV_LOW_GENERATION: // f1: Siang terik (lux_amb tinggi), baterai belum penuh, tapi daya/tegangan PV anjlok
      data.bhOk[0] = true; data.lux[0] = 2500.0f;
      data.pvOk    = true; data.pvV    = 3.20f; data.pvA = 0.010f; data.pvW = 0.03f;
      kfPvV.resetTo(3.20f); // Jaga agar NIS PV tenang sehingga masuk ke PV_LOW_GENERATION (bukan abnormal voltage)
      break;

    case PV_SENSOR_FAULT: // f2: Sensor INA219 PV lepas / NaN (Ditangkap oleh [L1])
      data.pvOk = false; data.pvV = NAN; data.pvA = NAN; data.pvW = NAN;
      break;

    case PV_ABNORMAL_VOLTAGE: // f3: Siang hari, tegangan PV berfluktuasi tajam -> NIS PV > 9.0 (Ditangkap [L3]+[L4])
      data.bhOk[0] = true; data.lux[0] = 2200.0f;
      data.pvOk    = true;
      if (faultStepCount == 1) kfPvV.resetTo(14.0f);
      data.pvV = (faultStepCount % 2 == 1) ? 18.5f : 11.5f;
      data.pvA = 0.85f;
      data.pvW = data.pvV * data.pvA;
      break;

    case BATTERY_LOW: // f4: Tegangan baterai drop ke 10.65V (< 11.0V) -> Ditangkap [L2] setelah 3 sampel
      data.battOk = true;
      data.battV  = 10.65f;
      break;

    case BATTERY_ABNORMAL: // f5: Tegangan baterai mengalami spike/sag mendadak di zona aman -> NIS Bat > 9.0 ([L3]+[L4])
      data.battOk = true;
      if (faultStepCount == 1) kfBattV.resetTo(12.20f);
      data.battV = (faultStepCount % 2 == 1) ? 12.85f : 11.65f;
      break;

    case BATTERY_OVERVOLTAGE: // f6: Tegangan baterai melonjak ke 15.05V (> 14.8V) -> Ditangkap [L2] setelah 3 sampel
      data.battOk = true;
      data.battV  = 15.05f;
      break;

    case LAMP_FAULT: { // f7: Lampu diberi Duty >= 50%, daya listrik normal, TAPI lux_lamp gelap (LED putus/tertutup)
      if (data.duty < (float)DUTY_MAX) { data.duty = (float)DUTY_MAX; prevDutyKF = (float)DUTY_MAX; rampSettlingSamples = 0; }
      float expW   = P_MAIN_IDLE_W + (data.duty / 100.0f) * NOMINAL_LAMP_W;
      data.mainOk  = true; data.mainV = 12.0f; data.mainW = expW; data.mainA = expW / 12.0f;
      kfMainP.resetTo(expW);
      data.bhOk[1] = true;
      if (faultStepCount == 1) kfLuxLamp.resetTo(80.0f);
      data.lux[1]  = 0.5f; // Gelap padahal daya mengalir normal
      break;
    }

    case LOAD_FAULT: { // f8: Lampu diberi Duty >= 50%, tapi beban terbuka / arus putus (P_main drop ke idle)
      if (data.duty < (float)DUTY_MAX) { data.duty = (float)DUTY_MAX; prevDutyKF = (float)DUTY_MAX; rampSettlingSamples = 0; }
      float expW  = P_MAIN_IDLE_W + (data.duty / 100.0f) * NOMINAL_LAMP_W;
      if (faultStepCount == 1) kfMainP.resetTo(expW);
      data.mainOk = true; data.mainV = 12.0f; data.mainA = 0.123f; data.mainW = 1.47f;
      break;
    }

    case LOAD_LEAKAGE: // f9: Lampu mati (Duty 0%), tapi ada arus bocor besar (0.48A / 5.7W)
      data.duty   = 0.0f; prevDutyKF = 0.0f; rampSettlingSamples = 0;
      data.mainOk = true; data.mainV = 11.9f; data.mainA = 0.480f; data.mainW = 5.71f;
      kfMainP.resetTo(5.71f);
      break;

    case LIGHT_SENSOR_FAULT: // f10: Sensor BH1750 gagal dibaca / NaN (Ditangkap oleh [L1])
      data.bhOk[0] = false; data.lux[0] = NAN;
      break;

    case THERMAL_ABNORMAL: // f11: Suhu NTC1 naik mendadak (+2.5 C/sampel) tapi masih di bawah 45 C -> NIS Temp > 9.0
      if (faultStepCount == 1) kfTempBat.resetTo(28.5f);
      kfTempBat.x    = 28.5f; // Tahan referensi prediktor agar lonjakan terus menghasilkan inovasi positif besar
      data.ntcT[0]   = 33.5f + (faultStepCount % 4) * 1.2f; // 33.5 C .. 37.1 C (< 45 C)
      break;

    case THERMAL_OVERLIMIT: // f12: Suhu NTC1 melewati batas keselamatan TBAT_MAX_ON (47.5 C > 45.0 C) -> Ditangkap [L2]
      data.ntcT[0] = 47.5f;
      break;

    case COMMUNICATION_FAULT: // f13: Modul LoRa gagal memancarkan paket >= 3x berturut-turut
      consecutiveLoraTimeout = 4;
      break;

    case COMMUNICATION_DATA_INVALID: // f14: Kegagalan serentak bus sensor utama (Ditangkap [L1])
      data.battOk = false; data.mainOk = false; data.pvOk = false;
      data.battV  = NAN;   data.mainV  = NAN;   data.pvV  = NAN;
      break;
  }
}

void evaluateAnomalyLayers() {
  // Deteksi apakah PWM sedang proses transisi (Soft-Ramp) atau baru saja berubah
  bool isRamping   = (hplPercent != targetPercent);
  bool dutyChanged = (prevDutyKF >= 0.0f && fabs(data.duty - prevDutyKF) >= 1.0f) || isRamping;
  prevDutyKF = data.duty;

  if (dutyChanged) {
    rampSettlingSamples = 2; // Beri waktu 2 sampel (2 detik) setelah PWM stabil
    if (data.mainOk)  kfMainP.resetTo(data.mainW);
    if (data.bhOk[1]) kfLuxLamp.resetTo(data.lux[1]);
  } else if (rampSettlingSamples > 0) {
    rampSettlingSamples--;
    if (data.mainOk)  kfMainP.resetTo(data.mainW);
    if (data.bhOk[1]) kfLuxLamp.resetTo(data.lux[1]);
  }

  // Ambil suhu hanya dari 3 sensor yang terpasang: NTC #1 (Baterai), NTC #3 (Heatsink/Ctrl), dan LM75B (Board)
  float tBat  = (!isnan(data.ntcT[NTC_BATT_INDEX]) && data.ntcT[NTC_BATT_INDEX] > -20.0f && data.ntcT[NTC_BATT_INDEX] < 120.0f)
                ? data.ntcT[NTC_BATT_INDEX] : NAN;
  float tNtc3 = (!isnan(data.ntcT[NTC_CTRL_INDEX]) && data.ntcT[NTC_CTRL_INDEX] > -20.0f && data.ntcT[NTC_CTRL_INDEX] < 120.0f)
                ? data.ntcT[NTC_CTRL_INDEX] : NAN;
  float tLm75 = (data.lmOk && !isnan(data.tBoard) && data.tBoard > -20.0f && data.tBoard < 120.0f)
                ? data.tBoard : NAN;

  // tCtrl adalah nilai tertinggi antara LM75B dan NTC #3
  float tCtrl = tLm75;
  if (!isnan(tNtc3) && (isnan(tCtrl) || tNtc3 > tCtrl)) tCtrl = tNtc3;

  float maxTemp = tCtrl;
  if (!isnan(tBat) && (isnan(maxTemp) || tBat > maxTemp)) maxTemp = tBat;

  // ===================================================================
  // [L1] LAPISAN 1: VALIDITAS DATA (Gagal Baca, NaN, Stuck, Rentang Fisik)
  // ===================================================================
  bool pvValid    = data.pvOk   && !isnan(data.pvV)   && (data.pvV >= -0.5f  && data.pvV <= 32.0f);
  bool battValid  = data.battOk && !isnan(data.battV) && (data.battV >= 3.0f && data.battV <= 25.0f);
  bool mainValid  = data.mainOk && !isnan(data.mainV) && (data.mainW >= -1.0f && data.mainW <= 150.0f);
  bool ambValid   = data.bhOk[0] && !isnan(data.lux[0]) && (data.lux[0] >= 0.0f && data.lux[0] <= LUX_AMB_MAX + 1.0f);
  bool lampValid  = data.bhOk[1] && !isnan(data.lux[1]) && (data.lux[1] >= 0.0f && data.lux[1] <= 100000.0f);
  bool tempValid  = !isnan(maxTemp) && (maxTemp >= -10.0f && maxTemp <= 110.0f);

  // Deteksi stuck (khusus saat nilai tidak nol tapi membeku persis selama 120 sampel)
  if (pvValid && data.pvV > 1.0f && stuckPvV.checkStuck(data.pvV, 0.0001f, 120))       pvValid = false;
  // nilai jenuh (matahari sangat terik) BUKAN sensor rusak -> tidak dihitung sebagai stuck
  if (ambValid && data.lux[0] > 5.0f && data.lux[0] < 0.99f * LUX_AMB_MAX
      && stuckLuxAmb.checkStuck(data.lux[0], 0.001f, 180)) ambValid = false;

  // Hasil L1 DISIMPAN dulu, tidak langsung return: sensor yang rusak tidak boleh
  // menutupi alarm keselamatan L2 dari sensor lain yang masih valid (mis. BH1750 lepas
  // tidak boleh membuat THERMAL_OVERLIMIT hilang).

  // ===================================================================
  // [L3] UPDATE KALMAN FILTER + NIS - hanya dengan data yang VALID
  //      (data rusak tidak boleh masuk ke filter)
  // ===================================================================
  float expMainP   = P_MAIN_IDLE_W + (data.duty / 100.0f) * NOMINAL_LAMP_W;
  float expLuxLamp = (data.duty / 100.0f) * NOMINAL_LAMP_LUX;

  if (pvValid)   { kfPvV.update(data.pvV); kfPvP.update(data.pvW); }
  if (battValid) { kfBattV.update(data.battV); kfBattI.update(data.battA); }
  if (mainValid) kfMainP.update(data.mainW, expMainP);
  if (lampValid) kfLuxLamp.update(data.lux[1], expLuxLamp);
  if (!isnan(tBat))  kfTempBat.update(tBat);   // NTC #1 (Baterai)
  if (!isnan(tNtc3)) kfTempNtc3.update(tNtc3); // NTC #3 (Heatsink/Controller)
  if (!isnan(tLm75)) kfTempLm75.update(tLm75); // LM75B (Board PCB)

  // ===================================================================
  // [L2] LAPISAN 2: BATAS KESELAMATAN + HISTERESIS (Prioritas Tertinggi)
  //      - Suhu (TBAT & TCTRL) : Persistensi = 1 sampel
  //      - Tegangan & PV       : Persistensi = N_PERSIST (3 sampel)
  // ===================================================================

  // 1. Evaluasi Suhu Baterai (1 sampel untuk ON, histeresis turun di bawah TBAT_MAX_OFF)
  if (!isnan(tBat)) {
    if (tBat >= TBAT_MAX_ON)       latchTbatOver = true;
    else if (tBat <= TBAT_MAX_OFF) latchTbatOver = false;
  }

  // 2. Evaluasi Suhu Controller/Board (1 sampel untuk ON, histeresis turun di bawah TCTRL_MAX_OFF)
  if (!isnan(tCtrl)) {
    if (tCtrl >= TCTRL_MAX_ON)       latchTctrlOver = true;
    else if (tCtrl <= TCTRL_MAX_OFF) latchTctrlOver = false;
  }

  // 3. Evaluasi Battery Overvoltage (N_PERSIST = 3 sampel untuk ON, histeresis VBAT_OV_OFF)
  //    Tegangan/PV hanya dievaluasi bila sensornya valid (NaN tidak boleh dianggap aman/bahaya)
  if (!battValid) {
    cntBattOV = cntBattLow = 0;          // status latch dipertahankan sampai data valid lagi
  } else if (data.battV >= VBAT_OV_ON) {
    if (++cntBattOV >= N_PERSIST) latchBattOV = true;
  } else {
    cntBattOV = 0;
    if (data.battV <= VBAT_OV_OFF) latchBattOV = false;
  }

  // 4. Evaluasi Battery Low Voltage (N_PERSIST = 3 sampel untuk ON, histeresis VBAT_LOW_OFF)
  if (!battValid) {
    // dilewati (lihat di atas)
  } else if (data.battV <= VBAT_LOW_ON) {
    if (++cntBattLow >= N_PERSIST) latchBattLow = true;
  } else {
    cntBattLow = 0;
    if (data.battV >= VBAT_LOW_OFF) latchBattLow = false;
  }

  // 5. Evaluasi PV Overvoltage / Overcurrent (N_PERSIST = 3 sampel untuk ON, histeresis VPV_MAX_OFF)
  if (!pvValid) {
    cntPvOver = 0;
  } else if (data.pvV >= VPV_MAX_ON || data.pvA >= IPV_MAX) {
    if (++cntPvOver >= N_PERSIST) latchPvOver = true;
  } else {
    cntPvOver = 0;
    if (data.pvV <= VPV_MAX_OFF && data.pvA < IPV_MAX) latchPvOver = false;
  }

  // ===================================================================
  // KELUARAN MULTI-ANOMALI
  //   - SEMUA anomali yang aktif dicatat di data.faultMask (bit k = kode k)
  //   - data.fault / data.label = kode UTAMA, yaitu yang pertama di-raise
  //     sesuai urutan prioritas L2 > L1 > L3/L4 (urutan pemanggilan di bawah)
  //   - Cek L3/L4 hanya memakai sensor yang VALID, jadi sensor rusak tidak
  //     menghasilkan alarm turunan palsu, tapi juga tidak menutupi alarm lain.
  // ===================================================================
  data.fault = NONE;
  data.label = getAnomalyLabel(NONE);
  data.faultMask = 0;
  auto raise = [](AnomalyCode c) {
    data.faultMask |= (uint16_t)(1u << (int)c);
    if (data.fault == NONE) { data.fault = c; data.label = getAnomalyLabel(c); }
  };

  // [L2] keselamatan
  if (latchTbatOver || latchTctrlOver) raise(THERMAL_OVERLIMIT);
  if (latchBattOV)                     raise(BATTERY_OVERVOLTAGE);
  if (latchBattLow)                    raise(BATTERY_LOW);
  if (latchPvOver)                     raise(PV_ABNORMAL_VOLTAGE);

  // [L1] validitas (per sensor, boleh lebih dari satu)
  if (!battValid && !mainValid && !pvValid) raise(COMMUNICATION_DATA_INVALID);
  if (!pvValid)                             raise(PV_SENSOR_FAULT);
  if (!ambValid || !lampValid)              raise(LIGHT_SENSOR_FAULT);
  if (!battValid || !mainValid || !tempValid) raise(COMMUNICATION_DATA_INVALID);

  // ===================================================================
  // [L4] LAPISAN 4: KLASIFIKASI KONTEKSTUAL (Gabungan L3 + Konteks Operasi)
  // ===================================================================
  bool isDaytime   = ambValid && (data.lux[0] > 1000.0f); // Siang hari (butuh lux_amb valid)
  // Histeresis SCC Keeper1220: arus PV ditahan saat V_bat ~14.x V, pulih saat V_bat < 13.0 V.
  // Latch ini harus sama dengan scc_hold di convert_aurora.py.
  static int8_t sccHold = -1;                 // -1 = belum diketahui (baru boot)
  float vbNow = kfBattV.x;
  if (sccHold < 0)
    sccHold = (vbNow >= 14.0f || (vbNow >= 13.0f && data.pvA < 0.05f && data.lux[0] >= 2000.0f)) ? 1 : 0;
  else if (vbNow >= 14.0f) sccHold = 1;
  else if (vbNow < 13.0f)  sccHold = 0;   // Keeper1220 4S LiFePO4: resume 13.0 V
  bool isFloatFull = (sccHold == 1);          // SCC hold / baterai penuh

  // 1. Kebocoran arus beban saat lampu mati stabil
  if (mainValid && rampSettlingSamples == 0 && data.duty == 0.0f
      && (kfMainP.x > (P_MAIN_IDLE_W + 2.0f) || data.mainA > 0.30f))
    raise(LOAD_LEAKAGE);

  // 2. Beban & lampu saat menyala stabil (duty >= 5% dan soft-ramp selesai). Sama dengan run_all.py
  if (rampSettlingSamples == 0 && data.duty >= 5.0f) {
    if (mainValid && (kfMainP.nis > NIS_THRESHOLD || kfMainP.x < (0.35f * expMainP) || kfMainP.x > (1.60f * expMainP)))
      raise(LOAD_FAULT);
    if (lampValid && (kfLuxLamp.x < (0.25f * expLuxLamp) || (kfLuxLamp.nis > NIS_THRESHOLD && kfLuxLamp.nu < 0.0f)))
      raise(LAMP_FAULT);
  }

  // 3. PV pada siang hari
  if (isDaytime && pvValid) {
    if (!isFloatFull && (kfPvV.x < 8.0f || kfPvP.x < 1.0f)) raise(PV_LOW_GENERATION);
    if (kfPvV.nis > NIS_THRESHOLD)                           raise(PV_ABNORMAL_VOLTAGE);
  }

  // 4. Dinamika baterai
  if (battValid && (kfBattV.nis > NIS_THRESHOLD || kfBattI.nis > (NIS_THRESHOLD * 1.5f)))
    raise(BATTERY_ABNORMAL);

  // 5. Dinamika termal (kenaikan mendadak NTC #1, NTC #3, atau LM75B)
  bool batThermalSpike  = (kfTempBat.initialized  && kfTempBat.nis  > NIS_THRESHOLD && kfTempBat.nu  > 0.0f);
  bool ntc3ThermalSpike = (kfTempNtc3.initialized && kfTempNtc3.nis > NIS_THRESHOLD && kfTempNtc3.nu > 0.0f);
  bool lm75ThermalSpike = (kfTempLm75.initialized && kfTempLm75.nis > NIS_THRESHOLD && kfTempLm75.nu > 0.0f);
  if (batThermalSpike || ntc3ThermalSpike || lm75ThermalSpike) raise(THERMAL_ABNORMAL);

  // 6. Kegagalan komunikasi LoRa
  if (consecutiveLoraTimeout >= 3) raise(COMMUNICATION_FAULT);
}

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
void identifyDevices(bool silent = false) {
  addrInaMain = addrInaPV = addrInaBatt = addrADS = addrLM75 = -1;

  uint8_t detectedIna[8];
  int inaCount = 0;

  for (uint8_t a = 0x40; a <= 0x4F; a++) {
    if (!i2cPresent(a)) continue;
    uint16_t r0 = 0, r3 = 0;

    if (i2cRead16(a, 0x00, r0) && r0 == 0x399F) {
      if (inaCount < 8) detectedIna[inaCount++] = a;
      continue;
    }

    if (!i2cRead16(a, 0x03, r3)) continue;
    if (r3 == 0x7FFF && addrADS < 0)                  addrADS = a;
    else if ((r3 & 0xFF80) == 0x5000 && addrLM75 < 0) addrLM75 = a;
  }

  for (int i = 0; i < inaCount; i++) {
    uint8_t a = detectedIna[i];
    if (a == ADDR_INA_MAIN_DEFAULT && addrInaMain < 0)      { addrInaMain = a; detectedIna[i] = 0; }
    else if (a == ADDR_INA_PV_DEFAULT && addrInaPV < 0)     { addrInaPV   = a; detectedIna[i] = 0; }
    else if (a == ADDR_INA_BATT_DEFAULT && addrInaBatt < 0) { addrInaBatt = a; detectedIna[i] = 0; }
  }

  for (int i = 0; i < inaCount; i++) {
    uint8_t a = detectedIna[i];
    if (a == 0) continue;
    if (addrInaMain < 0)      addrInaMain = a;
    else if (addrInaPV < 0)   addrInaPV   = a;
    else if (addrInaBatt < 0) addrInaBatt = a;
  }

  if (addrInaMain < 0 && i2cPresent(ADDR_INA_MAIN_DEFAULT)) addrInaMain = ADDR_INA_MAIN_DEFAULT;

  if (addrLM75 >= 0) {
    Wire.beginTransmission(addrLM75);
    Wire.write(0x01); Wire.write(0x00);
    Wire.endTransmission();
  }

  for (int i = 0; i < 2; i++) {
    hasBH[i] = i2cPresent(ADDR_BH1750[i]);
    if (hasBH[i]) {
      i2cWriteByte(ADDR_BH1750[i], 0x01);
      if (i == 0) {
        // Sensor LINGKUNGAN: MTreg = 31 (minimum) -> jangkauan ~121.000 lux.
        // Default (MTreg 69) jenuh di 54.612 lux; di siang terik nilai jenuh itu konstan
        // dan memicu detektor stuck -> LIGHT_SENSOR_FAULT palsu.
        i2cWriteByte(ADDR_BH1750[i], 0x40 | (BH_AMB_MTREG >> 5));
        i2cWriteByte(ADDR_BH1750[i], 0x60 | (BH_AMB_MTREG & 0x1F));
      }
      i2cWriteByte(ADDR_BH1750[i], 0x10);
    }
  }

  if (!silent) {
    auto pr = [](const char *name, int a) {
      if (a >= 0) Serial.printf("# %-12s -> 0x%02X\n", name, a);
      else        Serial.printf("# %-12s -> TIDAK TERDETEKSI\n", name);
    };
    Serial.println(F("# Identifikasi Perangkat I2C:"));
    pr("INA219 Main", addrInaMain);
    pr("INA219 PV",   addrInaPV);
    pr("INA219 BATT", addrInaBatt);
    pr("ADS1115",     addrADS);
    pr("LM75B",       addrLM75);
    pr("BH1750 Amb",  hasBH[0] ? ADDR_BH1750[0] : -1); // 0x5C (ADDR = VCC) -> Lingkungan
    pr("BH1750 Lamp", hasBH[1] ? ADDR_BH1750[1] : -1); // 0x23 (ADDR = GND) -> Lampu LED HPL
  }
}

void scanI2C() {
  Serial.println(F("\n# --- Scan I2C ---"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.printf("# Ditemukan 0x%02X\n", a); n++; }
  }
  if (n == 0) Serial.println(F("# Tidak ada device! Cek pull-up SDA/SCL & 3.3V."));
  identifyDevices(false);
  Serial.println(F("# ----------------"));
}

// =====================================================================
//                              Sensor
// =====================================================================
bool readINA219(uint8_t addr, float shuntOhm, float &busV, float &currentA, float &powerW) {
  uint16_t rawShunt, rawBus;
  if (!i2cRead16(addr, 0x01, rawShunt)) return false;
  if (!i2cRead16(addr, 0x02, rawBus))   return false;

  float shuntmV = (int16_t)rawShunt * 0.01f;
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
  if (i == 0) lux *= 69.0f / BH_AMB_MTREG;   // datasheet: lux = raw/1.2 * (69/MTreg)
  return true;
}

float ntcTempC(float v) {
  if (isnan(v) || v <= 0.05f || v >= 3.10f) return NAN;
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

  // 3. Hanya baca NTC yang terpasang: NTC1 (ADS ch 0) dan NTC3 (ADS ch 2)
  //    NTC2, NTC4, dan NTC5 dikunci NAN agar tidak membaca noise pin mengambang (floating)
  for (int i = 0; i < 5; i++) {
    data.ntcV[i] = NAN;
    data.ntcT[i] = NAN;
  }
  float vNtc1, vNtc3;
  if (readADS1115(0, vNtc1)) {
    data.ntcV[0] = vNtc1;
    data.ntcT[0] = ntcTempC(vNtc1); // NTC1
  }
  if (readADS1115(2, vNtc3)) {
    data.ntcV[2] = vNtc3;
    data.ntcT[2] = ntcTempC(vNtc3); // NTC3
  }

  // 4. Sensor Cahaya BH1750 x2:
  //    i = 0 -> ADDR_BH1750[0] = 0x5C (VCC) -> data.lux[0] = lux_amb (Lingkungan)
  //    i = 1 -> ADDR_BH1750[1] = 0x23 (GND) -> data.lux[1] = lux_lamp (Lampu LED HPL)
  for (int i = 0; i < 2; i++) {
    float l;
    data.bhOk[i] = readBH1750(i, l);
    data.lux[i]  = data.bhOk[i] ? l : NAN;
  }

  // 5. Sensor Gerak PIR
  data.pir = digitalRead(PIN_PIR) ? 1 : 0;

  // 5.5. Injeksi Anomali via Serial (Jika sedang mode uji f1..f14 / fauto, tanpa merusak hardware)
  applyFaultInjection();

  // 6. Jalankan Deteksi Anomali 4-Lapis (L1 - L4) berbasis Kalman Filter + NIS
  evaluateAnomalyLayers();
}

void addField(String &s, bool ok, float v, int dec) {
  s += ',';
  if (!ok || isnan(v)) s += "NA";
  else s += String(v, dec);
}

// Gabungkan semua label aktif dari faultMask, dipisah '|', mis. "BATTERY_LOW|LAMP_FAULT"
String maskLabels(uint16_t m) {
  if (m == 0) return String("NONE");
  String out;
  for (int k = 1; k <= 14; k++) if (m & (1u << k)) {
    if (out.length()) out += '|';
    out += getAnomalyLabel((AnomalyCode)k);
  }
  return out;
}

// Susun baris CSV standar (23 kolom) untuk LoRa & Serial:
// t,duty,V_main,I_main,P_main,V_pv,I_pv,P_pv,V_bat,I_bat,I_bat_sq,NTC1,NTC2,NTC3,NTC4,NTC5,T_LM75,lux_amb,lux_lamp,PIR,fault,label
String buildCsvRow() {
  String row = String(data.t, 2);
  addField(row, true,        data.duty,    1); // duty
  addField(row, data.mainOk, data.mainV,   3); // V_main (3 desimal: LSB INA219 = 4 mV)
  addField(row, data.mainOk, data.mainA,   4); // I_main
  addField(row, data.mainOk, data.mainW,   3); // P_main
  addField(row, data.pvOk,   data.pvV,     3); // V_pv
  addField(row, data.pvOk,   data.pvA,     4); // I_pv
  addField(row, data.pvOk,   data.pvW,     3); // P_pv
  addField(row, data.battOk, data.battV,   3); // V_bat
  addField(row, data.battOk, data.battA,   4); // I_bat
  addField(row, data.battOk, data.battAsq, 4); // I_bat_sq
  for (int i = 0; i < 5; i++) {
    addField(row, !isnan(data.ntcT[i]), data.ntcT[i], 1); // NTC1..NTC5
  }
  addField(row, data.lmOk,    data.tBoard, 2); // T_LM75
  addField(row, data.bhOk[0], data.lux[0], 1); // lux_amb
  addField(row, data.bhOk[1], data.lux[1], 1); // lux_lamp
  row += "," + String(data.pir);               // PIR
  row += "," + String(data.fault);             // fault (0..14)
  row += "," + String(data.label);             // label (NONE / Anomaly Code)
  row += "," + String(data.faultMask);         // fault_mask: semua anomali aktif (bit k = kode k)
  return row;
}

// Cetak header CSV untuk mempermudah olah data di Excel / Python
void printCsvHeader() {
  Serial.println(F("t,duty,V_main,I_main,P_main,V_pv,I_pv,P_pv,V_bat,I_bat,I_bat_sq,NTC1,NTC2,NTC3,NTC4,NTC5,T_LM75,lux_amb,lux_lamp,PIR,fault,label,fault_mask,kf_Vbat,kf_Pmain,kf_Vpv,nis_batV,nis_mainP,nis_pvV,nis_temp,gt_fault,gt_label,lora_tx"));
}

// Cetak baris CSV lengkap ke Serial Monitor (22 kolom utama + 7 kolom diagnostik Kalman/NIS + 2 kolom Ground Truth)
void printCsvSerial() {
  String line = buildCsvRow();
  float maxTempNis = max(kfTempBat.nis, max(kfTempNtc3.nis, kfTempLm75.nis));
  addField(line, kfBattV.initialized, kfBattV.x,   2);
  addField(line, kfMainP.initialized, kfMainP.x,   2);
  addField(line, kfPvV.initialized,   kfPvV.x,     2);
  addField(line, kfBattV.initialized, kfBattV.nis, 2);
  addField(line, kfMainP.initialized, kfMainP.nis, 2);
  addField(line, kfPvV.initialized,   kfPvV.nis,   2);
  addField(line, (kfTempBat.initialized || kfTempNtc3.initialized || kfTempLm75.initialized), maxTempNis, 2);
  line += "," + String(injectedFault);
  line += "," + String(getAnomalyLabel((AnomalyCode)injectedFault));
  line += String(",") + (loraTxFlag ? "1" : "0");   // lora_tx
  loraTxFlag = false;
  Serial.println(line);
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
  Serial.printf("%-12s : %d\n", "PIR",      data.pir);
  Serial.printf("%-12s : %d (%s)\n", "fault (pred)", data.fault, data.label);
  Serial.printf("%-12s : %u -> %s\n", "semua aktif", data.faultMask, maskLabels(data.faultMask).c_str());
  Serial.printf("%-12s : %d (%s)\n", "gt_fault",     injectedFault, getAnomalyLabel((AnomalyCode)injectedFault));
  Serial.printf("NIS (Bat/Main/PV/Temp) : %.2f / %.2f / %.2f / %.2f\n",
                kfBattV.nis, kfMainP.nis, kfPvV.nis, max(kfTempBat.nis, max(kfTempNtc3.nis, kfTempLm75.nis)));
  Serial.println(F("-------------------------------------------------------"));
}

// =====================================================================
//         Aktuator: Sinkronisasi Penuh LED HPL (Q3) & LED Internal
// =====================================================================
void applySyncedPWM(int pct) {
  hplPercent    = constrain(pct, 0, DUTY_MAX);
  currentPwmRaw = ((uint32_t)hplPercent * PWM_MAX) / 100;

  ledcWrite(PIN_PWM_HPL, currentPwmRaw);
  ledcWrite(PIN_LED_INT, currentPwmRaw);
}

void setManualPWM(int pct) {
  currentDimMode = MODE_MANUAL_PWM;
  targetPercent  = constrain(pct, 0, DUTY_MAX);
  if (!csvOnlyMode) {
    Serial.printf(">> [PWM MANUAL SOFT-RAMP] Target Duty = %d%% (Aktual sekarang: %d%%)\n",
                  targetPercent, hplPercent);
  }
}

void setGateToggleState(bool on) {
  gateOn        = on;
  targetPercent = on ? DUTY_MAX : 0;
  if (!csvOnlyMode) {
    Serial.printf(">> [%lu ms] [TOGGLE SOFT-RAMP] Target MOSFET LED HPL: %s (Target = %d%%)\n",
                  millis(), on ? "ON" : "OFF", targetPercent);
  }
}

void startToggleMode() {
  currentDimMode = MODE_TOGGLE;
  lastToggle = millis();
  if (!csvOnlyMode) Serial.println(F(">> Mode TOGGLE Aktif: ON/OFF selaras tiap 3 detik (dengan Soft-Ramp)"));
  setGateToggleState(false);
}

void startSweepMode() {
  currentDimMode = MODE_SWEEP;
  sweepDir = 1;
  lastSweep = millis();
  if (!csvOnlyMode) Serial.println(F(">> Mode AUTO-SWEEP Dimming Aktif: 0% <-> 100%"));
}

void startAdaptiveSimMode() {
  currentDimMode = MODE_ADAPTIVE_SIM;
  adaptStage     = 0;
  adaptTarget    = 20;
  targetPercent  = 20;
  lastAdaptState = millis();
  lastSweep      = millis();
  if (!csvOnlyMode) Serial.println(F(">> Mode ADAPTIVE SIMULASI (Timer) Aktif: 20% -> 100% -> 50% -> 0%"));
}

void startAdaptiveLuxMode() {
  currentDimMode = MODE_ADAPTIVE_LUX;
  if (!csvOnlyMode) {
    Serial.println(F(">> Mode ADAPTIVE TANPA PIR (Hanya BH1750 Ambient) Aktif!"));
    Serial.println(F("   (Semakin gelap lux_amb, duty cycle otomatis naik bertahap 0% -> 10% -> 20% -> 30% -> 40%)"));
  }
}

void startAdaptivePirMode() {
  currentDimMode   = MODE_ADAPTIVE_PIR;
  lastSeenPirCount = pirRiseCount;
  pirMotionActive  = false;
  if (!csvOnlyMode) {
    Serial.println(F(">> Mode ADAPTIVE DENGAN PIR (BH1750 Ambient + Sensor PIR) Aktif!"));
    Serial.printf("   (Siang=0%% | Standby=%d%% | Ada Gerakan PIR=%d%% -> Cooldown 20%% -> Standby)\n", standbyDuty, DUTY_MAX);
  }
}

// Hitung target PWM berdasarkan cahaya lingkungan BH1750 (ADDR=VCC / data.lux[0]) untuk Mode Tanpa PIR
int computeLuxOnlyTarget(float luxAmb) {
  if (isnan(luxAmb))     return 20;  // Fallback aman jika sensor belum siap
  if (luxAmb >= 150.0f)  return 0;   // Siang / Terang -> Lampu Mati (0%)
  if (luxAmb >= 80.0f)   return 10;  // Sore Terang    -> 10%
  if (luxAmb >= 35.0f)   return 20;  // Senja / Mendung-> 20%
  if (luxAmb >= 10.0f)   return 30;  // Malam Remang   -> 30%
  return DUTY_MAX;                   // Malam Gelap    -> DUTY_MAX (85%)
}

// Hitung target PWM berdasarkan BH1750 Ambient + Sensor Gerak PIR untuk Mode Dengan PIR
int computeLuxPirTarget(float luxAmb) {
  // Jika kondisi lingkungan masih terang (Siang >= 120 lux), lampu wajib OFF (0%)
  if (!isnan(luxAmb) && luxAmb >= 120.0f) {
    pirMotionActive = false;
    return 0;
  }

  // Cek apakah ada pemicu gerakan dari PIR (pin HIGH atau interrupt baru)
  uint32_t now = millis();
  if (digitalRead(PIN_PIR) == HIGH || pirRiseCount != lastSeenPirCount) {
    lastSeenPirCount = pirRiseCount;
    lastPirMotionMs  = now;
    pirMotionActive  = true;
  }

  // Jika baru ada gerakan dalam 8 detik terakhir -> Full Bright 100%
  if (pirMotionActive) {
    uint32_t elapsed = now - lastPirMotionMs;
    if (elapsed <= PIR_HOLD_FULL_MS) {
      return DUTY_MAX;
    } else if (elapsed <= (PIR_HOLD_FULL_MS + PIR_COOLDOWN_MS)) {
      return 20; // Fase Cooldown 4 detik (20%)
    } else {
      pirMotionActive = false;
    }
  }

  // Kondisi Malam Standby (Tidak ada gerakan): duty tetap 15%
  return standbyDuty;
}

void beep(int times, int onMs = 100, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_BUZZ, HIGH); delay(onMs);
    digitalWrite(PIN_BUZZ, LOW);  if (i < times - 1) delay(offMs);
  }
}


// =====================================================================
//        SESI LOGGING OTOMATIS (untuk pipeline Python / Kalman)
// =====================================================================
// Ketik: sesiD, sesiD2, sesiE, sesiF1, sesiF2   |   stop = hentikan sesi
// Setiap langkah: duty (atau -1 = mode a2), lama [detik], penanda fault, dan instruksi.
//   mark = -1 : tidak ada penanda
//   mark =  0 : AKHIR fault  (dicetak "#MARK,t,0")
//   mark >  0 : AWAL fault dengan kode tsb (dicetak "#MARK,t,kode")
// convert_aurora.py membaca baris #MARK untuk memberi label fault secara otomatis.
// Bunyi buzzer: 1x panjang = LAKUKAN aksi fault sekarang | 2x pendek = KEMBALIKAN normal |
//               1x pendek = ganti langkah | 3x = sesi selesai
// (struct SeqStep didefinisikan di bagian atas file - lihat catatan di sana)

const SeqStep SEQ_D[] = {   // TRAIN malam (~45 menit; level 0/10/20/30/40)
  {0,210,-1,nullptr},{10,210,-1,nullptr},{DUTY_MAX,210,-1,nullptr},{10,210,-1,nullptr},{30,210,-1,nullptr},
  {20,210,-1,nullptr},{10,210,-1,nullptr},{DUTY_MAX,210,-1,nullptr},{0,210,-1,nullptr},{10,210,-1,nullptr},
  {30,210,-1,nullptr},{DUTY_MAX,210,-1,nullptr},{10,210,-1,nullptr}};
const SeqStep SEQ_D2[] = {  // VAL malam (~20 menit)
  {10,200,-1,nullptr},{30,200,-1,nullptr},{10,200,-1,nullptr},{DUTY_MAX,200,-1,nullptr},{20,200,-1,nullptr},{0,200,-1,nullptr}};
const SeqStep SEQ_E[] = {   // TEST malam (~40 menit): 20 menit urutan duty + 20 menit mode a2
  {DUTY_MAX,200,-1,nullptr},{10,200,-1,nullptr},{20,200,-1,nullptr},{30,200,-1,nullptr},{0,200,-1,nullptr},{10,200,-1,nullptr},
  {-1,1200,-1,"Mode a2 (PIR) 20 menit: standby 10%, lewat di depan PIR sesekali (40% 8 s -> 20% 4 s)"}};
const SeqStep SEQ_F1[] = {  // LAMP_FAULT (~11 menit)
  {DUTY_MAX,60,-1,"Duty 40%, tunggu..."},
  {DUTY_MAX,120,7,"TUTUP sensor lampu (BH1750 0x23) SEKARANG"},
  {DUTY_MAX,120,0,"BUKA tutup sensor lampu"},
  {DUTY_MAX,120,7,"TUTUP sensor lampu SEKARANG"},
  {DUTY_MAX,120,0,"BUKA tutup sensor lampu"},
  {0,30,-1,nullptr}};
const SeqStep SEQ_F2[] = {  // LOAD_FAULT (~13 menit). Kabel dicabut/dipasang HANYA saat duty 0
  {0,60,-1,"Duty 0%. CABUT satu kabel lampu HPL sekarang (lampu mati, aman)"},
  {DUTY_MAX,120,8,"Duty 40% dengan kabel lampu tercabut"},
  {0,60,0,"Duty 0%. PASANG kembali kabel lampu"},
  {DUTY_MAX,120,-1,"Normal: duty 40% dengan lampu terpasang"},
  {0,60,-1,"Duty 0%. CABUT lagi kabel lampu"},
  {DUTY_MAX,120,8,"Duty 40% dengan kabel lampu tercabut"},
  {0,60,0,"Duty 0%. PASANG kembali kabel lampu"},
  {DUTY_MAX,120,-1,"Normal: duty 40%"},
  {0,30,-1,nullptr}};

// ---- Sesi SIANG (lampu mati, duty 0) ----
const SeqStep SEQ_A[]  = {{0,2700,-1,"Sesi A (train siang) 45 menit: jangan sentuh alat"}};
const SeqStep SEQ_A2[] = {{0,1200,-1,"Sesi A2 (val siang) 20 menit: jangan sentuh alat"}};
const SeqStep SEQ_B[]  = {{0,1800,-1,"Sesi B (test siang) 30 menit: jangan sentuh alat"}};
const SeqStep SEQ_C[] = {   // PV_LOW_GENERATION fisik (~39 menit). Tutup PANEL saja, sensor lux JANGAN ditutup
  {0,180,-1,"Normal 3 menit (panel terbuka)"},
  {0,180,1,"TUTUP panel 25% (kardus) SEKARANG"},
  {0,180,0,"BUKA tutup panel"},
  {0,180,1,"TUTUP panel 50% SEKARANG"},
  {0,180,0,"BUKA tutup panel"},
  {0,180,1,"TUTUP panel 100% SEKARANG"},
  {0,180,0,"BUKA tutup panel"},
  {0,180,1,"TUTUP panel 25% SEKARANG"},
  {0,180,0,"BUKA tutup panel"},
  {0,180,1,"TUTUP panel 50% SEKARANG"},
  {0,180,0,"BUKA tutup panel"},
  {0,180,1,"TUTUP panel 100% SEKARANG"},
  {0,180,0,"BUKA tutup panel, sesi hampir selesai"}};

// ---- DEMO rekaman: jeda 10 s, duty awal 5 s, lalu duty akhir ditahan sampai 'stop' ----
const SeqStep SEQ_DEMO1[] = {{0,10,-1,"Demo 1: mulai dalam 10 detik"},{3,5,-1,"Duty 3%"},{10,65000,-1,"Duty 10% (ketik stop untuk selesai)"}};
const SeqStep SEQ_DEMO2[] = {{0,10,-1,"Demo 2: mulai dalam 10 detik"},{5,5,-1,"Duty 5%"},{15,65000,-1,"Duty 15% (ketik stop untuk selesai)"}};
const SeqStep SEQ_DEMO3[] = {{0,10,-1,"Demo 3: mulai dalam 10 detik"},{2,5,-1,"Duty 2%"},{12,65000,-1,"Duty 12% (ketik stop untuk selesai)"}};

// ---- DEMO (rekam video) ----
// demo1..demo3 : mulai di duty rendah, tahan 10 s, lalu naik dan TETAP di duty tinggi
// turun1..turun3: mulai di duty tinggi, tahan 15 s, lalu turun dan TETAP di duty rendah
// Tanpa buzzer. Ketik 'stop' atau angka duty untuk keluar.
const SeqStep DEMO1[]  = {{3,10,-1,"Demo: duty 3%, naik ke 10% dalam 10 s"},  {10,1,-1,nullptr}};
const SeqStep DEMO2[]  = {{5,10,-1,"Demo: duty 5%, naik ke 15% dalam 10 s"},  {15,1,-1,nullptr}};
const SeqStep DEMO3[]  = {{2,10,-1,"Demo: duty 2%, naik ke 12% dalam 10 s"},  {12,1,-1,nullptr}};
const SeqStep TURUN1[] = {{10,15,-1,"Demo: duty 10% selama 15 s, lalu turun ke 3%"}, {3,1,-1,nullptr}};
const SeqStep TURUN2[] = {{15,15,-1,"Demo: duty 15% selama 15 s, lalu turun ke 5%"}, {5,1,-1,nullptr}};
const SeqStep TURUN3[] = {{12,15,-1,"Demo: duty 12% selama 15 s, lalu turun ke 2%"}, {2,1,-1,nullptr}};
bool seqKeepDuty = false;   // true untuk demo: duty terakhir dipertahankan, tanpa buzzer

const SeqStep *seqSteps = nullptr;
int         seqLen = 0, seqIdx = -1;
uint32_t    seqStepStart = 0;
const char *seqName = "";
bool        sessionActive = false;

void applySeqStep() {
  const SeqStep &st = seqSteps[seqIdx];
  if (st.duty >= 0) setManualPWM(st.duty);
  else              startAdaptivePirMode();
  if (st.mark >= 0) Serial.printf("#MARK,%.2f,%d\n", millis() / 1000.0f, st.mark);
  Serial.printf("# SESI %s langkah %d/%d: duty=%s, %u s%s%s\n", seqName, seqIdx + 1, seqLen,
                st.duty >= 0 ? String(st.duty).c_str() : "a2", st.sec,
                st.say ? " -> " : "", st.say ? st.say : "");
  if (seqKeepDuty)       { /* demo: tanpa buzzer */ }
  else if (st.mark > 0)  beep(1, 400);
  else if (st.mark == 0) beep(2, 120, 120);
  else                   beep(1, 60);
}

void startSession(const SeqStep *steps, int n, const char *name) {
  seqSteps = steps; seqLen = n; seqIdx = 0; seqName = name;
  sessionActive = true; seqStepStart = millis();
  uint32_t total = 0;
  for (int i = 0; i < n; i++) total += steps[i].sec;
  Serial.printf("# SESI %s MULAI (%lu menit). Ketik 'stop' untuk membatalkan.\n", name, (unsigned long)(total / 60));
  applySeqStep();
}

void stopSession(const char *why) {
  if (!sessionActive) return;
  sessionActive = false;
  // bila sesi berhenti di tengah fault, tutup penandanya
  for (int i = seqIdx; i >= 0; i--) {
    if (seqSteps[i].mark > 0) { Serial.printf("#MARK,%.2f,0\n", millis() / 1000.0f); break; }
    if (seqSteps[i].mark == 0) break;
  }
  setManualPWM(0);
  Serial.printf("# SESI %s %s\n", seqName, why);
  beep(3, 120, 120);
}

void updateSession() {
  if (!sessionActive) return;
  if (millis() - seqStepStart < (uint32_t)seqSteps[seqIdx].sec * 1000UL) return;
  seqStepStart = millis();
  if (++seqIdx >= seqLen) {
    seqIdx = seqLen - 1;
    if (seqKeepDuty) {                       // demo: berhenti tanpa mematikan lampu
      sessionActive = false;
      Serial.printf("# SESI %s SELESAI (duty dipertahankan)\n", seqName);
      return;
    }
    stopSession("SELESAI - simpan file sekarang");
    return;
  }
  applySeqStep();
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
  Serial.println(F("\n# --- Konfigurasi LoRa E220 ---"));
  loraSetMode(1, 1);
  while (LoRa.available()) LoRa.read();
  const uint8_t cmd[3] = {0xC1, 0x00, 0x08};
  LoRa.write(cmd, 3);

  uint8_t r[11]; int n = 0;
  uint32_t t0 = millis();
  while (n < 11 && millis() - t0 < 500) if (LoRa.available()) r[n++] = LoRa.read();
  loraSetMode(0, 0);

  if (n < 11 || r[0] != 0xC1) {
    Serial.printf("# GAGAL (terima %d byte). Cek wiring M0/M1/AUX/TX/RX.\n", n);
    return;
  }
  uint8_t reg0 = r[5], reg1 = r[6], ch = r[7], reg3 = r[8];
  const char *baud[] = {"1200","2400","4800","9600","19200","38400","57600","115200"};
  const char *air[]  = {"2.4k","2.4k","2.4k","4.8k","9.6k","19.2k","38.4k","62.5k"};
  Serial.printf("# Alamat     : 0x%02X%02X | UART: %s | Air: %s | CH: %d\n",
                r[3], r[4], baud[(reg0 >> 5) & 7], air[reg0 & 7], ch);
}

const uint8_t LORA_CHANNEL = 23;   // 873.125 MHz
void loraWriteConfig(uint8_t reg1 = 0x03, bool silent = false) {
  loraSetMode(1, 1);
  while (LoRa.available()) LoRa.read();
  const uint8_t cmd[11] = {0xC2, 0x00, 0x08,
                           0x00, 0x00,
                           0x62,
                           reg1,
                           LORA_CHANNEL,
                           0x03,
                           0x00, 0x00};
  LoRa.write(cmd, 11);
  uint8_t r[11]; int n = 0;
  uint32_t t0 = millis();
  while (n < 11 && millis() - t0 < 500) if (LoRa.available()) r[n++] = LoRa.read();
  loraSetMode(0, 0);
  if (!silent) {
    if (n >= 3 && r[0] == 0xC1) Serial.println(F("# CONFIG_OK"));
    else Serial.printf("# CONFIG GAGAL (terima %d byte)\n", n);
    loraReadConfig();
  }
}

uint8_t xorChecksum(const String &s) {
  uint8_t cs = 0;
  for (size_t i = 0; i < s.length(); i++) cs ^= (uint8_t)s[i];
  return cs;
}

// Format Paket LoRa tetap identik dengan sistem penerima:
// AUR,seq,t,duty,V_main,I_main,P_main,V_pv,I_pv,P_pv,V_bat,I_bat,I_bat_sq,NTC1,NTC2,NTC3,NTC4,NTC5,T_LM75,lux_amb,lux_lamp,PIR,fault,label,fault_mask*CS
// fault_mask: bit k = kode anomali k aktif (dashboard: (mask >> k) & 1)
void loraSend() {
  txSeq++;
  String p = "AUR," + String(txSeq) + "," + buildCsvRow();

  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X", xorChecksum(p));
  p += tail;

  if (!waitAux(500)) {
    consecutiveLoraTimeout++; // Hardware modul LoRa E220 sibuk / AUX stuck LOW
    if (!csvOnlyMode) Serial.println(F(">> LoRa: AUX LOW (modul sibuk), paket dibatalkan"));
    return;
  }
  LoRa.print(p);
  LoRa.print('\n');
  LoRa.flush();

  uint32_t tA = millis(); bool wentLow = false;
  while (millis() - tA < 100) if (digitalRead(PIN_LORA_AUX) == LOW) { wentLow = true; break; }
  uint32_t tLow = millis();
  if (wentLow) waitAux(3000);
  if (wentLow) loraTxFlag = true;   // tandai hanya bila modul BENAR-BENAR memancar (ada pulsa arus)
  consecutiveLoraTimeout = 0; // Reset timeout karena modul tidak stuck LOW saat waitAux(500)

  if (!csvOnlyMode) {
    if (wentLow) Serial.printf("   AUX LOW %lu ms (modul memancarkan)\n", millis() - tLow);
    else         Serial.println(F("   !! AUX tidak pernah LOW -> modul TIDAK memancarkan"));
  }

  txCount++;
  waitingAck = true; waitingAckSeq = txSeq; txTime = millis();
  if (!csvOnlyMode) {
    Serial.printf(">> LoRa TX #%lu (%d byte): %s\n", (unsigned long)txSeq, p.length() + 1, p.c_str());
  }
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
          if (!csvOnlyMode) {
            Serial.printf("<< LoRa ACK #%lu diterima, RTT %lu ms\n",
                          (unsigned long)seq, millis() - txTime);
          }
          waitingAck = false;
        }
      } else if (loraRxBuf.length() && !csvOnlyMode) {
        Serial.printf("<< LoRa RX: %s\n", loraRxBuf.c_str());
      }
      loraRxBuf = "";
    } else if (loraRxBuf.length() < 220) {
      loraRxBuf += c;
    }
  }
  if (waitingAck && millis() - txTime > ACK_TIMEOUT_MS) {
    // Jangan menaikkan consecutiveLoraTimeout saat ACK timeout agar pengujian mandiri
    // (tanpa board Receiver menyala) tidak dianggap COMMUNICATION_FAULT.
    if (!csvOnlyMode) {
      Serial.printf("!! LoRa: ACK #%lu tidak diterima (timeout)\n",
                    (unsigned long)waitingAckSeq);
    }
    waitingAck = false;
  }
}

// =====================================================================
//                         Kontrol Serial Monitor
// =====================================================================
void printHelp() {
  Serial.println(F("\n# --- PERINTAH SERIAL MONITOR ---"));
  Serial.println(F("#  v        : Toggle Mode CSV Murni <-> Mode Tabel Telemetri"));
  Serial.println(F("#  header   : Cetak baris judul kolom CSV"));
  Serial.println(F("#  0..100   : Set Duty Cycle PWM Manual (0% s/d 100%)"));
  Serial.println(F("#  + / -    : Naikkan / turunkan Duty Cycle sebesar 5%"));
  Serial.println(F("#  a1 / lux : Mode Adaptive TANPA PIR (Hanya mengandalkan BH1750 Ambient)"));
  Serial.println(F("#  a2 / pir : Mode Adaptive DENGAN PIR (BH1750 Ambient + Sensor Gerak PIR)"));
  Serial.println(F("#  a        : Mode Simulasi Adaptive Timer (10% -> 40% -> 20% -> 0%)"));
  Serial.println(F("#  d        : Mode Tes Auto-Sweep Dimming (0% <-> 100%)"));
  Serial.println(F("#  t        : Mode Toggle ON/OFF (0% <-> 100%) tiap 3 detik"));
  Serial.println(F("#  f0       : Matikan Simulasi Anomali (Kembali ke Sensor Asli / NONE)"));
  Serial.println(F("#  f1..f14  : Uji Injeksi Anomali Kode 1 s/d 14 via Software (Aman untuk Hardware!)"));
  Serial.println(F("#  fauto    : Uji Otomatis Berurutan seluruh Anomali f1 s/d f14 (10s Fault / 6s Normal)"));
  Serial.println(F("#  s        : Scan & identifikasi ulang bus I2C"));
  Serial.println(F("#  b        : Tes Buzzer (3x beep)"));
  Serial.println(F("#  l        : Paksa kirim paket data ke LoRa sekarang"));
  Serial.println(F("#  demo1 / demo2 / demo3  : jeda 10 s, lalu 3->10% / 5->15% / 2->12% (stop = selesai)"));
  Serial.println(F("#  sesiA / sesiA2 / sesiB : sesi siang otomatis (train / val / test)"));
  Serial.println(F("#  sesiC    : fault PV fisik (tutup panel, ikuti buzzer)"));
  Serial.println(F("#  sesiD / sesiD2 / sesiE : sesi malam otomatis (train / val / test)"));
  Serial.println(F("#  sesiF1 / sesiF2       : sesi fault fisik LAMP / LOAD (ikuti bunyi buzzer)"));
  Serial.println(F("#  demo1/demo2/demo3   : demo naik 3->10, 5->15, 2->12 (tahan 10 s dulu)"));
  Serial.println(F("#  turun1/turun2/turun3: demo turun 10->3, 15->5, 12->2 (tahan 15 s dulu)"));
  Serial.println(F("#  stop     : hentikan sesi otomatis"));
  Serial.println(F("#  h        : Tampilkan bantuan ini\n"));
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

  // Sesi logging otomatis
  if (in.equalsIgnoreCase("demo1")) { startSession(SEQ_DEMO1, 3, "DEMO1 (3% -> 10%)"); return; }
  if (in.equalsIgnoreCase("demo2")) { startSession(SEQ_DEMO2, 3, "DEMO2 (5% -> 15%)"); return; }
  if (in.equalsIgnoreCase("demo3")) { startSession(SEQ_DEMO3, 3, "DEMO3 (2% -> 12%)"); return; }
  if (in.equalsIgnoreCase("demo1"))  { seqKeepDuty = true; startSession(DEMO1,  2, "demo1 (3->10)");  return; }
  if (in.equalsIgnoreCase("demo2"))  { seqKeepDuty = true; startSession(DEMO2,  2, "demo2 (5->15)");  return; }
  if (in.equalsIgnoreCase("demo3"))  { seqKeepDuty = true; startSession(DEMO3,  2, "demo3 (2->12)");  return; }
  if (in.equalsIgnoreCase("turun1")) { seqKeepDuty = true; startSession(TURUN1, 2, "turun1 (10->3)"); return; }
  if (in.equalsIgnoreCase("turun2")) { seqKeepDuty = true; startSession(TURUN2, 2, "turun2 (15->5)"); return; }
  if (in.equalsIgnoreCase("turun3")) { seqKeepDuty = true; startSession(TURUN3, 2, "turun3 (12->2)"); return; }
  if (in.startsWith("sesi") || in.startsWith("SESI") || in.startsWith("Sesi")) seqKeepDuty = false;
  if (in.equalsIgnoreCase("sesiA"))  { startSession(SEQ_A,  1, "A (train siang)"); return; }
  if (in.equalsIgnoreCase("sesiA2")) { startSession(SEQ_A2, 1, "A2 (val siang)"); return; }
  if (in.equalsIgnoreCase("sesiB"))  { startSession(SEQ_B,  1, "B (test siang)"); return; }
  if (in.equalsIgnoreCase("sesiC"))  { startSession(SEQ_C,  sizeof(SEQ_C) / sizeof(SEQ_C[0]), "C (fault PV)"); return; }
  if (in.equalsIgnoreCase("sesiD"))  { startSession(SEQ_D,  sizeof(SEQ_D)  / sizeof(SEQ_D[0]),  "D (train malam)"); return; }
  if (in.equalsIgnoreCase("sesiD2")) { startSession(SEQ_D2, sizeof(SEQ_D2) / sizeof(SEQ_D2[0]), "D2 (val malam)"); return; }
  if (in.equalsIgnoreCase("sesiE"))  { startSession(SEQ_E,  sizeof(SEQ_E)  / sizeof(SEQ_E[0]),  "E (test malam)"); return; }
  if (in.equalsIgnoreCase("sesiF1")) { startSession(SEQ_F1, sizeof(SEQ_F1) / sizeof(SEQ_F1[0]), "F1 (LAMP_FAULT)"); return; }
  if (in.equalsIgnoreCase("sesiF2")) { startSession(SEQ_F2, sizeof(SEQ_F2) / sizeof(SEQ_F2[0]), "F2 (LOAD_FAULT)"); return; }
  if (in.equalsIgnoreCase("stop"))   { stopSession("DIBATALKAN"); return; }
  // Perintah lain (selain tampilan) saat sesi berjalan membatalkan sesi agar urutan tidak tercampur
  if (sessionActive && !in.equalsIgnoreCase("v") && !in.equalsIgnoreCase("header") && !in.equalsIgnoreCase("h")) {
    stopSession("DIBATALKAN oleh perintah manual");
  }

  if (in.equalsIgnoreCase("v")) {
    csvOnlyMode = !csvOnlyMode;
    if (csvOnlyMode) {
      printCsvHeader();
    } else {
      Serial.println(F(">> Mode Tabel Telemetri Aktif (ketik 'v' untuk kembali ke CSV Murni)"));
    }
  }
  else if (in.equalsIgnoreCase("header")) printCsvHeader();
  else if (in.equalsIgnoreCase("s")) scanI2C();
  else if (in.equalsIgnoreCase("t")) startToggleMode();
  else if (in.equalsIgnoreCase("d")) startSweepMode();
  else if (in.equalsIgnoreCase("a1") || in.equalsIgnoreCase("lux")) startAdaptiveLuxMode();
  else if (in.equalsIgnoreCase("a2") || in.equalsIgnoreCase("pir")) startAdaptivePirMode();
  else if (in.equalsIgnoreCase("a")) startAdaptiveSimMode();
  else if (in.equalsIgnoreCase("fauto")) startAutoFaultTest();
  else if ((in.startsWith("f") || in.startsWith("F")) && isNumericString(in.substring(1))) {
    int fCode = in.substring(1).toInt();
    setFaultInjection(fCode, false);
  }
  else if (in == "+")                setManualPWM(targetPercent + 5);
  else if (in == "-")                setManualPWM(targetPercent - 5);
  else if (in.equalsIgnoreCase("b")) { beep(3, 200, 150); }
  else if (in.equalsIgnoreCase("l")) { readAll(); loraSend(); }
  else if (in.equalsIgnoreCase("c")) loraReadConfig();
  else if (in.equalsIgnoreCase("w")) loraWriteConfig(0x03, false);
  else if (in.equalsIgnoreCase("h")) printHelp();
  else if (isNumericString(in))      setManualPWM(in.toInt());
}

// =====================================================================
//                              Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // IO13 mengalami short ke GND saat penyolderan -> WAJIB dikunci sebagai INPUT!
  pinMode(PIN_SHORTED, INPUT);

  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  pinMode(PIN_PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PIR), pirISR, RISING);

  ledcAttach(PIN_LED_INT, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWM_HPL, PWM_FREQ, PWM_RES);
  applySyncedPWM(0);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_NTC5, ADC_11db);

  pinMode(PIN_LORA_M0, OUTPUT);
  pinMode(PIN_LORA_M1, OUTPUT);
  pinMode(PIN_LORA_AUX, INPUT_PULLUP);
  LoRa.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
  loraSetMode(0, 0);
  loraWriteConfig(0x03, true); // Konfigurasi senyap agar tidak mengotori log CSV

  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  identifyDevices(true);       // Deteksi semua sensor secara senyap

  beep(2);
  // Mulai pada PWM 0% (Manual) agar aman saat pertama disambung ke Panel Surya + Baterai
  // Ketik angka 0..100, 't', 'd', atau 'a' kapan saja di Serial Monitor untuk mengubah mode PWM
  setManualPWM(0);
  lastLoRa = millis();

  // Langsung cetak header CSV pertama kali saat boot
  printCsvHeader();
}

void loop() {
  handleSerial();
  loraPoll();
  updateSession();

  if (currentDimMode == MODE_MANUAL_PWM) {
    // Soft-Ramp otomatis: meskipun diketik 0 -> 100 langsung, dinaikkan bertahap (5% tiap 100 ms)
    if (hplPercent != targetPercent && millis() - lastRampStep >= RAMP_INTERVAL_MS) {
      lastRampStep = millis();
      if (hplPercent < targetPercent) {
        applySyncedPWM(min(hplPercent + RAMP_STEP_PCT, targetPercent));
      } else {
        applySyncedPWM(max(hplPercent - RAMP_STEP_PCT, targetPercent));
      }
    }
  } else if (currentDimMode == MODE_TOGGLE) {
    if (millis() - lastToggle >= TOGGLE_PERIOD_MS) {
      lastToggle = millis();
      setGateToggleState(!gateOn);
    }
    if (hplPercent != targetPercent && millis() - lastRampStep >= RAMP_INTERVAL_MS) {
      lastRampStep = millis();
      if (hplPercent < targetPercent) applySyncedPWM(min(hplPercent + RAMP_STEP_PCT, targetPercent));
      else                            applySyncedPWM(max(hplPercent - RAMP_STEP_PCT, targetPercent));
    }
  } else if (currentDimMode == MODE_SWEEP) {
    if (millis() - lastSweep >= SWEEP_STEP_MS) {
      lastSweep = millis();
      int nextPct = hplPercent + sweepDir;
      if (nextPct >= DUTY_MAX) { nextPct = DUTY_MAX; sweepDir = -1; }
      if (nextPct <= 0)   { nextPct = 0;   sweepDir = 1;  }
      targetPercent = nextPct;
      applySyncedPWM(nextPct);
    }
  } else if (currentDimMode == MODE_ADAPTIVE_SIM) {
    const int targets[4]    = {10, DUTY_MAX, 20, 0};
    const uint32_t holds[4] = {4000, 5000, 4000, 3000};
    if (millis() - lastAdaptState >= holds[adaptStage]) {
      lastAdaptState = millis();
      adaptStage = (adaptStage + 1) % 4;
      adaptTarget = targets[adaptStage];
      targetPercent = adaptTarget;
    }
    if (millis() - lastSweep >= 25) {
      lastSweep = millis();
      if (hplPercent < adaptTarget)      applySyncedPWM(hplPercent + 1);
      else if (hplPercent > adaptTarget) applySyncedPWM(hplPercent - 1);
    }
  } else if (currentDimMode == MODE_ADAPTIVE_LUX) {
    // Mode Adaptive TANPA PIR (Hanya mengandalkan sensor BH1750 Ambient / data.lux[0])
    targetPercent = computeLuxOnlyTarget(data.lux[0]);
    if (hplPercent != targetPercent && millis() - lastRampStep >= RAMP_INTERVAL_MS) {
      lastRampStep = millis();
      if (hplPercent < targetPercent) applySyncedPWM(min(hplPercent + RAMP_STEP_PCT, targetPercent));
      else                            applySyncedPWM(max(hplPercent - RAMP_STEP_PCT, targetPercent));
    }
  } else if (currentDimMode == MODE_ADAPTIVE_PIR) {
    // Mode Adaptive DENGAN PIR (BH1750 Ambient + Sensor Gerak PIR)
    targetPercent = computeLuxPirTarget(data.lux[0]);
    if (hplPercent != targetPercent && millis() - lastRampStep >= RAMP_INTERVAL_MS) {
      lastRampStep = millis();
      if (hplPercent < targetPercent) applySyncedPWM(min(hplPercent + RAMP_STEP_PCT, targetPercent));
      else                            applySyncedPWM(max(hplPercent - RAMP_STEP_PCT, targetPercent));
    }
  }

  // Pembacaan periodik seluruh sensor + Deteksi Anomali Kalman L1-L4 + Output CSV
  if (millis() - lastRead >= READ_INTERVAL_MS) {
    lastRead = millis();
    readAll();
    if (csvOnlyMode) printCsvSerial();
    else             printAll();
  }

  // Pengiriman telemetri LoRa periodik
  if (millis() - lastLoRa >= LORA_INTERVAL_MS) {
    lastLoRa = millis();
    loraSend();
  }
}
