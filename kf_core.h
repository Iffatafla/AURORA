#pragma once
// AURORA - inti Kalman filter skalar + detektor anomali NIS (C++ murni, tanpa panggilan Arduino,
// sehingga bisa diuji di PC). Parameter datang dari params_<nama>.h buatan kf_identify.py.
//
// Model (satu keadaan x, N_U input u_j):
//   x_k = A*x_{k-1} + sum_j B_j*u_j,k + C + w   (w ~ N(0,Q))
//   z_k = x_k + v                               (v ~ N(0,R))
#include <stdint.h>

template <int NU>
struct KFParams {            // aggregate (tanpa default member) agar bisa constexpr di C++11
  float    A;
  float    B[NU];
  float    C, Q, R;
  float    nisThr;
  int      nisConsec;
  int      decim;           // jumlah tick dasar per langkah filter
  uint32_t dtMs;            // periode langkah filter = decim * periode tick
  int      winN;            // panjang jendela rata-rata NIS (2..KF_WIN_MAX)
  float    winHi;           // rata-rata NIS > winHi -> anomali (bias / noise)
  float    winLo;           // rata-rata NIS < winLo -> anomali (sensor beku)
};

constexpr int   KF_WIN_MAX  = 32;
constexpr float KF_NIS_CLIP = 10.828f;   // = NIS_CLIP di kf_identify.py

template <int NU>
struct ScalarKF {
  const KFParams<NU>& p;
  float x = 0.0f, P = 0.0f;
  float nu = 0.0f, S = 0.0f, nis = 0.0f;
  bool  init = false;

  explicit ScalarKF(const KFParams<NU>& params) : p(params) {}

  void step(float z, const float (&u)[NU]) {
    if (!init) {                       // keadaan awal = pengukuran pertama
      x = z; P = p.R; S = p.R; nu = 0.0f; nis = 0.0f; init = true;
      return;
    }
    float xp = p.A * x + p.C;
    for (int j = 0; j < NU; j++) xp += p.B[j] * u[j];
    const float Pp = p.A * P * p.A + p.Q;
    nu  = z - xp;
    S   = Pp + p.R;
    nis = (nu * nu) / S;
    const float K = Pp / S;
    x = xp + K * nu;
    P = (1.0f - K) * Pp;
  }

  // Pengukuran hilang (NaN / sedang TX LoRa) tetapi input diketahui: maju satu langkah tanpa update.
  // Tanpa ini, langkah yang dilewati saat input berubah menghasilkan NIS palsu besar.
  // Identik dengan cabang 'pred' di run_seg() kf_identify.py.
  void predictOnly(const float (&u)[NU]) {
    if (!init) return;
    float xp = p.A * x + p.C;
    for (int j = 0; j < NU; j++) xp += p.B[j] * u[j];
    x = xp;
    P = p.A * P * p.A + p.Q;
  }
};

// Dua detektor, hasil di-OR (identik dengan detect_all() di kf_identify.py):
//  titik  : NIS > nisThr selama nisConsec langkah berturut-turut
//  jendela: rata-rata NIS (tiap nilai dipotong di KF_NIS_CLIP) dari winN langkah terakhir
//           > winHi (bias/noise) atau < winLo (sensor beku). Aktif setelah jendela penuh.
enum AnomalyKind : uint8_t { AK_NONE = 0, AK_POINT = 1, AK_WIN_HIGH = 2, AK_WIN_LOW = 4 };

struct AnomalyDetector {
  int     run  = 0;
  bool    flag = false;
  uint8_t kind = AK_NONE;     // bitmask AnomalyKind (untuk log / LoRa)
  float   winMean = 0.0f;

  void update(float nis, float thr, int consec, int winN, float winHi, float winLo) {
    run = (nis > thr) ? run + 1 : 0;
    kind = (run >= consec) ? AK_POINT : AK_NONE;

    if (winN < 2) { flag = (kind != AK_NONE); return; }   // header lama (tanpa jendela): hanya detektor titik
    if (winN > KF_WIN_MAX) winN = KF_WIN_MAX;
    const float v = (nis < KF_NIS_CLIP) ? nis : KF_NIS_CLIP;
    buf[head] = v;
    head = (head + 1) % winN;
    if (cnt < winN) cnt++;
    if (cnt == winN) {
      float sum = 0.0f;                  // dijumlah ulang tiap langkah: tidak ada galat yang menumpuk
      for (int j = 0; j < winN; j++) sum += buf[j];
      winMean = sum / winN;
      if (winMean > winHi) kind |= AK_WIN_HIGH;
      if (winMean < winLo) kind |= AK_WIN_LOW;
    }
    flag = (kind != AK_NONE);
  }

 private:
  float buf[KF_WIN_MAX] = {};
  int   head = 0, cnt = 0;
};

// Satu subsistem: rata-rata blok (decimation) -> filter -> detektor.
// push() dipanggil tiap tick dasar; mengembalikan true saat filter benar-benar melangkah.
template <int NU>
struct Subsystem {
  ScalarKF<NU>    kf;
  AnomalyDetector det;
  float lastZ = 0.0f, lastNis = 0.0f;   // nilai langkah filter terakhir (untuk log)
  bool  flag  = false;

  explicit Subsystem(const KFParams<NU>& params) : kf(params) {}

  // Mulai ulang dari nol (dipakai saat subsistem diaktifkan lagi setelah dijeda, mis. PV pagi hari)
  void reset() {
    kf.init = false; kf.x = kf.P = kf.nu = kf.S = kf.nis = 0.0f;
    det = AnomalyDetector();
    flag = false; lastZ = lastNis = 0.0f;
    accZ = 0.0f; for (int j = 0; j < NU; j++) { accU[j] = 0.0f; accUo[j] = 0.0f; }
    n = nUo = ticks = 0;
  }

  // z = NAN untuk menandai pengukuran tidak dipakai (gagal baca, atau tick saat TX LoRa).
  bool push(float z, const float (&u)[NU]) {
    bool uok = true;
    for (int j = 0; j < NU; j++) uok = uok && (u[j] == u[j]);
    if (uok && z == z) {
      accZ += z;
      for (int j = 0; j < NU; j++) accU[j] += u[j];
      n++;
    } else if (uok) {                    // z hilang, u ada -> simpan u untuk langkah prediksi
      for (int j = 0; j < NU; j++) accUo[j] += u[j];
      nUo++;
    }
    if (++ticks < kf.p.decim) return false;
    ticks = 0;
    const int cnt = n, cntUo = nUo;
    n = nUo = 0;
    if (cnt == 0) {
      if (cntUo > 0) {                   // blok tanpa pengukuran: prediksi saja (tanpa NIS baru)
        float uo[NU];
        for (int j = 0; j < NU; j++) uo[j] = accUo[j] / cntUo;
        kf.predictOnly(uo);
      }
      accZ = 0; for (int j = 0; j < NU; j++) { accU[j] = 0; accUo[j] = 0; }
      return false;
    }
    for (int j = 0; j < NU; j++) accUo[j] = 0;
    float um[NU];
    for (int j = 0; j < NU; j++) { um[j] = accU[j] / cnt; accU[j] = 0; }
    const float zm = accZ / cnt;
    accZ = 0;
    const bool first = !kf.init;          // langkah pertama hanya inisialisasi (tanpa NIS)
    kf.step(zm, um);
    if (!first) det.update(kf.nis, kf.p.nisThr, kf.p.nisConsec, kf.p.winN, kf.p.winHi, kf.p.winLo);
    lastZ = zm; lastNis = kf.nis; flag = det.flag;
    return true;
  }

 private:
  float accZ = 0.0f, accU[NU] = {}, accUo[NU] = {};
  int   n = 0, nUo = 0, ticks = 0;
};

// Cek sensor beku pada nilai MENTAH (bukan z hasil kali): flag bila nilai persis sama selama
// nSame tick berturut-turut. Panggil tiap tick dasar dengan bacaan sensor (setelah fault injector),
// SEBELUM decimation. NaN dilewati (tidak mereset hitungan). Sama dengan detect_flat() di kf_identify.py.
struct FlatlineDetector {
  int   nSame;
  int   run = 0;
  bool  flag = false, have = false;
  float last = 0.0f;
  explicit FlatlineDetector(int n) : nSame(n) {}
  void reset() { run = 0; flag = false; have = false; }
  void update(float v) {
    if (nSame <= 0) return;                     // 0 = dimatikan
    if (v != v) return;                         // NaN
    run  = (have && v == last) ? run + 1 : 0;
    last = v; have = true;
    flag = (run >= nSame - 1);
  }
};
