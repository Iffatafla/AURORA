// Model baterai v2 (19 Nov 2026) - dibuat oleh fit_bat.py (lihat AURORA_Update_C501)
// subsistem : battery
// z = V_bat ; u = [I_bat, dI_bat] ; dt = 1.000 s ; decimate = 1
// STRUKTUR: A = 1, C = 0  ->  V(k) = V(k-1) + B1*I_bat + B2*dI_bat
//   (tegangan sebagai integral muatan + lompatan IR; TANPA titik keseimbangan)
//   Model lama (A=0,941, C=0,781) punya titik keseimbangan ~13,2 V -> BATTERY_ABNORMAL palsu
//   terus-menerus saat V_bat < ~13,3 V (log Siang_18_Nov).
// aktif bila: V_main>5 ; dilewati (reset) saat sapuan MPPT / lompatan arus > 1 A, + 4 sampel
// data      : train=day_A_2nov, night_Epre_3nov  val=day_A2_3nov, night_F1normal_2nov
//             test=day_B_3nov, night_Esim_3nov (0 alarm), Siang_18_Nov (3 kejadian / 2,45 jam)
// ambang    : titik = maks(6,635; 1,5 x maks min-5 NIS val) ; jendela = min(1,5 x maks rata-rata-10 val; 10,0)
#pragma once
#include <stdint.h>
#include "kf_core.h"

constexpr int BATTERY_NU = 2;
constexpr KFParams<2> BATTERY = {
  1.0f,  // A
  { -7.070561e-05f, -0.095417936f },  // B: [I_bat (V per A per s), dI_bat (V per A)]
  0.0f,  // C
  8.2046961e-07f,  // Q
  2.9961427e-06f,  // R
  10.4113f,  // ambang NIS
  5,  // sampel berturut-turut
  1,  // decimation
  1000,  // dt [ms]
  10,  // panjang jendela rata-rata NIS
  10.0f,  // ambang atas rata-rata NIS (NIS dipotong 10,828; 1,5 x maks val melebihi batas potong)
  -1.0f  // jendela bawah NONAKTIF: resolusi V INA219 4 mV -> NIS=0 sering pada data normal. Beku dicek lewat flatBatt (I_bat)
};
constexpr int BATTERY_FLAT_N = 30;
constexpr float BATTERY_TAU_S = 0.0f;   // A = 1: tidak ada konstanta waktu
constexpr float BATTERY_SKIP_DI_A = 1.0f;   // lompatan arus pemicu lewati
constexpr int   BATTERY_SKIP_N    = 4;      // sampel dilewati setelah pemicu
