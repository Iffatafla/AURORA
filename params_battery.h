// Dibuat oleh kf_identify.py - jangan diedit manual
// subsistem : battery
// z = V_bat ; u = [I_bat] ; dt = 1.000 s ; decimate = 1
// aktif bila: V_main>5   <- firmware HARUS memakai kondisi yang sama
// sampel dilewati bila: (tidak ada) = 1
// cek beku : I_bat ; N = 30 tick identik
// data      : train=../final/data/train/night_Epre_3nov.csv,../final/data/train/day_A_2nov.csv val=../final/data/val/night_F1normal_2nov.csv,../final/data/val/day_A2_3nov.csv test=../final/data/test/night_Esim_3nov.csv,../final/data/test/day_B_3nov.csv
// alarm palsu set test: 0.000 per jam
#pragma once
#include <stdint.h>
#include "kf_core.h"

constexpr int BATTERY_NU = 1;
constexpr KFParams<1> BATTERY = {
  0.94101019f,  // A
  { -0.010625919f },  // B (satu per input)
  0.78066891f,  // C
  0.00012766584f,  // Q
  3.0274341e-10f,  // R
  94.353721f,  // ambang NIS
  5,  // sampel berturut-turut
  1,  // decimation (tick dasar per langkah filter)
  1000,  // dt [ms]
  10,  // panjang jendela rata-rata NIS
  5.5215766f,  // ambang atas rata-rata NIS (bias / noise)
  0.010494055f  // ambang bawah rata-rata NIS (sensor beku)
};
constexpr int BATTERY_FLAT_N = 30;
constexpr float BATTERY_TAU_S = 16.447014f;
