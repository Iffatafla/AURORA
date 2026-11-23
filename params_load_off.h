// Dibuat oleh kf_identify.py - jangan diedit manual
// subsistem : load_off
// z = I_main ; u = [V_main] ; dt = 1.000 s ; decimate = 1
// aktif bila: duty<0.01;V_main>5   <- firmware HARUS memakai kondisi yang sama
// sampel dilewati bila: lora_tx = 1
// cek beku : (mati) ; N = 0 tick identik
// data      : train=../final/data/train/night_Epre_3nov.csv,../final/data/train/day_A_2nov.csv val=../final/data/val/night_F1normal_2nov.csv,../final/data/val/day_A2_3nov.csv test=../final/data/test/night_Esim_3nov.csv,../final/data/test/day_B_3nov.csv
// alarm palsu set test: 0.000 per jam
#pragma once
#include <stdint.h>
#include "kf_core.h"

constexpr int LOAD_OFF_NU = 1;
constexpr KFParams<1> LOAD_OFF = {
  0.076084579f,  // A
  { -0.0084803576f },  // B (satu per input)
  0.22526953f,  // C
  7.5343331e-05f,  // Q
  3.6689755e-05f,  // R
  71.365979f,  // ambang NIS
  5,  // sampel berturut-turut
  1,  // decimation (tick dasar per langkah filter)
  1000,  // dt [ms]
  10,  // panjang jendela rata-rata NIS
  9.2478849f,  // ambang atas rata-rata NIS (bias / noise)
  0.00083254548f  // ambang bawah rata-rata NIS (sensor beku)
};
constexpr int LOAD_OFF_FLAT_N = 0;
constexpr float LOAD_OFF_TAU_S = 0.38821237f;
