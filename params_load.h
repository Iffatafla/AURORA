// Dibuat oleh kf_identify.py - jangan diedit manual
// subsistem : load
// z = V_main,I_main ; u = [duty] ; dt = 1.000 s ; decimate = 1
// aktif bila: duty>=0.05   <- firmware HARUS memakai kondisi yang sama
// sampel dilewati bila: lora_tx = 1
// cek beku : I_main ; N = 30 tick identik
// data      : train=train/night_Dtr_4nov.csv val=val/night_Dva_4nov.csv test=test/night_D2_4nov.csv
// alarm palsu set test: 0.000 per jam
#pragma once
#include <stdint.h>
#include "kf_core.h"

constexpr int LOAD_NU = 1;
constexpr KFParams<1> LOAD = {
  0.71061632f,  // A
  { 8.4677272f },  // B (satu per input)
  0.49027491f,  // C
  0.9091183f,  // Q
  6.3441055e-07f,  // R
  24.1f,  // ambang NIS
  10,  // sampel berturut-turut
  1,  // decimation (tick dasar per langkah filter)
  1000,  // dt [ms]
  10,  // panjang jendela rata-rata NIS
  15.0f,  // ambang atas rata-rata NIS (dilonggarkan 4 Nov dari 4.52: transisi cepat PIR+closed-loop)
  0.13828302f  // ambang bawah rata-rata NIS (sensor beku)
};
constexpr int LOAD_FLAT_N = 30;
constexpr float LOAD_TAU_S = 2.9272065f;
