// Dibuat oleh kf_identify.py - jangan diedit manual
// subsistem : lamp_light
// z = lux_lamp ; u = [duty] ; dt = 1.000 s ; decimate = 1
// aktif bila: duty>=0.05   <- firmware HARUS memakai kondisi yang sama
// sampel dilewati bila: (tidak ada) = 1
// cek beku : lux_lamp ; N = 60 tick identik
// data      : train=train/night_Dtr_4nov.csv val=val/night_Dva_4nov.csv test=test/night_D2_4nov.csv
// alarm palsu set test: 0.000 per jam
#pragma once
#include <stdint.h>
#include "kf_core.h"

constexpr int LAMP_LIGHT_NU = 1;
constexpr KFParams<1> LAMP_LIGHT = {
  0.883637f,  // A
  { 69.731343f },  // B (satu per input)
  6.0499365f,  // C
  263.70745f,  // Q
  0.00062534892f,  // R
  74.8f,  // ambang NIS
  10,  // sampel berturut-turut
  1,  // decimation (tick dasar per langkah filter)
  1000,  // dt [ms]
  10,  // panjang jendela rata-rata NIS
  7.1734561f,  // ambang atas rata-rata NIS (bias / noise)
  0.00015105157f  // ambang bawah rata-rata NIS (sensor beku)
};
constexpr int LAMP_LIGHT_FLAT_N = 60;
constexpr float LAMP_LIGHT_TAU_S = 8.0834905f;
