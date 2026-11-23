// Dibuat oleh kf_identify.py - jangan diedit manual
// subsistem : lamp_light
// z = lux_lamp ; u = [duty,lux_amb] ; dt = 1.000 s ; decimate = 1
// aktif bila: duty>=0.05   <- firmware HARUS memakai kondisi yang sama
// sampel dilewati bila: (tidak ada) = 1
// cek beku : lux_lamp ; N = 10 tick identik
// data      : train=../final/data/train/night_Epre_3nov.csv val=../final/data/val/night_F1normal_2nov.csv test=../final/data/test/night_Esim_3nov.csv
// alarm palsu set test: 2.791 per jam
#pragma once
#include <stdint.h>
#include "kf_core.h"

constexpr int LAMP_LIGHT_NU = 2;
constexpr KFParams<2> LAMP_LIGHT = {
  0.60178895f,  // A
  { 405.2392f, 0.0f },  // B (satu per input)
  17.816102f,  // C
  673.74567f,  // Q
  0.0015977028f,  // R
  84.493186f,  // ambang NIS
  5,  // sampel berturut-turut
  1,  // decimation (tick dasar per langkah filter)
  1000,  // dt [ms]
  10,  // panjang jendela rata-rata NIS
  9.1409433f,  // ambang atas rata-rata NIS (bias / noise)
  0.0014769958f  // ambang bawah rata-rata NIS (sensor beku)
};
constexpr int LAMP_LIGHT_FLAT_N = 10;
constexpr float LAMP_LIGHT_TAU_S = 1.9690913f;
