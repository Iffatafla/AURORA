// Dibuat dari pv_static.py (3 Nov 2026) - model PV statis
// P_pv_exp = PV_K * lux_amb + PV_C ;  S = PV_S0SQ + PV_SRSQ * (PV_K * lux_amb)^2 ;  NIS = (P_pv - P_pv_exp)^2 / S
// aktif bila: lux_amb >= 2000, bukan scc_hold, bukan sapuan MPPT (V_pv >= 18 V & I_pv < 0.2 A, maks 10 s)
// data: train=day_A_2nov  val=day_A2_3nov  test=day_B_3nov  fault=pv_C_3nov
#pragma once
constexpr float PV_K      = 3.493892e-04f;   // W per lux
constexpr float PV_C      = -0.8832f;        // W
constexpr float PV_S0SQ   = 1.02791f;        // W^2
constexpr float PV_SRSQ   = 0.002310f;       // (relatif)^2
constexpr float PV_NIS_THR = 6.635f;         // chi2(1) 99%, diverifikasi 0 alarm di val
constexpr int   PV_CONSEC  = 5;
constexpr float PV_LUX_MIN = 2000.0f;
