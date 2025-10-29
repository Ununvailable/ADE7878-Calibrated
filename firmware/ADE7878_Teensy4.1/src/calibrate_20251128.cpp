/* Teensy 4.1 + ADE7878 (I2C) - 3-Phase Calibration System
 * 
 * Refactored architecture:
 * - Unified phase handling via PhaseConfig structure
 * - Centralized ZPSE encoding/decoding
 * - Single set of functions for all phases
 * - No code duplication
 * 
 * Hardware: 110V/60Hz, R1=1MΩ, R2=1kΩ, CT 2000:1, burden=200Ω
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "ADE7878_REG.h"

// ====== Register fallbacks ======
#ifndef PERIOD
#define PERIOD 0xE607
#endif

// ====== ZX bit masks ======
#ifndef ZXVA_BIT
#define ZXVA_BIT (1u << 0)
#endif
#ifndef ZXVB_BIT
#define ZXVB_BIT (1u << 1)
#endif
#ifndef ZXVC_BIT
#define ZXVC_BIT (1u << 2)
#endif
#ifndef ZXIA_BIT
#define ZXIA_BIT (1u << 3)
#endif
#ifndef ZXIB_BIT
#define ZXIB_BIT (1u << 4)
#endif
#ifndef ZXIC_BIT
#define ZXIC_BIT (1u << 5)
#endif

// ===== Hardware pins =====
#define RESET_PIN   22
#define PM0_PIN     20
#define PM1_PIN     25

// ===== I2C =====
#ifndef ADE_I2C_ADDR
#define ADE_I2C_ADDR  0x38
#endif

// ===== System constants =====
const int    LINE_FREQ_HZ   = 60;
const double R1_OHM         = 1'000'000.0;
const double R2_OHM         = 1'000.0;
const double CT_RATIO       = 2000.0;
const double BURDEN_OHM     = 200.0;

// PGA & ADC full-scale
#define  V_PGA              1
#define  I_PGA              1
#define  ADC_FS_V           (0.35355339059 / V_PGA)
#define  ADC_FS_I           (0.35355339059 / I_PGA)

// ADE internal constants
#define  PMAX               33516139
#define  FS_RMS_CODES       4191910u
#define  FS_POWER_SAMPLE    8000

// ===== Phase Configuration Structure =====
struct PhaseConfig {
  // Hardware registers
  uint16_t vrms_reg;
  uint16_t irms_reg;
  uint16_t watt_reg;
  uint16_t vgain_reg;
  uint16_t igain_reg;
  uint16_t ioffset_reg;
  uint16_t zxv_bit;
  uint16_t zxi_bit;
  
  // Calibration state
  double vrms_lsb;
  double irms_lsb;
  double watt_lsb;
  double last_vtest_vrms;
  double last_itest_arms;
  bool v_calibrated;
  bool i_calibrated;
  
  char name;
};

// Global phase array
PhaseConfig phases[3] = {
  // Phase A
  {AVRMS, AIRMS, AWATT, AVGAIN, AIGAIN, AIRMSOS,
   ZXVA_BIT, ZXIA_BIT, 0, 0, 0, 110, 10, false, false, 'A'},
  // Phase B
  {BVRMS, BIRMS, BWATT, BVGAIN, BIGAIN, BIRMSOS,
   ZXVB_BIT, ZXIB_BIT, 0, 0, 0, 110, 10, false, false, 'B'},
  // Phase C
  {CVRMS, CIRMS, CWATT, CVGAIN, CIGAIN, CIRMSOS,
   ZXVC_BIT, ZXIC_BIT, 0, 0, 0, 110, 10, false, false, 'C'}
};

// ========= I2C Communication =========
static volatile uint32_t i2c_err_streak = 0;

static bool ade_read_raw(uint16_t reg, uint8_t *buf, uint8_t nbytes) {
  for (uint8_t t = 0; t <= 3; t++) {
    Wire.beginTransmission((uint8_t)ADE_I2C_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)reg);
    if (Wire.endTransmission(false) != 0) {
      i2c_err_streak++;
      continue;
    }
    if (Wire.requestFrom((uint8_t)ADE_I2C_ADDR, nbytes) == nbytes) {
      for (uint8_t i = 0; i < nbytes; i++) buf[i] = Wire.read();
      i2c_err_streak = 0;
      return true;
    }
    i2c_err_streak++;
  }
  return false;
}

static uint32_t ade_read_u32(uint16_t reg, uint8_t nbytes) {
  uint8_t b[4] = {0, 0, 0, 0};
  if (!ade_read_raw(reg, b + (4 - nbytes), nbytes)) return 0;
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | 
         ((uint32_t)b[2] << 8) | b[3];
}

static bool ade_write(uint16_t reg, uint32_t data, uint8_t nbytes) {
  if (nbytes < 1 || nbytes > 4) return false;
  
  // Detect ZPSE registers (gain/offset family)
  bool isZPSE = (reg >= 0x4380 && reg <= 0x4386) || 
                (reg >= 0x4387 && reg <= 0x438D);
  
  uint32_t tx32 = data;
  
  if (isZPSE) {
    // Sign-extend 24-bit value
    int32_t s24 = (int32_t)(data & 0x00FFFFFF);
    if (s24 & 0x00800000) s24 |= 0xFF000000;
    
    // Build ZPSE word: sign nibble in bits[27:24]
    uint32_t signNibble = (s24 < 0) ? 0x0F000000 : 0x00000000;
    tx32 = signNibble | (uint32_t)(s24 & 0x00FFFFFF);
    nbytes = 4;
  }
  
  // Use tx32 (formatted), not data
  uint8_t b[4] = {
    (uint8_t)(tx32 >> 24), (uint8_t)(tx32 >> 16),
    (uint8_t)(tx32 >> 8),  (uint8_t)(tx32)
  };
  
  Wire.beginTransmission((uint8_t)ADE_I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  for (uint8_t i = 4 - nbytes; i < 4; i++) Wire.write(b[i]);
  
  return (Wire.endTransmission() == 0);
}

// ========= ZPSE Helpers =========
static int32_t zpse_decode(uint32_t raw) {
  int32_t s24 = (int32_t)(raw & 0x00FFFFFF);
  if (s24 & 0x00800000) s24 |= 0xFF000000;
  return s24;
}

static void write_gain(uint16_t gain_reg, int32_t value) {
  ade_write(gain_reg, (uint32_t)value, 4);
}

static int32_t read_gain(uint16_t gain_reg) {
  return zpse_decode(ade_read_u32(gain_reg, 4));
}

// ========= 24-bit Read Helpers =========
static int32_t ade_read_s24_zspe32(uint16_t reg) {
  uint32_t u = ade_read_u32(reg, 3);
  if (u & 0x800000) u |= 0xFF000000;
  return (int32_t)u;
}

static uint32_t ade_read_u24(uint16_t reg) {
  uint8_t b[3] = {0, 0, 0};
  if (!ade_read_raw(reg, b, 3)) return 0;
  return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
}

static int32_t ade_read_s24(uint16_t reg) {
  uint32_t u = ade_read_u24(reg);
  if (u & 0x800000) u |= 0xFF000000;
  return (int32_t)u;
}

static inline uint16_t ade_read_u16(uint16_t reg) {
  return (uint16_t)(ade_read_u32(reg, 2) & 0xFFFF);
}

// ========= STATUS1/MASK1 Helpers =========
#ifndef MASK1
#define MASK1 0xE50B
#endif
#ifndef STATUS1
#define STATUS1 0xE503
#endif

static inline bool ade_set_mask1(uint16_t mask) {
  return ade_write(MASK1, mask, 2);
}

static inline uint16_t ade_read_status1() {
  return ade_read_u16(STATUS1);
}

static inline bool ade_clear_status1(uint16_t bits) {
  return ade_write(STATUS1, bits, 2);
}

static bool wait_zx(uint16_t zx_bit, uint16_t timeout_ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeout_ms) {
    uint16_t st = ade_read_status1();
    if (st & zx_bit) {
      ade_clear_status1(zx_bit);
      return true;
    }
    delay(1);
  }
  return false;
}

// ========= Unified RMS Averaging =========
static double ade_read_avg24RMS(uint16_t reg, uint16_t samples, 
                                 uint16_t delay_ms = 8) {
  double acc = 0.0;
  uint16_t ok = 0;
  for (uint16_t i = 0; i < samples; i++) {
    uint32_t v = ade_read_s24(reg);
    // Serial.println(v);
    acc += (double)v;
    ok++;
    if (delay_ms) delay(delay_ms);
  }
  return ok ? (acc / ok) : 0.0;
}

static double avg_RMS_via_zx(uint16_t rms_reg, uint16_t zx_bit,
                               uint16_t ntrigs = 64, uint16_t timeout_ms = 200) {
  ade_set_mask1(zx_bit);
  double acc = 0.0;
  uint16_t ok = 0;
  
  for (uint16_t i = 0; i < ntrigs; i++) {
    if (!wait_zx(zx_bit, timeout_ms)) continue;
    uint32_t v = ade_read_u24(rms_reg);
    acc += (double)v;
    ok++;
  }
  
  ade_set_mask1(0x0000);
  return ok ? (acc / ok) : 0.0;
}

// ========= Basic Config =========
static void ade_basic_config() {
  ade_write(CFMODE, 0x0088, 2);
  ade_write(LCYCMODE, 0x0F, 1);
  ade_write(RUN, 0x0001, 2);
  ade_set_mask1(0x0000);
  
  // Clear all phase offsets
  for (int i = 0; i < 3; i++) {
    write_gain(phases[i].ioffset_reg, 0);
  }
}

// ========= Frequency Reading =========
static double read_line_freq_hz(uint8_t samples = 8, uint16_t delay_ms = 10) {
  double acc = 0.0;
  uint8_t ok = 0;
  
  for (uint8_t i = 0; i < samples; i++) {
    uint32_t per = ade_read_u32(PERIOD, 2);
    if (per > 0 && per < 2000) {
      acc += (double)per;
      ok++;
    }
    if (delay_ms) delay(delay_ms);
  }
  
  if (!ok) return 0.0;
  double per_avg = acc / ok;
  return (double)FS_POWER_SAMPLE / per_avg;
}

// ========= Unified Voltage Tuning =========
static bool tune_VRMS_to(PhaseConfig &ph, double V_TARGET_Vrms,
                         double tolerance_frac = 0.01,
                         uint8_t avg_samples = 64,
                         uint16_t delay_ms = 10,
                         uint32_t max_iters = 2000) {
  if (ph.vrms_lsb <= 0.0) {
    Serial.print(F("[Phase ")); Serial.print(ph.name);
    Serial.println(F("] VRMS_LSB not ready"));
    return false;
  }

  int32_t gain = read_gain(ph.vgain_reg);

  for (uint8_t it = 0; it < max_iters; it++) {
    double vrms_raw = avg_RMS_via_zx(ph.vrms_reg, ph.zxv_bit, 48, 250);
    if (vrms_raw <= 0.0) {
      vrms_raw = ade_read_avg24RMS(ph.vrms_reg, avg_samples, delay_ms);
    }

    double V_meas = vrms_raw * ph.vrms_lsb;
    if (V_meas <= 1e-12) {
      Serial.println(F("  Measurement too small"));
      return false;
    }

    double err_frac = (V_meas - V_TARGET_Vrms) / V_TARGET_Vrms;
    
    Serial.print(F("  it= ")); Serial.print(it); Serial.print(F("; "));
    Serial.print(F("V_meas= ")); Serial.print(V_meas, 6); Serial.print(F(" V; "));
    Serial.print(F("err= ")); Serial.print(err_frac * 100.0, 4); Serial.print(F(" %; "));
    Serial.print(F("int32_t VGAIN= ")); Serial.print(gain);
    Serial.println(F(""));

    if (fabs(err_frac) < tolerance_frac) {
      Serial.println(F("  [DONE] Target reached"));
      return true;
    }

    double k = V_TARGET_Vrms / V_meas;
    int32_t delta = (int32_t)llround((k - 1.0) * 8388608.0);

    const int32_t STEP_CLAMP = 300000;
    if (delta > STEP_CLAMP) delta = STEP_CLAMP;
    if (delta < -STEP_CLAMP) delta = -STEP_CLAMP;

    gain += delta;
    write_gain(ph.vgain_reg, gain);
    delay(250);
  }

  Serial.println(F("  Max iterations reached"));
  return false;
}

// ========= Unified Current Tuning =========
static bool tune_IRMS_to(PhaseConfig &ph, double I_TARGET_Arms,
                         double tolerance_frac = 0.01,
                         uint32_t avg_samples = 64,
                         uint16_t delay_ms = 8,
                         uint32_t max_iters = 2000) {
  if (ph.irms_lsb <= 0.0) {
    Serial.print(F("[Phase ")); Serial.print(ph.name);
    Serial.println(F("] IRMS_LSB not ready"));
    return false;
  }

  int32_t gain = read_gain(ph.igain_reg);
  const int32_t MAX_24 = 8388607;
  const int32_t MIN_24 = -8388608;

  for (uint8_t it = 0; it < max_iters; it++) {
    double irms_raw = avg_RMS_via_zx(ph.irms_reg, ph.zxi_bit, 48, 250);
    if (irms_raw <= 0.0) {
      irms_raw = ade_read_avg24RMS(ph.irms_reg, avg_samples, delay_ms);
    }

    double I_meas = irms_raw * ph.irms_lsb;
    if (I_meas <= 1e-12) {
      Serial.println(F("  Measurement too small"));
      return false;
    }

    double err_frac = (I_meas - I_TARGET_Arms) / I_TARGET_Arms;

    Serial.print(F("  it= ")); Serial.print(it); Serial.print(F("; "));
    Serial.print(F("I_meas= ")); Serial.print(I_meas, 6); Serial.print(F(" A; "));
    Serial.print(F("err= ")); Serial.print(err_frac * 100.0, 4); Serial.print(F(" %; "));
    Serial.print(F("int32_t IGAIN= ")); Serial.print(gain);
    Serial.println(F(""));

    if (fabs(err_frac) < tolerance_frac) {
      Serial.println(F("  [DONE] Target reached"));
      return true;
    }

    // Adaptive step scaling
    double k = I_TARGET_Arms / I_meas;
    double abs_err = fabs(err_frac);

    double step_factor;
    if      (abs_err > 0.80) step_factor = 1.0;
    else if (abs_err > 0.50) step_factor = 0.6;
    else if (abs_err > 0.20) step_factor = 0.3;
    else if (abs_err > 0.10) step_factor = 0.15;
    else                     step_factor = 0.05;

    int32_t delta = (int32_t)llround((k - 1.0) * 8388608.0 * step_factor);

    // Dynamic clamp
    int32_t step_limit = (int32_t)llround(600000.0 * (1.0 + 4.0 * abs_err));
    if (delta > step_limit) delta = step_limit;
    if (delta < -step_limit) delta = -step_limit;

    gain += delta;
    if (gain > MAX_24) gain = MAX_24;
    if (gain < MIN_24) gain = MIN_24;

    write_gain(ph.igain_reg, gain);
    delay(300);
  }

  Serial.println(F("  Max iterations reached"));
  return false;
}

// ========= Unified Voltage Calibration =========
static void calibrate_V(PhaseConfig &ph, double V_TEST_Vrms) {
  Serial.print(F("\n=== Calibrate Voltage Phase "));
  Serial.print(ph.name);
  Serial.println(F(" ==="));

  ph.last_vtest_vrms = V_TEST_Vrms;

  const double V_at_ADC = (R2_OHM / (R1_OHM + R2_OHM)) * V_TEST_Vrms;
  const double percentFS_V = V_at_ADC / ADC_FS_V;

  if (percentFS_V <= 0.0 || percentFS_V > 0.98) {
    Serial.println(F("  WARNING: %FS_V outside safe range (0..0.98)"));
  }

  // Set VLEVEL (only Phase A typically used)
//   if (ph.name == 'A') {
//     const double V_fullscale = V_TEST_Vrms / max(1e-12, percentFS_V);
//     const uint32_t VLEVEL_val = (uint32_t)(491520.0 * V_TEST_Vrms / V_fullscale);
//     ade_write(VLEVEL, VLEVEL_val, 4);
//     Serial.print(F("  VLEVEL=0x")); Serial.println(VLEVEL_val, HEX);
//     delay(300);
//   }

  // Set VLEVEL (all phases)
  const double V_fullscale = V_TEST_Vrms / max(1e-12, percentFS_V);
  const uint32_t VLEVEL_val = (uint32_t)(491520.0 * V_TEST_Vrms / V_fullscale);
  ade_write(VLEVEL, VLEVEL_val, 4);
  Serial.print(F("  VLEVEL=0x")); Serial.println(VLEVEL_val, HEX);
  delay(300);

  // Reset gain
  write_gain(ph.vgain_reg, 0);
  delay(100);
//   Serial.println(ade_read_s24(ph.vgain_reg));

  // Measure actual codes
  double vrms_raw = avg_RMS_via_zx(ph.vrms_reg, ph.zxv_bit, 64, 250);
  if (vrms_raw <= 0.0) {
    vrms_raw = ade_read_avg24RMS(ph.vrms_reg, 256, 12);
  }

  Serial.print(F("  Raw VRMS codes: ")); Serial.println(vrms_raw, 1);

  // Calculate LSB from actual measurement
  ph.vrms_lsb = V_TEST_Vrms / max(1.0, vrms_raw);

  // Calculate required gain
  double target_codes = (double)FS_RMS_CODES * percentFS_V;
  int32_t gain_val = (int32_t)llround(((target_codes / vrms_raw) - 1.0) * 8388608.0);
  write_gain(ph.vgain_reg, gain_val);

  Serial.print(F("  percentFS_V=")); Serial.println(percentFS_V, 6);
  Serial.print(F("  VGAIN=")); Serial.println(gain_val);
  Serial.print(F("  VRMS_LSB=")); Serial.println(ph.vrms_lsb, 10);

  delay(300);
//   double V_check = (double)ade_read_u24(ph.vrms_reg) * ph.vrms_lsb;
  double V_check = (double)ade_read_s24(ph.vrms_reg) * ph.vrms_lsb;
  Serial.print(F("  Initial check: ")); Serial.print(V_check, 3);
  Serial.println(F(" V"));

  // Fine-tune
  Serial.println(F(">> Fine-tuning VGAIN (target error < 1%)"));
  tune_VRMS_to(ph, V_TEST_Vrms);

  double V_final = (double)ade_read_s24(ph.vrms_reg) * ph.vrms_lsb;
  Serial.print(F("Final: ")); Serial.print(V_final, 6); Serial.println(F(" V"));

  ph.v_calibrated = true;
}

// ========= Unified Current Calibration =========
static void calibrate_I(PhaseConfig &ph, double I_TEST_Arms) {
  Serial.print(F("\n=== Calibrate Current Phase "));
  Serial.print(ph.name);
  Serial.println(F(" ==="));

  ph.last_itest_arms = I_TEST_Arms;

  const double I_sec = I_TEST_Arms / CT_RATIO;
  const double Iadc_Vrms = I_sec * BURDEN_OHM;
  const double percentFS_I = Iadc_Vrms / ADC_FS_I;

  Serial.print(F("  I_sec=")); Serial.print(I_sec * 1000, 3);
  Serial.println(F(" mA"));
  Serial.print(F("  I_adc=")); Serial.print(Iadc_Vrms, 4);
  Serial.println(F(" V"));
  Serial.print(F("  percentFS=")); Serial.print(percentFS_I * 100, 2);
  Serial.println(F("%"));

  if (percentFS_I <= 0.0 || percentFS_I > 0.98) {
    Serial.println(F("  WARNING: %FS_I outside safe range"));
  }

  // Reset gain
  write_gain(ph.igain_reg, 0);
  delay(100);

  // Measure actual codes
  double irms_raw = avg_RMS_via_zx(ph.irms_reg, ph.zxi_bit, 64, 250);
  if (irms_raw <= 0.0) {
    irms_raw = ade_read_avg24RMS(ph.irms_reg, 64, 8);
  }

  Serial.print(F("  Raw IRMS codes: ")); Serial.println(irms_raw, 1);

  // Calculate LSB from actual measurement
  ph.irms_lsb = I_TEST_Arms / max(1.0, irms_raw);

  // Calculate required gain
  double target_codes = (double)FS_RMS_CODES * percentFS_I;
  Serial.print(F("  Target codes: ")); Serial.println(target_codes, 1);
  
  int32_t gain_val = (int32_t)llround(((target_codes / irms_raw) - 1.0) * 8388608.0);
  write_gain(ph.igain_reg, gain_val);

  Serial.print(F("  percentFS_I=")); Serial.println(percentFS_I, 6);
  Serial.print(F("  IGAIN=")); Serial.println(gain_val);
  Serial.print(F("  IRMS_LSB=")); Serial.println(ph.irms_lsb, 10);

  delay(300);
//   double I_check = (double)ade_read_u24(ph.irms_reg) * ph.irms_lsb;
  double I_check = (double)ade_read_u24(ph.irms_reg) * ph.irms_lsb;
  Serial.print(F("  Initial check: ")); Serial.print(I_check, 6);
  Serial.println(F(" A"));

  // Fine-tune
  Serial.println(F(">> Fine-tuning IGAIN (target error < 1%)"));
  tune_IRMS_to(ph, I_TEST_Arms);

  double I_final = (double)ade_read_u24(ph.irms_reg) * ph.irms_lsb;
  Serial.print(F("Final: ")); Serial.print(I_final, 6); Serial.println(F(" A"));

  ph.i_calibrated = true;
}

// ========= Update WATT_LSB =========
static void update_WATT_LSB(PhaseConfig &ph) {
  if (ph.vrms_lsb <= 0 || ph.irms_lsb <= 0) {
    Serial.print(F("[Phase ")); Serial.print(ph.name);
    Serial.println(F("] Cannot compute WATT_LSB - calibrate V & I first"));
    return;
  }

  const double V_at_ADC = (R2_OHM / (R1_OHM + R2_OHM)) * ph.last_vtest_vrms;
  const double percentFS_V = V_at_ADC / ADC_FS_V;

  const double I_sec = ph.last_itest_arms / CT_RATIO;
  const double Iadc_Vrms = I_sec * BURDEN_OHM;
  const double percentFS_I = Iadc_Vrms / ADC_FS_I;

  const double scaleP = percentFS_V * percentFS_I;
  if (scaleP <= 0) {
    ph.watt_lsb = 0;
    return;
  }

  const double PF = 1.0;  // Assume unity power factor for calibration
  ph.watt_lsb = (ph.last_vtest_vrms * ph.last_itest_arms * PF) / (PMAX * scaleP);

  Serial.print(F("[Phase ")); Serial.print(ph.name);
  Serial.print(F("] WATT_LSB=")); Serial.println(ph.watt_lsb, 10);
}

// ========= Print Phase Reading =========
static void print_phase_reading(PhaseConfig &ph) {
  if (!ph.v_calibrated || !ph.i_calibrated) {
    Serial.print(F("[Phase ")); Serial.print(ph.name);
    Serial.println(F("] Not calibrated"));
    return;
  }

  double V = (double)ade_read_u24(ph.vrms_reg) * ph.vrms_lsb;
  double I = (double)ade_read_u24(ph.irms_reg) * ph.irms_lsb;
  double P = (double)ade_read_s24(ph.watt_reg) * ph.watt_lsb;

  Serial.print(F("[Phase ")); Serial.print(ph.name); Serial.print(F("] "));
  Serial.print(V, 2); Serial.print(F("V  "));
  Serial.print(I, 3); Serial.print(F("A  "));
  Serial.print(P, 2); Serial.println(F("W"));
}

// ========= Phase Selection Helper =========
static int select_phase() {
  Serial.print(F("Select phase [A/B/C]: "));
  uint32_t t0 = millis();
  
  while (millis() - t0 < 5000) {
    if (Serial.available()) {
      int c = Serial.read();
      Serial.println((char)c);
      if (c == 'A' || c == 'a') return 0;
      if (c == 'B' || c == 'b') return 1;
      if (c == 'C' || c == 'c') return 2;
    }
  }
  
  Serial.println(F("Timeout"));
  return -1;
}

// ========= Calibration Status =========
static void print_calibration_status() {
  Serial.println(F("\n=== Calibration Status ==="));
  for (int i = 0; i < 3; i++) {
    Serial.print(F("Phase ")); Serial.print(phases[i].name); Serial.print(F(": "));
    Serial.print(F("V=")); Serial.print(phases[i].v_calibrated ? "OK" : "--");
    Serial.print(F("  I=")); Serial.print(phases[i].i_calibrated ? "OK" : "--");
    Serial.println();
  }
}

// ========= Help Menu =========
static void print_help() {
  Serial.println(F("\n=== 3-PHASE CALIBRATION MENU ==="));
  Serial.println(F("[V] Calibrate Voltage (select phase)"));
  Serial.println(F("[I] Calibrate Current (select phase)"));
  Serial.println(F("[R] Read all phases"));
  Serial.println(F("[F] Read Frequency"));
  Serial.println(F("[S] Show calibration status"));
  Serial.println(F("[?] Help"));
  Serial.println();
}

// ========= Setup =========
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}  // Wait up to 3s for serial

  pinMode(RESET_PIN, OUTPUT);
  pinMode(PM0_PIN, INPUT);
  pinMode(PM1_PIN, INPUT);

  // Reset ADE7878 (active-low)
  digitalWrite(RESET_PIN, HIGH);
  delay(2);
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);

  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C
  delay(50);

  ade_basic_config();

  // Check version
  uint32_t ver = ade_read_u32(VERSION, 2);
  Serial.print(F("\nADE7878 VERSION=0x"));
  Serial.println(ver, HEX);

  if (ver == 0 || ver == 0xFFFF) {
    Serial.println(F("ERROR: Cannot communicate with ADE7878!"));
    Serial.println(F("Check I2C connections and address."));
  }

  Serial.println(F("\n>>> 3-PHASE CALIBRATION SYSTEM READY"));
  Serial.println(F(">>> Hardware: 110V/60Hz, CT 2000:1, Burden 200Ω"));
  print_help();
}

// ========= Main Loop =========
void loop() {
  if (Serial.available()) {
    int c = Serial.read();

    // ===== Voltage Calibration =====
    if (c == 'V' || c == 'v') {
      int phase_idx = select_phase();
      if (phase_idx < 0) return;

    //   Serial.print(F("Enter Vpp (default ~311.127 Vpp for 110Vrms): "));
      Serial.print(F("Enter Vrms: "));
      uint32_t t0 = millis();
      String s = "";
      
      while (millis() - t0 < 5000) {
        if (Serial.available()) {
          char ch = Serial.read();
          if (ch == '\n' || ch == '\r') break;
          if (isDigit(ch) || ch == '.') s += ch;
        }
      }

    //   double Vpp = s.length() ? s.toFloat() : (2.0 * 1.41421356237 * 110.0);
    //   double Vrms = Vpp / (2.0 * 1.41421356237);

      double Vrms = s.length() ? s.toFloat() : (2.0 * 1.41421356237 * 110.0);
      
    //   Serial.print(F("\n→ Vpp=")); Serial.print(Vpp, 3);
    //   Serial.print(F(" Vpp ~ Vrms=")); Serial.print(Vrms, 3);
    //   Serial.println(F(" V"));
      Serial.print(F("\n→ Vrms= ")); Serial.print(Vrms, 3);
      Serial.println(F(" V"));

      calibrate_V(phases[phase_idx], Vrms);
      update_WATT_LSB(phases[phase_idx]);
      print_help();
    }

    // ===== Current Calibration =====
    else if (c == 'I' || c == 'i') {
      int phase_idx = select_phase();
      if (phase_idx < 0) return;

      Serial.print(F("Enter I_TEST (Arms, default 10A): "));
      uint32_t t0 = millis();
      String s = "";
      
      while (millis() - t0 < 5000) {
        if (Serial.available()) {
          char ch = Serial.read();
          if (ch == '\n' || ch == '\r') break;
          if (isDigit(ch) || ch == '.') s += ch;
        }
      }

      double I_TEST = s.length() ? s.toFloat() : 10.0;
      Serial.print(F("\n→ I_TEST=")); Serial.print(I_TEST, 3);
      Serial.println(F(" A"));

      calibrate_I(phases[phase_idx], I_TEST);
      update_WATT_LSB(phases[phase_idx]);
      print_help();
    }

    // ===== Read All Phases =====
    else if (c == 'R' || c == 'r') {
      Serial.println(F("\n=== Phase Readings ==="));
      for (int i = 0; i < 3; i++) {
        print_phase_reading(phases[i]);
      }
      double f = read_line_freq_hz();
      Serial.print(F("Frequency: ")); Serial.print(f, 2);
      Serial.println(F(" Hz\n"));
    }

    // ===== Read Frequency Only =====
    else if (c == 'F' || c == 'f') {
      double f = read_line_freq_hz();
      Serial.print(F("Line frequency: ")); Serial.print(f, 3);
      Serial.println(F(" Hz"));
    }

    // ===== Show Calibration Status =====
    else if (c == 'S' || c == 's') {
      print_calibration_status();
    }

    // ===== Help =====
    else if (c == '?') {
      print_help();
    }
  }

  // Auto-print every 2 seconds if any phase is calibrated
  static uint32_t last_print = 0;
  if (millis() - last_print > 2000) {
    last_print = millis();
    
    bool any_calibrated = false;
    for (int i = 0; i < 3; i++) {
      if (phases[i].v_calibrated && phases[i].i_calibrated) {
        any_calibrated = true;
        break;
      }
    }

    if (any_calibrated) {
      Serial.println(F("\n--- Auto Reading ---"));
      for (int i = 0; i < 3; i++) {
        if (phases[i].v_calibrated && phases[i].i_calibrated) {
          print_phase_reading(phases[i]);
        }
      }
      double f = read_line_freq_hz();
      Serial.print(F("f=")); Serial.print(f, 2); Serial.println(F(" Hz"));
    }
  }
}