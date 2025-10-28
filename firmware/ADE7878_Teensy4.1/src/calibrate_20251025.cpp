/* Teensy 4.1 + ADE7878 (I2C)
 * Serial menu:
 *   V: Calibrate Voltage (nhập Vpp → đổi Vrms → calib AVRMS, dùng ZXVA)
 *   I: Calibrate Current (nhập Arms → calib BIRMS, dùng ZXIA)
 *   R: Read A-phase (V/A/W/Hz) using current LSB
 *   F: Read Frequency only (Hz)
 *
 * Default HW: 110V/60Hz, R1=1MΩ, R2=1kΩ, CT 100A/50mA (ratio=2000), burden=5Ω
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "ADE7878_REG.h"

// ====== PERIOD fallback (nếu header chưa define) ======
#ifndef PERIOD
#define PERIOD 0xE607
#endif

// ====== ZX bit masks (nếu header chưa có) ======
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

// ===== Pins =====
#define RESET_PIN   22
#define PM0_PIN     20
#define PM1_PIN     25

// ===== I2C =====
#ifndef ADE_I2C_ADDR
#define ADE_I2C_ADDR  0x38
#endif

// ===== System constants (sửa theo hệ thống) =====
const int    LINE_FREQ_HZ   = 60;          // 50/60 Hz
const double R1_OHM         = 1'000'000.0; // 1 MΩ
const double R2_OHM         = 1'000.0;     // 1 kΩ

// Current sensing HW
const double CT_RATIO       = 2000.0;      // 100A/50mA CT -> 2000:1
const double BURDEN_OHM     = 200;         // <-- sửa về 5Ω như ghi chú mặc định

// PGA & ADC full-scale (ADE7878: ±0.5 Vpeak ≈ 0.353553 Vrms tại ADC input)
#define  V_PGA              1
#define  I_PGA              1
#define  ADC_FS_V           (0.35355339059 / V_PGA) // Vrms full-scale kênh V @ ADC
#define  ADC_FS_I           (0.35355339059 / I_PGA) // Vrms full-scale kênh I @ ADC

// ===== ADE internal model constants (AN-1076) =====
#define  PMAX               33516139        // active power full-scale code (theo tài liệu)
#define  FS_RMS_CODES       4191910u        // RMS full-scale code (độc lập tần số)
#define  FS_POWER_SAMPLE    8000            // fS = 8 kHz

// ===== LSB & last calibration points =====
double VRMS_LSB = 0.0;     // V per LSB (AVRMS)
double IRMS_LSB = 0.0;     // A per LSB (BIRMS)
double WATT_LSB = 0.0;     // W per LSB (AWATT) — ước lượng theo mô hình

double last_VTEST_Vrms = 110.0; // V tại hiệu chuẩn
double last_ITEST_Arms = 10.0;  // A tại hiệu chuẩn
double last_PF         = 1.0;   // hệ số công suất giả định cho WATT_LSB

// ========= BỘ HÀM I2C DÙNG THEO MẪU BẠN ĐÃ CUNG CẤP =========
// (giữ nguyên cách đọc, bổ sung hàm ghi cùng “style”)

static volatile uint32_t i2c_err_streak = 0;

static bool ade_read_raw(uint16_t reg, uint8_t *buf, uint8_t nbytes) {
  for (uint8_t t=0; t<=3; t++) {
    Wire.beginTransmission((uint8_t)ADE_I2C_ADDR);
    Wire.write((uint8_t)(reg>>8)); Wire.write((uint8_t)reg);
    if (Wire.endTransmission(false)!=0) { i2c_err_streak++; continue; }
    if (Wire.requestFrom((uint8_t)ADE_I2C_ADDR, nbytes)==nbytes) {
      for (uint8_t i=0;i<nbytes;i++) buf[i]=Wire.read();
      i2c_err_streak=0; return true;
    }
    i2c_err_streak++;
  }
  return false;
}

static uint32_t ade_read_u32(uint16_t reg, uint8_t nbytes) {
  uint8_t b[4]={0,0,0,0};
  if(!ade_read_raw(reg,b+(4-nbytes),nbytes)) return 0;
  return ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
}

// Ghi theo cùng mẫu
static bool ade_write(uint16_t reg, uint32_t data, uint8_t nbytes) {
  if (nbytes < 1 || nbytes > 4) return false;
  // --- 1. Detect 32 ZPSE registers (gain/offset family) ---
  bool isZPSE =
    (reg >= 0x4380 && reg <= 0x4386) ||   // AIGAIN..NIGAIN
    (reg >= 0x4387 && reg <= 0x438D);     // AIRMSOS..NIRMSOS

  // --- 2. If ZPSE, sign-extend 24-bit value into 32-bit frame ---
  uint32_t tx32 = data;
  if (isZPSE) {
    int32_t s24 = (int32_t)(data & 0x00FFFFFFu);
    if (s24 & 0x00800000u) s24 |= 0xFF000000u;   // full sign-extend
    // Build ZPSE word: bits[31:28]=0, bits[27:24]=sign nibble
    uint32_t signNibble = (s24 < 0) ? 0x0F000000u : 0x00000000u;
    tx32 = signNibble | (uint32_t)(s24 & 0x00FFFFFFu);
    nbytes = 4;                                 // always transmit 4 bytes
  }
  uint8_t b[4] = {
    (uint8_t)(data >> 24), (uint8_t)(data >> 16),
    (uint8_t)(data >> 8),  (uint8_t)(data)
  };
  Wire.beginTransmission((uint8_t)ADE_I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  for (uint8_t i = 4 - nbytes; i < 4; i++) Wire.write(b[i]);
  return (Wire.endTransmission() == 0);
}

static inline uint32_t ade_zpse_write(int32_t s24) {
  // Clamp to ±2^23−1
  if (s24 > 0x7FFFFF) s24 = 0x7FFFFF;
  if (s24 < -0x800000) s24 = -0x800000;
  uint32_t u24 = (uint32_t)s24 & 0x00FFFFFF;
  uint32_t signNibble = (u24 & 0x00800000) ? 0x0F000000 : 0x00000000;
  return signNibble | u24;        // 32-bit word per 32 ZPSE spec
}


// Đọc 24-bit unsigned/signed
static uint32_t ade_read_u24(uint16_t reg) {
  uint8_t b[3] = {0,0,0};
  if (!ade_read_raw(reg, b, 3)) return 0;
  return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
}
static int32_t ade_read_s24(uint16_t reg) {
  uint32_t u = ade_read_u24(reg);
  if (u & 0x800000) u |= 0xFF000000;
  return (int32_t)u;
}

// Đọc 16/32-bit (ví dụ PERIOD 16-bit)
static inline uint16_t ade_read_u16(uint16_t reg) { return (uint16_t)(ade_read_u32(reg,2)&0xFFFF); }

// ========= MASK1 / STATUS1 helpers (ZX) =========
#ifndef MASK1
#define MASK1 0xE50B
#endif
#ifndef STATUS1
#define STATUS1 0xE503
#endif

static inline bool ade_set_mask1(uint16_t mask) { return ade_write(MASK1, mask, 2); }
static inline uint16_t ade_read_status1()       { return ade_read_u16(STATUS1); }
// STATUS1 là write-1-to-clear
static inline bool ade_clear_status1(uint16_t bits) { return ade_write(STATUS1, bits, 2); }

// Chờ một bit ZX lên 1
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

// ===== Helpers 24-bit average =====
static double readRMS24_avg(uint16_t reg, uint16_t samples, uint16_t delay_ms_between=8) {
  double acc = 0.0;
  uint16_t ok = 0;
  for (uint16_t i=0; i<samples; i++) {
    uint32_t v = ade_read_u24(reg);
    acc += (double)v; ok++;
    if (delay_ms_between) delay(delay_ms_between);
  }
  return ok ? (acc / ok) : 0.0;
}

// ===== Average RMS via Zero-Crossing (Voltage A / Current A) =====
static double avg_AVRMS_via_zx(uint16_t ntrigs = 64, uint16_t timeout_ms = 200) {
  ade_set_mask1(ZXVA_BIT);
  double acc = 0.0; uint16_t ok = 0;
  for (uint16_t i=0; i<ntrigs; i++) {
    if (!wait_zx(ZXVA_BIT, timeout_ms)) continue;
    uint32_t v = ade_read_u24(AVRMS);
    acc += (double)v; ok++;
  }
  ade_set_mask1(0x0000);
  return ok ? (acc / ok) : 0.0;
}

static double avg_AIRMS_via_zx(uint16_t ntrigs = 64, uint16_t timeout_ms = 200) {
  ade_set_mask1(ZXIA_BIT);
  double acc = 0.0; uint16_t ok = 0;
  for (uint16_t i=0; i<ntrigs; i++) {
    if (!wait_zx(ZXIA_BIT, timeout_ms)) continue;
    uint32_t v = ade_read_u24(AIRMS);
    acc += (double)v; ok++;
  }
  ade_set_mask1(0x0000);
  return ok ? (acc / ok) : 0.0;
}

static double avg_BIRMS_via_zx(uint16_t ntrigs = 64, uint16_t timeout_ms = 200) {
  ade_set_mask1(ZXIA_BIT);
  double acc = 0.0; uint16_t ok = 0;
  for (uint16_t i=0; i<ntrigs; i++) {
    if (!wait_zx(ZXIA_BIT, timeout_ms)) continue;
    uint32_t v = ade_read_u24(BIRMS);
    acc += (double)v; ok++;
  }
  ade_set_mask1(0x0000);
  return ok ? (acc / ok) : 0.0;
}

// ========= Basic config & helpers =========
static void ade_basic_config() {
  ade_write(CFMODE,   0x0088, 2); // bật CF pins nếu cần
  ade_write(LCYCMODE, 0x0F,   1);
  ade_write(RUN,      0x0001, 2);
  ade_set_mask1(0x0000);
}

static void print_help() {
  Serial.println();
  Serial.println(F("=== CALIB MENU ==="));
  Serial.println(F("[V] Calibrate Voltage (Nhập Vpp → đổi Vrms → AVRMS, dùng ZXVA)"));
  Serial.println(F("[I] Calibrate Current (Nhập Arms → BIRMS, dùng ZXIA)"));
  Serial.println(F("[R] Read A-phase (V/A/W/Hz)"));
  Serial.println(F("[F] Read Frequency only (Hz)"));
  Serial.println(F("[?] Help"));
  Serial.println();
}

// Frequency (Hz) from PERIOD (16-bit): f ≈ 8000 / PERIOD
static double read_line_freq_hz(uint8_t samples = 8, uint16_t delay_ms_between = 10) {
  double acc = 0.0;
  uint8_t ok = 0;
  for (uint8_t i = 0; i < samples; i++) {
    uint32_t per = ade_read_u32(PERIOD, 2);
    if (per > 0 && per < 2000) { acc += (double)per; ok++; }
    if (delay_ms_between) delay(delay_ms_between);
  }
  if (!ok) return 0.0;
  double per_avg = acc / ok;
  return (double)FS_POWER_SAMPLE / per_avg; // 8000 / PERIOD
}

// ========= Tuning (Voltage) =========
static bool tune_AVRMS_to(double V_TARGET_Vrms,
                          double tolerance_frac = 0.01, // 1%
                          uint8_t avg_samples = 64,
                          uint16_t delay_ms_between = 10,
                          uint8_t max_iters = 12)
{
  if (VRMS_LSB <= 0.0) {
    Serial.println(F("[tune_AVRMS_to] VRMS_LSB chua san sang. Calib V trước."));
    return false;
  }

  int32_t AVGAIN_now = (int32_t)ade_read_u32(AVGAIN, 4);

  for (uint8_t it = 0; it < max_iters; it++) {
    double avrms_raw = avg_AVRMS_via_zx(48, 250);
    if (avrms_raw <= 0.0) avrms_raw = readRMS24_avg(AVRMS, avg_samples, delay_ms_between);

    double V_meas    = avrms_raw * VRMS_LSB;
    if (V_meas <= 1e-12) {
      Serial.println(F("[tune_AVRMS_to] Đo AVRMS quá nhỏ/0."));
      return false;
    }

    double err_frac = (V_meas - V_TARGET_Vrms) / V_TARGET_Vrms;
    Serial.print(F("  it="));      Serial.print(it);
    Serial.print(F("  V_meas="));  Serial.print(V_meas, 6);
    Serial.print(F(" V  err="));   Serial.print(err_frac * 100.0, 4);
    Serial.println(F(" %"));

    if (fabs(err_frac) < tolerance_frac) {
      Serial.println(F("[tune_AVRMS_to] DONE: |error| dưới ngưỡng."));
      return true;
    }

    // xVGAIN = (Vref/Vmeas) - 1  (ghi ở Q.23)
    double  k   = V_TARGET_Vrms / V_meas;
    int32_t dAV = (int32_t)llround((k - 1.0) * 8388608.0); // 2^23

    const int32_t STEP_CLAMP = 300000;
    if (dAV >  STEP_CLAMP) dAV =  STEP_CLAMP;
    if (dAV < -STEP_CLAMP) dAV = -STEP_CLAMP;

    AVGAIN_now += dAV;
    ade_write(AVGAIN, (uint32_t)AVGAIN_now, 4);
    delay(250);
  }

  Serial.println(F("[tune_AVRMS_to] Hết số vòng lặp, chưa đạt sai số yêu cầu."));
  return false;
}

// ========= Tuning (Current) =========
// Adaptive AIRMS tuning with proportional gain adjustment
static bool tune_AIRMS_to(double I_TARGET_Arms,
                          double tolerance_frac = 0.01,
                          uint32_t avg_samples = 64,
                          uint16_t delay_ms_between = 8,
                          uint8_t max_iters = 24)    // allow more loops
{
  if (IRMS_LSB <= 0.0) {
    Serial.println(F("[tune_AIRMS_to] IRMS_LSB chưa sẵn sàng."));
    return false;
  }

  int32_t AIGAIN_now = (int32_t)ade_read_u32(AIGAIN, 4);
  const int32_t MAX_24 = (1 << 23) - 1;
  const int32_t MIN_24 = -(1 << 23);

  for (uint8_t it = 0; it < max_iters; it++) {

    double AIRMS_raw = avg_AIRMS_via_zx(48, 250);
    if (AIRMS_raw <= 0.0)
      AIRMS_raw = readRMS24_avg(AIRMS, avg_samples, delay_ms_between);

    double I_meas = AIRMS_raw * IRMS_LSB;
    if (I_meas <= 1e-12) {
      Serial.println(F("[tune_AIRMS_to] Đo AIRMS quá nhỏ/0."));
      return false;
    }

    double err_frac = (I_meas - I_TARGET_Arms) / I_TARGET_Arms;

    Serial.print(F("  it="));  Serial.print(it);
    Serial.print(F("  I_meas=")); Serial.print(I_meas, 6);
    Serial.print(F(" A  err="));  Serial.print(err_frac * 100.0, 4);
    Serial.println(F(" %"));

    if (fabs(err_frac) < tolerance_frac) {
      Serial.println(F("[tune_AIRMS_to] DONE: |error| dưới ngưỡng."));
      return true;
    }

    // ---------- Adaptive scaling ----------
    double k = I_TARGET_Arms / I_meas;     // ideal multiplicative factor
    double abs_err = fabs(err_frac);

    // Base proportional step
    double step_factor;
    if      (abs_err > 0.80) step_factor = 1.0;     // 100% of computed step
    else if (abs_err > 0.50) step_factor = 0.6;
    else if (abs_err > 0.20) step_factor = 0.3;
    else if (abs_err > 0.10) step_factor = 0.15;
    else                     step_factor = 0.05;    // fine tuning region

    // Convert to signed delta in Q23 domain
    int32_t dAI = (int32_t)llround((k - 1.0) * 8388608.0 * step_factor);

    // Dynamic clamp proportional to error magnitude
    int32_t step_limit = (int32_t)llround(600000.0 * (1.0 + 4.0 * abs_err));
    if (dAI >  step_limit) dAI =  step_limit;
    if (dAI < -step_limit) dAI = -step_limit;

    AIGAIN_now += dAI;
    if (AIGAIN_now > MAX_24) AIGAIN_now = MAX_24;
    if (AIGAIN_now < MIN_24) AIGAIN_now = MIN_24;

    // Build 32-bit ZPSE word and write
    uint32_t u24 = (uint32_t)AIGAIN_now & 0x00FFFFFFu;
    uint32_t signNibble = (AIGAIN_now < 0) ? 0x0F000000u : 0x00000000u;
    uint32_t tx32 = signNibble | u24;

    Serial.print(F("AIGAIN sent: "));
    Serial.println(AIGAIN_now);
    ade_write(AIGAIN, tx32, 4);
    delay(300);

    uint32_t rb32 = ade_read_u32(AIGAIN, 4);
    Serial.print(F("AIGAIN read: "));
    Serial.println(rb32);
  }

  Serial.println(F("[tune_AIRMS_to] Hết số vòng lặp, chưa đạt sai số yêu cầu."));
  return false;
}

static bool tune_BIRMS_to(double I_TARGET_Arms,
                          double tolerance_frac = 0.01, // 1%
                          uint8_t avg_samples = 64,
                          uint16_t delay_ms_between = 8,
                          uint8_t max_iters = 12)
{
  if (IRMS_LSB <= 0.0) {
    Serial.println(F("[tune_BIRMS_to] IRMS_LSB chua san sang. Calib I trước."));
    return false;
  }

  int32_t AIGAIN_now = (int32_t)ade_read_u32(AIGAIN, 4);

  for (uint8_t it = 0; it < max_iters; it++) {
    double BIRMS_raw = avg_BIRMS_via_zx(48, 250);
    if (BIRMS_raw <= 0.0) BIRMS_raw = readRMS24_avg(BIRMS, avg_samples, delay_ms_between);

    double I_meas    = BIRMS_raw * IRMS_LSB;
    if (I_meas <= 1e-12) {
      Serial.println(F("[tune_BIRMS_to] Đo BIRMS quá nhỏ/0."));
      return false;
    }

    double err_frac = (I_meas - I_TARGET_Arms) / I_TARGET_Arms;
    Serial.print(F("  it="));      Serial.print(it);
    Serial.print(F("  I_meas="));  Serial.print(I_meas, 6);
    Serial.print(F(" A  err="));   Serial.print(err_frac * 100.0, 4);
    Serial.println(F(" %"));

    if (fabs(err_frac) < tolerance_frac) {
      Serial.println(F("[tune_BIRMS_to] DONE: |error| dưới ngưỡng."));
      return true;
    }

    double  k   = I_TARGET_Arms / I_meas;
    int32_t dAI = (int32_t)llround((k - 1.0) * 8388608.0);

    const int32_t STEP_CLAMP = 300000;
    if (dAI >  STEP_CLAMP) dAI =  STEP_CLAMP;
    if (dAI < -STEP_CLAMP) dAI = -STEP_CLAMP;

    AIGAIN_now += dAI;
    ade_write(AIGAIN, (uint32_t)AIGAIN_now, 4);
    delay(250);
  }

  Serial.println(F("[tune_BIRMS_to] Hết số vòng lặp, chưa đạt sai số yêu cầu."));
  return false;
}

// ========= Calibrate Voltage =========
static void calibrate_V(double V_TEST_Vrms) {
  last_VTEST_Vrms = V_TEST_Vrms;

  // V tại ADC (Vrms) do cầu chia
  const double V_at_ADC     = (R2_OHM / (R1_OHM + R2_OHM)) * V_TEST_Vrms;
  const double percentFS_V  = V_at_ADC / ADC_FS_V; // so với FS Vrms tại ADC

  if (percentFS_V <= 0.0 || percentFS_V > 0.98) {
    Serial.println(F("[Calib V] CẢNH BÁO: %FS_V ngoài vùng an toàn (0..0.98). Kiểm tra divider."));
  }

  // VLEVEL theo AN-1076: VLEVEL = 491520 * (VN / VFS)
  // Ở đây suy ra VFS từ tỉ lệ phần trăm đang dùng:
  const double V_fullscale  = V_TEST_Vrms / max(1e-12, percentFS_V);
  const uint32_t VLEVEL_val = (uint32_t)((491520.0 * V_TEST_Vrms / max(1e-12, V_fullscale)));
  ade_write(VLEVEL, VLEVEL_val, 4);
  delay(300);

  // đọc AVRMS trung bình (đồng bộ ZX nếu có)
  double avrms_raw = avg_AVRMS_via_zx(64, 250);
  if (avrms_raw <= 0.0) avrms_raw = readRMS24_avg(AVRMS, 256, 12);

  // xVGAIN (Q.23) sao cho AVRMS * (1+VGAIN) đạt mức FS_RMS_CODES * percentFS_V
  const double target_codes = (double)FS_RMS_CODES * percentFS_V;
  const int32_t AVGAIN_val  = (int32_t)llround( ((target_codes / max(1.0, avrms_raw)) - 1.0) * 8388608.0 );
  ade_write(AVGAIN, AVGAIN_val, 4);
  delay(300);

  // Quy đổi LSB: Vrms = AVRMS * VRMS_LSB
  VRMS_LSB = V_TEST_Vrms / (FS_RMS_CODES * percentFS_V);

  Serial.println(F("\n[Calib V] DONE"));
  Serial.print(F("  V_TEST(Vrms)=")); Serial.print(V_TEST_Vrms, 4);
  Serial.print(F("  percentFS_V="));  Serial.println(percentFS_V, 6);
  Serial.print(F("  VLEVEL=0x"));     Serial.println(VLEVEL_val, HEX);
  Serial.print(F("  AVGAIN="));       Serial.println(AVGAIN_val);
  Serial.print(F("  VRMS_LSB="));     Serial.println(VRMS_LSB, 10);

  double V_now = (double)ade_read_u24(AVRMS) * VRMS_LSB;
  Serial.print(F("  Check AVRMS ≈ ")); Serial.print(V_now, 3); Serial.println(F(" V"));

  Serial.println(F(">> Tinh chỉnh AVGAIN (mục tiêu sai số < 1%)"));
  (void)tune_AVRMS_to(last_VTEST_Vrms, 0.01, 64, 10, 12);

  double V_after = (double)ade_read_u24(AVRMS) * VRMS_LSB;
  Serial.print(F("Kết thúc tune: AVRMS ≈ ")); Serial.print(V_after, 6); Serial.println(F(" V"));
}

// ========= Calibrate Current =========
static void calibrate_I(double I_TEST_Arms) {
  last_ITEST_Arms = I_TEST_Arms;

  // PRIMARY current voltage at burden (after CT scales it)
  // V_burden = (I_primary / CT_ratio) * R_burden
  const double I_sec        = I_TEST_Arms / CT_RATIO;  // 0.9A / 2000 = 0.45mA
  const double Iadc_Vrms    = I_sec * BURDEN_OHM;      // 0.45mA * 200Ω = 0.09V
  const double percentFS_I  = Iadc_Vrms / ADC_FS_I;    // 0.09V / 0.353V = 25.5%

  Serial.print(F(">>> DEBUG: I_sec=")); Serial.print(I_sec*1000, 3); Serial.println(F(" mA"));
  Serial.print(F(">>> DEBUG: V_adc=")); Serial.print(Iadc_Vrms, 4); Serial.println(F(" V"));
  Serial.print(F(">>> DEBUG: percentFS=")); Serial.print(percentFS_I*100, 2); Serial.println(F("%"));

  delay(300);

  double AIRMS_raw = avg_AIRMS_via_zx(64, 250);
  if (AIRMS_raw <= 0.0) AIRMS_raw = readRMS24_avg(AIRMS, 64, 8);

  Serial.print(F(">>> DEBUG: Raw AIRMS codes=")); Serial.println(AIRMS_raw, 1);

  const double target_codes = (double)FS_RMS_CODES * percentFS_I;
  Serial.print(F(">>> DEBUG: Target codes=")); Serial.println(target_codes, 1);

  // AIGAIN = (target/actual - 1) in Q23 format
  const int32_t AIGAIN_val = (int32_t)llround( ((target_codes / max(1.0, AIRMS_raw)) - 1.0) * 8388608.0 );
  // uint32_t u24 = (uint32_t)AIGAIN_val & 0x00FFFFFFu;
  // uint32_t signNibble = (AIGAIN_val < 0) ? 0x0F000000u : 0x00000000u;
  // uint32_t tx32 = signNibble | u24;
  ade_write(AIGAIN, AIGAIN_val, 4);

  // LSB converts codes to PRIMARY current (Amps)
  // IRMS_LSB = I_TEST_Arms / (FS_RMS_CODES * percentFS_I);
  IRMS_LSB = I_TEST_Arms / max(1.0, AIRMS_raw);  // LSB based on ACTUAL measurement

  Serial.println(F("\n[Calib I] DONE (sơ bộ)"));
  Serial.print(F("  I_TEST(Arms)=")); Serial.print(I_TEST_Arms, 4);
  Serial.print(F("  percentFS_I=")); Serial.println(percentFS_I, 6);
  Serial.print(F("  AIGAIN=")); Serial.println(AIGAIN_val);
  Serial.print(F("  IRMS_LSB=")); Serial.println(IRMS_LSB, 10);

  delay(300);
  double I_now = (double)ade_read_u24(AIRMS) * IRMS_LSB;
  Serial.print(F("  Check AIRMS ≈ ")); Serial.print(I_now, 6); Serial.println(F(" A"));

  Serial.println(F(">> Tinh chỉnh AIGAIN (mục tiêu sai số < 1%)"));
  bool okI = tune_AIRMS_to(last_ITEST_Arms, 0.01, 6400, 8, 200);

  double I_after = (double)ade_read_u24(AIRMS) * IRMS_LSB;
  Serial.print(F("Kết thúc tune: AIRMS ≈ ")); Serial.print(I_after, 6); Serial.println(F(" A"));
  if (!okI) {
    Serial.println(F("Cảnh báo: chưa đạt <1%. Có thể tăng max_iters/avg_samples hoặc giảm STEP_CLAMP."));
  }
}

// ========= Compute WATT_LSB (model-based nhẹ nhàng) =========
static void update_WATT_LSB() {
  if (VRMS_LSB <= 0 || IRMS_LSB <= 0) {
    Serial.println(F("\n[WATT LSB] Chưa đủ VRMS_LSB/IRMS_LSB. Calib V và I trước."));
    return;
  }

  const double V_at_ADC    = (R2_OHM / (R1_OHM + R2_OHM)) * last_VTEST_Vrms;
  const double percentFS_V = V_at_ADC / ADC_FS_V;

  const double I_sec       = last_ITEST_Arms / CT_RATIO;
  const double Iadc_Vrms   = I_sec * BURDEN_OHM;
  const double percentFS_I = Iadc_Vrms / ADC_FS_I;

  // W ≈ AWATT * WATT_LSB ; với công suất danh định: V*I*PF
  // PMAX tương ứng (FS_V * FS_I) * (1/2) * 2^19 (AN-1076).
  // Ước lượng LSB theo tỷ lệ phần FS đang dùng:
  const double scaleP = (percentFS_V * percentFS_I);
  if (scaleP <= 0) { WATT_LSB = 0; return; }

  WATT_LSB = (last_VTEST_Vrms * last_ITEST_Arms * last_PF) / (PMAX * scaleP);

  Serial.print(F("\n[WATT LSB] ")); Serial.println(WATT_LSB, 10);
  Serial.println(F("  (Khuyến nghị: nếu cần chính xác cao, calib năng lượng/CF theo AN-1076.)"));
}

// ========= Print A-phase reading =========
static void print_A_phase_reading() {
  double V_now = (double)ade_read_u24(AVRMS) * VRMS_LSB;
  double I_now = (double)ade_read_u24(BIRMS) * IRMS_LSB;
  double P_now = (double)ade_read_s24(AWATT) * WATT_LSB;
  double f_now = read_line_freq_hz();

  Serial.print(F("[A] V=")); Serial.print(V_now, 3); Serial.print(F(" V  "));
  Serial.print(F("I="));     Serial.print(I_now, 3); Serial.print(F(" A  "));
  Serial.print(F("P="));     Serial.print(P_now, 3); Serial.print(F(" W  "));
  Serial.print(F("f="));     Serial.print(f_now, 2); Serial.println(F(" Hz"));
}

// ========= Setup / Loop =========
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(RESET_PIN, OUTPUT);
  pinMode(PM0_PIN,   INPUT);
  pinMode(PM1_PIN,   INPUT);

  // Reset ADE (low-active)
  digitalWrite(RESET_PIN, HIGH); delay(2);
  digitalWrite(RESET_PIN, LOW);  delay(10);
  digitalWrite(RESET_PIN, HIGH); delay(10);

  Wire.begin();              // SDA=18, SCL=19 (Teensy 4.1)
  Wire.setClock(400000);     // 400 kHz
  delay(50);

  ade_basic_config();

  // Put into basic config later
  ade_write(AIRMSOS, 0, 4);
  Serial.print(F("Phase A offset: "));
  Serial.println(ade_read_u32(AIRMSOS,4));

  uint32_t ver = ade_read_u32(VERSION, 2);
  Serial.print(F("ADE7878 VERSION=0x")); Serial.println(ver, HEX);

  Serial.println(F("\n>>> HDSD:"));
  Serial.println(F("  - 'V': Calib Voltage (ZXVA)."));
  Serial.println(F("  - 'I': Calib Current (ZXIA)."));
  Serial.println(F("  - 'R': Đọc nhanh V/A/P/f theo LSB hiện tại."));
  Serial.println(F("  - 'F': Chỉ đọc tần số."));
  Serial.println(F("  - '?': Trợ giúp."));
  print_help();
}

void loop() {
  if (Serial.available()) {
    int c = Serial.read();

    if (c == 'V' || c == 'v') {
      Serial.println(F("\n== Calibrate Voltage =="));
      Serial.print(F("Nhập Vpp (mặc định ~311.127 Vpp ≈ 110 Vrms): "));
      uint32_t t0 = millis();
      String s = "";
      while (millis() - t0 < 5000) {
        if (Serial.available()) {
          char ch = Serial.read();
          if (ch == '\n' || ch == '\r') break;
          if (isDigit(ch) || ch == '.') s += ch;
        }
      }
      double Vpp  = s.length() ? s.toFloat() : (2.0 * 1.41421356237 * 110.0);
      double Vrms = Vpp / (2.0 * 1.41421356237);  // Vrms = Vpp /(2*sqrt(2))
      Serial.print(F("→ Vpp="));  Serial.print(Vpp, 3);
      Serial.print(F(" Vpp ~ Vrms=")); Serial.print(Vrms, 3); Serial.println(F(" V"));

      calibrate_V(Vrms);
      update_WATT_LSB();
      print_help();
    }

    else if (c == 'I' || c == 'i') {
      ade_write(AIGAIN, 0, 4);
      Serial.println(F("\n== Calibrate Current =="));
      Serial.print(F("Nhập I_TEST (Arms, mặc định 10): "));
      uint32_t t0 = millis();
      String s = "";
      while (millis() - t0 < 5000) {
        if (Serial.available()) {
          char ch = Serial.read();
          if (ch == '\n' || ch == '\r') break;
          if (isDigit(ch) || ch == '.') s += ch;
        }
      }
      double ITEST = s.length() ? s.toFloat() : 10.0;
      Serial.print(F("→ I_TEST=")); Serial.print(ITEST, 3); Serial.println(F(" A"));

      calibrate_I(ITEST);
      update_WATT_LSB();
      print_help();
    }

    else if (c == 'R' || c == 'r') {
      print_A_phase_reading();
    }

    else if (c == 'F' || c == 'f') {
      double f = read_line_freq_hz();
      Serial.print(F("Line frequency ≈ ")); Serial.print(f, 3); Serial.println(F(" Hz"));
    }

    else if (c == '?') {
      print_help();
    }
  }

  // Auto print every 1.5s if calibrated
  static uint32_t t = 0;
  if (millis() - t > 1500) {
    t = millis();
    if (VRMS_LSB > 0 && IRMS_LSB > 0 && WATT_LSB > 0) {
      print_A_phase_reading();
    }
  }
}
