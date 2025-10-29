/* Teensy 4.1 + ADE7878 (I2C) - 3-Phase Streaming System
 * 
 * REFACTORED with fixes:
 * - Proper ZPSE encoding/decoding for gain registers
 * - Correct zero-crossing synchronization per phase
 * - Per-phase calibration constants
 * - Unified phase handling structure
 * 
 * Features:
 * - Stream JSON @ 120 Hz over UDP when online
 * - Log to SD card when offline
 * - Real-time timestamps via TIME command
 * - ACK-based keepalive
 */

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <TimeLib.h>
#include "ADE7878_REG.h"

// =================== HARDWARE CONFIG ===================
#define RESET_PIN       22
#define PM0_PIN         20
#define PM1_PIN         25

#define ADE_I2C_ADDR    0x38
#define I2C_SPEED_HZ    400000

// =================== NETWORK CONFIG ===================
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress localIp(169, 254, 153, 177);
const uint16_t LOCAL_PORT = 6000;

EthernetUDP Udp;
IPAddress remoteIp;
uint16_t remotePort = 0;

const uint32_t ACK_TIMEOUT_MS = 1000;
uint32_t lastAckMs = 0;
uint32_t lastSentSeq = 0;

// =================== SD CARD CONFIG ===================
File logFile;
bool sdReady = false;
const uint16_t SD_FLUSH_EVERY = 60;
uint16_t sdLinesSinceFlush = 0;

// =================== STREAMING CONFIG ===================
#define STREAM_HZ           120
#define JSON_BUF_SZ         1024
#define READ_EXTRAS_EVERY   10

// =================== REGISTER FALLBACKS ===================
#ifndef PERIOD
#define PERIOD 0x0E6E
#endif

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

// =================== PHASE CONFIGURATION ===================
struct PhaseCalibration {
  // Register addresses
  uint16_t vrms_reg;
  uint16_t irms_reg;
  uint16_t watt_reg;
  uint16_t var_reg;
  uint16_t va_reg;
  uint16_t vgain_reg;
  uint16_t igain_reg;
  uint16_t voffset_reg;
  uint16_t ioffset_reg;
  uint16_t zxi_bit;
  
  // Calibration constants (set these from your calibration code)
  int32_t vgain_cal;
  int32_t igain_cal;
  int32_t voffset_cal;
  int32_t ioffset_cal;
  
  double vrms_lsb;
  double irms_lsb;
  double watt_lsb;
  
  char name;
};

// TODO: Replace these calibration values with YOUR actual calibrated values
PhaseCalibration phases[3] = {
  // Phase A
  {
    AVRMS, AIRMS, AWATT, AVAR, AVA,
    AVGAIN, AIGAIN, AVRMSOS, AIRMSOS, ZXIA_BIT,
    119751,    // vgain_cal - REPLACE WITH YOUR VALUE
    -286699,   // igain_cal - REPLACE WITH YOUR VALUE
    0,         // voffset_cal - REPLACE WITH YOUR VALUE
    0,         // ioffset_cal - REPLACE WITH YOUR VALUE
    8.47513E-05,   // vrms_lsb - REPLACE WITH YOUR VALUE
    0.0000008433,  // irms_lsb - REPLACE WITH YOUR VALUE
    0.02388575,    // watt_lsb - REPLACE WITH YOUR VALUE
    'A'
  },
  // Phase B
  {
    BVRMS, BIRMS, BWATT, BVAR, BVA,
    BVGAIN, BIGAIN, BVRMSOS, BIRMSOS, ZXIB_BIT,
    248334,    // vgain_cal - REPLACE WITH YOUR VALUE
    532840,   // igain_cal - REPLACE WITH YOUR VALUE
    0,         // voffset_cal - REPLACE WITH YOUR VALUE
    0,         // ioffset_cal - REPLACE WITH YOUR VALUE
    8.47513E-05,   // vrms_lsb - REPLACE WITH YOUR VALUE
    0.0000008433,  // irms_lsb - REPLACE WITH YOUR VALUE
    0.02388575,    // watt_lsb - REPLACE WITH YOUR VALUE
    'B'
  },
  // Phase C
  {
    CVRMS, CIRMS, CWATT, CVAR, CVA,
    CVGAIN, CIGAIN, CVRMSOS, CIRMSOS, ZXIC_BIT,  // ✓ Correct ZX bit
    248334,    // vgain_cal - REPLACE WITH YOUR VALUE
    -205997,   // igain_cal - REPLACE WITH YOUR VALUE
    0,         // voffset_cal - REPLACE WITH YOUR VALUE
    0,         // ioffset_cal - REPLACE WITH YOUR VALUE
    8.47513E-05,   // vrms_lsb - REPLACE WITH YOUR VALUE
    0.0000008433,  // irms_lsb - REPLACE WITH YOUR VALUE
    0.02388575,    // watt_lsb - REPLACE WITH YOUR VALUE
    'C'
  }
};

const uint32_t VLEVEL_CAL = 0x1720AF;

// =================== I2C ERROR HANDLING ===================
volatile uint8_t i2c_err_streak = 0;
const uint8_t I2C_ERR_LIMIT = 3;
const uint8_t I2C_RETRY = 2;

// =================== NETWORK STATE ===================
enum class NetState { OFFLINE, ONLINE };
NetState netState = NetState::OFFLINE;

// =================== I2C LOW-LEVEL ===================
static bool ade_read_raw(uint16_t reg, uint8_t *buf, uint8_t nbytes) {
  for (uint8_t t = 0; t <= I2C_RETRY; t++) {
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

static bool ade_write(uint16_t reg, uint32_t data, uint8_t nbytes) {
  if (nbytes < 1 || nbytes > 4) return false;
  
  // Detect ZPSE registers (gain/offset family)
  bool isZPSE = (reg >= 0x4380 && reg <= 0x4386) || 
                (reg >= 0x4387 && reg <= 0x438D);
  
  uint32_t tx32 = data;
  
  if (isZPSE && nbytes == 4) {
    // Sign-extend 24-bit value to 32-bit
    int32_t s24 = (int32_t)(data & 0x00FFFFFF);
    if (s24 & 0x00800000) s24 |= 0xFF000000;
    
    // Build ZPSE word: sign nibble in bits[27:24]
    uint32_t signNibble = (s24 < 0) ? 0x0F000000 : 0x00000000;
    tx32 = signNibble | (uint32_t)(s24 & 0x00FFFFFF);
  }
  
  // Use tx32 (formatted), not data
  uint8_t b[4] = {
    (uint8_t)(tx32 >> 24), (uint8_t)(tx32 >> 16),
    (uint8_t)(tx32 >> 8),  (uint8_t)(tx32)
  };
  
  for (uint8_t t = 0; t <= I2C_RETRY; t++) {
    Wire.beginTransmission((uint8_t)ADE_I2C_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)reg);
    for (uint8_t i = 4 - nbytes; i < 4; i++) Wire.write(b[i]);
    if (Wire.endTransmission() == 0) {
      i2c_err_streak = 0;
      return true;
    }
    i2c_err_streak++;
  }
  return false;
}

// =================== ZPSE HELPERS ===================
static int32_t zpse_decode(uint32_t raw) {
  int32_t s24 = (int32_t)(raw & 0x00FFFFFF);
  if (s24 & 0x00800000) s24 |= 0xFF000000;
  return s24;
}

static int32_t sign_extend_24(int32_t v) {
  if (v & 0x800000) v |= 0xFF000000;
  return v;
}

// =================== ADE CONFIGURATION ===================
static void ade_reset_hw() {
  digitalWrite(RESET_PIN, HIGH);
  delay(2);
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
}

static void ade_basic_config() {
  ade_write(CFMODE, 0x0088, 2);
  ade_write(LCYCMODE, 0x0F, 1);
  ade_write(RUN, 0x0001, 2);
}

static void ade_apply_calibration() {
  // Set voltage level (shared for all phases typically)
  ade_write(VLEVEL, VLEVEL_CAL, 4);
  delay(50);
  
  // Apply per-phase calibration
  for (int i = 0; i < 3; i++) {
    PhaseCalibration &ph = phases[i];
    
    // Write gains (ZPSE formatted automatically by ade_write)
    ade_write(ph.vgain_reg, (uint32_t)ph.vgain_cal, 4);
    ade_write(ph.igain_reg, (uint32_t)ph.igain_cal, 4);
    
    // Write offsets (24-bit, not ZPSE)
    ade_write(ph.voffset_reg, (uint32_t)ph.voffset_cal, 3);
    ade_write(ph.ioffset_reg, (uint32_t)ph.ioffset_cal, 3);
    
    delay(10);
  }
  
  delay(100);
}

static void verify_calibration() {
  Serial.println(F("\n=== Calibration Verification ==="));
  
  for (int i = 0; i < 3; i++) {
    PhaseCalibration &ph = phases[i];
    
    int32_t vgain_rb = zpse_decode(ade_read_u32(ph.vgain_reg, 4));
    int32_t igain_rb = zpse_decode(ade_read_u32(ph.igain_reg, 4));
    uint32_t voff_rb = ade_read_u32(ph.voffset_reg, 3);
    uint32_t ioff_rb = ade_read_u32(ph.ioffset_reg, 3);
    
    Serial.print(F("Phase ")); Serial.print(ph.name); Serial.println(F(":"));
    Serial.print(F("  VGAIN: wrote=")); Serial.print(ph.vgain_cal);
    Serial.print(F(" read=")); Serial.println(vgain_rb);
    Serial.print(F("  IGAIN: wrote=")); Serial.print(ph.igain_cal);
    Serial.print(F(" read=")); Serial.println(igain_rb);
    
    if (vgain_rb != ph.vgain_cal) {
      Serial.println(F("  ⚠️  VGAIN MISMATCH!"));
    }
    if (igain_rb != ph.igain_cal) {
      Serial.println(F("  ⚠️  IGAIN MISMATCH!"));
    }
  }
  
  Serial.println();
}

// =================== FREQUENCY READING ===================
static inline double read_line_freq_hz_raw() {
  uint16_t period = (uint16_t)(ade_read_u32(PERIOD, 2) & 0xFFFF);
  if (period < 100 || period > 65000) return 0.0;
  return 256000.0 / (double)period;
}

static inline double read_line_freq_hz_ema() {
  double f = read_line_freq_hz_raw();
  static bool init = false;
  static double ema = 60.0;
  const double alpha = 0.25;
  
  if (f <= 0.0 || f < 45.0 || f > 65.0) return ema;
  
  if (!init) {
    ema = f;
    init = true;
  } else {
    ema = alpha * f + (1.0 - alpha) * ema;
  }
  
  return ema;
}

// =================== SD CARD MANAGEMENT ===================
String nextLogFilename() {
  unsigned long t = now();
  
  if (t > 1500000000UL) {
    // Use timestamp-based filename
    unsigned y = year(t);
    unsigned mo = month(t);
    unsigned d = day(t);
    unsigned h = hour(t);
    unsigned mi = minute(t);
    unsigned s = second(t);
    
    char name[48];
    snprintf(name, sizeof(name), "[%04u%02u%02u%uH%02uM%02uS].log",
             y, mo, d, h, mi, s);
    
    if (!SD.exists(name)) return String(name);
    
    // Add suffix if collision
    for (int k = 1; k <= 99; k++) {
      char name2[56];
      snprintf(name2, sizeof(name2), "[%04u%02u%02u%uH%02uM%02uS-%02d].log",
               y, mo, d, h, mi, s, k);
      if (!SD.exists(name2)) return String(name2);
    }
  }
  
  // Fallback: sequential numbering
  for (uint32_t i = 1; i <= 99999; i++) {
    char name[24];
    snprintf(name, sizeof(name), "log%05lu.txt", (unsigned long)i);
    if (!SD.exists(name)) return String(name);
  }
  
  return String("log99999.txt");
}

bool openLogFile() {
  if (!sdReady) {
    sdReady = SD.begin(BUILTIN_SDCARD);
    if (!sdReady) {
      Serial.println(F("[SD] Init failed"));
      return false;
    }
  }
  
  if (logFile) return true;
  
  String fn = nextLogFilename();
  logFile = SD.open(fn.c_str(), FILE_WRITE);
  
  if (!logFile) {
    Serial.println(F("[SD] File open failed"));
    return false;
  }
  
  Serial.print(F("[SD] Logging to: ")); Serial.println(fn);
  sdLinesSinceFlush = 0;
  return true;
}

void closeLogFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
    Serial.println(F("[SD] File closed"));
  }
}

// =================== NETWORK STATE MANAGEMENT ===================
void goOffline(const char* reason) {
  if (netState != NetState::OFFLINE) {
    Serial.print(F("[NET] -> OFFLINE: ")); Serial.println(reason);
    netState = NetState::OFFLINE;
    openLogFile();
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void goOnline() {
  if (netState != NetState::ONLINE) {
    Serial.println(F("[NET] -> ONLINE"));
    closeLogFile();
    netState = NetState::ONLINE;
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// =================== STREAMING ===================
static char json_buf[JSON_BUF_SZ];
uint32_t seqNo = 0;

void sendFrameOrLog(const char* s) {
  if (netState == NetState::ONLINE && remotePort != 0) {
    Udp.beginPacket(remoteIp, remotePort);
    Udp.write((const uint8_t*)s, strlen(s));
    Udp.write('\n');
    Udp.endPacket();
    lastSentSeq = seqNo;
  } else {
    if (openLogFile()) {
      logFile.println(s);
      if (++sdLinesSinceFlush >= SD_FLUSH_EVERY) {
        logFile.flush();
        sdLinesSinceFlush = 0;
      }
    }
  }
}

void processUdpControl() {
  int n = Udp.parsePacket();
  
  while (n > 0) {
    uint8_t buf[80];
    int m = Udp.read(buf, min(n, (int)sizeof(buf)));
    
    // HELLO command
    if (m >= 5 && !memcmp(buf, "HELLO", 5)) {
      remoteIp = Udp.remoteIP();
      remotePort = Udp.remotePort();
      Serial.print(F("[ETH] HELLO from "));
      Serial.print(remoteIp);
      Serial.print(F(":"));
      Serial.println(remotePort);
      goOnline();
      lastAckMs = millis();
    }
    
    // TIME <epoch_s> command
    else if (m >= 6 && !memcmp(buf, "TIME ", 5)) {
      buf[m] = 0;
      const char* p = (const char*)buf + 5;
      unsigned long epoch = strtoul(p, nullptr, 10);
      
      if (epoch > 1500000000UL) {
        setTime(epoch);
        Serial.print(F("[TIME] Set to epoch "));
        Serial.println(epoch);
      }
    }
    
    // ACK command (binary: 'ACK' + 4-byte LE sequence number)
    else if (m == 7 && buf[0] == 'A' && buf[1] == 'C' && buf[2] == 'K') {
      uint32_t ackSeq = (uint32_t)buf[3] | 
                        ((uint32_t)buf[4] << 8) | 
                        ((uint32_t)buf[5] << 16) | 
                        ((uint32_t)buf[6] << 24);
      
      if (ackSeq == lastSentSeq) {
        lastAckMs = millis();
      }
    }
    
    n = Udp.parsePacket();
  }
}

void stream_frame_json() {
  bool do_extras = (READ_EXTRAS_EVERY > 0) && (seqNo % READ_EXTRAS_EVERY == 0);
  
  // Read raw register values for all phases
  uint32_t vrms_raw[3], irms_raw[3];
  int32_t watt_raw[3];
  
  for (int i = 0; i < 3; i++) {
    vrms_raw[i] = ade_read_u32(phases[i].vrms_reg, 4);
    irms_raw[i] = ade_read_u32(phases[i].irms_reg, 4);
    watt_raw[i] = sign_extend_24(ade_read_u32(phases[i].watt_reg, 4));

    // vrms_raw[i] = ade_read_s24(phases[i].vrms_reg); 
    // irms_raw[i] = ade_read_s24(phases[i].irms_reg); 
    // watt_raw[i] = ade_read_s24(phases[i].watt_reg); 
  }
  
  double freq = read_line_freq_hz_ema();
  
  // Apply LSB scaling
  double V[3], I[3], P[3], S[3], PF[3];
  
  for (int i = 0; i < 3; i++) {
    V[i] = (double)vrms_raw[i] * phases[i].vrms_lsb;
    I[i] = (double)irms_raw[i] * phases[i].irms_lsb;
    P[i] = (double)watt_raw[i] * phases[i].watt_lsb;
    S[i] = V[i] * I[i];
    PF[i] = (S[i] > 1e-9) ? (P[i] / S[i]) : 0.0;
  }
  
  bool i2c_lock = (i2c_err_streak >= I2C_ERR_LIMIT);
  uint32_t t_ms = millis();
  unsigned long ts = now();
  
  // Build JSON
  if (!do_extras) {
    snprintf(json_buf, JSON_BUF_SZ,
      "{\"n\":%lu,\"t_ms\":%lu,\"ts\":%lu,\"i2c_lock\":%s,\"f\":%.2f,"
      "\"A\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
      "\"B\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
      "\"C\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f}}",
      (unsigned long)seqNo, (unsigned long)t_ms, (unsigned long)ts,
      i2c_lock ? "true" : "false", freq,
      V[0], I[0], P[0], S[0], PF[0],
      V[1], I[1], P[1], S[1], PF[1],
      V[2], I[2], P[2], S[2], PF[2]
    );
  } else {
    // Read extra parameters
    int32_t var_raw[3];
    uint32_t va_raw[3];
    
    for (int i = 0; i < 3; i++) {
    //   var_raw[i] = sign_extend_24(ade_read_u32(phases[i].var_reg, 4));
    //   va_raw[i] = ade_read_u32(phases[i].va_reg, 4);
        var_raw[i] = ade_read_s24(phases[i].var_reg); 
        va_raw[i] = ade_read_u24(phases[i].va_reg); 
    }
    
    double Q[3], S_true[3];
    for (int i = 0; i < 3; i++) {
      Q[i] = (double)var_raw[i] * phases[i].watt_lsb;
      S_true[i] = (double)va_raw[i] * phases[i].watt_lsb;
    }
    
    snprintf(json_buf, JSON_BUF_SZ,
      "{\"n\":%lu,\"t_ms\":%lu,\"ts\":%lu,\"i2c_lock\":%s,\"f\":%.2f,"
      "\"A\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
      "\"B\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
      "\"C\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
      "\"extras\":{\"A\":{\"S_true\":%.3f,\"Q\":%.3f},"
      "\"B\":{\"S_true\":%.3f,\"Q\":%.3f},"
      "\"C\":{\"S_true\":%.3f,\"Q\":%.3f}}}",
      (unsigned long)seqNo, (unsigned long)t_ms, (unsigned long)ts,
      i2c_lock ? "true" : "false", freq,
      V[0], I[0], P[0], S[0], PF[0],
      V[1], I[1], P[1], S[1], PF[1],
      V[2], I[2], P[2], S[2], PF[2],
      S_true[0], Q[0],
      S_true[1], Q[1],
      S_true[2], Q[2]
    );
  }
  
  // Send or log
  sendFrameOrLog(json_buf);
  
  // I2C error recovery
  if (i2c_lock) {
    digitalWrite(LED_BUILTIN, HIGH);
    ade_reset_hw();
    delay(20);
    ade_basic_config();
    ade_apply_calibration();
    i2c_err_streak = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  seqNo++;
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  
  pinMode(RESET_PIN, OUTPUT);
  pinMode(PM0_PIN, INPUT);
  pinMode(PM1_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize ADE7878
  ade_reset_hw();
  Wire.begin();
  Wire.setClock(I2C_SPEED_HZ);
  Wire.setTimeout(2);
  delay(50);
  
  ade_basic_config();
  ade_apply_calibration();
  
  // Verify calibration
  uint32_t ver = ade_read_u32(VERSION, 2);
  Serial.print(F("\n{\"boot\":\"ok\",\"version\":0x"));
  Serial.print(ver, HEX);
  Serial.println(F("}"));
  
  if (ver == 0 || ver == 0xFFFF) {
    Serial.println(F("⚠️  ERROR: Cannot communicate with ADE7878!"));
    Serial.println(F("Check I2C connections and address."));
  } else {
    verify_calibration();
  }
  
  // Initialize Ethernet
  Ethernet.begin(mac, localIp);
  delay(100);
  Udp.begin(LOCAL_PORT);
  
  Serial.print(F("[ETH] IP: "));
  Serial.println(Ethernet.localIP());
  Serial.println(F("[ETH] Send 'HELLO' from PC to start streaming"));
  Serial.println(F("[ETH] Send 'TIME <epoch>' to set clock"));
  
  // Initialize SD card
  sdReady = SD.begin(BUILTIN_SDCARD);
  if (sdReady) {
    Serial.println(F("[SD] Ready"));
  } else {
    Serial.println(F("[SD] Not ready (will retry on demand)"));
  }
  
  // Start in offline mode
  netState = NetState::OFFLINE;
  openLogFile();
  
  Serial.println(F("\n=== STREAMING SYSTEM READY ===\n"));
}

// =================== MAIN LOOP ===================
void loop() {
  static uint32_t last_frame = 0;
  const uint32_t frame_period_ms = 1000 / STREAM_HZ;
  
  // Process incoming UDP control messages
  processUdpControl();
  
  // Check link status
  if (Ethernet.linkStatus() != LinkON) {
    goOffline("link down");
  } else if (netState == NetState::ONLINE) {
    // ACK timeout watchdog
    if ((millis() - lastAckMs) > ACK_TIMEOUT_MS) {
      goOffline("ack timeout");
    }
  }
  
  // Stream frame at fixed rate
  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - last_frame) >= frame_period_ms) {
    last_frame = now_ms;
    stream_frame_json();
  }
}