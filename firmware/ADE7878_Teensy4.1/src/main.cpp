// code 20250905 done
/* Teensy 4.1 + ADE7878 (I2C)
 * STREAM JSON @120 Hz, Ethernet-only with SD fallback & real-time timestamps.
 * - ONLINE: stream UDP, mỗi frame có "ts" (epoch seconds, UTC)
 * - OFFLINE: ghi mỗi frame JSON xuống SD
 * - Control: HELLO / TIME <epoch_s> / ACK + seq
 */

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <TimeLib.h>     // for setTime()/now()
#include "ADE7878_REG.h"

// =================== ETHERNET ===================
byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};
// IPAddress localIp(192,168,200,177);
IPAddress localIp(169,254,153,177);
const uint16_t LOCAL_PORT = 6000;

EthernetUDP Udp;
IPAddress remoteIp;
uint16_t  remotePort = 0;

const uint32_t ACK_TIMEOUT_MS = 1000;  // nếu >1s không thấy ACK -> offline
uint32_t lastAckMs = 0;
uint32_t lastSentSeq = 0;

// =================== SD ===================
File logFile;
bool sdReady = false;
const uint16_t SD_FLUSH_EVERY = 60;
uint16_t sdLinesSinceFlush = 0;

String nextLogFilename() {
  // Nếu đồng hồ đã được set (epoch > ~2017)
  unsigned long t = now(); // từ TimeLib
  if (t > 1500000000UL) {
    // Lấy UTC components
    unsigned y = year(t);
    unsigned mo = month(t);
    unsigned d = day(t);
    unsigned h = hour(t);       // giờ không pad 0 theo ví dụ bạn đưa
    unsigned mi = minute(t);
    unsigned s = second(t);

    char name[48];
    // Mẫu: [YYYYMMDD<H>H<MM>M<SS>S].log  -> ví dụ [202509054H25M00S].log
    snprintf(name, sizeof(name), "[%04u%02u%02u%uH%02uM%02uS].log",
             y, mo, d, h, mi, s);

    if (!SD.exists(name)) return String(name);

    // Hiếm khi trùng (cùng giây) -> thêm hậu tố -NN
    for (int k = 1; k <= 99; ++k) {
      char name2[56];
      snprintf(name2, sizeof(name2), "[%04u%02u%02u%uH%02uM%02uS-%02d].log",
               y, mo, d, h, mi, s, k);
      if (!SD.exists(name2)) return String(name2);
    }
    // nếu vẫn trùng, rơi xuống fallback
  }

  // Fallback cho trường hợp chưa có TIME: logNNNNN.txt
  for (uint32_t i = 1; i <= 99999; ++i) {
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
      Serial.println(F("[SD] begin failed."));
      return false;
    }
  }
  if (logFile) return true;
  String fn = nextLogFilename();
  logFile = SD.open(fn.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println(F("[SD] open file failed."));
    return false;
  }
  Serial.print(F("[SD] logging to ")); Serial.println(fn);
  sdLinesSinceFlush = 0;
  return true;
}

void closeLogFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
    Serial.println(F("[SD] file closed."));
  }
}

// =================== TRANSPORT STATE ===================
enum class NetState { OFFLINE, ONLINE };
NetState netState = NetState::OFFLINE;

// =================== ADE/I2C ===================
#ifndef PERIOD
#define PERIOD 0x0E6E
#endif

#define RESET_PIN   22
#define PM0_PIN     20
#define PM1_PIN     25

#ifndef ADE_I2C_ADDR
#define ADE_I2C_ADDR  0x38
#endif
#define I2C_SPEED_HZ  400000

#define STREAM_HZ         120
#define JSON_BUF_SZ       1024
#define READ_EXTRAS_EVERY 10

volatile uint8_t i2c_err_streak = 0;
const uint8_t    I2C_ERR_LIMIT  = 3;
const uint8_t    I2C_RETRY      = 2;

// ---- calib constants (yours) ----
const uint32_t VLEVEL_CAL = 0x1720AF;
const int32_t AVGAIN_CAL = 248334, AIGAIN_CAL =  -286699;
const int32_t BVGAIN_CAL = 248334, BIGAIN_CAL =  27077;
const int32_t CVGAIN_CAL = 248334, CIGAIN_CAL =  -78179;
// const int32_t AVGAIN_CAL = 1, AIGAIN_CAL =  1;
// const int32_t BVGAIN_CAL = 1, BIGAIN_CAL =  1;
// const int32_t CVGAIN_CAL = 1, CIGAIN_CAL =  1;
const double VRMS_LSB_A = 8.47513E-05, VRMS_LSB_B = 8.47513E-05, VRMS_LSB_C = 8.47513E-05;
const double IRMS_LSB_A = 0.0000008433,  IRMS_LSB_B = 0.0000008433,  IRMS_LSB_C = 0.0000008433;
const double WATT_LSB_A = 0.02388575,   WATT_LSB_B = 0.02388575,   WATT_LSB_C = 0.02388575;
#define AVRMSOS_CAL  (-3200)   // 24-bit signed
#define AIRMSOS_CAL  (-10153)
#define BVRMSOS_CAL  (-3200)   // 24-bit signed
#define BIRMSOS_CAL  (-10153)
#define CVRMSOS_CAL  (-3200)   // 24-bit signed
#define CIRMSOS_CAL  (-10153)
#define AVRMSOS_CAL  (0)   // 24-bit signed
#define AIRMSOS_CAL  (0)
#define BVRMSOS_CAL  (0)   // 24-bit signed
#define BIRMSOS_CAL  (0)
#define CVRMSOS_CAL  (0)   // 24-bit signed
#define CIRMSOS_CAL  (0)


// ---- helpers ----
static int32_t sign_extend_24(int32_t v) { if (v & 0x800000) v |= 0xFF000000; return v; }

static bool ade_write(uint16_t reg, uint32_t data, uint8_t nbytes) {
  if (nbytes < 1 || nbytes > 4) return false;
  uint8_t b[4] = {(uint8_t)(data>>24),(uint8_t)(data>>16),(uint8_t)(data>>8),(uint8_t)data};
  for (uint8_t t=0; t<=I2C_RETRY; t++) {
    Wire.beginTransmission((uint8_t)ADE_I2C_ADDR);
    Wire.write((uint8_t)(reg>>8)); Wire.write((uint8_t)reg);
    for (uint8_t i=4-nbytes; i<4; i++) Wire.write(b[i]);
    if (Wire.endTransmission()==0) { i2c_err_streak=0; return true; }
    i2c_err_streak++;
  }
  return false;
}

static bool ade_read_raw(uint16_t reg, uint8_t *buf, uint8_t nbytes) {
  for (uint8_t t=0; t<=I2C_RETRY; t++) {
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

// ---- freq via PERIOD ----
static inline double read_line_freq_hz_raw() {
  uint16_t period = (uint16_t)(ade_read_u32(PERIOD,2) & 0xFFFF);
  if (period<100 || period>65000) return 0.0;
  return 256000.0/(double)period;
}
static inline double read_line_freq_hz_ema() {
  double f = read_line_freq_hz_raw();
  static bool init=false; static double ema=60.0;
  const double a=0.25;
  if (f<=0.0 || f<45.0 || f>65.0) return ema;
  if (!init) { ema=f; init=true; } else { ema=a*f+(1.0-a)*ema; }
  return ema;
}

// ---- ADE init ----
static void ade_reset_hw() {
  digitalWrite(RESET_PIN,HIGH); delay(2);
  digitalWrite(RESET_PIN,LOW);  delay(10);
  digitalWrite(RESET_PIN,HIGH); delay(10);
}
static void ade_basic_config() {
  ade_write(CFMODE,0x0088,2);
  ade_write(LCYCMODE,0x0F,1);
  ade_write(RUN,0x0001,2);
}
static void ade_apply_calibration() {
  ade_write(VLEVEL, VLEVEL_CAL,4);
  ade_write(AIGAIN, AIGAIN_CAL,4);
  ade_write(BIGAIN, BIGAIN_CAL,4);
  ade_write(CIGAIN, CIGAIN_CAL,4);

  ade_write(AVGAIN, AVGAIN_CAL,4); 
  ade_write(BVGAIN, BVGAIN_CAL,4); 
  ade_write(CVGAIN, CVGAIN_CAL,4); 

  ade_write(AVRMSOS,(uint32_t)AVRMSOS_CAL,3);
  ade_write(BVRMSOS,(uint32_t)BVRMSOS_CAL,3);
  ade_write(CVRMSOS,(uint32_t)CVRMSOS_CAL,3);
  ade_write(AIRMSOS,(uint32_t)AIRMSOS_CAL,3);
  ade_write(BIRMSOS,(uint32_t)BIRMSOS_CAL,3);
  ade_write(CIRMSOS,(uint32_t)CIRMSOS_CAL,3);

}

// =================== STREAM ===================
static char json_buf[JSON_BUF_SZ];
uint32_t seqNo = 0;

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

void sendFrameOrLog(const char* s) {
  if (netState == NetState::ONLINE && remotePort!=0) {
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
  while (n>0) {
    uint8_t buf[80];
    int m = Udp.read(buf, min(n, (int)sizeof(buf)));
    // HELLO (ASCII)
    if (m>=5 && !memcmp(buf,"HELLO",5)) {
      remoteIp   = Udp.remoteIP();
      remotePort = Udp.remotePort();
      Serial.print(F("[ETH] HELLO from ")); Serial.print(remoteIp);
      Serial.print(F(":")); Serial.println(remotePort);
      goOnline();
      lastAckMs = millis();
    }
    // TIME <epoch_s> (ASCII)
    else if (m>=6 && !memcmp(buf,"TIME ",5)) {
      // parse integer seconds
      buf[m] = 0; // ensure zero-terminated
      const char* p = (const char*)buf + 5;
      unsigned long epoch = strtoul(p, nullptr, 10);
      if (epoch > 1500000000UL) { // sanity ~2017+
        setTime(epoch);  // TimeLib system clock
        Serial.print(F("[TIME] set to epoch ")); Serial.println(epoch);
      }
    }
    // ACK + seq (binary: 'A''C''K' + 4-byte LE)
    else if (m==7 && buf[0]=='A' && buf[1]=='C' && buf[2]=='K') {
      uint32_t ackSeq = (uint32_t)buf[3] | ((uint32_t)buf[4]<<8) | ((uint32_t)buf[5]<<16) | ((uint32_t)buf[6]<<24);
      if (ackSeq == lastSentSeq) {
        lastAckMs = millis();
      }
    }
    n = Udp.parsePacket();
  }
}

void stream_frame_json() {
  bool do_extras = (READ_EXTRAS_EVERY>0) && (seqNo % READ_EXTRAS_EVERY == 0);

  uint32_t avr = ade_read_u32(AVRMS,4), bvr = ade_read_u32(BVRMS,4), cvr = ade_read_u32(CVRMS,4);
  uint32_t air = ade_read_u32(AIRMS,4), bir = ade_read_u32(BIRMS,4), cir = ade_read_u32(CIRMS,4);
  int32_t  aw  = sign_extend_24(ade_read_u32(AWATT,4));
  int32_t  bw  = sign_extend_24(ade_read_u32(BWATT,4));
  int32_t  cw  = sign_extend_24(ade_read_u32(CWATT,4));
  double   f   = read_line_freq_hz_ema();

  double Va=(double)avr*VRMS_LSB_A, Vb=(double)bvr*VRMS_LSB_B, Vc=(double)cvr*VRMS_LSB_C;
  double Ia=(double)air*IRMS_LSB_A, Ib=(double)bir*IRMS_LSB_B, Ic=(double)cir*IRMS_LSB_C;
// test để kiểm tra raw value
  // double Va=(double)avr, Vb=(double)bvr, Vc=(double)cvr;
  // double Ia=(double)air, Ib=(double)bir, Ic=(double)cir;
  double Pa=(double)aw*WATT_LSB_A,  Pb=(double)bw*WATT_LSB_B,  Pc=(double)cw*WATT_LSB_C;

  double Sa=Va*Ia, Sb=Vb*Ib, Sc=Vc*Ic;
  double PFa = (Sa>1e-9)? Pa/Sa:0, PFb=(Sb>1e-9)? Pb/Sb:0, PFc=(Sc>1e-9)? Pc/Sc:0;

  bool i2c_lock = (i2c_err_streak >= I2C_ERR_LIMIT);

  uint32_t t_ms = millis();
  unsigned long ts = now();   // epoch seconds (UTC) — set bởi TIME từ app

  if (!do_extras) {
    snprintf(json_buf, JSON_BUF_SZ,
     "{\"n\":%lu,\"t_ms\":%lu,\"ts\":%lu,\"i2c_lock\":%s,\"f\":%.2f,"
     "\"A\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
     "\"B\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
     "\"C\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f}}",
      (unsigned long)seqNo,(unsigned long)t_ms,(unsigned long)ts,
      i2c_lock?"true":"false", f,
      Va,Ia,Pa,Sa,PFa,  Vb,Ib,Pb,Sb,PFb,  Vc,Ic,Pc,Sc,PFc
    );
  } else {
    int32_t  aq=sign_extend_24(ade_read_u32(AVAR,4));
    int32_t  bq=sign_extend_24(ade_read_u32(BVAR,4));
    int32_t  cq=sign_extend_24(ade_read_u32(CVAR,4));
    uint32_t asa=ade_read_u32(AVA,4), bsa=ade_read_u32(BVA,4), csa=ade_read_u32(CVA,4);

    double Qa=(double)aq*WATT_LSB_A, Qb=(double)bq*WATT_LSB_B, Qc=(double)cq*WATT_LSB_C;
    double SA=(double)asa*WATT_LSB_A, SB=(double)bsa*WATT_LSB_B, SC=(double)csa*WATT_LSB_C;

    snprintf(json_buf, JSON_BUF_SZ,
     "{\"n\":%lu,\"t_ms\":%lu,\"ts\":%lu,\"i2c_lock\":%s,\"f\":%.2f,"
     "\"A\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
     "\"B\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
     "\"C\":{\"V\":%.3f,\"I\":%.3f,\"P\":%.3f,\"S\":%.3f,\"PF\":%.3f},"
     "\"extras\":{\"A\":{\"S_true\":%.3f,\"Q\":%.3f},\"B\":{\"S_true\":%.3f,\"Q\":%.3f},\"C\":{\"S_true\":%.3f,\"Q\":%.3f}}}",
      (unsigned long)seqNo,(unsigned long)t_ms,(unsigned long)ts,
      i2c_lock?"true":"false", f,
      Va,Ia,Pa,Sa,PFa,  Vb,Ib,Pb,Sb,PFb,  Vc,Ic,Pc,Sc,PFc,
      SA,Qa, SB,Qb, SC,Qc
    );
  }

  // gửi / log
  sendFrameOrLog(json_buf);

  if (i2c_lock) {
    digitalWrite(LED_BUILTIN,HIGH);
    ade_reset_hw(); delay(20);
    ade_basic_config(); ade_apply_calibration();
    i2c_err_streak = 0;
    digitalWrite(LED_BUILTIN,LOW);
  }

  seqNo++;
}

// =================== SETUP / LOOP ===================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(RESET_PIN, OUTPUT);
  pinMode(PM0_PIN,   INPUT);
  pinMode(PM1_PIN,   INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // ADE
  ade_reset_hw();
  Wire.begin(); Wire.setClock(I2C_SPEED_HZ); Wire.setTimeout(2); delay(50);
  ade_basic_config(); ade_apply_calibration();

  uint32_t ver = ade_read_u32(VERSION, 2);
  Serial.print(F("{\"boot\":\"ok\",\"version\":")); Serial.print(ver);
  Serial.println(F("}"));

  // Ethernet
  Ethernet.begin(mac, localIp);
  delay(100);
  Udp.begin(LOCAL_PORT);
  Serial.print(F("[ETH] IP: ")); Serial.println(Ethernet.localIP());
  Serial.println(F("[ETH] Send HELLO + TIME <epoch_s> from PC to start."));

  // SD
  sdReady = SD.begin(BUILTIN_SDCARD);
  if (sdReady) Serial.println(F("[SD] ready"));
  else         Serial.println(F("[SD] not ready (will retry on demand)"));

  // thời gian chưa sync -> now() = 0; vẫn ghi "ts":0 cho tới khi TIME đến
  netState = NetState::OFFLINE;
  openLogFile();
}

void loop() {
  static uint32_t t0=0; const uint32_t period_ms = 1000/STREAM_HZ;

  // 1) Control
  processUdpControl();

  // 2) Link & ACK watchdog
  if (Ethernet.linkStatus()!=LinkON) {
    goOffline("link down");
  } else if (netState==NetState::ONLINE) {
    if ((millis() - lastAckMs) > ACK_TIMEOUT_MS) goOffline("ack timeout");
  }

  // 3) Push frame @120Hz
  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - t0) >= period_ms) { t0 = now_ms; stream_frame_json(); }
}