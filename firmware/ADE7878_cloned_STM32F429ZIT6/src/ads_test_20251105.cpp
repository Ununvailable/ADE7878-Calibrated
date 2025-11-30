/* STM32F429ZIT6 + ADS131M08 - 2kSPS Data Logger
 * 
 * Features:
 * - 2000 samples/sec per channel (8 channels)
 * - UDP streaming when online
 * - SD card logging when offline
 * - DRDY interrupt-driven acquisition
 * - Compatible with existing ADE7878 network protocol
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <STM32Ethernet.h>
#include <TimeLib.h>
#include "stm32f4xx_hal.h"


// =================== HARDWARE CONFIG ===================
// Use SPI1 on STM32F429ZI Discovery default pins:
// PA5 = SCK
// PA6 = MISO (ADS DOUT)
// PA7 = MOSI (ADS DIN)
// PE4 = CS (your custom choice)

#define ADS_MOSI_PIN    PE6
#define ADS_MISO_PIN    PE5
#define ADS_SCK_PIN     PE2
#define ADS_CS_PIN      PE4
#define ADS_DRDY_PIN    PD2
#define ADS_SYNC_PIN    PB1
#define ADS_CLKIN_PIN   PE3

// SD card on SPI3 (or SDIO if preferred)
#define SD_CS_PIN       PC11  // Adjust based on your board

// Create SPI object
SPIClass SPI_4(ADS_MOSI_PIN, ADS_MISO_PIN, ADS_SCK_PIN);

// Local static timer handle
static TIM_HandleTypeDef htim3;

// =============== SERIAL COMMUNICATION =================
HardwareSerial SerialComm(USART6); 

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

// =================== ADS131M08 REGISTERS ===================
// Minimal register set (from datasheet)
#define ADS_ID          0x0000
#define ADS_STATUS      0x0001
#define ADS_MODE        0x0002
#define ADS_CLOCK       0x0003
#define ADS_GAIN1       0x0004
#define ADS_GAIN2       0x0005

// Commands
#define CMD_NULL        0x0000
#define CMD_RESET       0x0011
#define CMD_STANDBY     0x0022
#define CMD_WAKEUP      0x0033

// =================== DATA BUFFERS ===================
#define BUFFER_SIZE     256
#define NUM_CHANNELS    8

volatile int32_t ch_buffers[NUM_CHANNELS][BUFFER_SIZE];
volatile uint16_t buf_write_idx = 0;
volatile uint32_t sample_count = 0;
volatile bool newDataReady = false;

// =================== LOGGING CONFIG ===================
File logFile;
bool sdReady = false;
const uint16_t SD_FLUSH_EVERY = 100;
uint16_t sdWritesSinceFlush = 0;

// =================== NETWORK STATE ===================
enum class NetState { OFFLINE, ONLINE };
NetState netState = NetState::OFFLINE;

// =================== SPI CONFIGURATION ===================
SPISettings adsSettings(8000000, MSBFIRST, SPI_MODE1); // 8 MHz, Mode 1

void ads_select() {
    digitalWrite(ADS_CS_PIN, LOW);
}

void ads_deselect() {
    digitalWrite(ADS_CS_PIN, HIGH);
}

// =================== ADS131M08 LOW-LEVEL I/O ===================

uint16_t ads_read_register(uint16_t reg, uint8_t* data, uint8_t nbytes) {
    uint16_t cmd = 0xA000 | (reg << 7) | ((nbytes - 1) & 0x7F);
    
    // Frame 1: Send RREG command
    SPI_4.beginTransaction(adsSettings);
    ads_select();
    SPI_4.transfer16(cmd);
    for (int i = 0; i < 9; i++) {
        SPI_4.transfer16(0x0000);
    }
    ads_deselect();
    SPI_4.endTransaction();
    
    delayMicroseconds(10);  // Add small delay between frames
    
    // Frame 2: Send NULL, receive register data
    SPI_4.beginTransaction(adsSettings);
    ads_select();
    uint16_t status = SPI_4.transfer16(CMD_NULL);
    
    // Read response
    for (uint8_t i = 0; i < nbytes; i += 2) {
        uint16_t word = SPI_4.transfer16(0x0000);
        if (i < nbytes) data[i] = word >> 8;
        if (i + 1 < nbytes) data[i + 1] = word & 0xFF;
    }
    
    // Complete frame
    for (int i = 0; i < (8 - nbytes/2); i++) {
        SPI_4.transfer16(0x0000);
    }
    
    ads_deselect();
    SPI_4.endTransaction();
    
    return status;
}

void ads_write_register(uint16_t reg, uint16_t value) {
    // Build WREG command: 011a_aaaa_a000_0000
    uint16_t cmd = 0x6000 | (reg << 7);
    
    SPI_4.beginTransaction(adsSettings);
    ads_select();
    
    SPI_4.transfer16(cmd);
    SPI_4.transfer16(value);
    
    // Padding to 10 words
    for (int i = 0; i < 8; i++) {
        SPI_4.transfer16(0x0000);
    }
    
    ads_deselect();
    SPI_4.endTransaction();
    
    delay(1); // Register write settling time
}

void ads_send_command(uint16_t cmd) {
    SPI_4.beginTransaction(adsSettings);
    ads_select();
    
    SPI_4.transfer16(cmd);
    
    // Padding
    for (int i = 0; i < 9; i++) {
        SPI_4.transfer16(0x0000);
    }
    
    ads_deselect();
    SPI_4.endTransaction();
}

// =================== ADS131M08 DATA ACQUISITION ===================

typedef struct {
    uint16_t status;
    int32_t ch_data[NUM_CHANNELS];
    uint16_t crc;
} ADSFrame;

ADSFrame ads_read_frame() {
    ADSFrame frame;
    
    SPI_4.beginTransaction(adsSettings);
    ads_select();
    
    // Send NULL command to retrieve data
    frame.status = SPI_4.transfer16(CMD_NULL);
    
    // Read 8 channels (24-bit each = 3 bytes)
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        // Read 24 bits as 3 bytes (MSB first)
        uint8_t b0 = SPI_4.transfer(0x00);
        uint8_t b1 = SPI_4.transfer(0x00);
        uint8_t b2 = SPI_4.transfer(0x00);
        
        int32_t raw = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
        
        // Sign-extend 24-bit to 32-bit
        if (raw & 0x00800000) {
            raw |= 0xFF000000;
        }
        
        frame.ch_data[ch] = raw;
    }
    
    // Read CRC (optional)
    frame.crc = SPI_4.transfer16(0x0000);
    
    ads_deselect();
    SPI_4.endTransaction();
    
    return frame;
}

// =================== INTERRUPT SERVICE ROUTINE ===================

void DRDY_ISR() {
    newDataReady = true;
}

// =================== ADS131M08 INITIALIZATION ===================

void ads_reset_hw() {
    // Hardware reset via SYNC/RESET pin
    pinMode(ADS_SYNC_PIN, OUTPUT);
    digitalWrite(ADS_SYNC_PIN, LOW);
    delay(10);
    digitalWrite(ADS_SYNC_PIN, HIGH);
    delay(10);
}

void ads_init() {
    // Configure CS pin
    pinMode(ADS_CS_PIN, OUTPUT);
    digitalWrite(ADS_CS_PIN, HIGH);
    
    // Hardware reset
    ads_reset_hw();
    delay(50);

    // ============ ENABLE INTERNAL OSCILLATOR ============
    // CLOCK register: 0xFF8E (bit 7 = 1 enables internal osc)
    //   [15:8] = 0xFF → All 8 channels enabled
    //   [7]    = 1    → Internal oscillator ENABLED ← Changed!
    //   [6]    = 0    → Internal reference
    //   [4:2]  = 011b → OSR = 1024
    //   [1:0]  = 10b  → High-resolution mode
    
    SerialComm.println(F("[ADS] Enabling internal oscillator..."));
    ads_write_register(ADS_CLOCK, 0xFF0E);  // Changed from 0xFF0E
    delay(100);  // Let oscillator stabilize
    
    // Verify DRDY starts toggling
    uint8_t status_data[2];
    ads_read_register(ADS_STATUS, status_data, 2);
    uint16_t status = (status_data[0] << 8) | status_data[1];
    
    SerialComm.print(F("[ADS] STATUS after osc enable: 0x"));
    SerialComm.println(status, HEX);
    
    // Send software reset command
    ads_send_command(CMD_RESET);
    delay(50);
    
    // Verify communication
    uint8_t id_data[2];
    ads_read_register(ADS_ID, id_data, 2);
    // uint16_t id = (id_data[0] << 8) | id_data[1];
    // uint16_t id = id_data[0] | (id_data[1]  << 8);

    // Fuck this
    SerialComm.print(F("  ID raw bytes: 0x"));
    SerialComm.print(id_data[0], HEX);
    SerialComm.print(" 0x");
    SerialComm.println(id_data[1], HEX);

    uint16_t id = (id_data[0] << 8) | id_data[1];
    SerialComm.print(F("  ID (MSB first): 0x"));
    SerialComm.println(id, HEX);

    id = (id_data[1] << 8) | id_data[0];
    SerialComm.print(F("  ID (LSB first): 0x"));
    SerialComm.println(id, HEX);
    
    SerialComm.print(F("ADS131M08 ID: 0x"));
    SerialComm.println(id, HEX);
    
    if ((id & 0xFF00) != 0x2800) {
        SerialComm.println(F("⚠️  ERROR: Invalid device ID!"));
        SerialComm.println(F("Check SPI connections."));
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    // Configure for 2kSPS
    // CLOCK register: 0xFF0E
    //   [15:8] = 0xFF → All 8 channels enabled
    //   [7]    = 0    → Internal oscillator disabled (using external clock)
    //   [6]    = 0    → Internal reference
    //   [4:2]  = 011b → OSR = 1024
    //   [1:0]  = 10b  → High-resolution mode
    ads_write_register(ADS_CLOCK, 0xFF0E);
    
    // MODE register: 0x0510
    //   [10]   = 1    → Reset cleared
    //   [9:8]  = 01b  → 24-bit word length
    //   [4]    = 1    → SPI timeout enabled
    //   [1:0]  = 00b  → DRDY on most lagging channel
    ads_write_register(ADS_MODE, 0x0510);
    
    // Set all gains to 1 (no amplification)
    ads_write_register(ADS_GAIN1, 0x0000); // Ch 0-3
    ads_write_register(ADS_GAIN2, 0x0000); // Ch 4-7

    
    
    SerialComm.println(F("ADS131M08 configured for 2kSPS"));
}

void ads_clkin_enable() {
    // === PE3 (TIM3_CH4) -> 8.192 MHz clock output ===
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    TIM_HandleTypeDef htim3;
    htim3.Instance = TIM3;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.RepetitionCounter = 0;

    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        timer_clk *= 2;  // APB1 timer clocks double when prescaled

    // uint32_t target = 8192000;
    uint32_t target = 2;
    uint32_t prescaler = 0;
    uint32_t period = (timer_clk / target) - 1;
    if (period > 0xFFFF) { // adjust if too big
        prescaler = (period / 0xFFFF);
        period = (timer_clk / (target * (prescaler + 1))) - 1;
    }

    htim3.Init.Prescaler = prescaler;
    htim3.Init.Period = period;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = period / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    SerialComm.print("[CLKOUT] PE3 TIM3_CH4 @ ");
    SerialComm.print(target);
    SerialComm.println(" Hz active");
    SerialComm.print("[CLKOUT] ARR="); SerialComm.println(period);
    SerialComm.print("[CLKOUT] APB1 Timer Clock="); SerialComm.println(timer_clk);
}


// =================== SD CARD MANAGEMENT ===================

String nextLogFilename() {
    unsigned long t = now();
    
    if (t > 1500000000UL) {
        char name[48];
        snprintf(name, sizeof(name), "log_%lu.bin", t);
        return String(name);
    }
    
    // Fallback: sequential numbering
    for (uint32_t i = 1; i <= 99999; i++) {
        char name[24];
        snprintf(name, sizeof(name), "log%05lu.bin", i);
        if (!SD.exists(name)) return String(name);
    }
    
    return String("log99999.bin");
}

bool openLogFile() {
    if (!sdReady) {
        sdReady = SD.begin(SD_CS_PIN);
        if (!sdReady) {
            SerialComm.println(F("[SD] Init failed"));
            return false;
        }
    }
    
    if (logFile) return true;
    
    String fn = nextLogFilename();
    logFile = SD.open(fn.c_str(), FILE_WRITE);
    
    if (!logFile) {
        SerialComm.println(F("[SD] File open failed"));
        return false;
    }
    
    SerialComm.print(F("[SD] Logging to: ")); SerialComm.println(fn);
    sdWritesSinceFlush = 0;
    return true;
}

void closeLogFile() {
    if (logFile) {
        logFile.flush();
        logFile.close();
        SerialComm.println(F("[SD] File closed"));
    }
}

// =================== NETWORK STATE MANAGEMENT ===================

void goOffline(const char* reason) {
    if (netState != NetState::OFFLINE) {
        SerialComm.print(F("[NET] -> OFFLINE: ")); SerialComm.println(reason);
        netState = NetState::OFFLINE;
        openLogFile();
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void goOnline() {
    if (netState != NetState::ONLINE) {
        SerialComm.println(F("[NET] -> ONLINE"));
        closeLogFile();
        netState = NetState::ONLINE;
        digitalWrite(LED_BUILTIN, LOW);
    }
}

// =================== DATA OUTPUT ===================

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int32_t ch_data[NUM_CHANNELS];
} BinaryLogEntry;

void log_sample_to_sd() {
    if (!openLogFile()) return;
    
    BinaryLogEntry entry;
    entry.timestamp_ms = millis();
    
    // Copy latest sample from circular buffer
    uint16_t idx = (buf_write_idx - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        entry.ch_data[ch] = ch_buffers[ch][idx];
    }
    
    logFile.write((uint8_t*)&entry, sizeof(entry));
    
    if (++sdWritesSinceFlush >= SD_FLUSH_EVERY) {
        logFile.flush();
        sdWritesSinceFlush = 0;
    }
}

void send_sample_udp() {
    if (remotePort == 0) return;
    
    char json[512];
    uint16_t idx = (buf_write_idx - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    
    // Build JSON compatible with ADE7878 format
    snprintf(json, sizeof(json),
        "{\"n\":%lu,\"t_ms\":%lu,\"ts\":%lu,"
        "\"ch0\":%ld,\"ch1\":%ld,\"ch2\":%ld,\"ch3\":%ld,"
        "\"ch4\":%ld,\"ch5\":%ld,\"ch6\":%ld,\"ch7\":%ld}",
        sample_count, millis(), now(),
        ch_buffers[0][idx], ch_buffers[1][idx], 
        ch_buffers[2][idx], ch_buffers[3][idx],
        ch_buffers[4][idx], ch_buffers[5][idx],
        ch_buffers[6][idx], ch_buffers[7][idx]
    );
    
    Udp.beginPacket(remoteIp, remotePort);
    Udp.write((const uint8_t*)json, strlen(json));
    Udp.write('\n');
    Udp.endPacket();
    
    lastSentSeq = sample_count;
}

// =================== UDP CONTROL ===================

void processUdpControl() {
    int n = Udp.parsePacket();
    
    while (n > 0) {
        uint8_t buf[80];
        int m = Udp.read(buf, min(n, (int)sizeof(buf)));
        
        // HELLO command
        if (m >= 5 && !memcmp(buf, "HELLO", 5)) {
            remoteIp = Udp.remoteIP();
            remotePort = Udp.remotePort();
            SerialComm.print(F("[ETH] HELLO from "));
            SerialComm.print(remoteIp);
            SerialComm.print(F(":"));
            SerialComm.println(remotePort);
            goOnline();
            lastAckMs = millis();
        }
        
        // TIME command
        else if (m >= 6 && !memcmp(buf, "TIME ", 5)) {
            buf[m] = 0;
            unsigned long epoch = strtoul((char*)buf + 5, nullptr, 10);
            if (epoch > 1500000000UL) {
                setTime(epoch);
                SerialComm.print(F("[TIME] Set to epoch "));
                SerialComm.println(epoch);
            }
        }
        
        // ACK command
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

extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_EnableOverDrive();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | 
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

// =================== SETUP ===================
void setup() {
    SystemClock_Config();
    SerialComm.setRx(PG9);
    SerialComm.setTx(PG14);
    SerialComm.begin(9600);
    delay(500);  // Give serial time to stabilize
    
    SerialComm.println(F("\n=== ADS131M08 + STM32F429 Data Logger ===\n"));
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // ============ CRITICAL: Configure CS BEFORE SPI.begin() ============
    pinMode(ADS_CS_PIN, OUTPUT);
    digitalWrite(ADS_CS_PIN, HIGH);  // Deselect initially

    // Configure CLKIN
    ads_clkin_enable();
    
    // Initialize SPI with explicit pin setup
    SPI_4.begin();
    
    // Configure SPI settings (slower speed for debugging)
    SPI_4.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // 1 MHz
    SPI_4.endTransaction();
    
    SerialComm.println(F("[SPI] Initialized at 1 MHz (debug mode)"));
    
    // Power-up delay for ADS131M08
    delay(2000);
    
    // Hardware reset
    pinMode(ADS_SYNC_PIN, OUTPUT);
    digitalWrite(ADS_SYNC_PIN, LOW);
    delay(50);
    digitalWrite(ADS_SYNC_PIN, HIGH);
    delay(1000);
    
    SerialComm.println(F("[ADS] Hardware reset complete"));

    // Enable internal oscillator before any SPI register read
    ads_write_register(ADS_CLOCK, 0xFF0E);  // bit7=1: enable internal oscillator
    delay(500);  // let oscillator stabilize
    
    // Test SPI communication with ID register
    SerialComm.println(F("[SPI] Reading ID register..."));
    
    uint8_t id_data[2];
    uint16_t status = ads_read_register(ADS_ID, id_data, 2);
    // uint16_t id = (id_data[0] << 8) | id_data[1];
    // uint16_t id = id_data[0] | (id_data[1] << 8);

    // Fuck this
    SerialComm.print(F("  ID raw bytes: 0x"));
    SerialComm.print(id_data[0], HEX);
    SerialComm.print(" 0x");
    SerialComm.println(id_data[1], HEX);

    uint16_t id = (id_data[0] << 8) | id_data[1];
    SerialComm.print(F("  ID (MSB first): 0x"));
    SerialComm.println(id, HEX);

    id = (id_data[1] << 8) | id_data[0];
    SerialComm.print(F("  ID (LSB first): 0x"));
    SerialComm.println(id, HEX);
    
    SerialComm.print(F("  Status: 0x"));
    SerialComm.println(status, HEX);
    SerialComm.print(F("  ID: 0x"));
    SerialComm.println(id, HEX);
    
    if (id == 0xFFFF) {
        SerialComm.println(F("⚠️  ERROR: Reading all 1s - SPI not working!"));
        SerialComm.println(F("Check:"));
        SerialComm.println(F("  1. SPI wiring (MOSI, MISO, SCK)"));
        SerialComm.println(F("  2. ADS131M08 power supply (2.7-3.6V)"));
        SerialComm.println(F("  3. CLKIN signal present"));
        SerialComm.println(F("  4. CS pin connected to PE4"));
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }
    
    if (id == 0x0000) {
        SerialComm.println(F("⚠️  ERROR: Reading all 0s - CS not working?"));
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(250);
        }
    }
    
    if ((id & 0xFF00) != 0x2800) {
    // if ((id & 0xFF00)) {
        SerialComm.print(F("⚠️  WARNING: Unexpected ID. Expected 0x28xx, got 0x"));
        SerialComm.println(id, HEX);
    } else {
        SerialComm.println(F("✓ ADS131M08 detected!"));
    }
    
    // Continue with rest of initialization...
    ads_init();

    // Basic comm test
    uint8_t test_data[2];
    ads_read_register(ADS_STATUS, test_data, 2);
    SerialComm.printf("STATUS: 0x%02X%02X\n", test_data[0], test_data[1]);

    // Configure DRDY interrupt
    pinMode(ADS_DRDY_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ADS_DRDY_PIN), DRDY_ISR, FALLING);
    
    // // Initialize Ethernet
    // Ethernet.begin(mac, localIp);
    // delay(200);
    // Udp.begin(LOCAL_PORT);
    
    // SerialComm.print(F("[ETH] IP: "));
    // SerialComm.println(Ethernet.localIP());
    // SerialComm.println(F("[ETH] Send 'HELLO' from PC to start streaming"));
    
    // // Initialize SD card
    // sdReady = SD.begin(SD_CS_PIN);
    // if (sdReady) {
    //     SerialComm.println(F("[SD] Ready"));
    // } else {
    //     SerialComm.println(F("[SD] Not ready (will retry on demand)"));
    // }
    
    // Start in offline mode
    // netState = NetState::OFFLINE;
    // openLogFile();
    
    digitalWrite(LED_BUILTIN, LOW);
    SerialComm.println(F("\n=== SYSTEM READY ===\n"));
}

// =================== MAIN LOOP ===================

void loop() {
    static uint32_t last_output = 0;
    static uint32_t last_stats = 0;
    static uint32_t last_check = 0;
    static uint32_t last_count = 0;

    // Process incoming UDP messages
    processUdpControl();
    
    // Check link status
    if (Ethernet.linkStatus() != LinkON) {
        goOffline("link down");
    } else if (netState == NetState::ONLINE) {
        if ((millis() - lastAckMs) > ACK_TIMEOUT_MS) {
            goOffline("ack timeout");
        }
    }
    
    // Handle new ADC data (interrupt-driven)
    if (newDataReady) {
        newDataReady = false;
        
        // Read all 8 channels
        ADSFrame frame = ads_read_frame();
        
        // Store in circular buffer
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            ch_buffers[ch][buf_write_idx] = frame.ch_data[ch];
        }
        
        buf_write_idx = (buf_write_idx + 1) % BUFFER_SIZE;
        sample_count++;
        
        // Output every 64 samples (32ms @ 2kSPS)
        if ((sample_count % 64) == 0) {
            if (netState == NetState::ONLINE) {
                // send_sample_udp();
            } else {
                // log_sample_to_sd();
            }
            // Print 8 channels (24-bit each = 3 bytes)
            // for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            //     SerialComm.print(frame.ch_data[ch]);
            //     SerialComm.print(',');
            // }
            SerialComm.print(frame.ch_data[0]);
            SerialComm.println();
        }
    }
    
    // Print statistics every 5 seconds
    // if (millis() - last_stats > 5000) {
    //     last_stats = millis();
        
    //     SerialComm.print(F("[STATS] Samples: "));
    //     SerialComm.print(sample_count);
    //     SerialComm.print(F(" | Rate: "));
    //     SerialComm.print(sample_count / (millis() / 1000.0));
    //     SerialComm.print(F(" SPS | State: "));
    //     SerialComm.println(netState == NetState::ONLINE ? "ONLINE" : "OFFLINE");
    // }

    // if (millis() - last_check > 1000) {
    //     SerialComm.printf("Rate: %lu SPS\n", sample_count - last_count);
    //     last_count = sample_count;
    //     last_check = millis();
    // }
}