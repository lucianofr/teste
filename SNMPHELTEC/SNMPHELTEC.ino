/******************************************************************************
 * SNMPHELTEC.ino  —  UPS SNMP v1 -> LoRaWAN LA915 (ABP) bridge
 *
 * Single-file Arduino IDE sketch.
 *
 * Board:    ESP32S3 Dev Module  (ESP32-S3 + SX1262)
 * Shield:   W5500 Ethernet
 * Region:   LA915  (AU915 PHY + RX1=5s / RX2=6s, no dwell, sub-band 1
 * per Everynet/Netmore docs — AllCom carrier in Brazil)
 *
 *
 * ============================================================================
 * ARDUINO IDE SETUP (do this once before compiling)
 * ============================================================================
 * 1. Boards Manager: install "esp32" by Espressif Systems.
 *    This sketch compiles on BOTH the 2.x and 3.x cores (the watchdog init
 *    is selected at compile time via ESP_ARDUINO_VERSION_MAJOR). 3.x is
 *    recommended but not required.
 * 2. Tools -> Board -> "ESP32S3 Dev Module".
 * 3. Library Manager — install:
 * - "RadioLib"                             by Jan Gromes        (>= 7.0.0)
 * - "Ethernet3"                            by sstaub (v1.6.x)
 * - "Arduino_SNMP_Manager"                 by shortbloke
 * - "U8g2"                                 by olikraus          (>= 2.34)
 *
 * 4. Nenhuma configuração externa de lib necessária. RadioLib usa AU915
 * com sub-band selecionável em runtime via LoRaWANNode constructor.
 *
 * 5. Compile, upload (115200 baud), open Serial Monitor at 115200.
 * ============================================================================
 *****************************************************************************/

#include <Arduino.h>
#include <SPI.h>
// sstaub/Ethernet3 (v1.6.x): drives the W5500 via its 8 hardware sockets
// directly — no LWIP, no MACRAW. Replaces EthernetESP32 which was dropping
// ~10% of UDP responses (firstResp=-1) on this hardware; root cause was
// inside the W5500-MACRAW <-> LWIP stack. Native sockets sidestep the whole
// problem. Configured with init(1) so the single UDP socket we need owns all
// 16 KB of the W5500's internal RX/TX buffer (huge headroom vs default 2 KB).
// INT pin (-1, not wired) is irrelevant — this lib polls W5500 status
// registers over SPI on demand.
#include <Ethernet3.h>
#include <EthernetUdp3.h>
#include <Arduino_SNMP_Manager.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>            // esp_restart()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"         // xTaskGetIdleTaskHandleForCPU() (core 2.x WDT)



// ---- TEMP SENSOR (DS18B20) -------------------------------------------------
// Set to 0 on boards with no DS18B20 fitted. When 0 the OneWire libraries are
// not compiled, GPIO23 is left untouched, payload bytes 26-27 carry the offline
// sentinel, and the invalid-read auto-reboot is disabled (so a sensor-less
// board does NOT reboot-loop). Runtime tunables in the config block below.
#define USE_TEMP_SENSOR       0

#if USE_TEMP_SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

// =============================================================================
// User-tunable configuration
// =============================================================================

// ---- BYPASS MODE -----------------------------------------------------------
// Set to 1 to skip the W5500 + SNMP code entirely and transmit synthetic UPS
// data over LoRaWAN every SNMP_POLL_INTERVAL_MS. Use to validate the LoRa
// radio path when the Ethernet shield is suspect or absent. The payload
// layout is identical so the LNS decoder needs no changes.
#define BYPASS_SNMP           0

#define SNMP_TARGET_IP        IPAddress(192, 168, 1, 3)
#define SNMP_COMMUNITY        "public"
#define SNMP_VERSION_V1       0                 // shortbloke lib: 0=v1, 1=v2c
// Local UDP port for SNMP GET/Response. The shortbloke library hardcodes 162
// (the SNMP TRAP port) inside its setUDP()/begin() path. Many UPS SNMP agents
// rate-limit or silently drop GET requests whose source port is 162 to avoid
// trap-loop attacks, which matches the field pattern of intermittent
// firstResp=-1 with healthy link/heap. Using a high ephemeral port bypasses
// the lib's hardcoded port by assigning gSnmp._udp directly (the field is
// public in Arduino_SNMP_Manager.h:122).
#define SNMP_LOCAL_PORT       33333
#define DS18B20_PIN           23
// Reboot the board after this many consecutive invalid DS18B20 reads
// (raw -127 C = DEVICE_DISCONNECTED_C). Mirrors the SNMP 3-fail -> esp_restart
// recovery policy. Only active when USE_TEMP_SENSOR=1.
#define TEMP_MAX_CONSEC_FAILS  3
#define TEMP_OFFLINE_C10       -1270   // payload sentinel: sensor offline/absent
#define TEMP_POLL_INTERVAL_MS  10000UL // DS18B20 sampling cadence (initially 10 s)
// Poll cadence dropped from 60 s -> 10 s so change-detect on critical OIDs
// (REQ-4) propagates state transitions in ~10 s instead of waiting for the
// 60 s heartbeat. Uplink TX is still rate-limited by UPLINK_PERIODIC_MS /
// UPLINK_MIN_INTERVAL_MS so the LNS does not see a 6x traffic increase.
#define SNMP_POLL_INTERVAL_MS  10000UL
#define SNMP_RESPONSE_WAIT_MS  2000UL           // wait between request and TX
#define UPLINK_PERIODIC_MS     600000UL          // heartbeat: forced uplink cadence
#define UPLINK_MIN_INTERVAL_MS 10000UL          // rate-limit between consecutive uplinks

// Hardware watchdog (ESP32 Task WDT). 15 s covers worst-case node.sendReceive()
// (~7 s on LA915 SF10) + SNMP retry window (~5 s) with margin. Panic on
// timeout → reboot. Fed at the top of loop() and around blocking radio calls.
#define WDT_TIMEOUT_S          15

// Failure tolerance threshold. Field logs on the prior EthernetESP32 path
// proved that mid-flight driver re-init (Ethernet.end+begin + W5500 RSTn
// pulse) did NOT clear the failure state — only a full esp_restart() did.
// We keep that direct-to-restart policy here. Transient single-poll losses
// still resolve themselves on the next 10 s cycle, so a threshold of 3
// consecutive fails (~30 s mute) balances noise tolerance vs reboot latency.
#define SNMP_RESTART_AFTER_FAILS 3

// W5500 static IP (edit for your LAN)
static const IPAddress LOCAL_IP    (192, 168, 1, 50);
static const IPAddress GATEWAY_IP  (192, 168, 1,  1);
static const IPAddress SUBNET_MASK (255, 255, 255, 0);
static const IPAddress DNS_IP      (8,   8,   8,   8);

// MAC for the W5500 (must be unique on the LAN). Edit for production.
static byte MAC_ADDR[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 };

// -----------------------------------------------------------------------------
// Pinos W5500
// -----------------------------------------------------------------------------
#define W5500_CS_PIN   10
#define W5500_RST_PIN  48
#define W5500_MOSI_PIN 11
#define W5500_MISO_PIN 13
#define W5500_SCK_PIN  12
#define W5500_INT_PIN  9

// =============================================================================
// LoRaWAN ABP credentials
// =============================================================================
// DevEUI / AppEUI — RadioLib LoRaWANNode takes uint64_t MSB-first.
// ABP runtime does NOT transmit them, but keeping them populated allows
// an OTAA migration later by calling node.beginOTAA(JOINEUI, DEVEUI, ...).

// DevEUI: f70a96163e29817c
static const uint64_t DEVEUI = 0xF70A96163E29817CULL;

// AppEUI / JoinEUI: 34e3f39dc13a92f6
static const uint64_t JOINEUI = 0x34E3F39DC13A92F6ULL;

// DevAddr: D4A65340
static const uint32_t DEVADDR = 0xD4A65340UL;

// NwkSKey (= NwkSEncKey in LoRaWAN 1.0.x): 3f439ce38b35ed3f32ab226916995438
static uint8_t NWKSKEY[16] = {
    0x3F, 0x43, 0x9C, 0xE3, 0x8B, 0x35, 0xED, 0x3F,
    0x32, 0xAB, 0x22, 0x69, 0x16, 0x99, 0x54, 0x38
};

// AppSKey: idêntica à NwkSKey (cadastro modo "criptografia APP" no portal).
static uint8_t APPSKEY[16] = {
    0x3F, 0x43, 0x9C, 0xE3, 0x8B, 0x35, 0xED, 0x3F,
    0x32, 0xAB, 0x22, 0x69, 0x16, 0x99, 0x54, 0x38
};

// =============================================================================
// SNMP OIDs — ORDER IS THE PAYLOAD ORDER (idx 0 -> idx 12)
// =============================================================================
static const char* OIDS[13] = {
    ".1.3.6.1.4.1.935.1.1.1.3.2.1.0",   //  0  voltageIn
    ".1.3.6.1.4.1.935.1.1.1.3.2.4.0",   //  1  frequencyIn
    ".1.3.6.1.4.1.935.1.1.1.3.2.5.0",   //  2  inputLineFailCause
    ".1.3.6.1.4.1.935.1.1.1.4.2.1.0",   //  3  voltageOut
    ".1.3.6.1.4.1.935.1.1.1.4.1.1.0",   //  4  outputStatus
    ".1.3.6.1.4.1.935.1.1.1.4.2.3.0",   //  5  upsLoad
    ".1.3.6.1.4.1.935.1.1.1.2.2.3.0",   //  6  batteryTemperature
    ".1.3.6.1.4.1.935.1.1.1.2.1.1.0",   //  7  batteryStatus
    ".1.3.6.1.4.1.935.1.1.1.2.2.1.0",   //  8  batteryCapacity
    ".1.3.6.1.4.1.935.1.1.1.2.2.2.0",   //  9  batteryVoltage
    ".1.3.6.1.4.1.935.1.1.1.2.1.2.0",   // 10  timeOnBattery
    ".1.3.6.1.4.1.935.1.1.1.2.2.5.0",   // 11  batteryReplacementIndicator
    ".1.3.6.1.4.1.935.1.1.1.2.2.7.0",   // 12  batteryCurrent
};
static const char* OID_NAMES[13] = {
    "voltageIn", "frequencyIn", "inputLineFailCause", "voltageOut",
    "outputStatus", "upsLoad", "batteryTemperature", "batteryStatus",
    "batteryCapacity", "batteryVoltage", "timeOnBattery",
    "batteryReplacementIndicator", "batteryCurrent"
};

// =============================================================================
// Globals
// =============================================================================

EthernetUDP    gUdp;
SNMPManager    gSnmp(SNMP_COMMUNITY);
SNMPGet        gReq(SNMP_COMMUNITY, SNMP_VERSION_V1);
ValueCallback* gCb[13]      = { nullptr };
int            gValues[13]  = { 0 };
#if USE_TEMP_SENSOR
OneWire           gOneWire(DS18B20_PIN);
DallasTemperature gTempSensor(&gOneWire);
uint8_t           gTempFailCount = 0;   // consecutive invalid reads -> reboot at TEMP_MAX_CONSEC_FAILS
uint32_t          gNextTempPollMs = 0;  // next DS18B20 sample due (millis)
#endif
int16_t           gTempC10 = TEMP_OFFLINE_C10;  // last reading, decidegrees; sentinel = offline/absent
uint8_t        gPayload[28] = { 0 };

uint32_t gNextPollMs   = 0;
uint32_t gSendAtMs     = 0;
bool     gSendPending  = false;
bool     gMidRetryDone = false;    // in-window retry latch (one extra GET per poll)

// Count of consecutive SNMP failures (0/13 or partial). Reset on any 13/13
// response. At SNMP_RESTART_AFTER_FAILS (3) triggers esp_restart() — the only
// recovery that empirically clears the wedged-network state.
uint8_t  gConsecFails  = 0;

// Diagnostic timing for failure root-cause analysis. gGetSentMs is the millis()
// stamp when the last GET was dispatched; gFirstRespMs is the delta (ms) to
// the first non-sentinel value appearing in gValues[]. Logged on every poll
// to differentiate "response never arrived" vs "response arrived too late".
uint32_t gGetSentMs    = 0;
int32_t  gFirstRespMs  = -1;     // -1 = no response observed in this window

// REQ-4: change-detect on critical OIDs. Sends an uplink off the 60 s
// heartbeat when any of the 4 monitored variables transitions.
// Indices into OIDS[]/gValues[]:
//   2  inputLineFailCause
//   4  outputStatus
//   7  batteryStatus
//  11  batteryReplacementIndicator
static const uint8_t CRITICAL_IDX[4] = { 2, 4, 7, 11 };
static const int32_t PREV_SENTINEL   = INT32_MIN;
int32_t  gPrevValues[4] = { PREV_SENTINEL, PREV_SENTINEL, PREV_SENTINEL, PREV_SENTINEL };
uint32_t gLastUplinkMs  = 0;

// -----------------------------------------------------------------------------
// OLED + comm-status tracking
// -----------------------------------------------------------------------------
// Heltec V3 OLED: SSD1306 128x64 over I2C
#define OLED_RST_PIN  21
#define OLED_SDA_PIN  17
#define OLED_SCL_PIN  18

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C gDisp(U8G2_R0, OLED_RST_PIN, OLED_SCL_PIN, OLED_SDA_PIN);

enum SnmpCommState : uint8_t { SNMP_S_INIT, SNMP_S_WAIT, SNMP_S_OK, SNMP_S_PARTIAL, SNMP_S_FAIL };
enum LoraCommState : uint8_t { LORA_S_INIT, LORA_S_TX, LORA_S_OK, LORA_S_FAIL };

// Sentinel used to detect which SNMP varbinds actually came back during the
// response window. INT32 value unlikely to occur as a real UPS reading.
static const int32_t SNMP_SENTINEL = 0x7FFFFFFE;

volatile SnmpCommState gSnmpState     = SNMP_S_INIT;
uint32_t               gSnmpOkCount   = 0;
uint32_t               gSnmpFailCount = 0;
uint32_t               gSnmpLastOkMs  = 0;
uint8_t                gSnmpLastRecv  = 0;

volatile LoraCommState gLoraState       = LORA_S_INIT;
uint32_t               gLoraTxCount     = 0;
uint32_t               gLoraDoneCount   = 0;
uint32_t               gLoraLastDoneMs  = 0;
int16_t                gLoraLastDlBytes = -1;

//uint32_t gOledNextDrawMs = 0;

// -----------------------------------------------------------------------------
// LoRaWAN session persistence (ESP32 NVS via Preferences)
// -----------------------------------------------------------------------------
Preferences gPrefs;

// -----------------------------------------------------------------------------
// RadioLib SX1262 + LoRaWANNode + Instância SPI Secundária
// -----------------------------------------------------------------------------
#define LORA_NSS   14
#define LORA_SCK   12
#define LORA_MOSI  11
#define LORA_MISO  13
#define LORA_RST   42
#define LORA_BUSY  38
#define LORA_DIO1  45
#define LORA_INT   45

// Instanciamos um barramento SPI separado para o SX1262 para evitar conflito com o W5500
SPIClass loraSPI(HSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// AU915, sub-band 1 (canais 0-7) per Everynet/AllCom AllManager Brasil.
const LoRaWANBand_t Region   = AU915;
const uint8_t       SUB_BAND = 1;

// LA915 requires RX1=5s / RX2=6s but RadioLib 7.x defaults to RX1=1s / RX2=2s
class LoRaWANNodeLA915 : public LoRaWANNode {
public:
    LoRaWANNodeLA915(PhysicalLayer* phy, const LoRaWANBand_t* band, uint8_t subBand)
        : LoRaWANNode(phy, band, subBand) {}
    void setRxDelaySec(uint8_t sec) {
        rxDelays[1] = (RadioLibTime_t)sec * 1000UL;
        rxDelays[2] = rxDelays[1] + 1000UL;
    }
};

LoRaWANNodeLA915 node(&radio, &Region, SUB_BAND);

// Session-buffer key in NVS
static const char* PREFS_KEY_SESSION = "session";
static const char* PREFS_KEY_NONCES  = "nonces";

// =============================================================================
// Forward declarations
// =============================================================================
//static void oledInit();
//static void oledDraw();
static void loraSetup();
static void loraSendUplink(bool isCritical = false);
static void loraSaveSession();
static void loraRestoreSession();
static void wdtInit();
#if !BYPASS_SNMP
static bool ethernetInit();
static void udpRefresh();
#endif
static bool criticalChanged();
static void handleCriticalAndUplink();

// =============================================================================
// Helpers
// =============================================================================
static int16_t clampToI16(int32_t v, const char* tag) {
    if (v > 32767) {
        Serial.printf("[WARN] %s overflow %ld -> 32767\n", tag, (long)v);
        return 32767;
    }
    if (v < -32768) {
        Serial.printf("[WARN] %s underflow %ld -> -32768\n", tag, (long)v);
        return -32768;
    }
    return (int16_t)v;
}

static void buildPayload() {
    Serial.println(F("[POLL] SNMP values:"));
    for (int i = 0; i < 13; i++) {
        Serial.printf("  [%2d] %-30s = %d\n", i, OID_NAMES[i], gValues[i]);
        int16_t v = clampToI16((int32_t)gValues[i], OID_NAMES[i]);
        gPayload[i * 2]     = (uint8_t)((v >> 8) & 0xFF);   // big-endian
        gPayload[i * 2 + 1] = (uint8_t)(v & 0xFF);
    }
    gPayload[26] = (uint8_t)((gTempC10 >> 8) & 0xFF);
    gPayload[27] = (uint8_t)(gTempC10 & 0xFF);
    Serial.printf("  [13] %-30s = %d (%.1f C)\n", "tempC10",
                  (int)gTempC10, gTempC10 / 10.0f);
    Serial.print(F("[PAYLOAD] 28B hex: "));
    for (int i = 0; i < 28; i++) {
        if (gPayload[i] < 0x10) Serial.print('0');
        Serial.print(gPayload[i], HEX);
    }
    Serial.println();
}

#if BYPASS_SNMP
static int16_t _jitter(int16_t base, int16_t spread, int16_t lo, int16_t hi) {
    int32_t v = (int32_t)base + (int32_t)random(-spread, spread + 1);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (int16_t)v;
}

static void fakeFillValues() {
    gValues[0]  = _jitter(2200, 30, 0, 3000);   // voltageIn  (×10)
    gValues[1]  = _jitter(600,   3, 0, 700);    // frequencyIn (×10)
    gValues[2]  = 1;                            // inputLineFailCause=none
    gValues[3]  = _jitter(2200, 10, 0, 3000);   // voltageOut (×10)
    gValues[4]  = 2;                            // outputStatus=onLine
    gValues[5]  = _jitter(45,    8, 0, 100);    // upsLoad %
    gValues[6]  = _jitter(28,    3, -40, 85);   // batteryTemperature °C
    gValues[7]  = 2;                            // batteryStatus=normal
    gValues[8]  = _jitter(95,    3, 0, 100);    // batteryCapacity %
    gValues[9]  = _jitter(540,   5, 0, 600);    // batteryVoltage (×10)
    gValues[10] = 0;                            // timeOnBattery s
    gValues[11] = 1;                            // batteryReplacementIndicator=no
    gValues[12] = _jitter(12,    4, -200, 200); // batteryCurrent A
}
#endif

// =============================================================================
// SNMP
// =============================================================================
static void snmpBindLocal() {
    gUdp.stop();
    gUdp.begin(SNMP_LOCAL_PORT);
    gSnmp._udp = &gUdp;
}

static void tempSetup() {
#if USE_TEMP_SENSOR
    gTempSensor.begin();
    uint8_t n = gTempSensor.getDeviceCount();
    Serial.printf("[TEMP] DS18B20 bus pin=%u, devices found=%u\n",
                  (unsigned)DS18B20_PIN, (unsigned)n);
#else
    Serial.println(F("[TEMP] DS18B20 disabled (USE_TEMP_SENSOR=0)"));
#endif
}

#if USE_TEMP_SENSOR
static void tempPoll() {
    gTempSensor.requestTemperatures();                 // ~750 ms blocking @ 12-bit
    float tempC = gTempSensor.getTempCByIndex(0);
    if (tempC == DEVICE_DISCONNECTED_C) {
        gTempC10 = TEMP_OFFLINE_C10;
        Serial.printf("[TEMP] invalid read (-127 C) consecFails=%u/%u\n",
                      (unsigned)(gTempFailCount + 1),
                      (unsigned)TEMP_MAX_CONSEC_FAILS);
        if (++gTempFailCount >= TEMP_MAX_CONSEC_FAILS) {
            Serial.printf("[TEMP] FATAL: %u consecutive invalid reads — esp_restart()\n",
                          (unsigned)gTempFailCount);
            Serial.flush();
            delay(50);
            esp_restart();
        }
    } else {
        gTempFailCount = 0;
        gTempC10 = clampToI16((int32_t)lroundf(tempC * 10.0f), "tempC10");
        Serial.printf("[TEMP] %.1f C\n", tempC);
    }
}
#endif

static void snmpSetup() {
    snmpBindLocal();
    for (int i = 0; i < 13; i++) {
        gCb[i] = gSnmp.addIntegerHandler(SNMP_TARGET_IP, OIDS[i], &gValues[i]);
    }
    Serial.printf("[SNMP] 13 integer handlers registered, local UDP port=%u\n",
                  (unsigned)SNMP_LOCAL_PORT);
}

static void snmpSendGet() {
    uint16_t drained = 0;
    while (gUdp.parsePacket() > 0) {
        gUdp.flush();
        drained++;
        if (drained > 8) break;
    }
    if (drained) {
        Serial.printf("[SNMP] drained %u stale UDP packet(s) pre-GET\n",
                      (unsigned)drained);
    }

    for (int i = 0; i < 13; i++) gValues[i] = (int)SNMP_SENTINEL;
    gFirstRespMs  = -1;
    gMidRetryDone = false;

    for (int i = 0; i < 13; i++) {
        gReq.addOIDPointer(gCb[i]);
    }
    gReq.setIP(Ethernet.localIP());
    gReq.setUDP(&gUdp);
    gReq.setRequestID((uint16_t)random(1, 65535));
    gGetSentMs = millis();
    bool sent = gReq.sendTo(SNMP_TARGET_IP);
    gReq.clearOIDList();
    gSnmpState = SNMP_S_WAIT;
    if (sent) {
        Serial.println(F("[SNMP] GET dispatched (13 OIDs)"));
    } else {
        Serial.println(F("[SNMP] GET dispatch FAILED (endPacket returned false)"));
    }
}

static void snmpResendInWindow() {
    for (int i = 0; i < 13; i++) {
        gReq.addOIDPointer(gCb[i]);
    }
    gReq.setIP(Ethernet.localIP());
    gReq.setUDP(&gUdp);
    gReq.setRequestID((uint16_t)random(1, 65535));
    bool sent = gReq.sendTo(SNMP_TARGET_IP);
    gReq.clearOIDList();
    Serial.printf("[SNMP] mid-window re-dispatch (no resp at %ldms) %s\n",
                  (long)(millis() - gGetSentMs),
                  sent ? "OK" : "FAILED");
}

static void snmpEvaluate() {
    uint8_t recv = 0;
    for (int i = 0; i < 13; i++) {
        if (gValues[i] != (int)SNMP_SENTINEL) recv++;
    }
    gSnmpLastRecv = recv;

    if (recv == 13) {
        gSnmpState     = SNMP_S_OK;
        gSnmpOkCount++;
        gSnmpLastOkMs  = millis();
    } else if (recv == 0) {
        gSnmpState     = SNMP_S_FAIL;
        gSnmpFailCount++;
    } else {
        gSnmpState     = SNMP_S_PARTIAL;
        gSnmpFailCount++;
    }
    Serial.printf("[SNMP] response window closed — %u/13 varbinds firstResp=%ldms\n",
                  (unsigned)recv, (long)gFirstRespMs);

    if (recv != 13) {
#if !BYPASS_SNMP
        Serial.printf("[DIAG] link=%s heapFree=%u heapLargest=%u remoteIP=%u.%u.%u.%u\n",
                      Ethernet.link() ? "UP" : "DOWN",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap(),
                      SNMP_TARGET_IP[0], SNMP_TARGET_IP[1],
                      SNMP_TARGET_IP[2], SNMP_TARGET_IP[3]);
#endif
    }

    for (int i = 0; i < 13; i++) {
        if (gValues[i] == (int)SNMP_SENTINEL) gValues[i] = 0;
    }
}

// =============================================================================
// LoRaWAN (RadioLib)
// =============================================================================
static void loraSetup() {
    Serial.println(F("[RL] radio.begin()"));
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[RL] FATAL radio.begin failed code=%d\n", (int)state);
        while (true) { delay(1000); }
    }

    // ESP32-S3 + SX1262. Max +22 dBm.
    // O método setOutputPower() do SX1262 recebe apenas a potência (sem arg PA_BOOST).
    radio.setOutputPower(22);

    Serial.println(F("[RL] node.beginABP()"));
    state = node.beginABP(DEVADDR, NULL, NULL, NWKSKEY, APPSKEY);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[RL] FATAL beginABP failed code=%d\n", (int)state);
        while (true) { delay(1000); }
    }

    node.setADR(false);

    loraRestoreSession();

    Serial.println(F("[RL] ABP activate"));
    state = node.activateABP();
    if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
        Serial.println(F("[RL] activateABP → NEW session (FCntUp=0)"));
    } else if (state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
        Serial.printf("[RL] activateABP → RESTORED (FCntUp=%lu)\n",
                      (unsigned long)node.getFCntUp());
    } else {
        Serial.printf("[RL] FATAL activateABP returned %d\n", (int)state);
        while (true) { delay(1000); }
    }

    node.setDatarate(2);
    node.setDwellTime(false);
    node.setRxDelaySec(5);

    Serial.printf("[RL] ABP activated — AU915 sub-band %u (channels 0-7 + ch64), DR2 SF10/125 kHz, 22 dBm, ADR off, dwell off, RX1=5s RX2=6s (LA915)\n",
                  (unsigned)SUB_BAND);
}

static void loraSendUplink(bool isCritical) {
    const uint8_t maxAttempts = isCritical ? 3 : 1;

    for (uint8_t att = 1; att <= maxAttempts; att++) {
        if (isCritical) {
            Serial.printf("[RL] confirmed uplink attempt %u/%u FCntUp=%lu\n",
                          (unsigned)att, (unsigned)maxAttempts,
                          (unsigned long)node.getFCntUp());
        } else {
            Serial.printf("[RL] uplink fPort=2 len=%u FCntUp=%lu\n",
                          (unsigned)sizeof(gPayload),
                          (unsigned long)node.getFCntUp());
        }

        gLoraState = LORA_S_TX;
        gLoraTxCount++;

        LoRaWANEvent_t evDown = { 0 };
        esp_task_wdt_reset();
        int16_t state = node.sendReceive(gPayload, sizeof(gPayload),
                                         /*fPort*/ 2,
                                         /*isConfirmed*/ isCritical,
                                         /*eventUp*/ NULL,
                                         /*eventDown*/ &evDown);
        esp_task_wdt_reset();

        if (state < RADIOLIB_ERR_NONE) {
            Serial.printf("[RL] uplink error code=%d\n", (int)state);
            gLoraState = LORA_S_FAIL;
            return;
        }

        gLoraState       = LORA_S_OK;
        gLoraDoneCount++;
        gLoraLastDoneMs  = millis();
        gLastUplinkMs    = gLoraLastDoneMs;

        if (state > 0) {
            Serial.printf("[RL] downlink received in RX%u\n", (unsigned)state);
            gLoraLastDlBytes = (int16_t)state;
        } else {
            gLoraLastDlBytes = 0;
            Serial.println(F("[RL] uplink TX OK, no downlink"));
        }

        loraSaveSession();
        esp_task_wdt_reset();

        if (!isCritical) {
            return;
        }

        if (evDown.confirming) {
            Serial.printf("[EVT] ACK received on attempt %u/%u\n",
                          (unsigned)att, (unsigned)maxAttempts);
            return;
        }
        Serial.printf("[EVT] no ACK on attempt %u/%u — retrying\n",
                      (unsigned)att, (unsigned)maxAttempts);
    }

    Serial.println(F("[EVT] ACK FAIL after 3 attempts — giving up"));
}

static void loraSaveSession() {
    uint8_t* nonces = node.getBufferNonces();
    if (!nonces) {
        Serial.println(F("[NVS] save SKIPPED — getBufferNonces() returned NULL"));
        return;
    }
    size_t wN = gPrefs.putBytes(PREFS_KEY_NONCES, nonces,
                                RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    if (wN != RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {
        Serial.printf("[NVS] nonces save FAILED — wrote %u of %u B\n",
                      (unsigned)wN,
                      (unsigned)RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
        return;
    }

    uint8_t* sess = node.getBufferSession();
    if (!sess) {
        Serial.println(F("[NVS] save SKIPPED — getBufferSession() returned NULL"));
        return;
    }
    size_t wS = gPrefs.putBytes(PREFS_KEY_SESSION, sess,
                                RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    if (wS != RADIOLIB_LORAWAN_SESSION_BUF_SIZE) {
        Serial.printf("[NVS] session save FAILED — wrote %u of %u B (NVS full or RO?)\n",
                      (unsigned)wS,
                      (unsigned)RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
        return;
    }
    Serial.printf("[NVS] saved nonces=%uB session=%uB FCntUp=%lu\n",
                  (unsigned)wN,
                  (unsigned)wS,
                  (unsigned long)node.getFCntUp());
}

static void loraRestoreSession() {
    size_t nLen = gPrefs.getBytesLength(PREFS_KEY_NONCES);
    size_t sLen = gPrefs.getBytesLength(PREFS_KEY_SESSION);
    if (nLen == 0 || sLen == 0) {
        Serial.println(F("[NVS] no prior nonces+session — starting fresh (FCntUp=0)"));
        return;
    }
    if (nLen != RADIOLIB_LORAWAN_NONCES_BUF_SIZE
        || sLen != RADIOLIB_LORAWAN_SESSION_BUF_SIZE) {
        Serial.printf("[NVS] buf size mismatch nonces=%u/%u session=%u/%u — ignoring\n",
                      (unsigned)nLen, (unsigned)RADIOLIB_LORAWAN_NONCES_BUF_SIZE,
                      (unsigned)sLen, (unsigned)RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
        return;
    }

    uint8_t nBuf[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
    gPrefs.getBytes(PREFS_KEY_NONCES, nBuf, sizeof(nBuf));
    int16_t st = node.setBufferNonces(nBuf);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("[NVS] setBufferNonces failed code=%d — fresh start\n", (int)st);
        return;
    }

    uint8_t sBuf[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
    gPrefs.getBytes(PREFS_KEY_SESSION, sBuf, sizeof(sBuf));
    st = node.setBufferSession(sBuf);
    if (st == RADIOLIB_ERR_NONE) {
        Serial.printf("[NVS] nonces+session restored — FCntUp=%lu\n",
                      (unsigned long)node.getFCntUp());
    } else {
        Serial.printf("[NVS] setBufferSession failed code=%d — fresh start\n", (int)st);
    }
}

// =============================================================================
// Watchdog (ESP32 Task WDT)
// =============================================================================
// Compatible with BOTH Arduino-ESP32 cores:
//   - 3.x (ESP-IDF 5.x): struct-based esp_task_wdt_init(&esp_task_wdt_config_t)
//   - 2.x (ESP-IDF 4.x): legacy esp_task_wdt_init(uint32_t timeout_s, bool panic)
// The original code only compiled on 3.x; on 2.0.17 the compiler errors with
// "'esp_task_wdt_config_t' was not declared in this scope". We branch on
// ESP_ARDUINO_VERSION_MAJOR so the right API is selected at compile time.
static void wdtInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // ---- Arduino-ESP32 3.x / ESP-IDF 5.x ----
    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdtCfg = {
        .timeout_ms     = (uint32_t)(WDT_TIMEOUT_S * 1000U),
        .idle_core_mask = (1 << 0) | (1 << 1),   // monitor both idle cores
        .trigger_panic  = true,
    };
    esp_err_t err = esp_task_wdt_init(&wdtCfg);
#else
    // ---- Arduino-ESP32 2.x / ESP-IDF 4.x ----
    // The core already armed the TWDT (default 5 s) on the per-core idle
    // tasks, and the legacy esp_task_wdt_init() refuses a second init
    // (ESP_ERR_INVALID_STATE). Unsubscribe the idle tasks and deinit first so
    // we can re-arm at WDT_TIMEOUT_S with panic enabled.
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
  #if !CONFIG_FREERTOS_UNICORE
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(1));
  #endif
    esp_task_wdt_deinit();
    esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
    if (err != ESP_OK) {
        Serial.printf("[WDT] init failed err=%d\n", (int)err);
    }
    esp_task_wdt_add(NULL);
    Serial.printf("[WDT] armed — timeout %us, panic on\n",
                  (unsigned)WDT_TIMEOUT_S);
}

// =============================================================================
// Ethernet (W5500) init / reinit
// =============================================================================
#if !BYPASS_SNMP
static void w5500HardReset() {
    pinMode(W5500_RST_PIN, OUTPUT);
    digitalWrite(W5500_RST_PIN, HIGH);
    delay(10);
    digitalWrite(W5500_RST_PIN, LOW);
    delay(5);
    digitalWrite(W5500_RST_PIN, HIGH);
    delay(150);
}

static bool ethernetInit() {
    Serial.println(F("[ETH] W5500 hardware reset")); Serial.flush();
    w5500HardReset();

    Serial.println(F("[ETH] Ethernet3 setCsPin/setRstPin + init(1) + begin")); Serial.flush();
    Ethernet.setCsPin(W5500_CS_PIN);
    Ethernet.setRstPin(W5500_RST_PIN);
    Ethernet.init(1);

    Ethernet.begin(MAC_ADDR, LOCAL_IP, SUBNET_MASK, GATEWAY_IP, DNS_IP);

    // Ethernet3's internal w5500::init() calls SPI.begin() with no arguments,
    // which pode resetar os pinos. Re-aplicamos o mapeamento do W5500 aqui.
    SPI.begin(W5500_SCK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN);

    Serial.printf("[ETH] W5500 readVersion=0x%02X (expect 0x04)\n",
                  w5500.readVersion());

    if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
        Serial.println(F("[ETH] FATAL: W5500 not detected (localIP=0.0.0.0)"));
        return false;
    }
    if (!Ethernet.link()) {
        Serial.println(F("[ETH] WARN: link DOWN (check cable)"));
    }
    Serial.print(F("[ETH] local IP = ")); Serial.println(Ethernet.localIP());

    return true;
}

static void udpRefresh() {
    snmpBindLocal();
    Serial.println(F("[UDP] socket refreshed post-TX"));
}
#endif // !BYPASS_SNMP

// =============================================================================
// Change detection on critical OIDs
// =============================================================================
static bool criticalChanged() {
    bool changed = false;
    for (uint8_t k = 0; k < 4; k++) {
        const int32_t cur  = (int32_t)gValues[CRITICAL_IDX[k]];
        const int32_t prev = gPrevValues[k];
        if (prev != PREV_SENTINEL && cur != prev) {
            Serial.printf("[EVT] %s changed: %ld -> %ld\n",
                          OID_NAMES[CRITICAL_IDX[k]], (long)prev, (long)cur);
            changed = true;
        }
        gPrevValues[k] = cur;
    }
    return changed;
}

static void handleCriticalAndUplink() {
    const bool     changed   = criticalChanged();
    const uint32_t now       = millis();
    const bool     heartbeat = (gLastUplinkMs == 0) ||
                               ((int32_t)(now - gLastUplinkMs) >= (int32_t)UPLINK_PERIODIC_MS);
    const bool     rateOk    = (gLastUplinkMs == 0) ||
                               ((int32_t)(now - gLastUplinkMs) >= (int32_t)UPLINK_MIN_INTERVAL_MS);

    if (heartbeat) {
        Serial.println(F("[UPLINK] heartbeat 60s"));
        buildPayload();
        loraSendUplink();
#if !BYPASS_SNMP
        udpRefresh();
#endif
    } else if (changed && rateOk) {
        Serial.println(F("[UPLINK] EVT critical change (confirmed)"));
        buildPayload();
        loraSendUplink(/*isCritical=*/true);
#if !BYPASS_SNMP
        udpRefresh();
#endif
    } else if (changed) {
        Serial.println(F("[UPLINK] critical changed but rate-limited — skipping TX"));
    }
}

// =============================================================================
// OLED
// =============================================================================
/*
static void oledInit() {
    pinMode(OLED_RST_PIN, OUTPUT);
    digitalWrite(OLED_RST_PIN, LOW);
    delay(20);
    digitalWrite(OLED_RST_PIN, HIGH);
    delay(20);

    gDisp.begin();
    gDisp.setBusClock(400000);
    gDisp.setContrast(128);
    gDisp.clearBuffer();
    gDisp.setFont(u8g2_font_6x10_tf);
    gDisp.drawStr(0, 10, "SNMPHELTEC");
    gDisp.drawStr(0, 24, "booting...");
    gDisp.sendBuffer();
}
*/
static const char* snmpStateLabel(SnmpCommState s) {
    switch (s) {
        case SNMP_S_OK:      return "OK";
        case SNMP_S_WAIT:    return "WAIT";
        case SNMP_S_PARTIAL: return "PART";
        case SNMP_S_FAIL:    return "FAIL";
        case SNMP_S_INIT:    default: return "INIT";
    }
}

static const char* loraStateLabel(LoraCommState s) {
    switch (s) {
        case LORA_S_TX:   return "TX";
        case LORA_S_OK:   return "OK";
        case LORA_S_FAIL: return "FAIL";
        case LORA_S_INIT: default: return "INIT";
    }
}
/*
static void oledDraw() {
    char buf[28];

    gDisp.clearBuffer();
    gDisp.setFont(u8g2_font_6x10_tf);

    gDisp.drawStr(0, 8, "SNMPHELTEC");
#if USE_TEMP_SENSOR
    char tbuf[10];
    if (gTempC10 == TEMP_OFFLINE_C10) {
        snprintf(tbuf, sizeof(tbuf), "OFF");
    } else {
        int a = abs((int)gTempC10);
        snprintf(tbuf, sizeof(tbuf), "%s%d.%01dC",
                 gTempC10 < 0 ? "-" : "", a / 10, a % 10);
    }
    gDisp.drawStr(128 - (int)strlen(tbuf) * 6, 8, tbuf);
#endif
    gDisp.drawHLine(0, 10, 128);

#if BYPASS_SNMP
    gDisp.drawStr(0, 22, "BYPASS  no W5500");
#else
    IPAddress ip = Ethernet.localIP();
    bool linkUp  = (Ethernet.link() != 0);
    snprintf(buf, sizeof(buf), "ETH %u.%u.%u.%u %s",
             ip[0], ip[1], ip[2], ip[3], linkUp ? "UP" : "DN");
    gDisp.drawStr(0, 22, buf);
#endif

    snprintf(buf, sizeof(buf), "SNMP %-4s ok=%lu",
             snmpStateLabel(gSnmpState), (unsigned long)gSnmpOkCount);
    gDisp.drawStr(0, 33, buf);

    uint32_t age = gSnmpLastOkMs ? (millis() - gSnmpLastOkMs) / 1000UL : 0UL;
    snprintf(buf, sizeof(buf), " err=%lu age=%lus rx=%u/13",
             (unsigned long)gSnmpFailCount,
             (unsigned long)age,
             (unsigned)gSnmpLastRecv);
    gDisp.drawStr(0, 43, buf);

    snprintf(buf, sizeof(buf), "LoRa %-4s tx=%lu ok=%lu",
             loraStateLabel(gLoraState),
             (unsigned long)gLoraTxCount,
             (unsigned long)gLoraDoneCount);
    gDisp.drawStr(0, 54, buf);

    uint32_t up = millis() / 1000UL;
    snprintf(buf, sizeof(buf), "up %02lu:%02lu:%02lu fc=%lu",
             up / 3600UL, (up / 60UL) % 60UL, up % 60UL,
             (unsigned long)node.getFCntUp());
    gDisp.drawStr(0, 64, buf);

    gDisp.sendBuffer();
}
*/

// =============================================================================
// Arduino entry points
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(50);
    Serial.println(F("\n=========================================="));
    Serial.println(F("   SNMPHELTEC  —  UPS SNMP -> LoRaWAN LA915"));
    Serial.println(F("=========================================="));
    Serial.printf("DevEUI:  %016llX\n", (unsigned long long)DEVEUI);
    Serial.printf("AppEUI:  %016llX\n", (unsigned long long)JOINEUI);
    Serial.printf("DevAddr: %08lX\n",   (unsigned long)DEVADDR);

    wdtInit();

//    Serial.println(F("[BOOT] oledInit")); Serial.flush();
//    oledInit();

    Serial.println(F("[BOOT] park CS lines HIGH")); Serial.flush();
    pinMode(W5500_CS_PIN, OUTPUT);
    digitalWrite(W5500_CS_PIN, HIGH);
    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH);

    // SPIs separadas para a V3.
    Serial.println(F("[BOOT] Configurando barramentos SPI (W5500 e LoRa)")); Serial.flush();
    SPI.begin(W5500_SCK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN);
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

#if BYPASS_SNMP
    Serial.println(F("[BYPASS] W5500 + SNMP disabled — synthetic data path"));
    gSnmpState = SNMP_S_OK;
#else
    if (!ethernetInit()) {
        while (true) { delay(1000); }
    }
    Serial.println(F("[BOOT] snmpSetup()")); Serial.flush();
    snmpSetup();
    Serial.println(F("[BOOT] snmpSetup done")); Serial.flush();
#endif
    tempSetup();
    esp_task_wdt_reset();

    if (!gPrefs.begin("lorawan", false)) {
        Serial.println(F("[NVS] FATAL: Preferences.begin(\"lorawan\") failed — "
                         "FCnt won't persist. Check NVS partition / erase flash."));
    } else {
        Serial.printf("[NVS] Preferences open (free entries: %u)\n",
                      (unsigned)gPrefs.freeEntries());
    }

    // ---------- LoRaWAN (RadioLib) ----------
    loraSetup();
    esp_task_wdt_reset();

    randomSeed(esp_random());

    gNextPollMs = millis() + 5000UL;
#if USE_TEMP_SENSOR
    gNextTempPollMs = millis() + 3000UL;
#endif
    Serial.println(F("[BOOT] running"));
//    oledDraw();
}

void loop() {
    esp_task_wdt_reset();

#if !BYPASS_SNMP
    gSnmp.loop();
    if (gFirstRespMs < 0 && gSnmpState == SNMP_S_WAIT) {
        for (int i = 0; i < 13; i++) {
            if (gValues[i] != (int)SNMP_SENTINEL) {
                gFirstRespMs = (int32_t)(millis() - gGetSentMs);
                break;
            }
        }
    }

    if (gSendPending && !gMidRetryDone && gFirstRespMs < 0
        && (int32_t)(millis() - gGetSentMs) >= (int32_t)(SNMP_RESPONSE_WAIT_MS / 2)) {
        gMidRetryDone = true;
        snmpResendInWindow();
    }
#endif

    const uint32_t now = millis();

    if ((int32_t)(now - gNextPollMs) >= 0) {
        gNextPollMs = now + SNMP_POLL_INTERVAL_MS;
#if BYPASS_SNMP
        fakeFillValues();
        gSnmpState     = SNMP_S_OK;
        gSnmpOkCount++;
        gSnmpLastOkMs  = now;
        gSnmpLastRecv  = 13;
        Serial.println(F("[BYPASS] synthetic UPS snapshot generated"));
        handleCriticalAndUplink();
#else
        snmpSendGet();
        gSendAtMs    = now + SNMP_RESPONSE_WAIT_MS;
        gSendPending = true;
#endif
    }

#if !BYPASS_SNMP
    if (gSendPending && (int32_t)(now - gSendAtMs) >= 0) {
        gSendPending = false;
        snmpEvaluate();

        if (gSnmpState == SNMP_S_OK) {
            gConsecFails = 0;
            handleCriticalAndUplink();
        } else {
            gConsecFails++;
            Serial.printf("[SNMP] %u/13 — uplink BLOCKED (consecFails=%u)\n",
                          (unsigned)gSnmpLastRecv,
                          (unsigned)gConsecFails);
            if (gConsecFails >= SNMP_RESTART_AFTER_FAILS) {
                Serial.printf("[ETH] %u consecutive failures — esp_restart()\n",
                              (unsigned)gConsecFails);
                Serial.flush();
                delay(50);
                esp_restart();
            }
        }
    }
#endif

#if USE_TEMP_SENSOR
    if (!gSendPending && (int32_t)(now - gNextTempPollMs) >= 0) {
        gNextTempPollMs = now + TEMP_POLL_INTERVAL_MS;
        tempPoll();
    }
#endif

//    if ((int32_t)(now - gOledNextDrawMs) >= 0) {
//        gOledNextDrawMs = now + 500UL;
//        oledDraw();
//    }

}
