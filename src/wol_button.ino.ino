/*
  ESP32 + W5500 Wake-on-LAN Button Box
  - WiFi OFF (power saving)
  - Bluetooth OFF (best-effort btStop)
  - LEARN MODE (hold button at boot): sniff raw Ethernet frame via W5500 MACRAW,
    extract SOURCE MAC, store to NVS.
  - NORMAL MODE: press button -> send WOL to stored MAC (3 packets).

  Wiring (VSPI):
    W5500 V    -> 3V3
    W5500 G    -> GND
    W5500 RST  -> GPIO4
    W5500 MISo -> GPIO19 (MISO)
    W5500 MOSI -> GPIO23 (MOSI)
    W5500 SCK  -> GPIO18
    W5500 CS   -> GPIO5
    W5500 INT  -> not used

  Button:
    GPIO27 -> button -> GND  (INPUT_PULLUP)
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_log.h>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include <Preferences.h>

// ---------- PINS ----------
static const int PIN_BTN       = 27;
static const int PIN_W5500_CS  = 5;
static const int PIN_W5500_RST = 4;

// ---------- STATIC LINK-LOCAL (NO DHCP HANGS) ----------
IPAddress LOCAL_IP    (169, 254, 10, 10);
IPAddress SUBNET_MASK (255, 255, 0, 0);

// ---------- WOL ----------
IPAddress BROADCAST_IP(255, 255, 255, 255);
const uint16_t WOL_PORT = 9;
const uint16_t LOCAL_UDP_PORT = 40000;

// ---------- RATE / RELIABILITY ----------
const int WOL_BURST = 3;          // send 3 packets per press for reliability
const uint32_t PRESS_LOCK_MS = 50;

// ---------- STORAGE ----------
Preferences prefs;
byte learnedMAC[6] = {0};
bool hasMAC = false;

// ---------- UDP ----------
EthernetUDP Udp;

// ============================================================
//  ESP32 radios OFF
// ============================================================
void disableRadios()
{
  esp_log_level_set("wifi", ESP_LOG_NONE);

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true, true);
  delay(10);
  esp_wifi_stop();
  esp_wifi_deinit();

#if defined(btStop)
  btStop();
#endif
}

// ============================================================
//  W5500 low-level SPI (for MACRAW learn mode)
//  Control byte: (BLOCK << 3) | (RWB << 2) | OM
//    - RWB: 0=READ, 1=WRITE
//    - OM : 0=VDM
//    - BLOCK: 0x00 common, 0x01 s0 reg, 0x02 s0 tx, 0x03 s0 rx, ...
// ============================================================
static inline uint8_t ctrlByte(uint8_t block, bool write)
{
  return (uint8_t)((block << 3) | ((write ? 1 : 0) << 2) | 0x00);
}

uint8_t w5500Read(uint8_t block, uint16_t addr)
{
  digitalWrite(PIN_W5500_CS, LOW);
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t)(addr & 0xFF));
  SPI.transfer(ctrlByte(block, false));
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_W5500_CS, HIGH);
  return v;
}

void w5500Write(uint8_t block, uint16_t addr, uint8_t v)
{
  digitalWrite(PIN_W5500_CS, LOW);
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t)(addr & 0xFF));
  SPI.transfer(ctrlByte(block, true));
  SPI.transfer(v);
  digitalWrite(PIN_W5500_CS, HIGH);
}

uint16_t w5500Read16(uint8_t block, uint16_t addr)
{
  uint16_t hi = w5500Read(block, addr);
  uint16_t lo = w5500Read(block, addr + 1);
  return (uint16_t)((hi << 8) | lo);
}

void w5500Write16(uint8_t block, uint16_t addr, uint16_t v)
{
  w5500Write(block, addr,     (uint8_t)(v >> 8));
  w5500Write(block, addr + 1, (uint8_t)(v & 0xFF));
}

void w5500Reset()
{
  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(100);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(500);
}

bool w5500Alive()
{
  // VERSIONR = 0x0039, should be 0x04 on W5500
  uint8_t v = w5500Read(0x00, 0x0039);
  return (v == 0x04);
}

// ============================================================
//  MACRAW learn mode (Socket 0)
//  Socket 0 register block = 0x01, RX block = 0x03
// ============================================================
static const uint8_t S0_REG = 0x01;
static const uint8_t S0_RX  = 0x03;

// Socket 0 registers (offsets)
static const uint16_t Sn_MR       = 0x0000;
static const uint16_t Sn_CR       = 0x0001;
static const uint16_t Sn_SR       = 0x0003;
static const uint16_t Sn_RX_RSR   = 0x0026;
static const uint16_t Sn_RX_RD    = 0x0028;
static const uint16_t Sn_RXBUF_SZ = 0x001E;

// Commands
static const uint8_t CR_OPEN = 0x01;
static const uint8_t CR_RECV = 0x40;
static const uint8_t MR_MACRAW = 0x04;

void w5500Sock0Command(uint8_t cmd)
{
  w5500Write(S0_REG, Sn_CR, cmd);
  // wait until command clears
  while (w5500Read(S0_REG, Sn_CR) != 0) delay(1);
}

bool macrawInit()
{
  // set RX buffer size for socket0 to 2KB (0x02) (enough for our sniffing)
  w5500Write(S0_REG, Sn_RXBUF_SZ, 0x02);

  // set MACRAW mode
  w5500Write(S0_REG, Sn_MR, MR_MACRAW);
  w5500Sock0Command(CR_OPEN);

  // crude check: SR should become SOCK_MACRAW (0x42) on many stacks,
  // but we won't rely on it. We'll just see if RX_RSR ever fills.
  return true;
}

// Read a single frame header enough to extract SRC MAC.
// W5500 MACRAW RX format includes 2-byte length then frame bytes.
// We'll read length (2 bytes) then first 14 bytes of frame (Ethernet header).
bool macrawReadSrcMac(byte outMac[6])
{
  uint16_t rxSize = w5500Read16(S0_REG, Sn_RX_RSR);
  if (rxSize < 2) return false;

  uint16_t rd = w5500Read16(S0_REG, Sn_RX_RD);

  // Helper to read from RX buffer at (rd) (VDM)
  auto readRxByte = [&](uint16_t raddr) -> uint8_t {
    // RX buffer address uses the raw pointer value; W5500 handles wrap internally
    return w5500Read(S0_RX, raddr);
  };

  // Read length (2 bytes)
  uint8_t lenHi = readRxByte(rd);
  uint8_t lenLo = readRxByte(rd + 1);
  uint16_t frameLen = (uint16_t)((lenHi << 8) | lenLo);

  if (frameLen < 14) {
    // consume it anyway
    uint16_t consume = (uint16_t)(frameLen + 2);
    w5500Write16(S0_REG, Sn_RX_RD, rd + consume);
    w5500Sock0Command(CR_RECV);
    return false;
  }

  // Read first 14 bytes of Ethernet frame (after the 2-byte length)
  uint16_t base = rd + 2;

  // Ethernet header: [0..5]=DST MAC, [6..11]=SRC MAC, [12..13]=Ethertype
  outMac[0] = readRxByte(base + 6);
  outMac[1] = readRxByte(base + 7);
  outMac[2] = readRxByte(base + 8);
  outMac[3] = readRxByte(base + 9);
  outMac[4] = readRxByte(base + 10);
  outMac[5] = readRxByte(base + 11);

  // consume whole frame from RX buffer (length + 2)
  uint16_t consume = (uint16_t)(frameLen + 2);
  w5500Write16(S0_REG, Sn_RX_RD, rd + consume);
  w5500Sock0Command(CR_RECV);

  return true;
}

// ============================================================
//  WOL sender (EthernetUDP)
// ============================================================
void sendWOLOnce(const byte mac[6])
{
  byte packet[102];

  for (int i = 0; i < 6; i++) packet[i] = 0xFF;
  for (int i = 1; i <= 16; i++)
    for (int j = 0; j < 6; j++)
      packet[i * 6 + j] = mac[j];

  int ok1 = Udp.beginPacket(BROADCAST_IP, WOL_PORT);
  size_t wrote = Udp.write(packet, sizeof(packet));
  int ok2 = Udp.endPacket();

  Serial.printf("WOL: begin=%d wrote=%u end=%d -> %02X:%02X:%02X:%02X:%02X:%02X\n",
                ok1, (unsigned)wrote, ok2,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ============================================================
//  Setup / Modes
// ============================================================
void loadStoredMac()
{
  prefs.begin("wolbox", true);
  hasMAC = prefs.getBool("valid", false);
  if (hasMAC) {
    prefs.getBytes("mac", learnedMAC, 6);
  }
  prefs.end();

  if (hasMAC) {
    Serial.printf("Stored MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  learnedMAC[0], learnedMAC[1], learnedMAC[2],
                  learnedMAC[3], learnedMAC[4], learnedMAC[5]);
  } else {
    Serial.println("No MAC stored yet.");
  }
}

void saveMac(const byte mac[6])
{
  prefs.begin("wolbox", false);
  prefs.putBytes("mac", mac, 6);
  prefs.putBool("valid", true);
  prefs.end();
}

void enterLearnMode()
{
  Serial.println("\n=== LEARN MODE ===");
  Serial.println("Keep connected to the PC NIC while PC is ON.");
  Serial.println("Trigger traffic on PC (unplug/replug cable, disable/enable NIC, open a webpage).");
  Serial.println("Waiting for first Ethernet frame...");

  // Low-level W5500 MACRAW sniff
  if (!w5500Alive()) {
    Serial.println("W5500 not responding (VERSIONR not 0x04). Check wiring.");
    while (true) delay(1000);
  }

  macrawInit();

  byte mac[6];
  uint32_t start = millis();
  while (true) {
    if (macrawReadSrcMac(mac)) {
      Serial.printf("LEARNED PC MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

      saveMac(mac);
      Serial.println("Saved to flash. Rebooting in 2 seconds...");
      delay(2000);
      ESP.restart();
    }

    if (millis() - start > 30000) {
      Serial.println("No frames seen in 30s. Make the PC generate traffic and try again.");
      start = millis();
    }

    delay(10);
  }
}

void setupNormalMode()
{
  Serial.println("\n=== NORMAL MODE ===");

  // Bring up Ethernet with static link-local (no DHCP hangs)
  Ethernet.init(PIN_W5500_CS);

  uint64_t chipid = ESP.getEfuseMac();
  byte deviceMac[6] = {
    0x02,
    (byte)(chipid >> 32),
    (byte)(chipid >> 24),
    (byte)(chipid >> 16),
    (byte)(chipid >> 8),
    (byte)(chipid)
  };

  Ethernet.begin(deviceMac, LOCAL_IP, IPAddress(0,0,0,0), IPAddress(0,0,0,0), SUBNET_MASK);
  delay(300);

  Serial.print("WOL box IP: ");
  Serial.println(Ethernet.localIP());

  Udp.begin(LOCAL_UDP_PORT);

  if (!hasMAC) {
    Serial.println("WARNING: No learned MAC stored. Hold button at boot to learn.");
  } else {
    Serial.println("Ready. Press button to send WOL.");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  disableRadios();

  pinMode(PIN_BTN, INPUT_PULLUP);

  // SPI init + stable slow freq for both MACRAW + Ethernet lib
  pinMode(PIN_W5500_CS, OUTPUT);
  digitalWrite(PIN_W5500_CS, HIGH);

  SPI.begin(18, 19, 23, PIN_W5500_CS);
  SPI.setFrequency(1000000);

  w5500Reset();

  loadStoredMac();

  // If button held at boot, learn MAC
  if (digitalRead(PIN_BTN) == LOW) {
    enterLearnMode();
  }

  setupNormalMode();
}

void loop()
{
  // One press = one action (burst WOL_BURST packets for reliability)
  if (digitalRead(PIN_BTN) == LOW) {
    if (!hasMAC) {
      Serial.println("No MAC stored. Hold button at boot to learn.");
    } else {
      for (int i = 0; i < WOL_BURST; i++) {
        sendWOLOnce(learnedMAC);
        delay(25);
      }
      Serial.println("WOL SENT.");
    }

    // wait release
    while (digitalRead(PIN_BTN) == LOW) delay(1);
    delay(PRESS_LOCK_MS);
  }
}
