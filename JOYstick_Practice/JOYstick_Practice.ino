#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/* ------------ Joystick pins (ADC-capable!) ------------ */
#define VRX_PIN    6   // ADC1_CH4
#define VRY_PIN    5  // ADC1_CH5
#define JOY_SW     4    // push-switch (to GND), use INPUT_PULLUP

/* ------------ nRF24 pins (ESP32) ------------ */
#define CE_PIN     7   // choose a free GPIO (not 6..11)
#define CSN_PIN    15
#define SCK_PIN    18
#define MISO_PIN   16   // (19 is the SPI default; 16 also works if wired)
#define MOSI_PIN   17

RF24 radio(CE_PIN, CSN_PIN);
const byte addr[6] = "00001";   // must match STM32

// 6-byte control packet the STM32 expects
struct __attribute__((packed)) ControlPkt {
  uint8_t hdr;    // 0xAA
  uint8_t seq;    // rolls over
  int8_t  vx;     // forward/back  [-127..127]
  int8_t  wz;     // turn          [-127..127]
  uint8_t flags;  // bit0 = e-stop (joystick press)
  uint8_t crc;    // CRC-8 over first 5 bytes (poly 0x07)
};

static uint8_t g_seq = 0;

/* ---------- helpers ---------- */
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// CRC-8 (poly 0x07, init 0x00)
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// map raw ADC (0..4095) to -1..+1 around a center
float adcToNorm(int raw, int mid) {
  float x = (raw - mid) / 2048.0f;   // 12-bit ADC
  return clampf(x, -1.0f, 1.0f);
}

// deadband + gentle expo
float shapeInput(float x, float deadband=0.06f, float expo=0.35f) {
  if (fabsf(x) < deadband) return 0.0f;
  float s = (x < 0) ? -1.0f : 1.0f;
  float m = (fabsf(x) - deadband) / (1.0f - deadband);
  m = clampf(m, 0.0f, 1.0f);
  float y = (1.0f - expo)*m + expo*(m*m*m);
  return s * y;
}

// centers (auto-calibrated on boot)
int MID_X = 2048;
int MID_Y = 2048;

void setup() {
  Serial.begin(115200);
  pinMode(JOY_SW, INPUT_PULLUP);

  analogReadResolution(12);     // 0..4095
  analogSetAttenuation(ADC_11db);

  // Start SPI on specified pins
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

  // Radio init
  if (!radio.begin()) {
    Serial.println("nRF24 init failed. Check wiring/power.");
    while (1) delay(10);
  }
  radio.setPALevel(RF24_PA_LOW);     // start low; raise if stable
  radio.setChannel(100);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);    // 1-byte CRC
  radio.setAutoAck(true);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(ControlPkt)); // 6 bytes
  radio.openWritingPipe(addr);
  radio.stopListening();

  Serial.println("TX ready (50 Hz). Hold joystick still for auto-center...");
  delay(300);

  // Auto-center
  long sx=0, sy=0; const int N=32;
  for (int i=0;i<N;i++){ sx += analogRead(VRX_PIN); sy += analogRead(VRY_PIN); delay(5); }
  MID_X = sx / N; MID_Y = sy / N;
  Serial.printf("Centers: X=%d  Y=%d\n", MID_X, MID_Y);
}

void loop() {
  static uint32_t last = 0;
  const uint32_t periodMs = 20;   // 50 Hz
  uint32_t now = millis();
  if (now - last < periodMs) return;
  last = now;

  // Read joystick
  int rawX = analogRead(VRX_PIN);
  int rawY = analogRead(VRY_PIN);
  bool swPressed = (digitalRead(JOY_SW) == LOW); // pull-up -> LOW when pressed

  // Normalize to -1..+1; invert Y so push-up = forward positive
  float w = adcToNorm(rawX, MID_X);    // turning
  float v = -adcToNorm(rawY, MID_Y);   // forward/back

  // Shape + limit
  w = shapeInput(w, 0.06f, 0.35f);
  v = shapeInput(v, 0.06f, 0.35f);
  const float LIMIT = 0.60f;           // raise after first tests
  w = clampf(w, -LIMIT, LIMIT);
  v = clampf(v, -LIMIT, LIMIT);

  // Quantize to packet units
  int8_t vx = (int8_t)roundf(v * 127.0f);
  int8_t wz = (int8_t)roundf(w * 127.0f);
  uint8_t flags = swPressed ? 0x01 : 0x00;

  // Build packet
  ControlPkt pkt;
  pkt.hdr   = 0xAA;
  pkt.seq   = g_seq++;
  pkt.vx    = vx;
  pkt.wz    = wz;
  pkt.flags = flags;
  pkt.crc   = crc8(reinterpret_cast<const uint8_t*>(&pkt), 5);

  // Send
  bool ok = radio.write(&pkt, sizeof(pkt));

  // Light debug at 5 Hz
  static uint32_t lastPrint = 0;
  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.printf("seq=%3u  vx=%4d  wz=%4d  flags=%02X  %s  rawX=%4d rawY=%4d\n",
                  pkt.seq, pkt.vx, pkt.wz, pkt.flags, ok ? "OK" : "TXERR",
                  rawX, rawY);
  }
} 