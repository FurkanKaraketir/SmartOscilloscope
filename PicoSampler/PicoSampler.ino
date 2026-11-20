// Smart Oscilloscope - sampler side (Raspberry Pi Pico 2 W)
// Samples A0 (GP26, 0-3.3 V) and sends one text line per sample, "mV\n", over UART to the
// CrowPanel 4.3" display (sketch: CrowPanelScope). A pot on A1 picks the sample period (the time
// zoom); the chosen period is sent as "T<microseconds>\n" when it changes and once a second.
// Slow periods stream live. Periods faster than the display can draw are captured as a burst of one
// screen width, then sent at a rate the display keeps up with, preceded by "F\n" (frame start).
//
// Arduino IDE: Boards Manager URL
//   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
//   Board: "Raspberry Pi Pico 2W"
// Wiring: Pico GP0 (UART0 TX, pin 1) -> CrowPanel UART1 RX (IO18), Pico GND -> CrowPanel GND.
//   Pot (any 1k-100k): ends to 3V3 OUT (pin 36) and GND (pin 33), wiper to GP27/A1 (pin 32).
// Input must stay inside 0-3.3 V.
// Test signal: jumper GP15 (pin 20) -> GP26/A0 (pin 31) to see a 100 Hz, 50% square wave, 0-3.3 V.
// Roasts: core 1 samples; core 0 (the only core allowed to use WiFi) watches the signal and, when it
// changes shape or timebase, asks OpenAI for a one-line sarcastic review, sent to the display as "R<text>\n".
// Copy secrets.example.h to secrets.h and fill in WiFi + OpenAI key. Library: ArduinoJson 7.

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

const char *MODEL = "gpt-5-nano";      // OpenAI's cheapest at the time of writing
const uint32_t ASK_GAP_MS = 20000;     // at most one roast per 20 s, however much the signal changes
const int REVIEW_CHARS = 78;           // one 480 px display line holds 80 size-1 chars; keep a margin
// Defined above every function: the Arduino IDE inserts auto-prototypes before the first one.
struct Shape {
  int lo, hi, mean, duty;  // raw ADC counts; duty = % of samples above the middle
  float cycles;            // full cycles across the frame, measured between first and last trigger edge
  const char *type;
};
// api.openai.com chains to GTS Root R4 (checked 2026-11). If OpenAI changes CA, roasts show "HTTP -1".
const char GTS_ROOT_R4[] = R"(
-----BEGIN CERTIFICATE-----
MIIDejCCAmKgAwIBAgIQf+UwvzMTQ77dghYQST2KGzANBgkqhkiG9w0BAQsFADBX
MQswCQYDVQQGEwJCRTEZMBcGA1UEChMQR2xvYmFsU2lnbiBudi1zYTEQMA4GA1UE
CxMHUm9vdCBDQTEbMBkGA1UEAxMSR2xvYmFsU2lnbiBSb290IENBMB4XDTIzMTEx
NTAzNDMyMVoXDTI4MDEyODAwMDA0MlowRzELMAkGA1UEBhMCVVMxIjAgBgNVBAoT
GUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBMTEMxFDASBgNVBAMTC0dUUyBSb290IFI0
MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAE83Rzp2iLYK5DuDXFgTB7S0md+8Fhzube
Rr1r1WEYNa5A3XP3iZEwWus87oV8okB2O6nGuEfYKueSkWpz6bFyOZ8pn6KY019e
WIZlD6GEZQbR3IvJx3PIjGov5cSr0R2Ko4H/MIH8MA4GA1UdDwEB/wQEAwIBhjAd
BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDwYDVR0TAQH/BAUwAwEB/zAd
BgNVHQ4EFgQUgEzW63T/STaj1dj8tT7FavCUHYwwHwYDVR0jBBgwFoAUYHtmGkUN
l8qJUC99BM00qP/8/UswNgYIKwYBBQUHAQEEKjAoMCYGCCsGAQUFBzAChhpodHRw
Oi8vaS5wa2kuZ29vZy9nc3IxLmNydDAtBgNVHR8EJjAkMCKgIKAehhxodHRwOi8v
Yy5wa2kuZ29vZy9yL2dzcjEuY3JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqG
SIb3DQEBCwUAA4IBAQAYQrsPBtYDh5bjP2OBDwmkoWhIDDkic574y04tfzHpn+cJ
odI2D4SseesQ6bDrarZ7C30ddLibZatoKiws3UL9xnELz4ct92vID24FfVbiI1hY
+SW6FoVHkNeWIP0GCbaM4C6uVdF5dTUsMVs/ZbzNnIdCp5Gxmx5ejvEau8otR/Cs
kGN+hr/W5GvT1tMBjgWKZ1i4//emhA1JG1BbPzoLJQvyEotc03lXjTaCzv8mEbep
8RqZ7a2CPsgRbuvTPBwcOMBBmuFeU88+FSBX6+7iP0il8b4Z0QFqIwwMHfs/L6K1
vepuoxtGzi4CZ68zJpiq1UvSqTbFJjtbD4seiMHl
-----END CERTIFICATE-----
)";

const uint8_t PIN = A0, POT_PIN = A1;
const uint8_t TEST_PWM_PIN = 15;  // 100 Hz is arduino-pico's analogWrite minimum
// Display grid is 50 samples per division, so these give
// 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000 ms per div. 10 us is near the ADC's limit (~3 us per read).
const uint32_t US_PER_SAMPLE[] = {10, 20, 40, 100, 200, 400, 1000, 2000, 4000, 10000, 20000};
const int STEPS = sizeof(US_PER_SAMPLE) / sizeof(US_PER_SAMPLE[0]);
const int DEFAULT_STEP = 6;  // 1 ms per sample, the display's default
const uint32_t LINE_US = 1000;  // fastest line rate the display is known to keep up with; lower it if yours does
const int FRAME = 450;          // samples per burst = CrowPanelScope's PLOT_W
const uint32_t BAUD = 921600;   // must match CrowPanelScope

// Pot position -> step, with hysteresis so a pot sitting on a boundary doesn't flicker.
int potStep(int raw, int cur) {
  const int BAND = 4096 / STEPS;  // not "BIN": Arduino's Print.h #defines BIN
  if (abs(raw - (cur * BAND + BAND / 2)) > BAND * 6 / 10) cur = constrain(raw / BAND, 0, STEPS - 1);
  return cur;
}

void waitUntil(uint32_t &next, uint32_t period) {
  while ((int32_t)(micros() - next) < 0) {}
  next += period;
}

void sendMv(int raw) {
  Serial1.print(raw * 3300UL / 4095);  // ADC reference is the 3.3 V rail
  Serial1.write('\n');
}

// Core handoff: core 0 sets wantSnap, core 1 copies its next full frame into snap and clears it.
volatile bool wantSnap = false, reviewReady = false;
uint16_t snap[FRAME];
volatile uint32_t snapUs;
char review[REVIEW_CHARS + 1];

void shareFrame(const uint16_t *buf, uint32_t us) {
  if (!wantSnap) return;
  memcpy(snap, buf, sizeof snap);
  snapUs = us;
  __sync_synchronize();
  wantSnap = false;
}

// Model text -> one protocol-safe line: newlines become spaces (they would end the UART line), and text
// past REVIEW_CHARS is cut at the last whole word, so the display never clips mid-word.
void toLine(const char *in, char *out) {
  int n = 0, lastSpace = -1;
  for (; *in && n < REVIEW_CHARS; in++, n++) {
    out[n] = (*in == '\n' || *in == '\r') ? ' ' : *in;
    if (out[n] == ' ') lastSpace = n;
  }
  if (*in && *in != ' ' && lastSpace > REVIEW_CHARS / 2) n = lastSpace;  // too long: drop the half word
  while (n && out[n - 1] == ' ') n--;
  out[n] = 0;
}

// Rough waveform classifier for one frame. Thresholds checked against synthetic waves in setup().
Shape analyze(const uint16_t *v) {
  Shape s = {4095, 0, 0, 0, 0, "flat DC"};
  long sum = 0;
  for (int i = 0; i < FRAME; i++) s.lo = min(s.lo, (int)v[i]), s.hi = max(s.hi, (int)v[i]), sum += v[i];
  s.mean = sum / FRAME;
  int amp = s.hi - s.lo, mid = (s.lo + s.hi) / 2;
  if (amp < 60) return s;  // under ~50 mV of wiggle
  int high = 0, rails = 0, rising = 0, falling = 0, edges = 0, turns = 0, first = 0, last = 0, ext = v[0];
  bool up = v[0] > mid, rise = true;
  float ms = 0;
  for (int i = 0; i < FRAME; i++) {
    high += v[i] > mid;
    rails += v[i] < s.lo + amp / 10 || v[i] > s.hi - amp / 10;
    float d = (v[i] - mid) / (amp / 2.0f);
    ms += d * d;
    if (i) rising += v[i] > v[i - 1] + 2, falling += v[i] < v[i - 1] - 2;
    if (up ? v[i] < mid - amp / 4 : v[i] > mid + amp / 4) {  // Schmitt trigger edge: ignores small noise
      up = !up, last = i;
      if (++edges == 1) first = i;
    }
    if (rise ? v[i] > ext : v[i] < ext) ext = v[i];
    else if (abs(v[i] - ext) > amp / 8) rise = !rise, ext = v[i], turns++;  // direction change bigger than amp/8
  }
  s.cycles = edges > 1 ? (edges - 1) / 2.0f * FRAME / (last - first) : 0;
  s.duty = high * 100 / FRAME;
  float rms = sqrtf(ms / FRAME);  // of the wave scaled to -1..1: square 1, sine 0.71, triangle/saw 0.58
  if (turns > edges * 3 / 2 + 2) s.type = "noise";  // clean waves turn about once per edge
  else if (edges < 2) s.type = "slow drift, under one cycle on screen";
  else if (rails > FRAME * 8 / 10) s.type = abs(s.duty - 50) <= 5 ? "square wave" : "pulse train";
  else if (rms > 0.64f) s.type = "sine wave";
  else if (rising > 3 * falling || falling > 3 * rising) s.type = "sawtooth";
  else s.type = "triangle wave";
  return s;
}

float volts(int raw) { return raw * 3.3f / 4095; }

void askRoast(const Shape &s, uint32_t us, char *out) {
  String trace;  // 75 points, each the average of 6 samples: smoother than skipping, same shape
  for (int i = 0; i < FRAME; i += 6) {
    int sum = 0;
    for (int j = 0; j < 6; j++) sum += snap[i + j];
    trace += String(sum * 3300L / 6 / 4095) + (i + 6 < FRAME ? "," : "");
  }
  float windowS = FRAME * us * 1e-6f;
  char facts[400];
  snprintf(facts, sizeof facts,
           "Signal: %s. Frequency: %s%.3g Hz. Range %.2f to %.2f V (%.2f Vpp), average %.2f V, above the middle "
           "%d%% of the time. Screen spans %.3g ms. Trace in mV, 75 evenly spaced points: ",
           s.type, s.cycles ? "" : "unknown, under ", s.cycles ? s.cycles / windowS : 1 / windowS, volts(s.lo),
           volts(s.hi), volts(s.hi - s.lo), volts(s.mean), s.duty, windowS * 1000);
  JsonDocument req;
  req["model"] = MODEL;
  req["reasoning_effort"] = "minimal";
  req["max_completion_tokens"] = 300;
  req["messages"][0]["role"] = "system";
  req["messages"][0]["content"] =
    "You are Scopey, a burned-out 1980s bench oscilloscope who now does roast comedy about whatever signal "
    "is on your screen. Reply with ONE savage, witty line of at most 70 characters: that is a hard limit, "
    "shorter is funnier. Name something real about the signal (its shape, frequency, voltage or duty cycle) "
    "and twist it into an electronics pun or a comparison to something everyday. Be mean to the signal, "
    "never to people. Plain ASCII only: no emoji, no quotes, no hashtags. Never reuse your previous joke.";
  req["messages"][1]["role"] = "user";
  req["messages"][1]["content"] = String(facts) + trace + ". Your previous roast: " + (out[0] ? out : "none");
  String body;
  serializeJson(req, body);

  static WiFiClientSecure tls;
  static X509List ca(GTS_ROOT_R4);
  tls.setTrustAnchors(&ca);
  HTTPClient http;
  http.setTimeout(15000);
  http.begin(tls, "https://api.openai.com/v1/chat/completions");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " OPENAI_KEY);
  int code = http.POST(body);
  if (code != 200) {
    Serial.printf("OpenAI HTTP %d: %s\n", code, http.getString().c_str());
    snprintf(out, REVIEW_CHARS + 1, "(critic unavailable: HTTP %d)", code);
  } else {
    JsonDocument res;
    deserializeJson(res, http.getString());
    toLine(res["choices"][0]["message"]["content"] | "(critic speechless)", out);
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  if (potStep(0, 5) != 0 || potStep(4095, 0) != STEPS - 1 || potStep(2046, 5) != 5 || potStep(2242, 5) != 5 ||
      potStep(2300, 5) != 6)
    Serial.println("self-check FAILED: potStep");
  char t[REVIEW_CHARS + 1];
  toLine("a\nb\rc", t);
  bool ok = !strcmp(t, "a b c");
  String longText;
  for (int i = 0; i < 20; i++) longText += "word ";
  toLine(longText.c_str(), t);
  if (!ok || strlen(t) > REVIEW_CHARS || !longText.startsWith(t) || !String(t).endsWith("word"))
    Serial.println("self-check FAILED: toLine");

  const char *want[] = {"square wave", "pulse train", "sine wave", "triangle wave", "sawtooth", "flat DC", "noise"};
  for (int k = 0; k < 7; k++) {  // 5 cycles of 90 samples each
    for (int i = 0; i < FRAME; i++) {
      int p = i % 90;
      snap[i] = k == 0 ? (p < 45 ? 4000 : 50)
              : k == 1 ? (p < 15 ? 4000 : 50)
              : k == 2 ? 2025 + 1975 * sinf(2 * PI * p / 90)
              : k == 3 ? 50 + (p < 45 ? p : 90 - p) * 88
              : k == 4 ? 50 + p * 44
              : k == 5 ? 2000
                       : 1000 + random(2000);
    }
    Shape s = analyze(snap);
    if (strcmp(s.type, want[k]) || (k < 5 && fabsf(s.cycles - 5) > 0.2f))
      Serial.printf("self-check FAILED: analyze %s -> %s, %.2f cycles\n", want[k], s.type, s.cycles);
  }
}

// Core 0: decide when a roast is worth it (first frame, or shape/level/timebase changed) and fetch it.
void loop() {
  static Shape p = {-1};
  static uint32_t pUs, lastAsk;
  delay(2000);
  if (WiFi.status() != WL_CONNECTED) return (void)WiFi.begin(WIFI_SSID, WIFI_PASS);  // blocks core 0 only
  if (time(nullptr) < 1700000000 && !(NTP.begin("pool.ntp.org"), NTP.waitSet())) return;  // TLS needs the date

  wantSnap = true;
  while (wantSnap) delay(10);
  __sync_synchronize();
  Shape s = analyze(snap);
  bool changed = p.lo < 0 || s.type != p.type || abs(s.lo - p.lo) > 300 || abs(s.hi - p.hi) > 300 ||
                 fabsf(s.cycles - p.cycles) > p.cycles / 5 + 0.5f || snapUs != pUs;
  if (!changed || (lastAsk && millis() - lastAsk < ASK_GAP_MS)) return;
  p = s, pUs = snapUs, lastAsk = millis();

  while (reviewReady) delay(10);  // core 1 hasn't sent the previous one yet
  askRoast(s, snapUs, review);
  Serial.printf("Roast: %s\n", review);
  __sync_synchronize();
  reviewReady = true;
}

// Core 1: sampling and the UART link, untouched by network stalls on core 0.
void setup1() {
  analogReadResolution(12);
  analogWriteFreq(100);
  analogWrite(TEST_PWM_PIN, 128);  // default range 0..255, so 50% duty
  Serial1.begin(BAUD);  // UART0 on GP0 (TX) / GP1 (RX)
}

void loop1() {
  static uint32_t next = micros();
  static int step = DEFAULT_STEP;
  static uint16_t buf[FRAME];
  uint32_t us = US_PER_SAMPLE[step];
  if (us >= LINE_US) {
    static int i = 0;
    waitUntil(next, us);
    sendMv(buf[i] = analogRead(PIN));
    if (++i == FRAME) i = 0, shareFrame(buf, us);
  } else {
    uint32_t t = micros();
    for (int i = 0; i < FRAME; i++) {
      waitUntil(t, us);
      buf[i] = analogRead(PIN);
    }
    shareFrame(buf, us);
    Serial1.write("F\n");
    next = micros();
    for (int i = 0; i < FRAME; i++) {
      waitUntil(next, LINE_US);
      sendMv(buf[i]);
    }
  }

  static uint32_t lastPot = 0, lastSent = 0;
  if (millis() - lastPot >= 50) {
    lastPot = millis();
    int s = potStep(analogRead(POT_PIN), step);
    if (s != step || millis() - lastSent >= 1000) {  // resend so a display that reset catches up
      step = s;
      lastSent = millis();
      Serial1.printf("T%lu\n", US_PER_SAMPLE[step]);
    }
  }
  if (reviewReady) {
    __sync_synchronize();
    Serial1.printf("R%s\n", review);
    reviewReady = false;
  }
}
