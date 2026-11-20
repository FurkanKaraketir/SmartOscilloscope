// Smart Oscilloscope - display side (Elecrow CrowPanel 4.3" ESP32-S3 HMI, 480x272)
// Receives "mV\n" sample lines and "T<us per sample>\n" time-zoom lines
// and "F\n" burst-frame starts and "R<text>\n" sarcastic reviews from PicoSampler over UART1 and draws them as a sweeping trace,
// the way an analog scope draws: the trace moves left to right and overwrites the previous sweep.
//
// Arduino IDE: Boards Manager "esp32" by Espressif (3.x), library "GFX Library for Arduino" (1.6.x)
//   Board: ESP32S3 Dev Module, PSRAM: QSPI PSRAM, Flash Size: 4MB,
//   Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
// Wiring: CrowPanel UART1 RX (IO18) <- Pico GP0, GND <- Pico GND.
// Grid: 0.5 V per horizontal line, 50 samples (columns) per vertical line; ms/div follows the Pico's pot.

#include <Arduino_GFX_Library.h>

const uint32_t BAUD = 921600;  // must match PicoSampler
const int UART_RX = 18, UART_TX = 17;  // CrowPanel 4.3" UART1 connector
const int BACKLIGHT = 2;
const int W = 480, H = 272;
const int TOP = 20, LEFT = 30, BOTTOM = 20;  // margins: readout, volt labels, time labels + review line
const int PLOT_W = W - LEFT, PLOT_H = H - TOP - BOTTOM, PLOT_Y1 = TOP + PLOT_H - 1;
const int MV_DIV = 500, COLS_DIV = 50;
const int GAP = 4;  // blank columns ahead of the trace, so you can see where it is
const uint16_t BG = 0x0000, GRID = 0x2945, LABEL = 0x8410, TRACE = 0xFFE0, REVIEW = 0xFD20;

// Pin map and timing from Elecrow's CrowPanel 4.3" Arduino tutorial.
Arduino_ESP32RGBPanel *panel = new Arduino_ESP32RGBPanel(
  40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
  45, 48, 47, 21, 14 /* R0..R4 */,
  5, 6, 7, 15, 16, 4 /* G0..G5 */,
  8, 3, 46, 9, 1 /* B0..B4 */,
  0, 8, 4, 43 /* hsync polarity, front porch, pulse width, back porch */,
  0, 8, 4, 12 /* vsync polarity, front porch, pulse width, back porch */,
  1 /* pclk_active_neg */, 7000000 /* pclk Hz */);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(W, H, panel, 0, true);

int lastMv = 0;
long usPerSample = 1000;  // PicoSampler default until its first "T" line

int toY(int mv) {
  mv = constrain(mv, 0, 3300);  // UART data is untrusted: never draw outside the plot
  return PLOT_Y1 - (long)mv * (PLOT_H - 1) / 3300;
}

void clearColumn(int x) {  // x is a plot column, 0..PLOT_W-1
  gfx->drawFastVLine(LEFT + x, TOP, PLOT_H, x % COLS_DIV == 0 ? GRID : BG);
  for (int mv = 0; mv <= 3300; mv += MV_DIV) gfx->drawPixel(LEFT + x, toY(mv), GRID);
}

void drawScale() {
  gfx->setTextSize(1);
  gfx->setTextColor(LABEL, BG);
  for (int mv = 0; mv <= 3300; mv += MV_DIV) {
    gfx->setCursor(2, toY(mv) - 3);
    gfx->printf("%.1fV", mv / 1000.0);
  }
  gfx->fillRect(LEFT, H - 18, PLOT_W, 8, BG);
  for (int x = 0; x < PLOT_W; x += COLS_DIV) {
    gfx->setCursor(LEFT + x, H - 18);
    gfx->printf("%gms", x * usPerSample / 1000.0);
  }
  gfx->setCursor(W - 6 * 22, 7);
  gfx->printf("0.5 V/div %5g ms/div", COLS_DIV * usPerSample / 1000.0);
}

int sweepX = 0;

void plot(int mv) {
  static int prevY;
  int &x = sweepX;
  x = (x + 1) % PLOT_W;
  clearColumn((x + GAP) % PLOT_W);
  int y = toY(mv);
  if (x) gfx->drawLine(LEFT + x - 1, prevY, LEFT + x, y, TRACE);  // no segment across the wrap
  prevY = y;
  lastMv = mv;
}

void drawReadout() {
  gfx->setTextSize(2);
  gfx->setCursor(4, 2);
  gfx->setTextColor(TRACE, BG);
  gfx->printf("%.3f V", constrain(lastMv, 0, 3300) / 1000.0);
}

void drawReview(const char *s) {
  gfx->fillRect(0, H - 8, W, 8, BG);
  gfx->setTextSize(1);
  gfx->setTextColor(REVIEW, BG);
  gfx->setCursor(0, H - 8);
  for (int i = 0; s[i] && i < W / 6; i++) gfx->write(s[i] >= ' ' && s[i] <= '~' ? s[i] : '?');  // untrusted text
}

void setup() {
  Serial.begin(115200);
  if (toY(0) != PLOT_Y1 || toY(3300) != TOP || toY(-5) != PLOT_Y1 || toY(9999) != TOP)
    Serial.println("self-check FAILED: toY");

  pinMode(BACKLIGHT, OUTPUT);
  digitalWrite(BACKLIGHT, HIGH);
  if (!gfx->begin()) Serial.println("display init FAILED");
  gfx->fillScreen(BG);
  for (int x = 0; x < PLOT_W; x++) clearColumn(x);
  drawScale();

  Serial1.setRxBufferSize(4096);  // covers the short stalls while the readout is redrawn
  Serial1.begin(BAUD, SERIAL_8N1, UART_RX, UART_TX);
}

void loop() {
  static char line[96];  // fits an 80-char review
  static int len = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      line[len] = 0;
      if (line[0] == 'T') {
        long us = constrain(atol(line + 1), 1, 100000);  // untrusted: keep labels sane
        if (us != usPerSample) {
          usPerSample = us;
          drawScale();
        }
      } else if (line[0] == 'R') {
        drawReview(line + 1);
      } else if (line[0] == 'F') {
        sweepX = PLOT_W - 1;  // burst frame starts: next sample lands in column 0
      } else if (len) {
        plot(atoi(line));
      }
      len = 0;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    }
  }

  static uint32_t lastReadout = 0;
  if (millis() - lastReadout >= 200) {
    lastReadout = millis();
    drawReadout();
  }
}
