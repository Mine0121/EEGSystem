/*********************************************************************
 * TGAM 512 Hz Raw EEG → FFT → β/α (8 s avg) + Signal 表示 + Splash
 *********************************************************************/
#include <M5Unified.h>
#include "FFT.h"
#include <math.h>               // sqrtf

#ifdef TWO_PI               
#undef TWO_PI
#endif
#define TWO_PI 6.28318530f

/* ---- 定数 ---- */
constexpr uint16_t FS_HZ   = 512;
constexpr uint16_t FFT_N   = 512;
constexpr uint8_t  AVG_N   = 8;
constexpr uint16_t DISP_W  = 240;
constexpr uint8_t  GRAPH_H = 80;
constexpr int16_t  GRAPH_Y0 = 134;

/* ---- UART ---- */
HardwareSerial TGAMSerial(1);
constexpr int TGAM_RX_PIN = 25;
constexpr int TGAM_TX_PIN = -1;

/* ---- FFT ---- */
static float fftInput [FFT_N];
static float fftOutput[FFT_N];
static fft_config_t* fftPlan = nullptr;
static uint16_t samplePos    = 0;

/* ---- 移動平均・履歴 ---- */
static float alphaBuf[AVG_N] = {};
static float betaBuf [AVG_N] = {};
static uint8_t avgPos        = 0;

static float  ratioHist[DISP_W] = {};
static uint8_t histPos          = 0;

/* ---- Signal 品質 ---- */
static uint8_t signalQuality = 200;     // 0=good, 200=bad

/* ---- スプラッシュ ---- */
static bool     readyGraph  = false;
static uint32_t splashTick = 0;

/* ===================================================== *
 *                        画面描画                        *
 * ===================================================== */
void drawGraph(float latest)
{
  /*
  // 文字エリア：40px (2 行)
  M5.Display.fillRect(0, 0, DISP_W, 40, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  M5.Display.setCursor(0, 2);
  M5.Display.printf("beta/alpha %.2f", latest);

  M5.Display.setCursor(0, 20);
  M5.Display.printf("Signal %-3u", signalQuality);

  // スケール決定
  float vmin = 1e9f, vmax = -1e9f;
  for (uint16_t i = 0; i < DISP_W; ++i) {
    vmin = fminf(vmin, ratioHist[i]);
    vmax = fmaxf(vmax, ratioHist[i]);
  }
  vmin = fmaxf(0.2f, vmin);
  vmax = fminf(5.0f, vmax);
  float scale = (vmax > vmin) ? GRAPH_H / (vmax - vmin) : 1.0f;

  // グラフエリア消去
  M5.Display.fillRect(0, GRAPH_Y0 - GRAPH_H + 1, DISP_W, GRAPH_H, TFT_BLACK);

  // 描画
  for (uint16_t x = 0; x < DISP_W; ++x) {
    uint16_t idx = (histPos + x) % DISP_W;
    int h = int((ratioHist[idx] - vmin) * scale);
    h = h < 0 ? 0 : (h > GRAPH_H ? GRAPH_H : h);
    M5.Display.drawFastVLine(x, GRAPH_Y0 - h, h == 0 ? 1 : h, TFT_CYAN);
  }
  */
}

/* ---- FFT → β/α 計算 ---- */
void executeFFTandStoreRatio()
{
  fft_execute(fftPlan);

  float alpha = 0, beta = 0;
  for (uint16_t k = 8;  k <= 12; ++k) {
    float re = fftOutput[2*k  ], im = fftOutput[2*k+1];
    alpha += sqrtf(re*re + im*im);
  }
  alpha /= 5.0f;
  for (uint16_t k = 13; k <= 30; ++k) {
    float re = fftOutput[2*k  ], im = fftOutput[2*k+1];
    beta  += sqrtf(re*re + im*im);
  }
  beta /= 18.0f;

  alphaBuf[avgPos] = alpha;
  betaBuf [avgPos] = beta;
  avgPos = (avgPos + 1) % AVG_N;

  float aSum = 0, bSum = 0;
  for (uint8_t i = 0; i < AVG_N; ++i) { aSum += alphaBuf[i]; bSum += betaBuf[i]; }
  float ratio = (aSum > 1e-6f) ? bSum / aSum : 0.f;

  ratioHist[histPos] = ratio;
  histPos = (histPos + 1) % DISP_W;

  drawGraph(ratio);
  readyGraph = true;                       // 初回でスプラッシュ解除
}

/* ---- Raw 1 サンプル投入 ---- */
void pushEEGSample(int16_t raw)
{
  fftInput[samplePos++] = static_cast<float>(raw);
  if (samplePos >= FFT_N) { samplePos = 0; executeFFTandStoreRatio(); }
}

/* ---- TGAM 受信 ---- */
uint8_t readTGAMByte() { while (!TGAMSerial.available()) M5.update(); return TGAMSerial.read(); }

void parseTGAM()
{
  if (readTGAMByte() != 0xAA) return;
  if (readTGAMByte() != 0xAA) return;

  uint8_t len = readTGAMByte();
  uint8_t sum = 0; static uint8_t buf[170];
  for (uint8_t i = 0; i < len; ++i) { buf[i] = readTGAMByte(); sum += buf[i]; }
  if (readTGAMByte() != uint8_t(255 - sum)) return;

  for (uint8_t i = 0; i < len; ++i) {
    uint8_t code = buf[i];
    if (code == 0x02 && i + 1 < len) {     // Signal Quality
      signalQuality = buf[++i];
    }
    else if (code == 0x80 && i + 3 < len) { // Raw EEG
      ++i; int16_t raw = int16_t(buf[i+1] << 8 | buf[i+2]); pushEEGSample(raw); i += 2;
    }
    else if (code < 0x80) ++i;             // 1B データ
    else              i += buf[++i];       // N バイトスキップ
  }
}

/* ---- スプラッシュ ---- */
void drawSplash()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(20, 50);
  M5.Display.print("Waiting for TGAM");
  splashTick = millis();
}

/* ---------------- setup ---------------- */
void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);

  TGAMSerial.begin(57600, SERIAL_8N1, TGAM_RX_PIN, TGAM_TX_PIN);
  Serial.begin(115200);

  fftPlan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fftInput, fftOutput);
  if (!fftPlan) { Serial.println("FFT init failed"); while (1) {} }

  drawSplash();
}

/* ---------------- loop ----------------- */
void loop()
{
  M5.update();
  parseTGAM();

  if (!readyGraph && millis() - splashTick > 1000) {   // スプラッシュ点滅
    splashTick += 1000;
    M5.Display.print('.');
    if (M5.Display.getCursorX() > 220) {
      M5.Display.fillRect(140,50,90,16,TFT_BLACK);
      M5.Display.setCursor(140,50);
    }
  }
}
