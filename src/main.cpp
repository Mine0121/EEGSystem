#include <M5Unified.h>          // 画面とボタン・電源管理など
#include <arduinoFFT.h>         // FFT で β/α 比算出
HardwareSerial TGAMSerial(1);   // UART1 を TGAM 専用に使う

// --- ピン設定 ---
constexpr int TGAM_RX_PIN = 25;     // TGAM TX → G25
constexpr int TGAM_TX_PIN = -1;     // 送信しない

// --- TGAM パケット解析用 ---
uint8_t  payload[169];
uint8_t  payloadLen   = 0;
uint8_t  checksumRx   = 0;
uint8_t  checksumCalc = 0;
bool     bigPacket    = false;

uint8_t  poorQuality = 0;
uint8_t  attention   = 0;
uint8_t  meditation  = 0;

// --- FFT 用バッファと計算設定 ---
constexpr size_t SAMPLE_SIZE  = 512;   // 512 サンプル (約1秒)
constexpr size_t UPDATE_STEP  = 32;    // 32 サンプルごとに更新
constexpr size_t AVG_COUNT    = 8;     // FFT 結果を8回平均

int16_t rawBuffer[SAMPLE_SIZE];        // 生波形リングバッファ
size_t  rawIndex     = 0;              // 書き込み位置
size_t  sampleTotal  = 0;              // 累積サンプル数

double  vReal[SAMPLE_SIZE];            // FFT 入力 (実数部)
double  vImag[SAMPLE_SIZE];            // FFT 入力 (虚数部)

arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLE_SIZE, 512);

float   graphBuffer[120];             // グラフ表示用履歴
size_t  graphIndex = 0;
size_t  graphNum   = 0;

float   ratioBuffer[AVG_COUNT];       // β/α 比平均用
size_t  ratioPos   = 0;

float   avgBA      = 0.0f;            // 最新の平均β/α比

// 1 バイトだけブロッキングで読む（TGAMSerial 版）
uint8_t readTGAMByte() {
  while (!TGAMSerial.available()) {
    M5.update();                 // ボタンや電源管理を止めない
  }
  return TGAMSerial.read();
}

// 画面に最新値を出す
void drawValues() {
  M5.Display.clear(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);       // 16×32 ドット相当
  M5.Display.printf("Signal: %3u\n", poorQuality);
  M5.Display.printf("Attend : %3u\n", attention);
  M5.Display.printf("Meditat: %3u\n", meditation);
  M5.Display.printf("B/A   : %4.2f\n", avgBA);
}

// β/α 比を計算
float calcBetaAlpha() {
  for (size_t i = 0; i < SAMPLE_SIZE; ++i) {
    size_t idx = (rawIndex + i) % SAMPLE_SIZE;
    vReal[i] = rawBuffer[idx];
    vImag[i] = 0.0;
  }

  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  double alpha = 0.0;
  for (uint8_t i = 8; i <= 12; ++i) alpha += vReal[i];
  double beta = 0.0;
  for (uint8_t i = 13; i <= 30; ++i) beta += vReal[i];

  if (alpha == 0.0) return 0.0f;
  return beta / alpha;
}

// グラフ描画（右端に1点追加）
void drawGraph(float ratio) {
  constexpr int graphX = 0;
  constexpr int graphY = 80;
  constexpr int graphH = 40;
  constexpr int graphW = 120;

  graphBuffer[graphIndex] = ratio;
  graphIndex = (graphIndex + 1) % graphW;
  if (graphNum < graphW) graphNum++;

  M5.Display.fillRect(graphX, graphY - graphH, graphW, graphH, TFT_BLACK);
  for (size_t i = 0; i < graphNum; ++i) {
    size_t idx = (graphIndex + i) % graphW;
    int h = static_cast<int>(graphBuffer[idx] * 10.0f);
    if (h > graphH) h = graphH;
    M5.Display.drawFastVLine(graphX + i, graphY - h, h, TFT_GREEN);
  }
}

void setup() {
  // 画面 / I²C / IMU などを初期化
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);       // 横置き表示

  // TGAM UART
  TGAMSerial.begin(57600, SERIAL_8N1, TGAM_RX_PIN, TGAM_TX_PIN);

  // デバッグ／ログ用 USB シリアル
  Serial.begin(115200);
  Serial.println("\nTGAM monitor started");
}

void loop() {
  M5.update();                     // 必須

  /* ---------------- 同期バイト 0xAA 2 連続待ち ---------------- */
  if (readTGAMByte() != 0xAA) return;
  if (readTGAMByte() != 0xAA) return;

  /* ---------------- ペイロード長と内容取得 ---------------- */
  payloadLen = readTGAMByte();
  if (payloadLen > sizeof(payload)) return;  // ガード

  checksumCalc = 0;
  for (uint8_t i = 0; i < payloadLen; ++i) {
    payload[i] = readTGAMByte();
    checksumCalc += payload[i];
  }
  checksumRx   = readTGAMByte();
  checksumCalc = 255 - checksumCalc;

  if (checksumRx != checksumCalc) return;    // チェックサム NG

  /* ---------------- データ行をパース ---------------- */
  bigPacket = false;
  poorQuality = 200;  attention = 0;  meditation = 0;

  for (uint8_t i = 0; i < payloadLen; ++i) {
    switch (payload[i]) {
      case 0x02:   // 信号品質
        poorQuality = payload[++i];
        bigPacket   = true;
        break;
      case 0x04:   // 注意度
        attention   = payload[++i];
        break;
      case 0x05:   // 瞑想度
        meditation  = payload[++i];
        break;
      case 0x80:   // Raw EEG (2 バイト ×2)
        if (i + 3 < payloadLen && payload[i+1] == 0x02) {
          int16_t v = (static_cast<int16_t>(payload[i+2]) << 8) | payload[i+3];
          rawBuffer[rawIndex] = v;
          rawIndex = (rawIndex + 1) % SAMPLE_SIZE;
          sampleTotal++;
          if (sampleTotal >= SAMPLE_SIZE && (sampleTotal % UPDATE_STEP) == 0) {
            float ratio = calcBetaAlpha();
            drawGraph(ratio);

            ratioBuffer[ratioPos % AVG_COUNT] = ratio;
            ratioPos++;
            size_t count = ratioPos < AVG_COUNT ? ratioPos : AVG_COUNT;
            float sum = 0.0f;
            for (size_t n = 0; n < count; ++n) sum += ratioBuffer[n];
            avgBA = sum / count;
            drawValues();
          }
        }
        i += 3;
        break;
      case 0x83:   // EEG バンドパワー (24 バイト)
        i += 25;
        break;
      default:
        /* length=1 row はここを通る */
        break;
    }
  }

  /* ---------------- 表示とシリアル転送 ---------------- */
  if (bigPacket) {                 // poorQuality を含む完全パケットだけ処理
    Serial.printf("Signal=%u Attn=%u Med=%u\n",
                  poorQuality, attention, meditation);
    drawValues();
  }
}
