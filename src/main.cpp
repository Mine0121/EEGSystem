#include <M5Unified.h>          // 画面とボタン・電源管理など
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
