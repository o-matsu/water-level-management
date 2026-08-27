// =====================================================================
// 田圃水管理ロボット ファームウェア v0.3.1-dbg (CALIB診断ログ版)
// 対象: ESP32 (Freenove WROOM 38pin) / Arduino IDE
//
// v0.3.1: 生存確認/即時起床はBOOTボタン(GPIO0, ext0起床)に変更。
//         ENボタンはハードリセットでRTCメモリが消える(=CALIB要求になる)ため使用しない。
//         ⚠BOOT押しっぱなしでEN/電源投入すると書き込みモードに入るので注意。
// v0.3: 心拍方式を廃止。スリープ10分周期に統合、生存確認はENボタン起床。
//        起床時5秒の操作受付ウィンドウ(LED点灯)→SW即時実行。gpio_holdでスリープ中の
//        リレーIN浮きを固定(チャタリング対策)。
// v0.2.1: ホールセンサ電源ON時の偽エッジ(+1)対策: カウンタクリアを電源ON後に移動
// v0.2の変更点:
//  - 自動ホーミング全廃 → CALIBモード(現地で人間がゼロ点・全開位置を登録)
//  - 巻き戻り問題の構造的解消(繰り出しは常に登録済みゼロ点まで)
//  - 心拍LED方式: 10秒ごとに起床しチカッ(生存表示)。消灯=異常の意味論
//  - 手動SWは心拍起床時のポーリングで検出(押しっぱなしで最大10秒以内に反応)
//  - TURNS_OPENはNVS保存(電源断でも生存)、現在位置はRTCメモリ
//  - CALIB放置30分 → LED消灯で完全停止(復帰は電源再投入のみ)
//
// LED意味論(v0.3):
//  BOOTボタン押下→点灯  = 正常(起床5秒の操作受付ウィンドウ)
//  点灯しっぱなし        = 動作中(開閉・検証中)
//  ゆっくり点滅(1秒周期) = CALIB待機中(位置登録を求めている)
//  BOOTを押しても消灯のまま = 異常(電源喪失/ファーム停止/CALIB放置タイムアウト)
// =====================================================================

#include <Preferences.h>

#define BENCH_MODE 0   // 卓上: 1(シリアルで水位指令) / 実運用: 0(電極センサ)

// ---------------- ピン割当(結線表v1準拠) ----------------
const int PIN_RELAY_OPEN  = 26;
const int PIN_RELAY_CLOSE = 27;
const int PIN_HALL_POWER  = 25;
const int PIN_HALL        = 35;
const int PIN_ACS         = 34;
const int PIN_SW_OPEN     = 33;
const int PIN_SW_CLOSE    = 13;
const int PIN_ELEC_DRIVE  = 4;
const int PIN_ELEC_HIGH   = 36;
const int PIN_ELEC_LOW    = 39;
const int PIN_LED         = 2;    // 内蔵LED(出力専用として使用)

const int RELAY_ON  = HIGH;       // ★実機確認済: HIGHでON
const int RELAY_OFF = LOW;

// ---------------- 実測定数(卓上試験採取) ----------------
const int ACS_IDLE      = 1965;
const int ACS_JAM_DELTA = 150;

// ---------------- 機構・タイミング定数 ----------------
const unsigned long MS_PER_TURN        = 1500;   // 10RPM 磁石4つ
const unsigned long TURN_TIMEOUT_MARGIN = 4000;
const unsigned long SLEEP_US           = 10ULL * 60 * 1000000; // スリープ10分(水位サイクルと同一)
// const unsigned long SLEEP_US           = 10ULL * 1000000; // スリープ10秒(短縮)
const unsigned long SW_WINDOW_MS       = 5000;   // 起床後の手動SW受付ウィンドウ
const int  VERIFY_COUNT                = 6;
const unsigned long VERIFY_INTERVAL_MS = 10000;
const int  WATER_ADC_THRESHOLD         = 2000;   // ★電極実測後に調整
const unsigned long CALIB_CONFIRM_MS   = 10000;  // 10秒放置で確定
const unsigned long CALIB_TIMEOUT_MS   = 30UL * 60 * 1000; // 30分で完全停止

// ---------------- 永続状態 ----------------
#define RTC_MAGIC 0xA5A57002
RTC_DATA_ATTR uint32_t rtcMagic = 0;
RTC_DATA_ATTR int      rtcPosition = -1;   // 現在位置[回転] 0=全閉 / -1=不定

Preferences prefs;
int turnsOpen = 0;                          // 全開回転数(NVSからロード)

// ---------------- 実行時状態 ----------------
volatile int hallCount = 0;
volatile unsigned long lastHallMs = 0;

enum GateResult { GATE_OK, GATE_JAM, GATE_TIMEOUT };

// =====================================================================
// 低レベル
// =====================================================================
volatile unsigned long pulseMs[32];
volatile int pulseIdx = 0;
void IRAM_ATTR onHallPulse() {
  unsigned long now = millis();
  if (now - lastHallMs > 50) {
    hallCount++;
    if (pulseIdx < 32) pulseMs[pulseIdx++] = now;
    lastHallMs = now;
  }
}

void relayAllOff() {
  digitalWrite(PIN_RELAY_OPEN,  RELAY_OFF);
  digitalWrite(PIN_RELAY_CLOSE, RELAY_OFF);
}

int readACS() {
  long sum = 0;
  for (int i = 0; i < 32; i++) { sum += analogRead(PIN_ACS); delay(2); }
  return sum / 32;
}

bool isOverCurrent() { return abs(readACS() - ACS_IDLE) > ACS_JAM_DELTA; }

void hallPower(bool on) {
  digitalWrite(PIN_HALL_POWER, on ? HIGH : LOW);
  if (on) delay(100);
}

void ledBlink(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH); delay(onMs);
    digitalWrite(PIN_LED, LOW);  delay(offMs);
  }
}

// =====================================================================
// ゲート駆動(回転数指定・電流監視・タイムアウト)
// =====================================================================
GateResult driveTurns(bool dirOpen, int turns) {
  if (turns <= 0) return GATE_OK;
  hallPower(true);
  delay(100);          // センサ起動過渡(電源ON時の偽エッジ)が落ち着くまで待つ
  hallCount = 0;       // ★クリアは電源ON後・アタッチ直前(v0.2.1修正)
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onHallPulse, FALLING);

  unsigned long timeout = (unsigned long)turns * MS_PER_TURN + TURN_TIMEOUT_MARGIN;
  unsigned long start = millis();
  digitalWrite(dirOpen ? PIN_RELAY_OPEN : PIN_RELAY_CLOSE, RELAY_ON);
  delay(300);  // 突入マスク

  GateResult result = GATE_OK;
  while (hallCount < turns) {
    if (millis() - start > timeout) { result = GATE_TIMEOUT; break; }
    if (isOverCurrent())            { result = GATE_JAM;     break; }
    delay(10);
  }
  relayAllOff();
  detachInterrupt(digitalPinToInterrupt(PIN_HALL));
  hallPower(false);
  delay(300);
  return result;
}

// =====================================================================
// 開閉シーケンス
//  失敗時: 位置信頼を失う → rtcPosition=-1 → 次回起動でCALIB要求(LEDゆっくり点滅)
//  → 30分放置で消灯 → 「消灯=異常」の意味論に合流
// =====================================================================
bool openGate() {
  if (rtcPosition < 0 || turnsOpen <= 0) return false;
  if (rtcPosition >= turnsOpen) return true;
  digitalWrite(PIN_LED, HIGH);  // 動作中=点灯

  GateResult r = driveTurns(true, turnsOpen - rtcPosition);
  digitalWrite(PIN_LED, LOW);
  if (r == GATE_OK) { rtcPosition = turnsOpen; return true; }

  Serial.printf("[OPEN] fail(%d) -> position lost\n", r);
  relayAllOff();
  driveTurns(false, 1);   // 安全側: 少し繰り出して張力を抜く
  rtcPosition = -1;       // 次回起動でCALIB要求
  return false;
}

bool closeGate() {
  if (rtcPosition < 0) return false;
  if (rtcPosition == 0) return true;
  digitalWrite(PIN_LED, HIGH);

  // 繰り出しは常に「登録ゼロ点ぴったりまで」= 巻き戻りは構造的に起きない
  GateResult r = driveTurns(false, rtcPosition);
  digitalWrite(PIN_LED, LOW);
  if (r == GATE_JAM) {    // 繰り出しJAM=ワイヤー絡み等の異常
    Serial.println("[CLOSE] jam -> position lost");
    rtcPosition = -1;
    return false;
  }
  rtcPosition = 0;        // TIMEOUTは着座後の空転猶予とみなしOK側に倒す
  return true;
}

// =====================================================================
// 水位判定
// =====================================================================
#if BENCH_MODE
RTC_DATA_ATTR char benchWater = 'h';  // 'h'=水位高(閉でよい) 'l'=水位低(開けたい)
bool waterHighRequested() { return benchWater != 'l'; }
#else
bool electrodeWet(int pin) {
  digitalWrite(PIN_ELEC_DRIVE, HIGH);
  delay(30);
  long sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogRead(pin); delay(2); }
  digitalWrite(PIN_ELEC_DRIVE, LOW);
  int adc = sum / 8;
  Serial.printf("[ELEC] pin=%s adc=%d -> %s\n",
                pin == PIN_ELEC_HIGH ? "VP(上)" : "VN(下)", adc,
                adc < WATER_ADC_THRESHOLD ? "WET" : "DRY");
  return adc < WATER_ADC_THRESHOLD;
}
bool waterHighRequested() {
  bool hi = electrodeWet(PIN_ELEC_HIGH);
  bool lo = electrodeWet(PIN_ELEC_LOW);
  if (hi) return true;
  if (!lo) return false;
  return (rtcPosition == 0);  // 中間帯は現状維持
}
#endif

bool verifyRequest(bool wantHigh) {
  digitalWrite(PIN_LED, HIGH);  // 検証中=点灯
  Serial.printf("[VERIFY] start (need %d consecutive)\n", VERIFY_COUNT);
  for (int i = 0; i < VERIFY_COUNT; i++) {
    delay(VERIFY_INTERVAL_MS);
    bool now = waterHighRequested();
    Serial.printf("[VERIFY] %d/%d: wantHigh=%d now=%d\n", i + 1, VERIFY_COUNT, wantHigh, now);
    if (now != wantHigh) {
      Serial.println("[VERIFY] flipped -> cancel");
      digitalWrite(PIN_LED, LOW);
      return false;
    }
  }
  Serial.println("[VERIFY] confirmed");
  digitalWrite(PIN_LED, LOW);
  return true;
}

// =====================================================================
// CALIBモード
//  操作: 開SW/閉SW押下中のみジョグ駆動。10秒放置で確定。
//  1回目の確定=ゼロ点(速点滅×3) → 2回目の確定=全開位置(速点滅×5, NVS保存)
//  → 自動で全閉まで繰り出して通常運用へ
//  30分無操作 → LED消灯で永久停止(電源再投入のみで復帰)
// =====================================================================
void jog(bool dirOpen, int &netTurns) {
  hallPower(true);
  delay(100);          // v0.2.1: 起動過渡の偽エッジ対策
  hallCount = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onHallPulse, FALLING);
  digitalWrite(dirOpen ? PIN_RELAY_OPEN : PIN_RELAY_CLOSE, RELAY_ON);
  while (digitalRead(dirOpen ? PIN_SW_OPEN : PIN_SW_CLOSE) == LOW) delay(20);
  relayAllOff();
  detachInterrupt(digitalPinToInterrupt(PIN_HALL));
  hallPower(false);
  netTurns += dirOpen ? hallCount : -hallCount;
  Serial.printf("[JOG] dir=%s hallCount=%d -> netTurns=%d (GPIO35 now=%d)\n",
                dirOpen ? "OPEN" : "CLOSE", hallCount, netTurns, digitalRead(PIN_HALL));
  for (int i = 1; i < pulseIdx; i++) {
    Serial.printf("  pulse[%d] interval = %lu ms\n", i, pulseMs[i] - pulseMs[i-1]);
  }
  pulseIdx = 0;
  delay(300);
}

void calibMode() {
  Serial.println("[CALIB] enter. jog with SW, leave 10s to confirm.");
  int phase = 0;          // 0=ゼロ点待ち 1=全開位置待ち
  int netTurns = 0;       // ゼロ点確定後の累積回転
  bool touched = false;   // このフェーズで一度でも操作されたか
  unsigned long lastAction = millis();
  unsigned long enterTime  = millis();
  unsigned long lastBlink  = 0;

  while (true) {
    // ゆっくり点滅(1秒周期)
    if (millis() - lastBlink > 500) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      lastBlink = millis();
    }
    // 30分放置 → 完全停止(消灯=異常の意味論)
    if (millis() - enterTime > CALIB_TIMEOUT_MS) {
      Serial.println("[CALIB] timeout -> full stop (power cycle to recover)");
      digitalWrite(PIN_LED, LOW);
      relayAllOff();
      esp_deep_sleep_start();   // 起床要因なし=永久停止
    }
    // ジョグ
    if (digitalRead(PIN_SW_OPEN) == LOW)  { digitalWrite(PIN_LED, HIGH); jog(true,  netTurns); touched = true; lastAction = millis(); enterTime = millis(); }
    if (digitalRead(PIN_SW_CLOSE) == LOW) { digitalWrite(PIN_LED, HIGH); jog(false, netTurns); touched = true; lastAction = millis(); enterTime = millis(); }

    // 10秒放置で確定(そのフェーズで一度は操作があった場合のみ)
    if (touched && millis() - lastAction > CALIB_CONFIRM_MS) {
      if (phase == 0) {
        netTurns = 0;                       // ここがゼロ点
        ledBlink(3, 100, 100);
        Serial.println("[CALIB] zero registered");
        phase = 1; touched = false; lastAction = millis(); enterTime = millis();
      } else {
        Serial.printf("[CALIB] confirm check: netTurns=%d\n", netTurns);
        if (netTurns <= 0) {                // 全開が0以下は無効。やり直し待ち
          ledBlink(10, 50, 50);
          Serial.println("[CALIB] invalid open position (<=0). jog again.");
          touched = false; lastAction = millis();
          continue;
        }
        turnsOpen = netTurns;
        prefs.begin("gate", false);
        prefs.putInt("turnsOpen", turnsOpen);
        prefs.end();
        ledBlink(5, 100, 100);
        Serial.printf("[CALIB] open position registered: %d turns\n", turnsOpen);

        // 登録した全開位置から全閉へ戻して運用開始
        rtcPosition = netTurns;
        rtcMagic = RTC_MAGIC;
        closeGate();
        Serial.println("[CALIB] done -> normal operation");
        return;
      }
    }
    delay(20);
  }
}

// =====================================================================
// 通常サイクル(起床ごとに実行 / タイマー起床とEN起床は区別しない)
// =====================================================================
void wakeCycle() {
  // 起床表示 兼 操作受付ウィンドウ(5秒): LED点灯のままSWを待つ
  digitalWrite(PIN_LED, HIGH);
  unsigned long t0 = millis();
  while (millis() - t0 < SW_WINDOW_MS) {
    if (digitalRead(PIN_SW_OPEN) == LOW)  { openGate();  goToSleep(); }
    if (digitalRead(PIN_SW_CLOSE) == LOW) { closeGate(); goToSleep(); }
    delay(20);
  }
  digitalWrite(PIN_LED, LOW);

#if BENCH_MODE
  // 卓上: シリアル指令があれば反映('l'=水位低/'h'=水位高)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'l' || c == 'h') benchWater = c;
  }
#endif
  bool wantHigh = waterHighRequested();
  bool shouldClose = wantHigh;
  bool isClosed = (rtcPosition == 0);
  Serial.printf("[CYCLE] wantHigh=%d position=%d(%s) -> %s\n",
                wantHigh, rtcPosition, isClosed ? "closed" : "open",
                (shouldClose != isClosed) ? "state change -> verify" : "no change");
  if (shouldClose != isClosed) {
    if (verifyRequest(wantHigh)) {
      if (shouldClose) closeGate();
      else             openGate();
    }
  }
  goToSleep();
}

void goToSleep() {
  relayAllOff();
  hallPower(false);
  digitalWrite(PIN_LED, LOW);
  // 位置を失っていたら眠らずCALIBへ(再起動で入口から)
  if (rtcPosition < 0) { rtcMagic = 0; ESP.restart(); }
  // スリープ中のリレーIN浮き対策: OFFレベルでピン状態を固定
  gpio_hold_en((gpio_num_t)PIN_RELAY_OPEN);
  gpio_hold_en((gpio_num_t)PIN_RELAY_CLOSE);
  gpio_deep_sleep_hold_en();
  esp_sleep_enable_timer_wakeup(SLEEP_US);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // BOOTボタン(GPIO0=LOW)で即時起床
  Serial.flush();
  esp_deep_sleep_start();
}

// =====================================================================
void setup() {
  // 前回スリープ時のピン固定を解除してから初期化
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)PIN_RELAY_OPEN);
  gpio_hold_dis((gpio_num_t)PIN_RELAY_CLOSE);
  digitalWrite(PIN_RELAY_OPEN,  RELAY_OFF);
  digitalWrite(PIN_RELAY_CLOSE, RELAY_OFF);
  pinMode(PIN_RELAY_OPEN,  OUTPUT);
  pinMode(PIN_RELAY_CLOSE, OUTPUT);
  pinMode(PIN_HALL_POWER, OUTPUT); digitalWrite(PIN_HALL_POWER, LOW);
  pinMode(PIN_HALL, INPUT);
  pinMode(PIN_SW_OPEN,  INPUT_PULLUP);
  pinMode(PIN_SW_CLOSE, INPUT_PULLUP);
  pinMode(PIN_ELEC_DRIVE, OUTPUT); digitalWrite(PIN_ELEC_DRIVE, LOW);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(0, INPUT_PULLUP);  // BOOTボタン(ext0起床用)

  Serial.begin(115200);
  analogSetPinAttenuation(PIN_ACS, ADC_11db);
  analogSetPinAttenuation(PIN_ELEC_HIGH, ADC_11db);
  analogSetPinAttenuation(PIN_ELEC_LOW,  ADC_11db);

  prefs.begin("gate", true);
  turnsOpen = prefs.getInt("turnsOpen", 0);
  prefs.end();

  // 入口の2段の門:
  //  位置不定 or 全開回転数未登録 → CALIB / それ以外 → 通常サイクル
  if (rtcMagic != RTC_MAGIC || rtcPosition < 0 || turnsOpen <= 0) {
    calibMode();          // 完了時のみ戻ってくる(rtcMagic等は設定済み)
  }
  wakeCycle();            // 1起床分を実行して眠る(戻らない)
}

void loop() {}  // 全処理はsetup内で完結(毎起床=毎setup)
