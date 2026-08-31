// =====================================================================
// 田圃水管理ロボット ファームウェア v0.5.2 (ハイブリッド位置制御版)
// 対象: ESP32 (Freenove WROOM 38pin) / Arduino IDE
//
// v0.5.2: 一晩の現場ログより: 電極閾値270に(水膜1900をWET誤判定しゲートが開かなかった)。
//         上のみ濡れの矛盾状態は現状維持。磁石位相を学習し任意ゼロ点でもエッジ同期が効くように
//         (従来は磁石が整数位置にある前提で、位相が350mcを超えるCALIBでは全エッジ棄却→時間補間のみだった)。
// v0.5.1: リレー投入ノイズの偽エッジ対策(マスク・棄却・学習の妥当範囲)、磁石検出幅の学習とB側エッジ再同期。
// v0.5.0: 位置をホールエッジ+時間補間で 1/1000カウント(mc)単位で管理。
//         - 開はFALLING、閉はRISINGを数える(どちらも磁石の同じ側=同じ物理点で止まる)
//         - エッジ間は直近の実測速度(ms/カウント, 方向別に学習)で補間
//         - CALIBのゼロ点・全開もmc精度で登録、閉はCLOSE_BIAS_MC手前で止めて弛みを作らない
//         現場実測: 全開3カウントでは1カウント(31.5mm)の量子化が致命的だったため。
// v0.4.0: LittleFSに /gate.csv としてイベントログを追記。
//         jogにJAM監視、CLOSEのパルスなしTIMEOUTを位置喪失扱い、電極は1回通電で両読み、
//         VERIFY中も手動SWを受付。
//         時刻は電源投入からの経過秒(gettimeofday: ディープスリープを跨いで進み続ける)。
//         読み出し: USB接続→BOOTで起床→5秒窓の間にシリアルで d(ダンプ)/i(残量)/x(消去)。
//         コマンド処理ごとに窓を5秒延長するのでダンプ中に切れない。
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
// LED意味論(v0.3以降):
//  BOOTボタン押下→点灯  = 正常(起床5秒の操作受付ウィンドウ)
//  点灯しっぱなし        = 動作中(開閉・検証中)
//  ゆっくり点滅(1秒周期) = CALIB待機中(位置登録を求めている)
//  BOOTを押しても消灯のまま = 異常(電源喪失/ファーム停止/CALIB放置タイムアウト)
// =====================================================================

#include <Preferences.h>
#include <LittleFS.h>
#include <time.h>

#define BENCH_MODE  0   // 卓上: 1(シリアルで水位指令) / 実運用: 0(電極センサ)
#if BENCH_MODE
RTC_DATA_ATTR char benchWater = 'h';  // 'h'=水位高(閉でよい) 'l'=水位低(開けたい)
#endif

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
const int           MS_PER_COUNT_DEF   = 1300;   // 1カウント(磁石1個分)の初期想定時間。駆動ごとに実測で学習
const unsigned long TURN_TIMEOUT_MARGIN = 4000;
const uint64_t      SLEEP_US           = 10ULL * 60 * 1000000; // スリープ10分(水位サイクルと同一) ※64bit: 32bitだと2時間超で桁あふれ
// const uint64_t      SLEEP_US           = 10ULL * 1000000; // スリープ10秒(短縮)
const unsigned long SW_WINDOW_MS       = 5000;   // 起床後の手動SW受付ウィンドウ
const int  VERIFY_COUNT                = 6;
const unsigned long VERIFY_INTERVAL_MS = 10000;
const int  WATER_ADC_THRESHOLD         = 270;    // 現場実測(2026-08-31): 水没116〜231 / 水膜(水面より上)310〜1900 / 乾燥4095。水膜をDRYに倒す
const unsigned long CALIB_CONFIRM_MS   = 10000;  // 10秒放置で確定
const unsigned long CALIB_TIMEOUT_MS   = 30UL * 60 * 1000; // 30分で完全停止

// ---------------- 位置(mc = 1/1000カウント, 1カウント≒31.5mm) ----------------
const int32_t POS_UNKNOWN   = INT32_MIN;
const int32_t CLOSE_BIAS_MC = 30;     // 閉はゼロ点のこれだけ手前で止める(≒1mm)。弛み→蓋の傾きを防ぐ
const int32_t OPEN_MIN_MC   = 500;    // 全開登録の最小値(これ未満は無効)
const int     MS_PER_COUNT_MIN = 300, MS_PER_COUNT_MAX = 5000;  // 学習に使うパルス間隔の妥当範囲
const int32_t EDGE_SNAP_TOL_MC = 350;   // 推定位置からこれ以上離れたエッジは偽エッジとして無視
const int32_t ZONE_MIN_MC = 50, ZONE_MAX_MC = 700;  // 磁石検出幅の妥当範囲
const unsigned long INRUSH_MASK_MS = 300;           // リレー投入直後: 電流監視もエッジも無視(ノイズ)

// ---------------- ログ ----------------
const char*  LOG_PATH      = "/gate.csv";
const size_t LOG_MAX_BYTES = 1024UL * 1024;   // 約170日分(1シーズン)
const size_t LOG_LINE_BYTES = 40;             // 1行の目安
const size_t LOG_BYTES_PER_DAY = LOG_LINE_BYTES * (86400ULL * 1000000 / SLEEP_US); // ≒6KB(BOOT+CYCLE想定)

// ---------------- 型(Arduinoの自動プロトタイプは最初の関数の直前に挿入されるため、全関数より前に置く) ----------------
enum GateResult { GATE_OK, GATE_JAM, GATE_TIMEOUT };
struct MoveResult  { GateResult r; int edges; };
struct WaterSample { bool high; int adcHi; int adcLo; };   // adc=-1: BENCH(電極なし)

// ---------------- 永続状態 ----------------
#define RTC_MAGIC 0xA5A57003
RTC_DATA_ATTR uint32_t rtcMagic = 0;
RTC_DATA_ATTR int32_t  rtcPosMc = POS_UNKNOWN;   // 現在位置[mc] 0=全閉(ゼロ点)
RTC_DATA_ATTR int      rtcMsPerCount[2] = { MS_PER_COUNT_DEF, MS_PER_COUNT_DEF };  // [0]=閉 [1]=開 の学習速度
RTC_DATA_ATTR int32_t  rtcZoneMc = 0;            // 磁石の検出幅[mc](0=未学習)。B側エッジでの再同期に使う
RTC_DATA_ATTR int32_t  rtcPhaseMc = -1;          // 磁石A側位置の座標内オフセット[mc, mod 1000](-1=未学習)。
                                                 //  ゼロ点は任意の位置なので磁石は整数位置にいるとは限らない
int32_t mod1000(int32_t v) { return ((v % 1000) + 1000) % 1000; }

Preferences prefs;
int32_t openMc = 0;                         // 全開位置[mc](NVSからロード)

const char* gateResultName(GateResult r) {
  return r == GATE_OK ? "ok" : r == GATE_JAM ? "jam" : "timeout";
}
bool posKnown() { return rtcPosMc != POS_UNKNOWN; }
bool isClosed() { return posKnown() && rtcPosMc < openMc / 2; }

// ---------------- ホールエッジ(ISR→メインループへ受け渡し) ----------------
volatile unsigned long lastHallMs = 0;
volatile uint32_t edgeMs[8];
volatile uint8_t  edgeLvl[8];
volatile uint8_t  edgeHead = 0, edgeTail = 0;

bool fsReady = false;

// =====================================================================
// ログ(LittleFS /gate.csv 追記)
//  時刻列 = 電源投入からの経過秒。ESP32のシステム時刻はRTCスロークロックで
//  ディープスリープ中も進むため、RTCモジュールなしで起床・スリープをまたいだ通し時間になる。
//  上限LOG_MAX_BYTESに達したら追記を止める(消去はシリアル 'x')。
// =====================================================================
uint32_t uptimeSec() { return (uint32_t)time(nullptr); }

void logInit() {
  fsReady = LittleFS.begin(true);   // 未フォーマットなら自動フォーマット
  if (!fsReady) Serial.println("[LOG] LittleFS mount failed");
}

void logEvent(const char* fmt, ...) {
  char line[112];
  int n = snprintf(line, sizeof(line), "%lu,", (unsigned long)uptimeSec());
  va_list ap;
  va_start(ap, fmt);
  n += vsnprintf(line + n, sizeof(line) - n - 1, fmt, ap);
  va_end(ap);
  n = min(n, (int)sizeof(line) - 2);   // vsnprintfは切り詰め前の長さを返すので実長に丸める('\n'+'\0'分を残す)
  line[n++] = '\n'; line[n] = '\0';
  Serial.printf("[LOG] %s", line);
  if (!fsReady) return;
  File f = LittleFS.open(LOG_PATH, FILE_APPEND);
  if (!f) { Serial.println("[LOG] open failed"); return; }
  if (f.size() + n <= LOG_MAX_BYTES) f.write((const uint8_t*)line, n);
  else                               Serial.println("[LOG] full (send 'x' to erase)");
  f.close();
}

void logDump() {
  if (!fsReady) { Serial.println("[LOG] no fs"); return; }
  File f = LittleFS.open(LOG_PATH, FILE_READ);   // 無効なFileはavailable()=0
  Serial.printf("--- BEGIN %s (%u bytes) ---\n", LOG_PATH, f ? (unsigned)f.size() : 0);
  uint8_t buf[256];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    Serial.write(buf, n);
  }
  f.close();
  Serial.printf("--- END %s ---\n", LOG_PATH);
}

void logInfo() {
  if (!fsReady) { Serial.println("[LOG] no fs"); return; }
  size_t used = 0;
  File f = LittleFS.open(LOG_PATH, FILE_READ);
  if (f) { used = f.size(); f.close(); }
  size_t total = LittleFS.totalBytes(), fsUsed = LittleFS.usedBytes();
  size_t cap = min(LOG_MAX_BYTES, total);
  Serial.printf("[LOG] %s: %u / %u bytes (%u%%), free=%u bytes (~%u days), fs total=%u used=%u, uptime=%lus\n",
                LOG_PATH, (unsigned)used, (unsigned)cap, (unsigned)(used * 100 / cap),
                (unsigned)(cap - used), (unsigned)((cap - used) / LOG_BYTES_PER_DAY),
                (unsigned)total, (unsigned)fsUsed, (unsigned long)uptimeSec());
}

bool eraseArmed = false;   // 'x'は2回連続で消去(誤タイプ対策)
void logErase() {
  if (!fsReady) { Serial.println("[LOG] no fs"); return; }
  if (!eraseArmed) { eraseArmed = true; Serial.println("[LOG] send 'x' again to erase"); return; }
  eraseArmed = false;
  LittleFS.remove(LOG_PATH);
  Serial.println("[LOG] erased");
  logEvent("ERASE");
}

// シリアルコマンド処理。何か処理したら true(呼び元で受付窓を延長する)
bool handleSerialCommand() {
  bool handled = false;
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'd': logDump();  break;
      case 'i': logInfo();  break;
      case 'x': logErase(); break;
#if BENCH_MODE
      case 'l': case 'h': benchWater = c; break;
#endif
      default: continue;   // 改行等は無視(窓も延長しない)
    }
    if (c != 'x') eraseArmed = false;   // 間に別コマンドが挟まったら消去は解除
    handled = true;
  }
  return handled;
}

// 起床要因: timer / button(BOOT, ext0) / reset(位置喪失後のESP.restart等ソフトリセット) / cold(電源投入・EN)
const char* wakeCauseName() {
  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_TIMER: return "timer";
    case ESP_SLEEP_WAKEUP_EXT0:  return "button";
    default: break;
  }
  return (esp_reset_reason() == ESP_RST_SW) ? "reset" : "cold";
}

// =====================================================================
// 低レベル
// =====================================================================
// CHANGEで両エッジを取り、レベルと時刻をリングに積む(判定はメインループ側)
void IRAM_ATTR onHallEdge() {
  unsigned long now = millis();
  if (now - lastHallMs < 20) return;   // デバウンス
  lastHallMs = now;
  edgeMs[edgeHead]  = now;
  edgeLvl[edgeHead] = digitalRead(PIN_HALL);
  edgeHead = (edgeHead + 1) & 7;
}

void relayAllOff() {
  digitalWrite(PIN_RELAY_OPEN,  RELAY_OFF);
  digitalWrite(PIN_RELAY_CLOSE, RELAY_OFF);
}

int readACS() {   // 8サンプル≒16ms(駆動ループの停止分解能を保つため短め)
  long sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogRead(PIN_ACS); delay(2); }
  return sum / 8;
}

bool isOverCurrent(int acs) { return abs(acs - ACS_IDLE) > ACS_JAM_DELTA; }

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
// ゲート駆動(ハイブリッド位置制御)
//  位置[mc]をエッジで同期し、エッジ間は実測速度で時間補間する。
//  開: FALLING(磁石がセンサに到達) / 閉: RISING(センサが同じ側から離脱) を数える
//   → 方向によらず「磁石のA側」= 整数カウント位置で同期される。
//  反対側(B側 = 整数+検出幅)のエッジも、検出幅を学習済みなら再同期に使う
//   → 閉の最終区間(最後の同期点→ゼロ点)が 1カウント から 検出幅 に縮み、補間誤差が減る。
//  現場実測(2026-08-30)でリレー投入時の偽エッジが毎回1個入り速度学習を壊していたため、
//  突入マスク中のエッジ破棄・推定位置から遠いエッジの棄却・速度学習の妥当範囲を設けた。
//  targetMc: 目標位置(POS_UNKNOWNなら目標なし=holdPinを離すまで)
//  holdPin : 押している間だけ動かすSW(-1なら使わない)
//  tag     : ログのイベント名。1駆動=1行 "tag,結果,開始mc,終了mc,エッジ数,ms/カウント,ACS"
// =====================================================================
MoveResult moveGate(bool dirOpen, int32_t targetMc, int holdPin, const char* tag) {
  int32_t startMc = posKnown() ? rtcPosMc : 0;
  int dir = dirOpen ? 1 : -1;
  bool hasTarget = (targetMc != POS_UNKNOWN);
  if (hasTarget && (dirOpen ? startMc >= targetMc : startMc <= targetMc)) return { GATE_OK, 0 };

  int &msPer = rtcMsPerCount[dirOpen ? 1 : 0];
  hallPower(true);
  delay(100);                       // センサ起動過渡(電源ON時の偽エッジ)が落ち着くまで待つ
  edgeHead = edgeTail = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onHallEdge, CHANGE);

  unsigned long timeout = hasTarget
      ? (unsigned long)(labs(targetMc - startMc) / 1000.0 * msPer) + TURN_TIMEOUT_MARGIN
      : 0;
  unsigned long start = millis();
  int32_t  anchorMc = startMc;      // 直近の同期位置
  unsigned long anchorMs = start;   // その時刻
  unsigned long prevCountedMs = 0;  // 直近の計数エッジ時刻(速度学習用)
  unsigned long lastASideMs = 0;    // 直近のA側エッジ時刻(検出幅学習用: 開方向のみ)
  int edges = 0;
  int acs = -1;
  GateResult result = GATE_OK;

  digitalWrite(dirOpen ? PIN_RELAY_OPEN : PIN_RELAY_CLOSE, RELAY_ON);
  delay(INRUSH_MASK_MS);            // 突入マスク(この間も位置は時間で進める)
  unsigned long maskUntil = start + INRUSH_MASK_MS;

  auto estimate = [&](unsigned long atMs) -> int32_t {
    if ((long)(atMs - anchorMs) < 0) atMs = anchorMs;   // 基準より古い時刻(起動直後の偽エッジ等)は進めない
    return anchorMc + dir * (int32_t)((atMs - anchorMs) * 1000UL / (unsigned long)msPer);
  };

  while (true) {
    // エッジ処理
    while (edgeTail != edgeHead) {
      unsigned long t = edgeMs[edgeTail];
      bool aSide = dirOpen ? (edgeLvl[edgeTail] == LOW) : (edgeLvl[edgeTail] == HIGH);  // 磁石A側(整数位置)のエッジか
      edgeTail = (edgeTail + 1) & 7;
      if ((long)(t - maskUntil) < 0) continue;            // 突入マスク中=リレーノイズ
      int32_t est = estimate(t);
      int32_t snapped;
      if (aSide) {
        if (rtcPhaseMc < 0) {           // 位相未学習: 最初のA側エッジの推定位置をそのまま磁石位置として学習
          rtcPhaseMc = mod1000(est);
          snapped = est;
        } else {
          snapped = (int32_t)lroundf((est - rtcPhaseMc) / 1000.0f) * 1000 + rtcPhaseMc;   // 位相+整数カウント
        }
      } else {
        if (rtcZoneMc <= 0 || rtcPhaseMc < 0) {
          // 検出幅未学習: 開方向ならA側からの経過で幅を学習(B側は位相+整数+幅)
          if (dirOpen && lastASideMs) {
            int32_t w = (int32_t)((t - lastASideMs) * 1000UL / (unsigned long)msPer);
            if (w >= ZONE_MIN_MC && w <= ZONE_MAX_MC) rtcZoneMc = w;
          }
          continue;
        }
        snapped = (int32_t)lroundf((est - rtcPhaseMc - rtcZoneMc) / 1000.0f) * 1000 + rtcPhaseMc + rtcZoneMc;
      }
      if (labs(est - snapped) > EDGE_SNAP_TOL_MC) {         // 推定と合わない=偽エッジ
        Serial.printf("[MOVE] reject edge: est=%ld snap=%ld\n", (long)est, (long)snapped);
        continue;
      }
      if (aSide) {
        if (prevCountedMs) {                                 // 速度学習(A側エッジ間=1カウント)
          long iv = t - prevCountedMs;
          if (iv >= MS_PER_COUNT_MIN && iv <= MS_PER_COUNT_MAX &&
              iv >= msPer * 7 / 10 && iv <= msPer * 13 / 10) msPer = (msPer + iv) / 2;
        }
        prevCountedMs = t; lastASideMs = t; edges++;
      } else if (dirOpen && lastASideMs) {                   // 検出幅学習(開方向: A側→B側の経過)
        int32_t w = (int32_t)((t - lastASideMs) * 1000UL / (unsigned long)msPer);
        if (w >= ZONE_MIN_MC && w <= ZONE_MAX_MC) rtcZoneMc = (rtcZoneMc + w) / 2;
      }
      anchorMc = snapped; anchorMs = t;
    }
    int32_t pos = estimate(millis());
    if (hasTarget && (dirOpen ? pos >= targetMc : pos <= targetMc)) { anchorMc = targetMc; anchorMs = millis(); break; }
    if (holdPin >= 0 && digitalRead(holdPin) != LOW) break;
    if (timeout && millis() - start > timeout) { result = GATE_TIMEOUT; break; }
    acs = readACS();
    if (isOverCurrent(acs)) { result = GATE_JAM; break; }
    delay(5);
  }
  relayAllOff();
  int32_t endMc = estimate(millis());
  detachInterrupt(digitalPinToInterrupt(PIN_HALL));
  hallPower(false);
  rtcPosMc = endMc;
  logEvent("%s,%s,%ld,%ld,%d,%d,%ld,%d", tag, gateResultName(result), (long)startMc, (long)endMc, edges, msPer, (long)rtcZoneMc, acs);
  delay(300);
  return { result, edges };
}

// =====================================================================
// 開閉シーケンス
//  失敗時: 位置信頼を失う → rtcPosMc=POS_UNKNOWN → 次回起動でCALIB要求(LEDゆっくり点滅)
//  → 30分放置で消灯 → 「消灯=異常」の意味論に合流
// =====================================================================
bool openGate() {
  if (!posKnown() || openMc <= 0) return false;
  if (rtcPosMc >= openMc) return true;
  digitalWrite(PIN_LED, HIGH);  // 動作中=点灯
  MoveResult m = moveGate(true, openMc, -1, "OPEN");
  digitalWrite(PIN_LED, LOW);
  if (m.r == GATE_OK) return true;

  Serial.printf("[OPEN] fail(%d) -> position lost\n", m.r);
  moveGate(false, rtcPosMc - 1000, -1, "RETREAT");   // 安全側: 1カウント繰り出して張力を抜く(結果は見ない: どのみち位置は破棄しCALIBへ)
  rtcPosMc = POS_UNKNOWN; rtcPhaseMc = -1;
  return false;
}

bool closeGate() {
  if (!posKnown()) return false;
  if (rtcPosMc <= CLOSE_BIAS_MC) return true;
  digitalWrite(PIN_LED, HIGH);
  // 繰り出しは常に「登録ゼロ点のCLOSE_BIAS_MC手前まで」= 弛みも巻き戻りも作らない
  MoveResult m = moveGate(false, CLOSE_BIAS_MC, -1, "CLOSE");
  digitalWrite(PIN_LED, LOW);
  if (m.r == GATE_JAM) {                       // 繰り出しJAM=ワイヤー絡み等の異常
    Serial.println("[CLOSE] jam -> position lost");
    rtcPosMc = POS_UNKNOWN; rtcPhaseMc = -1;
    return false;
  }
  if (m.r == GATE_TIMEOUT && m.edges == 0) {   // 1エッジも来ないTIMEOUT=ホールセンサ無応答
    Serial.println("[CLOSE] timeout with no hall edges -> position lost");
    rtcPosMc = POS_UNKNOWN; rtcPhaseMc = -1;
    return false;
  }
  return true;   // エッジありのTIMEOUTは速度想定より遅かっただけとみなす(位置はmoveGateの推定値)
}

// =====================================================================
// 水位判定
// =====================================================================
#if BENCH_MODE
WaterSample sampleWater() { return { benchWater != 'l', -1, -1 }; }
#else
bool wet(int adc) { return adc < WATER_ADC_THRESHOLD; }
int avgAdc(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogRead(pin); delay(2); }
  return sum / 8;
}
// 1回の通電で上下両方を読む(電食を抑えるため通電時間を最小に)
void readElectrodes(int &adcHi, int &adcLo) {
  digitalWrite(PIN_ELEC_DRIVE, HIGH);
  delay(30);
  adcHi = avgAdc(PIN_ELEC_HIGH);
  adcLo = avgAdc(PIN_ELEC_LOW);
  digitalWrite(PIN_ELEC_DRIVE, LOW);
  Serial.printf("[ELEC] VP(上)=%d %s / VN(下)=%d %s\n",
                adcHi, wet(adcHi) ? "WET" : "DRY", adcLo, wet(adcLo) ? "WET" : "DRY");
}
WaterSample sampleWater() {
  WaterSample w = { false, -1, -1 };
  readElectrodes(w.adcHi, w.adcLo);
  bool hiWet = wet(w.adcHi), loWet = wet(w.adcLo);
  if (hiWet && loWet)        w.high = true;
  else if (!hiWet && !loWet) w.high = false;
  else                       w.high = isClosed();  // 中間帯(下のみ濡れ) / 矛盾(上のみ濡れ=水膜・断線等)は現状維持
  return w;
}
#endif

// 手動SWが押されていれば手動動作を実行して眠る(戻らない)。窓・VERIFY中の共通処理
void manualSwitchCheck() {
  if (digitalRead(PIN_SW_OPEN) == LOW)  { logEvent("MANUAL,open,%ld",  (long)rtcPosMc); openGate();  goToSleep(); }
  if (digitalRead(PIN_SW_CLOSE) == LOW) { logEvent("MANUAL,close,%ld", (long)rtcPosMc); closeGate(); goToSleep(); }
}

// 待機しつつ手動SW/シリアルを見る(delayの置き換え)
void waitWatching(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    manualSwitchCheck();
    handleSerialCommand();
    delay(20);
  }
}

bool verifyRequest(bool wantHigh) {
  digitalWrite(PIN_LED, HIGH);  // 検証中=点灯
  Serial.printf("[VERIFY] start (need %d consecutive)\n", VERIFY_COUNT);
  for (int i = 0; i < VERIFY_COUNT; i++) {
    waitWatching(VERIFY_INTERVAL_MS);   // 最大60秒の間も手動SWを優先する
    WaterSample w = sampleWater();
    Serial.printf("[VERIFY] %d/%d: wantHigh=%d now=%d\n", i + 1, VERIFY_COUNT, wantHigh, w.high);
    if (w.high != wantHigh) {
      Serial.println("[VERIFY] flipped -> cancel");
      logEvent("VERIFY,cancel,%d,%d,%d", i + 1, w.adcHi, w.adcLo);
      digitalWrite(PIN_LED, LOW);
      return false;
    }
  }
  Serial.println("[VERIFY] confirmed");
  logEvent("VERIFY,confirmed,%d", VERIFY_COUNT);
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
void jog(bool dirOpen) {
  int pin = dirOpen ? PIN_SW_OPEN : PIN_SW_CLOSE;
  MoveResult m = moveGate(dirOpen, POS_UNKNOWN, pin, "JOG");   // 押している間だけ。JAMで自動停止
  Serial.printf("[JOG] dir=%s -> pos=%ld mc%s\n", dirOpen ? "OPEN" : "CLOSE", (long)rtcPosMc,
                m.r == GATE_JAM ? " JAM -> stopped" : "");
  if (m.r == GATE_JAM) {
    ledBlink(10, 50, 50);   // 速点滅×10=異常(全開無効と同じ合図)。SWを離すまで待って再開
    while (digitalRead(pin) == LOW) delay(20);
  }
}

void calibMode() {
  Serial.println("[CALIB] enter. jog with SW, leave 10s to confirm. (serial d/i/x also accepted)");
  logEvent("CALIB,enter,%ld,%ld", posKnown() ? (long)rtcPosMc : -1L, (long)openMc);
  if (posKnown() && rtcPhaseMc >= 0) rtcPhaseMc = mod1000(rtcPhaseMc - rtcPosMc);  // 座標を張り替えても磁石位相は維持
  else rtcPhaseMc = -1;
  rtcPosMc = 0;           // ゼロ点確定までは仮の座標系(rtcMagicは未設定のまま=再起動すればCALIBに戻る)
  int phase = 0;          // 0=ゼロ点待ち 1=全開位置待ち
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
      logEvent("CALIB,timeout");
      LittleFS.end();
      digitalWrite(PIN_LED, LOW);
      relayAllOff();
      esp_deep_sleep_start();   // 起床要因なし=永久停止
    }
    // シリアルコマンド(d/i/x): 持ち帰り時はRTCメモリ消失でここに来るのでCALIB中も受け付ける
    if (handleSerialCommand()) { enterTime = millis(); lastAction = millis(); }  // ダンプ中に10秒確定が走らないよう両方延長
    // ジョグ
    if (digitalRead(PIN_SW_OPEN) == LOW)  { digitalWrite(PIN_LED, HIGH); jog(true);  touched = true; lastAction = millis(); enterTime = millis(); }
    if (digitalRead(PIN_SW_CLOSE) == LOW) { digitalWrite(PIN_LED, HIGH); jog(false); touched = true; lastAction = millis(); enterTime = millis(); }

    // 10秒放置で確定(そのフェーズで一度は操作があった場合のみ)
    if (touched && millis() - lastAction > CALIB_CONFIRM_MS) {
      if (phase == 0) {
        if (rtcPhaseMc >= 0) rtcPhaseMc = mod1000(rtcPhaseMc - rtcPosMc);  // ゼロ点移動に合わせ位相も張り替え
        rtcPosMc = 0;                       // ここがゼロ点
        ledBlink(3, 100, 100);
        Serial.println("[CALIB] zero registered");
        logEvent("CALIB,zero");
        phase = 1; touched = false; lastAction = millis(); enterTime = millis();
      } else {
        Serial.printf("[CALIB] confirm check: pos=%ld mc\n", (long)rtcPosMc);
        if (rtcPosMc < OPEN_MIN_MC) {       // 全開が小さすぎる/ゼロ点より閉側は無効。やり直し待ち
          ledBlink(10, 50, 50);
          Serial.println("[CALIB] invalid open position. jog again.");
          logEvent("CALIB,invalid,%ld", (long)rtcPosMc);
          touched = false; lastAction = millis();
          continue;
        }
        openMc = rtcPosMc;
        prefs.begin("gate", false);
        prefs.putInt("openMc", openMc);
        prefs.putInt("msClose", rtcMsPerCount[0]);
        prefs.putInt("msOpen",  rtcMsPerCount[1]);
        prefs.putInt("zoneMc",  rtcZoneMc);
        prefs.end();
        ledBlink(5, 100, 100);
        Serial.printf("[CALIB] open position registered: %ld mc (%.2f counts)\n", (long)openMc, openMc / 1000.0);
        logEvent("CALIB,open,%ld,%d,%d,%ld", (long)openMc, rtcMsPerCount[0], rtcMsPerCount[1], (long)rtcZoneMc);

        // 登録した全開位置から全閉へ戻して運用開始
        rtcMagic = RTC_MAGIC;
        closeGate();
        Serial.println("[CALIB] done -> normal operation");
        logEvent("CALIB,done,%ld", (long)rtcPosMc);
        return;
      }
    }
    delay(20);
  }
}

// =====================================================================
// 通常サイクル(起床ごとに実行 / タイマー起床とBOOT起床で処理は同じ。要因はBOOT行に記録)
// =====================================================================
void wakeCycle() {
  // 起床表示 兼 操作受付ウィンドウ(5秒): LED点灯のままSW/シリアルコマンドを待つ
  //  シリアル: d=ログダンプ i=残量 x=消去(BENCH: l/h=水位指令)。処理のたびに窓を延長。
  digitalWrite(PIN_LED, HIGH);
  unsigned long t0 = millis();
  while (millis() - t0 < SW_WINDOW_MS) {
    manualSwitchCheck();
    if (handleSerialCommand()) t0 = millis();
    delay(20);
  }
  digitalWrite(PIN_LED, LOW);

  WaterSample w = sampleWater();
  bool shouldClose = w.high;
  bool closed = isClosed();
  const char* action = (shouldClose == closed) ? "keep" : shouldClose ? "close" : "open";
  Serial.printf("[CYCLE] wantHigh=%d position=%ld mc(%s) -> %s\n",
                w.high, (long)rtcPosMc, closed ? "closed" : "open", action);
  logEvent("CYCLE,%d,%d,%ld,%s", w.adcHi, w.adcLo, (long)rtcPosMc, action);
  if (shouldClose != closed) {
    if (verifyRequest(w.high)) {
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
  LittleFS.end();
  if (!posKnown()) { rtcMagic = 0; ESP.restart(); }
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
  // core 3.x では出力化前のdigitalWriteは無効(ログだけ出て捨てられる)ため、pinMode直後にOFFを明示する
  pinMode(PIN_RELAY_OPEN,  OUTPUT); digitalWrite(PIN_RELAY_OPEN,  RELAY_OFF);
  pinMode(PIN_RELAY_CLOSE, OUTPUT); digitalWrite(PIN_RELAY_CLOSE, RELAY_OFF);
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
  openMc = prefs.getInt("openMc", 0);
  if (rtcMagic != RTC_MAGIC) {   // cold/reset時は学習速度をNVSから復元(スリープ跨ぎはRTC値をそのまま使う)
    rtcMsPerCount[0] = prefs.getInt("msClose", MS_PER_COUNT_DEF);
    rtcMsPerCount[1] = prefs.getInt("msOpen",  MS_PER_COUNT_DEF);
    rtcZoneMc        = prefs.getInt("zoneMc",  0);
  }
  prefs.end();

  logInit();
  logEvent("BOOT,%s,%ld,%ld", wakeCauseName(), posKnown() ? (long)rtcPosMc : -1L, (long)openMc);

  // 入口の2段の門:
  //  位置不定 or 全開回転数未登録 → CALIB / それ以外 → 通常サイクル
  if (rtcMagic != RTC_MAGIC || !posKnown() || openMc <= 0) {
    calibMode();          // 完了時のみ戻ってくる(rtcMagic等は設定済み)
  }
  wakeCycle();            // 1起床分を実行して眠る(戻らない)
}

void loop() {}  // 全処理はsetup内で完結(毎起床=毎setup)
