#include "TM1650.h"
#include "EEPROM.h"
#include "OneButton.h"

/******************** 可靠性开关 ********************/
// 独立看门狗：一旦程序真的跑飞，2 秒内自动复位，不再需要手动断电。
// 同时能帮你判断"卡死"的性质：
//   - 看门狗能救回来  -> 软件跑飞 / 死循环
//   - 看门狗也救不回 -> 硬件闩锁或电源塌陷（需查继电器续流与去耦）
// 调试下载阶段可改为 0 临时关闭。
#define USE_IWDG 1
#define IWDG_TIMEOUT_MS 2000

// 硬件引脚定义
#define CLK PF1       // TM1650时钟
#define DIO PF0       // TM1650数据
#define TEST_BTN PA4  // 测试键
#define FUNC_BTN PB6  // 功能键
#define EN PB2        // 升压使能
#define VSW PB1       // 升压切换
#define RELAY PA5     // 继电器控制
#define BUZZER PB3    // 蜂鸣器
#define NO1 PA0       // 常开触点1
#define NO2 PA7       // 常开触点2
#define NC1 PA1       // 常闭触点1
#define NC2 PB0       // 常闭触点2

// 全局状态变量
bool BUZZER_STATE = false;  // 蜂鸣器状态
uint8_t RELAY_STATE = 0;    // 继电器状态 0:释放 1:吸合
uint8_t contact_form = 0;   // 触点类型 1-8
uint8_t voltage = 12;       // 当前电压

// 触点状态
// 用「数组 + 引用」让去抖后的稳定值只有一份来源，
// 避免"采样值"与"去抖值"两份副本不同步造成的误判。
// 索引约定：0=NO1 1=NO2 2=NC1 3=NC2
#define IDX_NO1 0
#define IDX_NO2 1
#define IDX_NC1 2
#define IDX_NC2 3
static bool contact_state[4] = { false, false, false, false };
bool& NO1_STATE = contact_state[IDX_NO1];
bool& NO2_STATE = contact_state[IDX_NO2];
bool& NC1_STATE = contact_state[IDX_NC1];
bool& NC2_STATE = contact_state[IDX_NC2];

/******************** 继电器动作保护窗（按键去抖） ********************/
// 去抖只服务一个目的：测试键按下/释放（或识别流程）导致继电器动作后，
// 触点有 ~10ms 机械延时 + 弹跳，这段时间采到的电平不代表真实状态。
// 若拿去当判据，ForceTest 会拿到「新继电器状态 + 旧触点值」而误鸣一声。
//
// 保护窗 = 继电器动作后的 RELAY_GUARD_MS 内，ForceTest 静音待判。
// 窗口一过，ForceTest 直接读引脚原始电平、逐循环判定，灵敏响应——
// 压力测试中真实故障的报警延迟就是主循环周期，没有滤波拖累。
//
// （旧版的三级滤波：消隐 + 稳定计数 + all_stable 闸门已删除——
//   那套会让所有时段的触点变化都延迟 40ms 才被采纳，与"灵敏响应"冲突。）
#define RELAY_GUARD_MS 50  // 需 > 继电器 ~10ms 机械延时 + 弹跳，留足余量

static const uint8_t contact_pin[4] = { NO1, NO2, NC1, NC2 };

uint32_t relay_change_time = 0;  // 最近一次继电器驱动电平变化的时刻

// 继电器是否仍在动作保护窗内（此期间触点电平不可信）
bool contactsSettling() {
  return (millis() - relay_change_time) < RELAY_GUARD_MS;
}

/******************** 串口事件追踪（排障用，已注释） ********************/
// 【已禁用】置 1 后，串口会在"继电器动作 / 触点稳定 / 报警判定翻转"三个时刻各打印一行，
// 用来确认某一次鸣响到底是判据误判还是真实故障。需要排障时取消注释即可恢复。
// #define DEBUG_TRACE 0
//
// #if DEBUG_TRACE
// static void traceEvent(const char *tag) {
//   Serial.print(millis());
//   Serial.print("  ");
//   Serial.print(tag);
//   Serial.print("  RLY=");
//   Serial.print(RELAY_STATE);
//   Serial.print("  NO1=");
//   Serial.print(NO1_STATE);
//   Serial.print(" NO2=");
//   Serial.print(NO2_STATE);
//   Serial.print(" NC1=");
//   Serial.print(NC1_STATE);
//   Serial.print(" NC2=");
//   Serial.print(NC2_STATE);
//   Serial.print("  BZ=");
//   Serial.println(digitalRead(BUZZER));
// }
// #else
// #define traceEvent(tag) ((void)0)
// #endif

// 继电器统一入口：所有动作都走这里，记录动作时刻供保护窗判断
void setRelay(bool on) {
  digitalWrite(RELAY, on ? HIGH : LOW);
  RELAY_STATE = on ? 1 : 0;
  relay_change_time = millis();
  // traceEvent(on ? "RLY->ON" : "RLY->OFF");  // 调试追踪已禁用
}

bool BEEP_ON = false;    //蜂鸣器状态
bool last_beep = false;  // ForceTest报警状态记录（beep序列结束后需复位）

static bool last_btn_state = HIGH;    // 按钮前次状态
static uint32_t last_press_time = 0;  // 最后按下时间

//测试键模式
uint8_t TEST_MODE = 0;  //  0：点动 1：自锁

//数码管显示管理
bool show_contact = false;       // 正在短时显示触点类型
uint32_t contact_show_time = 0;  // 触点类型开始显示的时刻

TM1650 DigitalTube(CLK, DIO);  // 数码管
OneButton button;

/******************** 核心功能函数 ********************/
// 非阻塞蜂鸣器状态机
bool beep_active = false;   // beep序列进行中（此期间蜂鸣器由状态机独占）
bool beep_phase = false;    // 当前相位 true=鸣响 false=静音
uint8_t beep_left = 0;      // 剩余鸣响次数
uint32_t beep_timer = 0;

void beep(uint8_t times) {
  beep_left = times;
  beep_phase = true;
  beep_active = true;
  beep_timer = millis();
  digitalWrite(BUZZER, HIGH);
}

// 在loop()中每次循环调用，驱动beep的响/停时序
void beepUpdate() {
  if (!beep_active) return;
  uint32_t now = millis();
  if (now - beep_timer < 100) return;
  beep_timer = now;
  if (beep_phase) {             // 鸣响100ms结束，转静音
    beep_phase = false;
    digitalWrite(BUZZER, LOW);
  } else {                      // 静音100ms结束
    if (--beep_left > 0) {      // 继续下一声
      beep_phase = true;
      digitalWrite(BUZZER, HIGH);
    } else {                    // 序列结束，归还蜂鸣器
      beep_active = false;
      BUZZER_STATE = false;
      last_beep = false;        // 复位报警状态，若故障仍在ForceTest立即恢复鸣响
    }
  }
}

// 显示当前电压
void displayVoltage() {
  DigitalTube.displayChar(0, '0' + voltage / 10);
  DigitalTube.displayChar(1, '0' + voltage % 10);
}

// 短时显示触点类型（1s后由主循环恢复电压显示）
void showContactType() {
  const char *labels[] = { "--", "1A", "1B", "1C", "2A", "2B", "2C", "AB", "BA" };
  const char *s = labels[contact_form <= 8 ? contact_form : 0];
  DigitalTube.displayChar(0, s[0]);
  DigitalTube.displayChar(1, s[1]);
  show_contact = true;
  contact_show_time = millis();
}

// 立即采纳某路触点的当前电平（识别流程专用：此时继电器已稳定 300ms）。
// contact_state 只服务识别流程；ForceTest 不用它，直接读引脚。
void contactCommit(uint8_t idx) {
  contact_state[idx] = digitalRead(contact_pin[idx]);
}

/******************** 触点类型识别（非阻塞状态机） ********************/
// 【重要修改】原实现在 OneButton 的长按回调里连续 delay(300) x3，阻塞主循环 900ms。
// 这段时间里 loop 停摆：按键扫描、数码管刷新、蜂鸣器时序全部冻结，
// 而继电器又正在高频通断（强干扰源），极易造成状态错乱甚至跑飞。
// 现改为每 300ms 推进一步，由主循环驱动，全程不阻塞。
#define CA_IDLE 0     // 空闲
#define CA_SETTLE 1   // 继电器已释放，等待稳定后采样常闭
#define CA_PICKED 2   // 继电器已吸合，等待稳定后采样常开
#define CA_FINISH 3   // 已断开，收尾判定

uint8_t ca_step = CA_IDLE;
uint32_t ca_timer = 0;

// 长按回调：只置标志，绝不做耗时操作
void contact_arrangement() {
  if (ca_step != CA_IDLE) return;   // 上一次识别未完成，忽略重复触发

  if (!BEEP_ON) {
    setRelay(false);   // 释放继电器，开始识别
    ca_step = CA_SETTLE;
    ca_timer = millis();
  } else {
    contact_form = 0;
    BEEP_ON = false;
    showContactType();
    beep(3);
  }
}

// 主循环每轮调用，推进识别流程
void contactUpdate() {
  if (ca_step == CA_IDLE) return;
  if (millis() - ca_timer < 300) return;
  ca_timer = millis();

  switch (ca_step) {
    case CA_SETTLE:  // 释放 300ms 后采样常闭
      contactCommit(IDX_NC1);
      contactCommit(IDX_NC2);
      setRelay(true);
      ca_step = CA_PICKED;
      break;

    case CA_PICKED:  // 吸合 300ms 后采样常开
      contactCommit(IDX_NO1);
      contactCommit(IDX_NO2);
      setRelay(false);
      ca_step = CA_FINISH;
      break;

    case CA_FINISH: {  // 收尾：同步状态 + 判定 + 提示
      // 原代码遗留问题已修复：识别结束时继电器已断开，但 RELAY_STATE 仍停留在
      // 旧值，导致 ForceTest 用错误的吸合/释放状态判断。现在 setRelay() 会
      // 自动保持 RELAY_STATE 与实际驱动电平严格同步，这里无需再手动清零。
      ca_step = CA_IDLE;

      const uint8_t state_mask = (NO1_STATE << 3) | (NO2_STATE << 2)
                                 | (NC1_STATE << 1) | NC2_STATE;

      switch (state_mask) {
        case 0b1000: contact_form = 1; break;  // 1A
        case 0b0010: contact_form = 2; break;  // 1B
        case 0b1010: contact_form = 3; break;  // 1C
        case 0b1100: contact_form = 4; break;  // 2A
        case 0b0011: contact_form = 5; break;  // 2B
        case 0b1111: contact_form = 6; break;  // 2C
        case 0b1001: contact_form = 7; break;  // 1A + 1B
        case 0b0110: contact_form = 8; break;  // 1B + 1A
        default: contact_form = 0;             // 未知
      }
      BEEP_ON = true;

      // 识别结束后重新同步测试键历史状态，避免残留电平被误判为一次按下
      last_btn_state = digitalRead(TEST_BTN);

      showContactType();
      beep(3);
      break;
    }

    default:
      ca_step = CA_IDLE;
      break;
  }
}

// 电压切换回调
void VoltageSwitch() {
  switch (voltage) {
    case 5:
      voltage = 12;
      digitalWrite(EN, HIGH);
      digitalWrite(VSW, LOW);
      break;
    case 12:
      voltage = 24;
      digitalWrite(EN, HIGH);
      digitalWrite(VSW, HIGH);
      break;
    case 24:
      voltage = 5;
      digitalWrite(EN, LOW);
      break;
  }

  // 显示电压
  displayVoltage();

  // 升压切换瞬间电流冲击最大，让电源先稳几毫秒再驱动蜂鸣器，
  // 避免与继电器/蜂鸣器同时拉低 VCC。
  delay(5);

  // 蜂鸣器提示
  beep(1);
}

void SW_MODE() {
  if (TEST_MODE == 0) {
    TEST_MODE = 1;
  } else {
    TEST_MODE = 0;
  }
  beep(2);
}

// 压力测试逻辑
// 判定数据源：直接 digitalRead 四路触点引脚，逐循环刷新，无任何滤波。
// 灵敏度 = 主循环周期（微秒级），真实故障立即报警。
// 唯一的例外是保护窗：继电器刚动作后的 RELAY_GUARD_MS 内触点电平不可信，
// 静音待判，避免「新继电器状态 + 旧触点值」的错配误鸣（按键去抖）。
void ForceTest() {
  bool should_beep = false;

  if (contact_form == 0) return;
  if (beep_active) return;  // beep序列进行中，蜂鸣器由状态机驱动

  // 继电器动作保护窗：先静音并复位报警状态，窗口过后立即重新判定
  if (contactsSettling()) {
    if (BUZZER_STATE) {
      BUZZER_STATE = false;
      digitalWrite(BUZZER, LOW);
    }
    last_beep = false;
    return;
  }

  // 直接采样引脚原始电平（不经任何滤波，灵敏响应）
  bool no1 = digitalRead(NO1);
  bool no2 = digitalRead(NO2);
  bool nc1 = digitalRead(NC1);
  bool nc2 = digitalRead(NC2);

  switch (RELAY_STATE) {
    case 0:  // 释放状态
      switch (contact_form) {
        case 1: should_beep = no1; break;
        case 2:
        case 3: should_beep = !nc1 || no1; break;
        case 4: should_beep = no1 || no2; break;
        case 5: should_beep = !nc1 || !nc2; break;
        case 6: should_beep = !nc1 || !nc2 || no1 || no2; break;
        case 7: should_beep = no1 || !nc2; break;
        case 8: should_beep = no2 || !nc1; break;
        default: break;
      }
      break;

    case 1:  // 吸合状态
      switch (contact_form) {
        case 1:
        case 3: should_beep = !no1 || nc1; break;
        case 4: should_beep = !no1 || !no2; break;
        case 5: should_beep = nc1 || nc2; break;
        case 6: should_beep = !no1 || !no2 || nc1 || nc2; break;
        case 7: should_beep = !no1 || nc2; break;
        case 8: should_beep = !no2 || nc1; break;
        default: break;
      }
      break;
    default:
      break;
  }

  // 追踪放在 BEEP_ON 判断之外：这样即使蜂鸣器功能关闭，也能从串口看出
  // 判据本身有没有误翻转，便于区分"误鸣"和"真实故障"。
  // 【已禁用】if (should_beep != last_beep) {
  //   traceEvent(should_beep ? "ALARM+" : "ALARM-");
  // }

  if (should_beep != last_beep && BEEP_ON == true) {
    BUZZER_STATE = should_beep;
    digitalWrite(BUZZER, BUZZER_STATE);
    last_beep = should_beep;
  }
}

/******************** 看门狗 ********************/
// 【注意】不能用 HAL 的 IWDG：
// BSP 的 cores/AirMCU/air/airyyxx_hal_conf.h 把 HAL_IWDG_MODULE_ENABLED 放在 #if 0
// 的 "Unused HAL modules" 块里（注释：IWDG built-in library uses LL），
// 所以 SrcWrapper 编出来的 air001xx_hal_iwdg.c 是空的，链接时报
// undefined reference to HAL_IWDG_Init / HAL_IWDG_Refresh。
// LL 驱动是头文件里的 static inline，不依赖模块开关，也不需要链接库。
#if USE_IWDG
#include "air001xx_ll_iwdg.h"

// LSI ≈ 32kHz，64 分频后计数频率 ≈ 500Hz，即每个计数值约 2ms
void IWDG_Config(uint32_t timeout_ms) {
  uint32_t reload = timeout_ms / 2;
  if (reload > 0x0FFF) reload = 0x0FFF;
  if (reload < 2)      reload = 2;

  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
  LL_IWDG_SetReloadCounter(IWDG, reload);

  // 等分频/重装寄存器同步完成（PVU/RVU 清零），带超时，防止异常时卡在这
  uint32_t guard = 0;
  while (!LL_IWDG_IsReady(IWDG)) {
    if (++guard > 200000UL) break;
  }

  LL_IWDG_Enable(IWDG);            // 配置就位后再启动；启动后无法关闭，只能复位
  LL_IWDG_ReloadCounter(IWDG);     // 喂一次，让计数器从满值开始递减
}

static inline void IWDG_Kick(void) {
  LL_IWDG_ReloadCounter(IWDG);
}
#else
#define IWDG_Config(ms)
#define IWDG_Kick()
#endif

/******************** 初始化配置 ********************/
void setup() {
  //串口初始化
  Serial.begin(115200);

#if USE_IWDG
  // 先判断上次复位原因，这是定位卡死性质的关键线索
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    Serial.println("!! WARNING: last reset was triggered by IWDG (code hung) !!");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
#endif

  Serial.println("Author: Latiaomaibu");
  Serial.println("Build Date: 20260903");
  Serial.println("Firmware Version: 1.4");
  Serial.println("What are you looking at?");

  // 引脚初始化
  pinMode(NO1, INPUT_PULLDOWN);
  pinMode(NO2, INPUT_PULLDOWN);
  pinMode(NC1, INPUT_PULLDOWN);
  pinMode(NC2, INPUT_PULLDOWN);
  pinMode(TEST_BTN, INPUT_PULLUP);
  pinMode(EN, OUTPUT);
  pinMode(VSW, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // 功能键外部下拉，接VCC有效，按下为高电平（activeLow=false）
  // 由浮空 INPUT 改为 INPUT_PULLDOWN：继电器动作时的空间干扰会在浮空引脚上
  // 感应出高电平，造成误判长按并再次触发继电器，形成正反馈。
  button.setup(FUNC_BTN, INPUT_PULLDOWN, false);
  button.setDebounceMs(50);
  button.setClickMs(400);
  button.setPressMs(1000);   // 长按阈值由默认 800ms 提到 1000ms，减少误触发
  button.attachClick(VoltageSwitch);
  //  button.attachDoubleClick(SW_MODE);
  button.attachLongPressStart(contact_arrangement);

  setRelay(false);  // 上电确保继电器释放（同时启动消隐窗口）
  digitalWrite(BUZZER, LOW);
  digitalWrite(EN, HIGH);  // 默认12V
  digitalWrite(VSW, LOW);

  // 数码管初始化
  DigitalTube.setBrightness(1);
  for (char i = 0; i < 3; i++) {
    DigitalTube.clearBit(i);
  }
  // 显示12V
  displayVoltage();

#if USE_IWDG
  IWDG_Config(IWDG_TIMEOUT_MS);  // 所有初始化完成后再启动看门狗
#endif
}

/******************** 主循环 ********************/
void loop() {
  uint32_t now = millis();

  // 按键状态检测（OneButton内部自带毫秒级消抖，高频tick无害且更精准）
  button.tick();
  // 蜂鸣器非阻塞时序
  beepUpdate();
  // 触点识别状态机（非阻塞）
  contactUpdate();

  // 10ms节拍任务
  static uint32_t last_task_time = 0;
  if (now - last_task_time >= 10) {
    last_task_time = now;

    // 触点类型显示超时1s，恢复电压显示
    if (show_contact && (now - contact_show_time > 1000)) {
      show_contact = false;
      displayVoltage();
    }

    // 识别流程进行中时，跳过按键与触点采样，避免和继电器动作打架
    if (ca_step == CA_IDLE) {
      // 按钮状态读取
      bool current_btn = digitalRead(TEST_BTN);

      // 模式切换核心逻辑：边沿触发 + 30ms去抖
      // 去抖窗口内的边沿不动作也不更新历史状态，避免丢失释放沿
      if (current_btn != last_btn_state) {
        if ((now - last_press_time) > 30) {
          last_press_time = now;
          last_btn_state = current_btn;
          if (TEST_MODE) {  // 自锁模式：按下翻转
            if (current_btn == LOW) {
              setRelay(RELAY_STATE == 0);  // 翻转并立即生效
            }
          } else {  // 点动模式：跟随按键
            setRelay(current_btn == LOW);
          }
        }
      }
    }
  }

  //调试信息
  //Serial.print(voltage);
  //Serial.print(TEST_MODE);
  //Serial.print(contact_form);
  //Serial.print(RELAY_STATE);
  //Serial.print(NO1_STATE);
  //Serial.print(NO2_STATE);
  //Serial.print(NC1_STATE);
  //Serial.println(NC2_STATE);

  // 执行压力测试（识别期间跳过，避免蜂鸣器与继电器动作叠加）
  if (ca_step == CA_IDLE) {
    ForceTest();
  }

#if USE_IWDG
  IWDG_Kick();
#endif
}
