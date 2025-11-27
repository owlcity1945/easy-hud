#define BLINKER_PRINT Serial
#define BLINKER_BLE
#include <Blinker.h>

#include <TFT_eSPI.h>
#include <SPI.h>

#include "esp32_tft.h"

#include <Arduino.h>
#include "OneButton.h"

//增加I2C姿态传感器
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno(55);

//增加与stm32通信
#include <HardwareSerial.h>
HardwareSerial mySerial(1);  //使用Serial


// 控件定义（按钮 + 数值）
BlinkerButton button_inc_pemh("btn-pemh-inc");
BlinkerButton button_dec_pemh("btn-pemh-dec");
BlinkerButton button_inc_peml("btn-peml-inc");
BlinkerButton button_dec_peml("btn-peml-dec");
BlinkerButton airpump("airpump");  //带自锁充气电磁阀按键，长按充气保持，短按停止
BlinkerButton launch("launch");    //发射按键
BlinkerButton vent("vent");        //泄压按键


BlinkerNumber pemh_display("pemh");
BlinkerNumber peml_display("peml");
BlinkerNumber p_xs_display("p_xs");
BlinkerNumber airpump_display("airpump");

BlinkerNumber Number1("num-valve");
BlinkerSlider Slider1("ran-valve");//滑块

const uint8_t PIN_INPUT = 35;  // C 相
OneButton button(PIN_INPUT, true);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

void setup() {
  Serial.begin(115200);

#if defined(BLINKER_PRINT)
  BLINKER_DEBUG.stream(BLINKER_PRINT);
#endif

  // 设置舵机1
  ledcSetup(channel1, freq, resolution);
  ledcAttachPin(servo1, channel1);
  // 设置舵机2
  ledcSetup(channel2, freq, resolution);
  ledcAttachPin(servo2, channel2);

  mySerial.begin(9600, SERIAL_8N1, 16, 17);  //esp32通信设置串口，16为RX，17为TX,使用 8 数据位、1 停止位、无奇偶校验

  Wire.begin(18, 19);     // BHO055姿态传感器 I2C 引脚：SDA = GPIO18, SCL = GPIO19
  Wire.setClock(100000);  // 100kHz，提高兼容性

  if (!bno.begin()) {
    Serial.println("未找到 BNO055");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);  // 使用外部晶振（如果有）
  Serial.println("BNO055 初始化完成");

  tft.init();
  tft.setRotation(0);
  spr.setColorDepth(16);  // ✅ 改为16位色深（RGB565）  // 使用 8 位颜色模式（256色）
  spr.createSprite(WIDTH, HEIGHT);
  tft.fillScreen(TFT_BLACK);

  tft.setTextSize(2);  // 设置文字大小
  tft.setCursor(20, 20);
  tft.print("Connecting to Bluetooth...");  // 显示连接 Wi-Fi 的提示
  tft.fillScreen(TFT_BLACK);                // 清空屏幕


  // 初始化blinker
  Blinker.begin();
  Blinker.attachData(data_read);
  // 按钮回调绑定
  button_inc_pemh.attach(button_inc_pemh_callback);
  button_dec_pemh.attach(button_dec_pemh_callback);
  button_inc_peml.attach(button_inc_peml_callback);
  button_dec_peml.attach(button_dec_peml_callback);
  airpump.attach(airpump_callback);
  launch.attach(launch_callback);
  vent.attach(vent_callback);

  Slider1.attach(slider1_callback);//增加滑动条

  // 注册心跳包回调（主动反馈）
  Blinker.attachHeartbeat(heartbeat);

  //pwm 调光初始化
  // ledcSetup(channel, freq, resolution);  //设置通道
  // ledcAttachPin(pwm, channel);           //通道与引脚连接
  // ledcWrite(channel, 255);

  pinMode(32, INPUT);  //编码器配置
  pinMode(33, INPUT);
  pinMode(PIN_INPUT, INPUT);

  digitalWrite(32, HIGH);  //turn pullup resistor on
  digitalWrite(33, HIGH);  //turn pullup resistor on

  attachInterrupt(32, Encoder, CHANGE);  //中断配置
  attachInterrupt(33, Encoder, CHANGE);

  pinMode(BEEP_PIN, OUTPUT);  //BEEP
  Beep_off;
  Turn_Beep();

  //多功能按键
  button.reset();  //清除一下按钮状态机的状态
  button.attachClick(click);
  button.attachDoubleClick(doubleclick);
  button.attachLongPressStart(longPressStart);
  button.attachDuringLongPress(duringLongPress);
  button.attachLongPressStop(longPressStop);

  ledcWrite(channel1, 307);  //关闭0°
  ledcWrite(channel2, 307);  //关闭0°
}


void loop() {
  Blinker.run();

  button.tick();
  words_display();

  if (counter >= 360) counter = 360;
  else if (counter <= 0) counter = 0;
}

// 增加 pemh
void button_inc_pemh_callback(const String &state) {
  if (state == "tap") {
    pemh += 0.1f;
    if (pemh > 10.0f) pemh = 10.0f;
    button_inc_pemh.print("on");
    pemh_display.print(pemh);
    send_to_stm32();
    Turn_Beep();
  }
}

// 减少 pemh
void button_dec_pemh_callback(const String &state) {
  if (state == "tap") {
    pemh -= 0.1f;
    if (pemh < 0.1f) pemh = 0.1f;
    button_dec_pemh.print("on");
    pemh_display.print(pemh);
    send_to_stm32();
    Turn_Beep();
  }
}

// 增加 peml
void button_inc_peml_callback(const String &state) {
  if (state == "tap") {
    peml += 0.1f;
    if (peml > 9.0f) peml = 9.0f;
    button_inc_peml.print("on");
    peml_display.print(peml);
    send_to_stm32();
    Turn_Beep();
  }
}

// 减少 peml
void button_dec_peml_callback(const String &state) {
  if (state == "tap") {
    peml -= 0.1f;
    if (peml < 0.0f) peml = 0.0f;
    button_dec_peml.print("on");
    peml_display.print(peml);
    send_to_stm32();
    Turn_Beep();
  }
}


//控制stm32充气泵开关
bool airpump_state = false;  // 设备的开关状态，默认关闭

// 当手机端点击开关时，调用此回调函数
void airpump_callback(const String &state) {

  airpump_state = !airpump_state;  // 每次按下切换状态

  if (state == "press" || state == "pressup") {  //长按app充气按
    airpump_state = true;                        // 开启设备
    airpump.print("on");
    send_to_stm32();
    Turn_Beep();
  } else {
    airpump_state = false;  // 关闭设备
    airpump.print("off");
    send_to_stm32();
    Turn_Beep();
  }
}

void launch_callback(const String &state) {
  if (state == "tap") {
    launch.print("on");
    ledcWrite(channel1, 102);  //开启+90° 2500us / 20000us * 4096
    Turn_Beep();
    delay(500);
    ledcWrite(channel1, 307);  //关闭0° 1500us / 20000us * 4096
    Turn_Beep();
  }

}

void vent_callback(const String &state) {
  if (state == "tap") {
    vent.print("on");
    ledcWrite(channel2, 512);  //打开-90°
    Turn_Beep();
    delay(500);
    ledcWrite(channel2, 307);  //关闭0° 1500us / 20000us * 4096
    Turn_Beep();
  }

}


// 接收手机数值控件直接设置数值（非按钮）
void data_read(const String &data) {
  if (data.startsWith("pemh")) {
    pemh = data.substring(4).toFloat();
    pemh_display.print(pemh);
  }
  if (data.startsWith("peml")) {
    peml = data.substring(4).toFloat();
    peml_display.print(peml);
  }
  airpump_display.print(airpump_state);
}
// 心跳包回调函数
void heartbeat() {

  Blinker.vibrate();         // 触发手机震动反馈
  p_xs_display.print(p_xs);  // 显示实时气压
}

void slider1_callback(int32_t value) {
  pemh = value;
  send_to_stm32();
  Number1.print(pemh);
  Turn_Beep();
}