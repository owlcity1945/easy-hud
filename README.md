# 战机HUD
## 简介
这是一个由esp32-s驱动lcd屏幕显示战斗机俯仰梯姿态并通过准直透镜投射到55分光镜的小项目，只是简单模拟一下。
想要实现虚像在无限远处可换成焦距更短的菲涅尔透镜，便宜又轻量化装在无人机上简直合适不过了。这个版本是全功能板蓝牙通信板可连接手机控制舵机，串口实现和stm32交换数据（可忽略）
## 所需硬件
esp32-s开发板-----------or esp32-s3双核板代码我也写出来了，那个版本采用双核心写的，代码更简洁一些。

姿态传感器bno055------- 虽稍贵但可直取里面的驱动函数，性能稳定，重要的是芯片里有地磁角，温度等功能。

1.3英寸tft显示屏--------驱动类型是st7789,分辨率240*240。

360度编码器开关 --------方便调节hud的颜色，单击可开启自动追踪，不依赖传感器。

蜂鸣器 -----------------用三极管驱动（动手能力强可自行添加）

400孔面包板 ------------方便搭建各种硬件

杜邦线 -----------------公母头都各买一些

3d打印hud壳子-----------我已绘制好放在文件里了格式是stl。

双凸透镜----------------或者菲涅尔透镜 直径40mm,焦距28

五五半透分光棱镜 --------尺寸我会放在图库里

## 硬件连接
//TFT引脚

//#define TFT_MOSI 13

//#define TFT_SCLK 14

//#define TFT_DC    4  // Data Command control pin

//#define TFT_RST   5  // Reset pin (could connect to RST pin)

//编码器

// int aState = digitalRead(32);  // A 相

// int bState = digitalRead(33);  // B 相

// const uint8_t PIN_INPUT = 35;  // C 相

// BHO055姿态传感器 

// I2C 引脚：SDA = GPIO18, SCL = GPIO19

//esp32通信设置串口

// 16为RX，17为TX

//蜂鸣器引脚

// BEEP_PIN 2

arduino ide里需要本地配置的文件路径C:\Users\你的电脑\Documents\Arduino\libraries\TFT_eSPI，打开User_Setup.H这个头文件确保以下代码是非注释项：

##
#define USER_SETUP_INFO "User_Setup"

#define ST7789_DRIVER      // Full configuration option, define additional parameters below for this display

#define TFT_WIDTH  240 // ST7789 240 x 240 and 240 x 320

#define TFT_HEIGHT 240 // ST7789 240 x 240

#define TFT_MOSI 11 // In some display driver board, it might be written as "SDA" and so on.SDA:13

#define TFT_SCLK 12 //SCLK:14

#define TFT_DC 13  // Data Command control pin DC:4

#define TFT_RST 14  // Reset pin (could connect to Arduino RESET pin)RES:5

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH

#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters

#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters

#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm

#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.

#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.

#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

 #define SPI_FREQUENCY  40000000//如果是esp32-s3双核版本可选80000000

// Optional reduced SPI frequency for reading TFT

#define SPI_READ_FREQUENCY  20000000

// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:

#define SPI_TOUCH_FREQUENCY  2500000
