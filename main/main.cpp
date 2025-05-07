#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#ifdef ARDUINO_ARCH_SAMD
  #include <Adafruit_ZeroDMA.h>
#endif
#ifdef ARDUINO_ARCH_ESP32
  #include <Arduino.h>
  // 为ESP32定义TFT_SPI别名，以便统一代码
  #define TFT_SPI SPI
#endif

// 定义eyeInfo_t结构体
typedef struct {
  int8_t  select;       // 每个眼睛的屏幕选择引脚
  int8_t  wink;         // 眨眼按钮引脚 (如果没有则为 -1)
  uint8_t rotation;     // 显示旋转值 (0-3)
} eyeInfo_t;

#include "config.h"     // ****** CONFIGURATION IS DONE IN HERE ******

extern void user_setup(void); // Functions in the user*.cpp files
extern void user_loop(void);

// 引入原始Arduino程序
extern void setup(void);
extern void loop(void);

extern "C" void app_main(void) {
  // 初始化Arduino库
  initArduino();

  // 调用原始Arduino程序的setup函数
  setup();

  // ESP-IDF需要无限循环来持续运行
  while (true) {
    // 调用原始Arduino程序的loop函数
    loop();
    
    // 添加一个短暂的延迟，防止看门狗触发
    vTaskDelay(1);
  }
} 