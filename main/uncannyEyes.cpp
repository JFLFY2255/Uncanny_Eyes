#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#ifdef ARDUINO_ARCH_SAMD
  #include <Adafruit_ZeroDMA.h>
#endif
#ifdef ARDUINO_ARCH_ESP32
  #include <Arduino.h>
  // 为ESP32定义TFT_SPI别名，以便统一代码
  #define TFT_SPI SPI
  // 定义LED_BUILTIN（ESP32上可能没有定义）
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 2 // ESP32通常使用GPIO2作为内置LED
  #endif
#endif

// 与main.cpp中保持一致的结构体定义
typedef struct {
  int8_t  select;       // 每个眼睛的屏幕选择引脚
  int8_t  wink;         // 眨眼按钮引脚 (如果没有则为 -1)
  uint8_t rotation;     // 显示旋转值 (0-3)
} eyeInfo_t;

// GRAPHICS SETTINGS FOR EYES
#define DEFINE_EYEINFO  // 定义在这个文件中实际声明eyeInfo数组
#include "config.h"     // ****** CONFIGURATION IS DONE IN HERE ******

extern void user_setup(void); // Functions in the user*.cpp files
extern void user_loop(void);

// 函数声明
void setup(void);
void loop(void);
void drawEye(uint8_t e, uint16_t iScale, uint8_t scleraX, uint8_t scleraY, uint8_t uT, uint8_t lT);

#ifdef PIXEL_DOUBLE
  // For the 240x240 TFT, pixels are rendered in 2x2 blocks for an
  // effective resolution of 120x120. M0 boards just don't have the
  // space or speed to handle an eye at the full resolution of this
  // display (and for M4 boards, take a look at the M4_Eyes project
  // instead). 120x120 doesn't quite match the resolution of the
  // TFT & OLED this project was originally developed for. Rather
  // than make an entirely new alternate set of graphics for every
  // eye (would be a huge undertaking), this currently just crops
  // four pixels all around the perimeter.
  #define SCREEN_X_START 4
  #define SCREEN_X_END   (SCREEN_WIDTH - 4)
  #define SCREEN_Y_START 4
  #define SCREEN_Y_END   (SCREEN_HEIGHT - 4)
#else
  #define SCREEN_X_START 0
  #define SCREEN_X_END   SCREEN_WIDTH
  #define SCREEN_Y_START 0
  #define SCREEN_Y_END   SCREEN_HEIGHT
#endif

#if defined(_ADAFRUIT_ST7789H_)
  typedef Adafruit_ST7789  displayType; // Using 240x240 TFT display(s)
#elif defined(_ADAFRUIT_ST7735H_) || defined(_ADAFRUIT_ST77XXH_)
  typedef Adafruit_ST7735  displayType; // Using TFT display(s)
#else
  typedef Adafruit_SSD1351 displayType; // Using OLED display(s)
#endif

// A simple state machine is used to control eye blinks/winks:
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing
#define DEBLINK 2       // Eyelid is currently opening
typedef struct {
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;    // Duration of blink state (micros)
  uint32_t startTime;   // Time (micros) of last state change
} eyeBlink;

#define NUM_EYES (sizeof eyeInfo / sizeof eyeInfo[0]) // config.h pin list

struct {                // One-per-eye structure
  displayType *display; // -> OLED/TFT object
  eyeBlink     blink;   // Current blink/wink state
} eye[NUM_EYES];

#ifdef ARDUINO_ARCH_SAMD
  // SAMD boards use DMA (Teensy uses SPI FIFO instead):
  // Two single-scanline pixel buffers are used for DMA,
  // alternating rendering and transferring between them.
  // Though you'd think fewer larger transfers would improve speed,
  // multi-line buffering made no appreciable difference.
  uint8_t           dmaIdx = 0; // Active DMA buffer # (alternate fill/send)
  Adafruit_ZeroDMA  dma;
 #ifdef PIXEL_DOUBLE
  uint32_t          dmaBuf[2][120]; // Two 120-pixel buffers (32bit for doubling)
  DmacDescriptor   *descriptor[2];  // Pair of descriptors for doubled scanlines
 #else
  uint16_t          dmaBuf[2][ST77XX_SCREEN_WIDTH]; // 使用屏幕宽度定义缓冲区大小
  DmacDescriptor   *descriptor;     // Single active DMA descriptor
 #endif
  // DMA transfer-in-progress indicator and callback
  static volatile bool dma_busy = false;
  static void dma_callback(Adafruit_ZeroDMA *dma) { dma_busy = false; }
#elif defined(ARDUINO_ARCH_NRF52)
  uint8_t           dmaIdx = 0; // Active DMA buffer # (alternate fill/send)
 #ifdef PIXEL_DOUBLE
  uint32_t          dmaBuf[2][240]; // Two 240-pixel buffers (32bit for doubling)
 #else
  uint16_t          dmaBuf[2][ST77XX_SCREEN_WIDTH]; // 使用屏幕宽度定义缓冲区大小
 #endif
#elif defined(ARDUINO_ARCH_ESP32)
  uint8_t           dmaIdx = 0; // Active DMA buffer # (alternate fill/send)
 #ifdef PIXEL_DOUBLE
  uint32_t          dmaBuf[2][240]; // Two 240-pixel buffers (32bit for doubling)
 #else
  uint16_t          dmaBuf[2][ST77XX_SCREEN_WIDTH]; // 使用屏幕宽度定义缓冲区大小
 #endif
#endif

uint32_t startTime;  // For FPS indicator

#if defined(SYNCPIN) && (SYNCPIN >= 0)
#include <Wire.h>
// If two boards are synchronized over I2C, this struct is passed from one
// to other. No device-independent packing & unpacking is performed...both
// boards are expected to be the same architecture & endianism.
struct {
  uint16_t iScale;  // These are basically the same arguments as
  uint8_t  scleraX; // drawEye() expects, explained in that function.
  uint8_t  scleraY;
  uint8_t  uT;
  uint8_t  lT;
} syncStruct = { 512,
  (SCLERA_WIDTH-SCREEN_WIDTH)/2, (SCLERA_HEIGHT-SCREEN_HEIGHT)/2, 0, 0 };

void wireCallback(int n) {
  if(n == sizeof syncStruct) {
    // Read 'n' bytes from I2C into syncStruct
    uint8_t *ptr = (uint8_t *)&syncStruct;
    for(uint8_t i=0; i < sizeof syncStruct; i++) {
      ptr[i] = Wire.read();
    }
  }
}

bool receiver = false;
#endif // SYNCPIN

// INITIALIZATION -- runs once at startup ----------------------------------

void setup(void) {
  uint8_t e; // Eye index, 0 to NUM_EYES-1

#if defined(SYNCPIN) && (SYNCPIN >= 0) // If using I2C sync...
  pinMode(SYNCPIN, INPUT_PULLUP);      // Check for jumper to ground
  if(!digitalRead(SYNCPIN)) {          // If there...
    receiver = true;                   // Set this one up as receiver
    Wire.begin(SYNCADDR);
    Wire.onReceive(wireCallback);
  } else {
    Wire.begin();                      // Else set up as sender
  }
#endif

  Serial.begin(115200);
  //while (!Serial);
  Serial.println("Init");
  Serial.print("Screen size: ");
  Serial.print(ST77XX_SCREEN_WIDTH);
  Serial.print("x");
  Serial.println(ST77XX_SCREEN_HEIGHT);
  
#ifdef ARDUINO_ARCH_ESP32
  randomSeed(analogRead(36)); // Seed random() from floating analog input on ESP32
#else
  randomSeed(analogRead(A3)); // Seed random() from floating analog input
#endif

#ifdef DISPLAY_BACKLIGHT
  // Enable backlight pin, initially off
  Serial.println("Backlight off");
  pinMode(DISPLAY_BACKLIGHT, OUTPUT);
  digitalWrite(DISPLAY_BACKLIGHT, LOW);
#endif

#ifdef ARDUINO_ARCH_ESP32
  // Initialize SPI for ESP32
  TFT_SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
#endif

  user_setup();

  // Initialize eye objects based on eyeInfo list in config.h:
  for(e=0; e<NUM_EYES; e++) {
    Serial.print("Create display #"); Serial.println(e);
#if defined(_ADAFRUIT_ST7789H_) // 240x240 TFT
    eye[e].display     = new displayType(&TFT_SPI, eyeInfo[e].select,
                           DISPLAY_DC, -1);
#elif defined(_ADAFRUIT_ST7735H_) || defined(_ADAFRUIT_ST77XXH_) // 128x128 TFT
    eye[e].display     = new displayType(&TFT_SPI, eyeInfo[e].select,
                           DISPLAY_DC, -1);
#else // OLED
    eye[e].display     = new displayType(128, 128, &TFT_SPI,
                           eyeInfo[e].select, DISPLAY_DC, -1);
#endif
    eye[e].blink.state = NOBLINK;
    // If project involves only ONE eye and NO other SPI devices, its
    // select line can be permanently tied to GND and corresponding pin
    // in config.h set to -1.  Best to use it though.
    if(eyeInfo[e].select >= 0) {
      pinMode(eyeInfo[e].select, OUTPUT);
      digitalWrite(eyeInfo[e].select, HIGH); // Deselect them all
    }
    // Also set up an individual eye-wink pin if defined:
    if(eyeInfo[e].wink >= 0) pinMode(eyeInfo[e].wink, INPUT_PULLUP);
  }
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
  pinMode(BLINK_PIN, INPUT_PULLUP); // Ditto for all-eyes blink pin
#endif

#if defined(DISPLAY_RESET) && (DISPLAY_RESET >= 0)
  // Because both displays share a common reset pin, -1 is passed to
  // the display constructor above to prevent the begin() function from
  // resetting both displays after one is initialized.  Instead, handle
  // the reset manually here to take care of both displays just once:
  Serial.println("Reset displays");
  pinMode(DISPLAY_RESET, OUTPUT);
  digitalWrite(DISPLAY_RESET, LOW);  delay(1);
  digitalWrite(DISPLAY_RESET, HIGH); delay(50);
  // Alternately, all display reset pin(s) could be connected to the
  // microcontroller reset, in which case DISPLAY_RESET should be set
  // to -1 or left undefined in config.h.
#else
  // If no DISPLAY_RESET pin is defined, the board might have an auto-
  // reset circuit for the display (e.g. TFT Gizmo). Pause for just a
  // tiny moment to allow it to work, otherwise commands will be issued
  // too soon and the display won't correctly initialize.
  delay(75);
#endif

  // After all-displays reset, now call init/begin func for each display:
  for(e=0; e<NUM_EYES; e++) {
#if defined(_ADAFRUIT_ST7789H_) // 240x240 TFT
    eye[e].display->init(240, 240);
    
#elif defined(_ADAFRUIT_ST7735H_) || defined(_ADAFRUIT_ST77XXH_)
    Serial.print("Init ST77xx display #"); Serial.println(e);
    
    #ifdef ARDUINO_ARCH_ESP32
    // ESP32需要额外的显示初始化步骤
    if(DISPLAY_RESET >= 0) {
      pinMode(DISPLAY_RESET, OUTPUT);
      digitalWrite(DISPLAY_RESET, HIGH);
      delay(50);
      digitalWrite(DISPLAY_RESET, LOW);
      delay(50);
      digitalWrite(DISPLAY_RESET, HIGH);
      delay(50);
    }
    #endif
    
    eye[e].display->initR(INITR_18BLACKTAB);
    // eye[e].display->initR(INITR_144GREENTAB);
    // eye[e].display->initR(INITR_MINI160x80);
    
    #ifdef ARDUINO_ARCH_ESP32
    // ESP32需要额外延迟
    delay(100); 
    #endif
#else // OLED
    eye[e].display->begin(SPI_FREQ);
#endif
    Serial.println("Rotate");
    eye[e].display->setRotation(eyeInfo[e].rotation);
  }
  Serial.println("done");

#if defined(DISPLAY_BACKLIGHT) // If LED connection for backlight is defined...
  Serial.println("Backlight on");
  analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_MAX);
#endif

#ifdef ARDUINO_ARCH_SAMD
  // Set up SPI DMA on SAMD boards:
 #ifdef PIXEL_DOUBLE
  int bytesPerPixel  = 4;           // 32-bit pixels
  int bytesPerBuffer = bytesPerPixel * 120; // 2 scanlines, each 120 pixels
  descriptor[0] = dma.addDescriptor(
    dmaBuf[0],                      // Source address
    (void *)(&SPI->DATA.reg),       // Destination address (SPI peripheral register)
    bytesPerBuffer,                 // Bytes to transfer
    DMA_BEAT_SIZE_BYTE,             // Beat size
    true,                           // Source address increment
    false);                         // Destination address increment
  descriptor[1] = dma.addDescriptor(
    dmaBuf[1],                      // Source address
    (void *)(&SPI->DATA.reg),       // Destination address (SPI peripheral register)
    bytesPerBuffer,                 // Bytes to transfer
    DMA_BEAT_SIZE_BYTE,             // Beat size
    true,                           // Source address increment
    false);                         // Destination address increment
  dma.setCallback(dma_callback);    // Set up transfer-done callback
 #else // Not PIXEL_DOUBLE
  int bytesPerPixel = 2;            // 16-bit pixels
  int bytesPerBuffer = bytesPerPixel * ST77XX_SCREEN_WIDTH; // 1 scanline
  descriptor = dma.addDescriptor(
    dmaBuf[0],                      // Source address
    (void *)(&SPI->DATA.reg),       // Destination address (SPI peripheral register)
    bytesPerBuffer,                 // Bytes to transfer
    DMA_BEAT_SIZE_BYTE,             // Beat size
    true,                           // Source address increment
    false);                         // Destination address increment
  dma.setCallback(dma_callback);    // Set up transfer-done callback
 #endif
  dma.allocate();
#endif // End SAMD-specific SPI DMA init

#ifdef ARDUINO_ARCH_ESP32
  // 设置ESP32数据缓冲区
  #ifdef PIXEL_DOUBLE
  int bytesPerPixel = 4;            // 32-bit pixels
  #else
  int bytesPerPixel = 2;            // 16-bit pixels
  #endif
#endif

  startTime = millis(); // For frame-rate calculation
}

// MAIN LOOP -- runs continuously after setup() ----------------------------

void loop() {
  static uint16_t       // Range is 0-1023:
    irisZ   = map(IRIS_MIN + IRIS_MAX, 0, 1023, IRIS_MIN, IRIS_MAX),
    eyeX    = 512, eyeY = 512, // 512,512 is center
    eyeOldX = 512, eyeOldY = 512,
    mouseX  = 512, mouseY = 512,
    frame   = 0;
  static int8_t
    eyeCoeff = 15;     // Filter coefficient (smaller = slower motion)
  static int32_t
    eyeXfilt = (int32_t)eyeX << 16,
    eyeYfilt = (int32_t)eyeY << 16;
  static uint8_t
    eyeIndex         = 0, // Eye index for animation
    blink[NUM_EYES]    = { 0 },  // 用0初始化，与数组大小匹配
    blinkCountdown     = 100,
    gazeCountdown      = 75,
    moveCoeff          = 10,
    joystickLastX = 0, joystickLastY = 0;
  uint8_t
    i;
  uint32_t
    t = millis(); // Time at start of function

  // Read and process input first

  // Position of pupil may be controlled by user code with joystick,
  // or may be automated with "puppet" or auto-glance modes...
#if defined(JOYSTICK_X_PIN) && (JOYSTICK_X_PIN >= 0) && \
    defined(JOYSTICK_Y_PIN) && (JOYSTICK_Y_PIN >= 0)
  // Read X/Y from 2-axis joystick
  uint8_t
    x = map(analogRead(JOYSTICK_X_PIN), 0, 1023, 0, 255),
    y = map(analogRead(JOYSTICK_Y_PIN), 0, 1023, 0, 255);
  if(joystickLastX != x || joystickLastY != y) {
    // Only if position changed since last frame -- otherwise
    // same random movement is used (no need to re-seed random() here)
    joystickLastX = x;
    joystickLastY = y;
    // Scale raw 0-255 value to -512 (left/up) to +512 (right/down)
    mouseX = map(x, 0, 255, -512, 512);
    mouseY = map(y, 0, 255, -512, 512);
#ifdef JOYSTICK_X_FLIP
    mouseX = -mouseX;
#endif
#ifdef JOYSTICK_Y_FLIP
    mouseY = -mouseY;
#endif
    eyeX = 512 + mouseX;
    eyeY = 512 + mouseY;
  }
#else // Autonomous X/Y eye motion
  // Periodically initiates motion to a new random point, random speed
  if(--gazeCountdown <= 0) { // Current gaze position attrition
    // New random point determined by random() function
    // We're adding 1/4 +/- 1/8 of full iris motion range
    #ifdef IRIS_SMOOTH
      // With IRIS_SMOOTH enabled, eye movements are slower
      // random(0, 513) is 0 to 512 inclusive (small move)
      // random(0, 1025) is 0 to 1024 inclusive (large move)
      // Thanks to David for pointing out a previous mathsy mistake
      eyeX = eyeXfilt >> 16;
      eyeY = eyeYfilt >> 16;
      if(random(3) < 2) { // 2/3 of time: smaller, shorter movements
        eyeX += random(-128, 129); // +/- 128
        eyeY += random(-128, 129);
        gazeCountdown = random(2, 15);
      } else {  // 1/3 of time: longer, wider movements
        eyeX += random(-384, 385); // +/- 384
        eyeY += random(-384, 385);
        gazeCountdown = random(15, 100);
      }
    #else
      // With IRIS_SMOOTH disabled, eye movements are instant
      // Old format behavior: both random(a,b) and random(a, b)
      // produce results up to but NOT including upper limit
      // random(0, 513) is 0 to 512 inclusive (small move)
      // random(0, 1025) is 0 to 1024 inclusive (large move)
      // That's weird and had me confused for a bit.
      if(random(3) < 2) { // 2/3 of time: smaller, shorter movements
        eyeX = 512 + random(-128, 128); // +/- 128
        eyeY = 512 + random(-128, 128);
        gazeCountdown = random(3, 12);
      } else {  // 1/3 of time: longer, wider movements
        eyeX = 512 + random(-512, 512); // +/- 512
        eyeY = 512 + random(-256, 256);
        gazeCountdown = random(30, 60);
      }
    #endif
    if(eyeX < 0)            eyeX = 0;
    else if(eyeX > 1023)    eyeX = 1023;
    if(eyeY < 0)            eyeY = 0;
    else if(eyeY > 1023)    eyeY = 1023;
    moveCoeff = random(5, 20);     // Movement speed/filter
  }
#endif // JOYSTICK_X/Y_PIN

#if defined(LIGHT_PIN) && (LIGHT_PIN >= 0)
 #ifdef LIGHT_PIN_FLIP
  int reading = 1023 - analogRead(LIGHT_PIN);
 #else
  int reading = analogRead(LIGHT_PIN);
 #endif
 #ifdef LIGHT_MIN
  if(reading < LIGHT_MIN)      reading = LIGHT_MIN; // Clamp light sensor range
 #endif
 #ifdef LIGHT_MAX
  if(reading > LIGHT_MAX)      reading = LIGHT_MAX;
 #endif
 #if defined(LIGHT_CURVE)
  // See notes about this math in the uncannyEyes.ino file.
  // The "curve" value (here 'LIGHT_CURVE' defined in config.h)
  // can be set to 0.0 to 1.0 (0.0 being no effect, 1.0 being
  // the strongest effect) -- it's an exponential scale, not linear.
  float v = (float)reading / (float)(LIGHT_MAX - LIGHT_MIN);
  // Scale reading by a 2nd-degree polynomial (triangular curve)
  v = LIGHT_CURVE - LIGHT_CURVE * (1.0 - v) * (1.0 - v);
  if(v < 0.0)      v = 0.0;
  else if(v > 1.0) v = 1.0;
  // And re-scale to iris range
  irisZ = (uint16_t)(v *
    (float)(IRIS_MAX - IRIS_MIN) + (float)IRIS_MIN + 0.5);
 #else
  // No curve, just linear scaling
  irisZ = map(reading, LIGHT_MIN, LIGHT_MAX, IRIS_MIN, IRIS_MAX);
 #endif
#else
 #ifdef IRIS_SMOOTH
  // Autonomous iris motion uses a slow filtered approach
  irisZ = (irisZ * 15 + (IRIS_MIN + IRIS_MAX - irisZ) +
    random(0, 4)) / 16;
 #else
  // Autonomous iris motion uses a more abrupt approach
  if(random(6) == 0) { // Random but more steady than perlin noise
    irisZ = random(IRIS_MIN, IRIS_MAX);
  }
 #endif
#endif // LIGHT_PIN

#ifdef AUTOBLINK
  // Similar to the autonomous eye movement above -- blink start
  // times and durations are random (within ranges).
  if((t - eye[0].blink.startTime) >= eye[0].blink.duration) {
    // All done with this blink?  Pick random time for next...
    uint32_t blinkTime;
    blinkTime = t + random(3000, 10000);  
    if(++eye[0].blink.state > DEBLINK) { // Deblinking finished?
      eye[0].blink.state = NOBLINK;      // No longer blinking
    } else { // Advancing from ENBLINK to DEBLINK mode
      blinkTime = t + random(50, 125);    // Blink for 50-125 ms
    }
    for(i=0; i<NUM_EYES; i++) {
      eye[i].blink.startTime = blinkTime;
      eye[i].blink.duration  = blinkTime - t;
    }
  }
#endif

  // Process blinks (reduce durations by half for second eye)
  for(i=0; i<NUM_EYES; i++) {
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
    // Check if blink button pressed (active low)?
    if(digitalRead(BLINK_PIN) == LOW) {
      // Not blinking; start a blink, independent of auto-blink
      if(eye[i].blink.state == NOBLINK) {
        eye[i].blink.state     = ENBLINK;
        eye[i].blink.startTime = t;
        eye[i].blink.duration  = random(50, 125);
      }
    }
#endif
    // Per-eye wink buttons do similarly, but there's no debounce
    // logic, so they'd typically be used with toggle switches
    uint8_t winkPin = eyeInfo[i].wink;
    if((winkPin >= 0) && (digitalRead(winkPin) == LOW)) {
      // Wink this eye now (will deblink on its own)
      eye[i].blink.state     = ENBLINK;
      eye[i].blink.startTime = t;
      eye[i].blink.duration  = random(50, 125);
    }
    switch(eye[i].blink.state) {
     case NOBLINK:
      break;
     case ENBLINK: // eye starting to blink...
      if(i > 0) { // set duration for right eye
        eye[i].blink.duration = eye[0].blink.duration * 2 / 3;
      }
      break;
     case DEBLINK: // eye starting to un-blink
      if(i > 0) { // set duration for right eye
        eye[i].blink.duration = eye[0].blink.duration * 2 / 3;
      }
      break;
    }
    if(eye[i].blink.state > NOBLINK) {
      // Blinking
      // Both blink state and blink duration depend on eye type
      // Might go through a state transition during this interval
      uint32_t elapsed = t - eye[i].blink.startTime;
      if(elapsed >= eye[i].blink.duration) {
        if(eye[i].blink.state == ENBLINK) {
          // Enblinking done, then deblinking...
          eye[i].blink.state     = DEBLINK;
          eye[i].blink.startTime = t;
          eye[i].blink.duration  = random(50, 125);
        } else {
          // Deblinking done, back to NOBLINK
          eye[i].blink.state = NOBLINK;
        }
      }
    }
  }

  // Calculate a filtered eye position
#ifdef IRIS_SMOOTH
  // Autonomous eye movement uses a slow filtered approach
  uint16_t newX, newY;
  uint32_t divider = (eyeCoeff + 1);

  // horizontal position
  if(eyeX < (eyeXfilt >> 16)) {
    newX = eyeXfilt - ((eyeXfilt - ((int32_t)eyeX << 16)) / divider);
  } else {
    newX = eyeXfilt + ((((int32_t)eyeX << 16) - eyeXfilt) / divider);
  }
  eyeXfilt = newX;
  eyeOldX = newX >> 16;

  // vertical position
  if(eyeY < (eyeYfilt >> 16)) {
    newY = eyeYfilt - ((eyeYfilt - ((int32_t)eyeY << 16)) / divider);
  } else {
    newY = eyeYfilt + ((((int32_t)eyeY << 16) - eyeYfilt) / divider);
  }
  eyeYfilt = newY;
  eyeOldY = newY >> 16;

  eyeCoeff = moveCoeff;
#else
  eyeOldX = eyeX; // New pos = old pos
  eyeOldY = eyeY;
#endif

  // Update blink/wink animation
  uint32_t blinkTimer = t;
  uint32_t timeElapsed = 0;
  float    timeElapsedDivisor = 1.0;
  // Calculate upper/lower lid values
  uint8_t upperLidWeight = 0;        // Closed
  uint8_t lowerLidWeight = 0;        // Open

  for(i=0; i<NUM_EYES; i++) {
    // Get elapsed time from blink/wink start
    if(eye[i].blink.state != NOBLINK) {
      timeElapsed  = blinkTimer - eye[i].blink.startTime;
      if(timeElapsed > eye[i].blink.duration) {
        timeElapsed = eye[i].blink.duration;
      }
      // Calculate how far open the eye is
      timeElapsedDivisor = (float)timeElapsed /
                      (float)eye[i].blink.duration;
    }
    // Eyelid positions
    // Eye is in one of THREE states: actively opening/closing,
    // or fully open.
    static uint8_t  ul, ll;
    if(eye[i].blink.state == NOBLINK) {
      // Eye is fully open
      upperLidWeight = 0;        // Closed
      lowerLidWeight = 0;        // Open
    } else if(eye[i].blink.state == ENBLINK) {
      // Eye is being closed
      upperLidWeight = (float)255.0 * timeElapsedDivisor;
      lowerLidWeight = upperLidWeight;
    } else {  // DEBLINK
      // Eye is being opened
      upperLidWeight = (float)255.0 * (1.0 - timeElapsedDivisor);
      lowerLidWeight = upperLidWeight;
    }

    // Now draw the eye at the new position
    // Eyelid opening/closing is done by varying curves on upper and lower eyelids.
    // (Curvature is calculated on-the-fly but both are from lookup tables. When
    // lids are "open" the upper/lower curvature tables are used as-is, but
    // when closing or opening, one table is initially overridden by a flatter
    // average value, then a weighted average of the two is used for in-between.)
    // We defer the bulk of this to the drawEye() function.
#ifdef TRACKING
    user_loop();
    drawEye(i, irisZ, 
      map(eyeOldX, 0, 1023, SCREEN_X_START, SCREEN_X_END),
      map(eyeOldY, 0, 1023, SCREEN_Y_START, SCREEN_Y_END),
      upperLidWeight, lowerLidWeight);
#else
    drawEye(i, irisZ, 
      map(eyeOldX, 0, 1023, SCREEN_X_START, SCREEN_X_END),
      map(eyeOldY, 0, 1023, SCREEN_Y_START, SCREEN_Y_END),
      0, 0);
#endif
  }

  if(++frame >= 3) {
    // FPS indicator (once per 3 frames)
    uint16_t t2 = micros() - t;
    if(t2 >= 10000) { // At 300 FPS or slower, reduce flickering
      digitalWrite(LED_BUILTIN, HIGH);
      delayMicroseconds(t2 / 4);
      digitalWrite(LED_BUILTIN, LOW);
    }
    frame = 0;
    //Serial.print((1000000L + (millis() - startTime) / 2) / (millis() - startTime));
    //Serial.println(" fps");
  }
}

// EYE-RENDERING FUNCTION --------------------------------------------------

void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint16_t iScale,  // Scale factor for iris (0-1023)
  uint8_t  scleraX, // First pixel X offset into sclera image
  uint8_t  scleraY, // First pixel Y offset into sclera image
  uint8_t  uT,      // Upper eyelid threshold value
  uint8_t  lT) {    // Lower eyelid threshold value

#if defined(SYNCPIN) && (SYNCPIN >= 0)
  if(!e && !receiver) { // Eye 0 (left) on sender?
    // Stuff values into syncStruct and send to receiver
    syncStruct.iScale  = iScale;
    syncStruct.scleraX = scleraX;
    syncStruct.scleraY = scleraY;
    syncStruct.uT      = uT;
    syncStruct.lT      = lT;
    Wire.beginTransmission(SYNCADDR);
    Wire.write((char *)&syncStruct, sizeof syncStruct);
    Wire.endTransmission();
  }
#endif

  displayType *d = eye[e].display;
  
  // ESP32配置屏幕刷新区域 - 使用屏幕实际宽高
  #if defined(ARDUINO_ARCH_ESP32) && defined(_ADAFRUIT_ST7735H_)
  d->setAddrWindow(0, 0, ST77XX_SCREEN_HEIGHT, ST77XX_SCREEN_WIDTH);
  #else
  d->setAddrWindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
  #endif
  
  // 这里简化实现，直接绘制一个基本的眼睛
  // 在实际项目中，应该使用更复杂的渲染逻辑
  
  // 清除屏幕
  for(int y=0; y<SCREEN_HEIGHT; y++) {
    for(int x=0; x<SCREEN_WIDTH; x++) {
      d->drawPixel(x, y, 0xFFFF); // 白色背景
    }
  }
  
  // 绘制虹膜
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;
  int irisRadius = map(iScale, 0, 1023, SCREEN_WIDTH/6, SCREEN_WIDTH/3);
  
  for(int y=centerY-irisRadius; y<=centerY+irisRadius; y++) {
    for(int x=centerX-irisRadius; x<=centerX+irisRadius; x++) {
      if(((x-centerX)*(x-centerX) + (y-centerY)*(y-centerY)) <= irisRadius*irisRadius) {
        d->drawPixel(x, y, 0x07E0); // 绿色虹膜
      }
    }
  }
  
  // 绘制瞳孔
  int pupilRadius = irisRadius / 2;
  for(int y=centerY-pupilRadius; y<=centerY+pupilRadius; y++) {
    for(int x=centerX-pupilRadius; x<=centerX+pupilRadius; x++) {
      if(((x-centerX)*(x-centerX) + (y-centerY)*(y-centerY)) <= pupilRadius*pupilRadius) {
        d->drawPixel(x, y, 0x0000); // 黑色瞳孔
      }
    }
  }
  
  // 处理眼睑
  if(uT > 0 || lT > 0) {
    int upperLidPos = map(uT, 0, 255, 0, centerY);
    int lowerLidPos = map(lT, 0, 255, SCREEN_HEIGHT, centerY);
    
    // 上眼睑
    for(int y=0; y<upperLidPos; y++) {
      for(int x=0; x<SCREEN_WIDTH; x++) {
        d->drawPixel(x, y, 0xF800); // 红色上眼睑
      }
    }
    
    // 下眼睑
    for(int y=lowerLidPos; y<SCREEN_HEIGHT; y++) {
      for(int x=0; x<SCREEN_WIDTH; x++) {
        d->drawPixel(x, y, 0xF800); // 红色下眼睑
      }
    }
  }
} 