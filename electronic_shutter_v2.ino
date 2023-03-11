#include "Adafruit_NeoPixel.h"
#include "Adafruit_FreeTouch.h"
#include "HID-Project.h"  // https://github.com/NicoHood/HID

#include "RTCCounter.h"

// Create the neopixel strip with the built in definitions NUM_NEOPIXEL and PIN_NEOPIXEL
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

int16_t neo_brightness = 255; // initialize with max so no PWM on the LEDs

uint8_t started = 0;
uint8_t mode = 0;
uint16_t mode_index = 0;

uint8_t led_current = 0;
uint8_t led_last = 0;
uint16_t led_change = 0;
uint8_t led_click = 0;

#define FRAMES  (10)

//#define DELAY_48MHZ   3840             // 80us
//#define DELAY_48MHZ   4320             // 90us
#define DELAY_48MHZ   4800             // 100us

static const uint32_t modeColors[][FRAMES] = {
  { 0x00FF0000, 0x00000000, 0x0000FF00, 0x00000000, 0x000000FF, 0x00000000, 0x00FFFF00, 0x00000000, 0x00FFFFFF, 0x00000000 },  // RoGoBoYoWo mode0
  { 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },  // WWWWWooooo mode1
  { 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000 },  // WoWoWoWoWo mode2
  { 0x00FF0000, 0x00FF0000, 0x0000FF00, 0x0000FF00, 0x000000FF, 0x000000FF, 0x00FF0000, 0x0000FF00, 0x000000FF, 0x0000FF00 },  // RRGGBBRGBG mode3
  { 0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF, 0x00000000, 0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF, 0x00000000 },  // RGBWoRGBWo mode4
  { 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000 },  // RoRoRoRoRo mode5
  { 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000 },  // GoGoGoGoGo mode6
  { 0X000000FF, 0X00000000, 0X000000FF, 0X00000000, 0X000000FF, 0X00000000, 0X000000FF, 0X00000000, 0X000000FF, 0X00000000 },  // BoBoBoBoBo mode7
  { 0X00FF0000, 0X00FF0000, 0X00FF0000, 0X00FF0000, 0X00FF0000, 0X00000000, 0X00000000, 0X00000000, 0X00000000, 0X00000000 },  // RRRRRooooo mode8
  { 0X0000FF00, 0X0000FF00, 0X0000FF00, 0X0000FF00, 0X0000FF00, 0X00000000, 0X00000000, 0X00000000, 0X00000000, 0X00000000 },  // GGGGGooooo mode9
  { 0X000000FF, 0X000000FF, 0X000000FF, 0X000000FF, 0X000000FF, 0X00000000, 0X00000000, 0X00000000, 0X00000000, 0X00000000 },  // BBBBBooooo mode10
};

#define MODES (sizeof(modeColors)/(sizeof(uint32_t)*FRAMES))

#undef USE_SERIAL

void setup() 
{
#ifdef USE_SERIAL
  Serial.begin(9600);
#endif

  strip.begin();
  strip.setBrightness(neo_brightness);
  strip.setPixelColor( 0, modeColors[0][0] );
  strip.show(); 

#ifdef USE_SERIAL
  Serial.println("Starting");
#endif

  //pinMode(PIN_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_SWITCH, OUTPUT );
  digitalWrite(PIN_SWITCH, 1);

  GCLK->CLKCTRL.reg = 0x401B; // Enable TC3 clock domain with same clock as CPU

  TC3->COUNT16.CTRLA.bit.MODE = 0;
  TC3->COUNT16.CTRLBCLR.bit.DIR = 1; // Count down
  TC3->COUNT16.CTRLBCLR.bit.ONESHOT = 1; // Disable oneshot
  TC3->COUNT16.CTRLA.bit.WAVEGEN = 0;
  
  TC3->COUNT16.COUNT.reg = 0xFFFFUL - DELAY_48MHZ + 1;
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;  

  // Sends a clean report to the host. This is important on any Arduino type.
  BootKeyboard.begin();
}

void ledBlink()
{
  uint32_t microsecs;

#ifdef USE_SERIAL
  Serial.print("mode=");Serial.println(mode);
#endif

  // Clear interrupt, reset counter to overflow
  TC3->COUNT16.INTFLAG.bit.OVF = 1;
  TC3->COUNT16.COUNT.reg = 0xFFFFUL - DELAY_48MHZ + 1;

  // Off we go
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;  

  while(1) {

    strip.setPixelColor( 0, modeColors[mode][mode_index] );

#ifdef USE_SERIAL
    Serial.print("mode_index=");Serial.println(mode_index);
    Serial.print("mode_color=");Serial.println(modeColors[mode][mode_index],16);
#endif    

    mode_index = (mode_index + 1) % FRAMES;
    digitalWrite(PIN_SWITCH, mode_index % 2);
    strip.show();

    // wait for the frame to complete
    while (!TC3->COUNT16.INTFLAG.bit.OVF);

    // reset frame
    TC3->COUNT16.COUNT.reg = 0xFFFFUL - DELAY_48MHZ + 1;
    TC3->COUNT16.INTFLAG.bit.OVF = 1;
  }
}

void usbKeyboardConfig()
{
  // mimic an led
  led_current = (BootKeyboard.getLeds() & 0x2);

#ifdef USE_SERIAL
  Serial.print("led_change=");Serial.println(led_change);
  Serial.print("led_current=");Serial.println(led_current);
  Serial.print("mode=");Serial.println(mode);
#endif

  if (led_current != led_last) {
    if (!led_current && (led_change < 40)) 
      led_click = 1;
    led_change = 0;
  }
  else if (led_change < 100)  {
    led_change++;
  }
  else {
    started = 1;
  }

  led_last = led_current;

  if (led_click)
  {
    mode = (mode + 1) % MODES;
    led_click = 0;
  }

  strip.setPixelColor( 0, modeColors[mode][mode_index] );
  strip.show();
  mode_index = (mode_index + 1) % FRAMES;
}

void loop() 
{
  int toggle = 0;

  if (started == 0) {
    usbKeyboardConfig();
  } else {
    ledBlink();
  }

#if 0
  uint32_t counter = 0;

#if 0  
  Serial.println("Starting");
  Serial.print("CPUSEL=");Serial.println(PM->CPUSEL.reg,16);
  Serial.print("APBAMASK=");Serial.println(PM->APBAMASK.reg,16);
  Serial.print("APBBMASK=");Serial.println(PM->APBBMASK.reg,16);
  Serial.print("APBCSEL=");Serial.println(PM->APBCSEL.reg,16);
  Serial.print("APBCMASK=");Serial.println(PM->APBCMASK.reg,16);
  Serial.print("TC3.COUNT16.CTRLA=");Serial.println(TC3->COUNT16.CTRLA.reg,16);
  Serial.print("TC3.COUNT16.INTENCLR=");Serial.println(TC3->COUNT16.INTENCLR.reg,16);
  Serial.print("TC3.COUNT16.EVCTRL=");Serial.println(TC3->COUNT16.EVCTRL.reg,16);
  Serial.print("TC3.COUNT16.INTFLAG=");Serial.println(TC3->COUNT16.INTFLAG.reg,16);
  Serial.print("TC3.COUNT16.STATUS=");Serial.println(TC3->COUNT16.STATUS.reg,16);
  Serial.print("TC3.COUNT16.COUNT=");Serial.println(TC3->COUNT16.COUNT.reg,16);

  for (int t = 0; t < 0x25; t++)
  {
    *(uint8_t *)0x40000C02UL = t;
    //   GCLK->CLKCTRL.=t;
    Serial.print("GCLK");Serial.print(t,16);Serial.print("=");Serial.println(GCLK->CLKCTRL.reg,16);
  }

  Serial.print("TC3.COUNT16.COUNT=");Serial.println(TC3->COUNT16.COUNT.reg,16);
#endif

  if (TC3->COUNT16.CTRLA.bit.ENABLE != 1) {
    TC3->COUNT16.CTRLA.bit.MODE = 0;
    TC3->COUNT16.CTRLBCLR.bit.DIR = 1; // Count down
    TC3->COUNT16.CTRLBCLR.bit.ONESHOT = 1; // Disable oneshot
    TC3->COUNT16.CTRLA.bit.WAVEGEN = 0;
    TC3->COUNT16.COUNT.reg = 0xFFFFUL - 4800;
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;  
  }
  
//  TC3->COUNT16.COUNT.reg = 4800;
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;  

  while (1) {
    counter = 0;
    while (!TC3->COUNT16.INTFLAG.bit.OVF) {
        counter++;
    }
    TC3->COUNT16.COUNT.reg = 0xFFFFUL - 4800;
    TC3->COUNT16.INTFLAG.bit.OVF = 1;

    digitalWrite(PIN_SWITCH, toggle);
    toggle = 1 - toggle;

   // Serial.print("Clearing ");Serial.println(counter);
  }
#endif

  delay(100);
}
