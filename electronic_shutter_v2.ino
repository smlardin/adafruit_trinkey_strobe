#include "Adafruit_NeoPixel.h"
#include "Adafruit_FreeTouch.h"
#include "HID-Project.h"  // https://github.com/NicoHood/HID

#include "RTCCounter.h"

// Create the neopixel strip with the built in definitions NUM_NEOPIXEL and PIN_NEOPIXEL
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

int16_t neo_brightness = 255; // initialize with 20 brightness (out of 255)

uint8_t started = 0;
uint8_t mode = 0;
uint16_t mode_index = 0;

uint8_t led_current = 0;
uint8_t led_last = 0;
uint16_t led_change = 0;
uint8_t led_click = 0;

uint8_t blink = 0;

#define FRAMES  (10)

uint32_t modeColors[][FRAMES] = {
  { 0x00FF0000, 0x00000000, 0x0000FF00, 0x00000000, 0x000000FF, 0x00000000, 0x00FFFF00, 0x00000000, 0x00FFFFFF, 0x00000000 },  // RoGoBoYoWo
  { 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },  // WWWWWooooo
  { 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FFFFFF, 0x00000000 },  // WoWoWoWoWo
  { 0x00FF0000, 0x00FF0000, 0x0000FF00, 0x0000FF00, 0x000000FF, 0x000000FF, 0x00FF0000, 0x0000FF00, 0x000000FF, 0x0000FF00 },  // RRGGBBRGBG
  { 0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF, 0x00000000, 0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF, 0x00000000 },  // RGBWoRGBWo
  { 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000, 0X00FF0000, 0X00000000 },  // RoRoRoRoRo
  { 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000, 0X0000FF00, 0X00000000 },  // GoGoGoGoGo
  { 0X000000FF, 0X00000000, 0X000000FF, 0X00000000, 0X000000FF, 0X00000000, 0X000000FF, 0X00000000, 0X000000FF, 0X00000000 },  // BoBoBoBoBo
  { 0X00FF0000, 0X00FF0000, 0X00FF0000, 0X00FF0000, 0X00FF0000, 0X00000000, 0X00000000, 0X00000000, 0X00000000, 0X00000000 },  // RRRRRooooo
  { 0X0000FF00, 0X0000FF00, 0X0000FF00, 0X0000FF00, 0X0000FF00, 0X00000000, 0X00000000, 0X00000000, 0X00000000, 0X00000000 },  // GGGGGooooo
  { 0X000000FF, 0X000000FF, 0X000000FF, 0X000000FF, 0X000000FF, 0X00000000, 0X00000000, 0X00000000, 0X00000000, 0X00000000 },  // BBBBBooooo
};

#define MODES (sizeof(modeColors)/(sizeof(uint32_t)*FRAMES))

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

  // Sends a clean report to the host. This is important on any Arduino type.
  BootKeyboard.begin();
}

void ledBlink()
{
  uint32_t microsecs;

#ifdef USE_SERIAL
  Serial.print("mode=");Serial.println(mode);
#endif

  while(1) {
    
    microsecs = micros() + 98;
    while (microsecs < 98) {
      microsecs = micros() + 98;
    }

    strip.setPixelColor( 0, modeColors[mode][mode_index] );
#ifdef USE_SERIAL
    Serial.print("mode_index=");Serial.println(mode_index);
    Serial.print("mode_color=");Serial.println(modeColors[mode][mode_index],16);
#endif    
    mode_index = (mode_index + 1) % FRAMES;
    digitalWrite(PIN_SWITCH, mode_index % 2);
    strip.show();
  
    while (micros() < microsecs);
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
  if (started == 0) {
    usbKeyboardConfig();
  } else {
    ledBlink();
  }

  delay(60);
}
