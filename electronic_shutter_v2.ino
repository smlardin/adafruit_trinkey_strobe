#include "Adafruit_NeoPixel.h"
#include "Adafruit_FreeTouch.h"
#include "HID-Project.h"  // https://github.com/NicoHood/HID

#include "RTCCounter.h"

// Create the neopixel strip with the built in definitions NUM_NEOPIXEL and PIN_NEOPIXEL
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_FreeTouch qt = Adafruit_FreeTouch(PIN_TOUCH, OVERSAMPLE_4, RESISTOR_50K, FREQ_MODE_NONE);

int16_t neo_brightness = 255; // initialize with max so no PWM on the LEDs

uint8_t started = 0;
uint8_t mode = 0;
uint16_t mode_index = 0;

uint8_t led_current = 0;
uint8_t led_last = 0;
uint16_t led_change = 0;
uint8_t led_click = 0;

#define MAX_FRAMES  (12)

#define DELAY_10US    480              // 10us
#define DELAY_80US    3840             // 80us
#define DELAY_90US    4322             // 90us
#define DELAY_100US   4800             // 100us
#define DELAY_125US   6000             // 125us 1/8000
#define DELAY_150US   7200             // 150us 
#define DELAY_175US   8400             // 175us
#define DELAY_200US   9600             // 200us
#define DELAY_500US   24000            // 500us
#define DELAY_1MS     48000            // 1ms

typedef struct {
  uint16_t  clock_delay;
  uint16_t  frames;
  uint32_t  colors[MAX_FRAMES];
} run_mode_t;

static const run_mode_t modes[] = {
  { DELAY_100US, 10, 0x00FF0000, 0x00000000, 0x0000FF00, 0x00000000, 0x000000FF, 0x00000000, 0x00FFFF00, 0x00000000, 0x00FFFFFF, 0x00000000                       },  
  { DELAY_100US,  5, 0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF, 0x00000000,                                                                                  },  
  { DELAY_100US,  2, 0x00FFFFFF, 0x00000000,                                                                                                                      },  
  { DELAY_200US,  2, 0x00FFFF00, 0x00000000,                                                                                                                      },  
  { DELAY_100US,  2, 0X00FF0000, 0X00000000,                                                                                                                      },  
  { DELAY_125US, 12, 0x0000FF00, 0x00000000, 0x00FFFFFF, 0x00000000, 0x000000FF, 0x00000000, 0x00FFFFFF, 0x00000000, 0x00FF0000, 0x00000000, 0x00FFFFFF, 0x000000 },  
  { DELAY_100US,  2, 0X0000FF00, 0X00000000,                                                                                                                      },  
  { DELAY_150US, 10, 0x000000FF, 0x00000000, 0x0000FF00, 0x00000000, 0x00FF0000, 0x00000000, 0x0000FFFF, 0x00000000, 0x00FFFFFF, 0x00000000                       },  
  { DELAY_100US,  2, 0X000000FF, 0X00000000,                                                                                                                      },  
  { DELAY_175US, 10, 0x00FFFFFF, 0x00000000, 0x0000FF00, 0x00000000, 0x0000FFFF, 0x00000000, 0x00FF0000, 0x00000000, 0x000000FF, 0x00000000                       },  
  { DELAY_100US, 10, 0x00FFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000                       },
  { DELAY_500US,  4, 0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF                                                                                               },
  { DELAY_1MS,    8, 0x00FF0000, 0x00FFFF00, 0x0000FF00, 0x0000FFFF, 0x000000FF, 0x00FF00FF, 0x00FFFFFF, 0x00000000                                               },
};

#define MODES (sizeof(modes)/sizeof(run_mode_t))

#undef USE_SERIAL

void setup() 
{
#ifdef USE_SERIAL
  Serial.begin(9600);
#endif

  strip.begin();
  strip.setBrightness(neo_brightness);
  strip.setPixelColor( 0, modes[0].colors[0] );
  strip.show(); 

  qt.begin();

#ifdef USE_SERIAL
  Serial.println("Starting");
#endif

  pinMode(PIN_SWITCH, OUTPUT );
  digitalWrite(PIN_SWITCH, 1);

  GCLK->CLKCTRL.reg = 0x401B; // Enable TC3 clock domain with same clock as CPU

  TC3->COUNT16.CTRLA.bit.MODE = 0;
  TC3->COUNT16.CTRLBSET.bit.DIR = 1; // Count down
  TC3->COUNT16.CTRLBCLR.bit.ONESHOT = 1; // Disable oneshot

  TC3->COUNT16.EVCTRL.bit.EVACT = 1; //Retrigger
  TC3->COUNT16.EVCTRL.bit.OVFEO = 1; // Overflow/underflow enable

  TC3->COUNT16.INTENCLR.bit.MC1 = 1; // Disable interrupts we aren't interested in
  TC3->COUNT16.INTENCLR.bit.MC0 = 1;
  TC3->COUNT16.INTENCLR.bit.SYNCRDY = 1;

  TC3->COUNT16.INTFLAG.reg = 0xFF; // Clear any pending interrupts

  TC3->COUNT16.CC[0].reg = DELAY_100US;

  TC3->COUNT16.CTRLA.bit.WAVEGEN = 1; // CC0 used for top
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;  

    // Sends a clean report to the host. This is important on any Arduino type.
  BootKeyboard.begin();
}

void ledBlink()
{
  uint16_t clock_delay = modes[mode].clock_delay;
  uint8_t frames = modes[mode].frames;

#ifdef USE_SERIAL
  Serial.print("mode=");Serial.println(mode);
#endif

  // Clear interrupt, reset counter to overflow
  TC3->COUNT16.INTFLAG.bit.OVF = 1;

  TC3->COUNT16.CC[0].reg = clock_delay;

  // Off we go
  TC3->COUNT16.CTRLBSET.bit.CMD = 0x1; // Retrigger

  while(1) {

    strip.setPixelColor( 0, modes[mode].colors[mode_index] );

#ifdef USE_SERIAL
    Serial.print("mode_index=");Serial.println(mode_index);
    Serial.print("mode_color=");Serial.println(modes[mode].colors[mode_index],16);
#endif    

    mode_index = (mode_index + 1) % frames;
    strip.show();

    // wait for the frame to complete
    while (!TC3->COUNT16.INTFLAG.bit.OVF);

    // most accurate for the pin if we set now
    digitalWrite(PIN_SWITCH, mode_index % 2);

    // clear the interrupts
    TC3->COUNT16.INTFLAG.reg = 0xff;
  }
}

uint8_t touch_debounce = 0;

void usbKeyboardConfig()
{
  uint16_t touch = qt.measure();

  // mimic an led
  led_current = (BootKeyboard.getLeds() & 0x2);

#ifdef USE_SERIAL
  Serial.print("led_change=");Serial.println(led_change);
  Serial.print("led_current=");Serial.println(led_current);
  Serial.print("mode=");Serial.println(mode);
#endif

  if (led_current != led_last) {
    if (!led_current && (led_change < 20)) 
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

  // touching to change modes
  touch_debounce = (touch_debounce << 1) | ((touch > 500)?1:0);
  if (touch_debounce == 0x1F)
  {
    mode = (mode + 1) % MODES;
    led_change = 0;
  }
  else if ((touch_debounce == 0x3F) || (touch_debounce == 0x7F) || (touch_debounce == 0xFF))
  {
    strip.setPixelColor( 0, 0 );
    strip.show();
    led_change = 0;
  }

  mode_index = (mode_index + 1) % modes[mode].frames;
  strip.setPixelColor( 0, modes[mode].colors[mode_index] );
  strip.show();
}

void loop() 
{

  if (started == 0) {
    usbKeyboardConfig();
  } else {
    ledBlink();
  }

  delay(100);
}