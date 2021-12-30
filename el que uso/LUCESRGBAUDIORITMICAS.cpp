#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <EEPROM.h>
#define PIN 6
#define N_PIXELS 68
#define BG 0
#define COLOR_ORDER GRB // Try mixing up the letters (RGB, GBR, BRG, etc) for a whole new world of color combinations
#define BRIGHTNESS 255  // 0-255, higher number is brighter.
#define LED_TYPE WS2812B
#define MIC_PIN A0         // Microphone is attached to this analog pin
#define DC_OFFSET 0        // DC offset in mic signal - if unusure, leave 0
#define NOISE 15           // Noise/hum/interference in mic signal
#define SAMPLES 10         // Length of buffer for dynamic level adjustment
#define TOP (N_PIXELS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 12       // Rate of peak falling dot
#define N_PIXELS_HALF (N_PIXELS / 2)
#define GRAVITY -9.81    // Downward (negative) acceleration of gravity in m/s^2
#define h0 1             // Starting height, in meters, of the ball (strip length)
#define NUM_BALLS 3      // Number of bouncing balls you want (recommend < 7, but 20 is fun in its own way)
#define SPEED .20        // Amount to increment RGB color by each cycle
#define SAMPLE_WINDOW 10 // Sample window for average level
//int brightnessPin = A0, potPin = A1;
#define COLOR_MIN 0
#define COLOR_MAX 255
#define DRAW_MAX 100
#define SEGMENTS 3           // 4 Number of segments to carve amplitude bar into
#define COLOR_WAIT_CYCLES 10 // Loop cycles to wait between advancing pixel origin
#define qsubd(x, b) ((x > b) ? b : 0)
#define qsuba(x, b) ((x > b) ? x - b : 0) // Analog Unsigned subtraction macro. if result <0, then => 0. By Andrew Tuline.
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define INPUT_FLOOR 10     //Lower range of analogRead input
#define INPUT_CEILING 1023 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)300 (150)
//config for balls
float h[NUM_BALLS];                       // An array of heights
float vImpact0 = sqrt(-2 * GRAVITY * h0); // Impact velocity of the ball when it hits the ground if "dropped" from the top of the strip
float vImpact[NUM_BALLS];                 // As time goes on the impact velocity will change, so make an array to store those values
float tCycle[NUM_BALLS];                  // The time since the last time the ball struck the ground
int pos[NUM_BALLS];                       // The integer position of the dot on the strip (LED index)
long tLast[NUM_BALLS];                    // The clock time of the last ground strike
float COR[NUM_BALLS];                     // Coefficient of Restitution (bounce damping)

float
    greenOffset = 30,
    blueOffset = 150;

byte
    peak = 0,     // Used for falling dot
    dotCount = 0, // Frame counter for delaying dot-falling speed
    volCount = 0; // Frame counter for storing past volume data
int
    reading,
    vol[SAMPLES],  // Collection of prior volume samples
    lvl = 30,       // Current "dampened" audio level
    minLvlAvg = 00, // For dynamic adjustment of graph low & high
    maxLvlAvg = 512;

int brightnessValue, prevBrightnessValue;
int sensorDeviationBrightness = 3;
int sensitivityValue = true;  // 0 - 255, initial value (value read from the potentiometer if useSensorValues = true)
int maxSensitivity = 2 * 255; // let the 'volume' go up to 200%!
int ledBrightness = 255;      // 0 - 255, initial value (value read from the potentiometer if useSensorValues = true)
int val;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, PIN, NEO_GRB + NEO_KHZ800);

// FOR SYLON ETC
uint8_t thisbeat = 23;
uint8_t thatbeat = 28;
uint8_t thisfade = 2;  // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255; // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255;

//FOR JUGGLE
uint8_t numdots = 4;  // Number of dots in use.
uint8_t faderate = 2; // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;  // Incremental change in hue between each dot.
uint8_t thishue = 0;  // Starting hue.
uint8_t curhue = 0;
uint8_t thisbright = 255; // How bright should the LED/display be.
uint8_t basebeat = 5;
uint8_t max_bright = 255;

// Twinkle
float redStates[N_PIXELS];
float blueStates[N_PIXELS];
float greenStates[N_PIXELS];
float Fade = 0.96;

// Vu meter 4
const uint32_t Red = strip.Color(255, 0, 0);
const uint32_t Yellow = strip.Color(255, 255, 0);
const uint32_t Green = strip.Color(0, 255, 0);
const uint32_t Blue = strip.Color(0, 0, 255);
const uint32_t White = strip.Color(255, 255, 255);
const uint32_t Dark = strip.Color(0, 0, 0);
unsigned int sample;

CRGB leds[N_PIXELS];

int myhue = 0;

// constants used here to set pin numbers:
const int buttonPin = 5; // the number of the pushbutton pin

// Variables will change:

int buttonPushCounter = 0; // counter for the number of button presses
int buttonState = 0;       // current state of the button
int lastButtonState = 0;

//Ripple variables
int color;
int center = 0;
int step = -1;
int maxSteps = 8;
float fadeRate = 0.80;
int diff;

//background color
uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;

//VARS DEL OTRO ARDUINO

byte dotHangCount = 0; //Frame counter for holding peak dot
#define PEAK_HANG 24   //Time of pause before peak dot falls
#define PEAK_FALL2 8   //Rate of falling peak dot
#define DRAW_MAX 100

#define NSAMPLES 64
unsigned int samplearray[NSAMPLES];
unsigned long samplesum = 0;
unsigned int sampleavg = 0;
int samplecount = 0;
//unsigned int sample = 0;
unsigned long oldtime = 0;
unsigned long newtime = 0;
#define POT_PIN 4 //4

//vu ripple
uint8_t colour;
uint8_t myfade = 255; // Starting brightness.
#define maxsteps 6    // 16 Case statement wouldn't allow a variable.
int peakspersec = 0;
int peakcount = 0;
uint8_t bgcol = 0;
int thisdelay = 20;

boolean
    growing = false,
    fall_from_left = true;

int
    origin = 0,
    color_wait_count = 0,
    scroll_color = COLOR_MIN,
    last_intensity = 0,
    intensity_max = 0,
    origin_at_flip = 0;

uint32_t
    draw[DRAW_MAX];

//new ripple vu
uint8_t timeval = 20;   // Currently 'delay' value. No, I don't use delays, I use EVERY_N_MILLIS_I instead.
uint16_t loops = 0;     // Our loops per second counter.
bool samplepeak = 0;    // This sample is well above the average, and is a 'peak'.
uint16_t oldsample = 0; // Previous sample is used for peak detection and for 'on the fly' values.
bool thisdir = 0;

CRGBPalette16 currentPalette(OceanColors_p);
CRGBPalette16 targetPalette(CloudColors_p);

//background color

TBlendType currentBlending;

void setup()
{

  //analogReference(EXTERNAL);
  pinMode(buttonPin, INPUT);
  //pinMode(buttonPin, OUTPUT);

  //initialize the buttonPin as output
  digitalWrite(buttonPin, HIGH);

  //initialize the serial port
  Serial.begin(9600);
  //Serial.begin(115200);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  delay(2000); // power-up safety delay

  FastLED.addLeds<WS2812B, PIN, COLOR_ORDER>(leds, N_PIXELS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  memset(vol, 0, sizeof(vol));

  for (int i = 0; i < NUM_BALLS; i++)
  { // Initialize variables
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;            // Balls start on the ground
    vImpact[i] = vImpact0; // And "pop" up at vImpact0
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i) / pow(NUM_BALLS, 2);
  }
}

float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve)
{

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10)
    curve = 10;
  if (curve < -10)
    curve = -10;

  curve = (curve * -.1);  // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin)
  {
    inputValue = originalMin;
  }
  if (inputValue > originalMax)
  {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin)
  {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange; // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax)
  {
    return 0;
  }

  if (invFlag == 0)
  {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  }
  else // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}

void loop()
{

  /*
   brightnessValue = analogRead(brightnessPin);
  brightnessValue = map(brightnessValue, 0, 1023, 0, 255);
  
  if (abs(brightnessValue - prevBrightnessValue) > sensorDeviationBrightness) {
    ledBrightness = brightnessValue;
    strip.setBrightness(ledBrightness);
    prevBrightnessValue = brightnessValue;
  }
  */
  //for mic
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;
  // end mic
  buttonPushCounter = EEPROM.read(0);
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState)
  {
    // if the state has changed, increment the counter
    if (buttonState == HIGH)
    {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;
      Serial.println("on");
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
      if (buttonPushCounter >= 14)
      {
        buttonPushCounter = 1;
      }
      EEPROM.write(0, buttonPushCounter);
    }
    else
    {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state,
  //for next time through the loop

  lastButtonState = buttonState;


 
  
  switch (buttonPushCounter)
  {


     case 1:
    buttonPushCounter == 1;
    {
      Vu1(); // SEE
      break;
    }
     case 2 :
    buttonPushCounter == 1;
    {
      Vu2(); // SEE
      break;
    }
     case 3:
    buttonPushCounter == 1;
    {
      Vu3(); // SEE
      break;
    }
     case 4:
    buttonPushCounter == 1;
    {
      Vu4(); // SEE
      break;
    }
     case 5:
    buttonPushCounter == 1;
    {
      Vu6(); // SEE
      break;
    }
     case 6:
    buttonPushCounter == 1;
    {
      Vu8(); // SEE
      break;
    }
     case 7:
    buttonPushCounter == 1;
    {
      Vu9(); // SEE
      break;
    }
     case 8:
    buttonPushCounter == 1;
    {
      Vu10(); // SEE
      break;
    }
     case 9:
    buttonPushCounter == 1;
    {
      Vu11(); // SEE
      break;
    }
     case 10:
    buttonPushCounter == 1;
    {
      Vu12(); // SEE
      break;
    }
     case 11:
    buttonPushCounter == 1;
    {
      Vu13(); // SEE
      break;
    }
     case 12:
    buttonPushCounter == 1;
    {
      ripple(); // SEE
      break;
    }
     case 13:
    buttonPushCounter == 1;
    {
      ripple2(); // SEE
      break;
    }
     case 14:
    buttonPushCounter == 1;
    {
      Twinkle(); // SEE
      break;
    }
     case 15:
    buttonPushCounter == 1;
    {
      pattern3(); // SEE
      break;
    }
     case 16:
    buttonPushCounter == 1;
    {
      Balls(); // SEE
      break;
    }
  
  

    
    
   
    
  

  }

  
}

void Vu1() /// OK
{

  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 0 - DC_OFFSET);         // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;         // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++)
  {
    if (i >= height)
      strip.setPixelColor(i, 0, 0, 0);
    else
      strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));
  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS - 1)
    strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }

  vol[volCount] = n; // Save sample for dynamic leveling
  if (++volCount >= SAMPLES)
    volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
    maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85)
  {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else
  {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void Vu2() /// OK
{

  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  //val = (analogRead(potPin));
  //val= map(val, 0, 1023, -10, 6);
  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 0 - DC_OFFSET);         // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum

  /*
      if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }

        */
  lvl = ((lvl * 7) + n) >> 3; // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS_HALF; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(N_PIXELS_HALF - i - 1, 0, 0, 0);
      strip.setPixelColor(N_PIXELS_HALF + i, 0, 0, 0);
    }
    else
    {
      uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
      strip.setPixelColor(N_PIXELS_HALF - i - 1, color);
      strip.setPixelColor(N_PIXELS_HALF + i, color);
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS_HALF - 1)
  {
    uint32_t color = Wheel(map(peak, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS_HALF - peak - 1, color);
    strip.setPixelColor(N_PIXELS_HALF + peak, color);
  }

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }

  vol[volCount] = n; // Save sample for dynamic leveling
  if (++volCount >= SAMPLES)
    volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
    maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

//here................

////OK
void Vu3()
{
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 0 - DC_OFFSET);         // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum

  if (val < 0)
  {
    n = n / (val * (-1));
  }
  if (val > 0)
  {
    n = n * val;
  }

  lvl = ((lvl * 7) + n) >> 3; // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top

  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255)
    greenOffset = 0;
  if (blueOffset >= 255)
    blueOffset = 0;

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(i, 0, 0, 0);
    }
    else
    {
      strip.setPixelColor(i, Wheel(
                                 map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)));
    }
  }
  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS - 1)
    strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }
  strip.show(); // Update strip

  vol[volCount] = n;
  if (++volCount >= SAMPLES)
  {
    volCount = 0;
  }

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
    {
      minLvl = vol[i];
    }
    else if (vol[i] > maxLvl)
    {
      maxLvl = vol[i];
    }
  }

  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
  {
    maxLvl = minLvl + TOP;
  }
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

////OK
void Vu4()
{
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 0 - DC_OFFSET);         // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum

  if (val < 0)
  {
    n = n / (val * (-1));
  }
  if (val > 0)
  {
    n = n * val;
  }

  lvl = ((lvl * 7) + n) >> 3; // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255)
    greenOffset = 0;
  if (blueOffset >= 255)
    blueOffset = 0;

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS_HALF; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(N_PIXELS_HALF - i - 1, 0, 0, 0);
      strip.setPixelColor(N_PIXELS_HALF + i, 0, 0, 0);
    }
    else
    {
      uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, (int)greenOffset, (int)blueOffset));
      strip.setPixelColor(N_PIXELS_HALF - i - 1, color);
      strip.setPixelColor(N_PIXELS_HALF + i, color);
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS_HALF - 1)
  {
    uint32_t color = Wheel(map(peak, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS_HALF - peak - 1, color);
    strip.setPixelColor(N_PIXELS_HALF + peak, color);
  }

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }

  vol[volCount] = n; // Save sample for dynamic leveling
  if (++volCount >= SAMPLES)
    volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
    maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}


/// NO
void Vu5()
{

  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);       // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;         // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS_HALF; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(N_PIXELS_HALF - i - 1, 0, 0, 0);
      strip.setPixelColor(N_PIXELS_HALF + i, 0, 0, 0);
    }
    else
    {
      uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
      strip.setPixelColor(N_PIXELS_HALF - i - 1, color);
      strip.setPixelColor(N_PIXELS_HALF + i, color);
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS_HALF - 1)
  {
    uint32_t color = Wheel(map(peak, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS_HALF - peak - 1, color);
    strip.setPixelColor(N_PIXELS_HALF + peak, color);
  }

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }

  vol[volCount] = n; // Save sample for dynamic leveling
  if (++volCount >= SAMPLES)
    volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
    maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}


///OK
void Vu6()
{
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;                 // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;

  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(MIC_PIN);
    if (sample < 1024)
    {
      if (sample > signalMax)
      {
        signalMax = sample;
      }
      else if (sample < signalMin)
      {
        signalMin = sample;
      }
    }
  }
  peakToPeak = signalMax - signalMin;

  // Serial.println(peakToPeak);

  for (int i = 0; i <= N_PIXELS_HALF - 1; i++)
  {
    uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS - i, color);
    strip.setPixelColor(0 + i, color);
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, N_PIXELS_HALF, 0, peakToPeak, 2);

  if (c < peak)
  {
    peak = c;         // Keep dot on top
    dotHangCount = 0; // make the dot hang before falling
  }
  if (c <= strip.numPixels())
  { // Fill partial column with off pixels
    drawLine(N_PIXELS_HALF, N_PIXELS_HALF - c, strip.Color(0, 0, 0));
    drawLine(N_PIXELS_HALF, N_PIXELS_HALF + c, strip.Color(0, 0, 0));
  }

  y = N_PIXELS_HALF - peak;
  uint32_t color1 = Wheel(map(y, 0, N_PIXELS_HALF - 1, 30, 150));
  strip.setPixelColor(y - 1, color1);
  //strip.setPixelColor(y-1,Wheel(map(y,0,N_PIXELS_HALF-1,30,150)));

  y = N_PIXELS_HALF + peak;
  strip.setPixelColor(y, color1);
  //strip.setPixelColor(y+1,Wheel(map(y,0,N_PIXELS_HALF+1,30,150)));

  strip.show();

  // Frame based peak dot animation
  if (dotHangCount > PEAK_HANG)
  { //Peak pause length
    if (++dotCount >= PEAK_FALL2)
    { //Fall rate
      peak++;
      dotCount = 0;
    }
  }
  else
  {
    dotHangCount++;
  }
}


/// NO 
void Vu7()
{

  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);       // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;         // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS_HALF; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(N_PIXELS_HALF - i - 1, 0, 0, 0);
      strip.setPixelColor(N_PIXELS_HALF + i, 0, 0, 0);
    }
    else
    {
      uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
      strip.setPixelColor(N_PIXELS_HALF - i - 1, color);
      strip.setPixelColor(N_PIXELS_HALF + i, color);
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS_HALF - 1)
  {
    uint32_t color = Wheel(map(peak, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS_HALF - peak - 1, color);
    strip.setPixelColor(N_PIXELS_HALF + peak, color);
  }

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }

  vol[volCount] = n; // Save sample for dynamic leveling
  if (++volCount >= SAMPLES)
    volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
    maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

/// SI ESTA PARADO 
void Vu8()
{
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);       // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;         // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;
  if (height > peak)
    peak = height; // Keep 'peak' dot at top

  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255)
    greenOffset = 0;
  if (blueOffset >= 255)
    blueOffset = 0;

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(i, 0, 0, 0);
    }
    else
    {
      strip.setPixelColor(i, Wheel(
                                 map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)));
    }
  }
  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS - 1)
    strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL)
  { //fall rate

    if (peak > 0)
      peak--;
    dotCount = 0;
  }
  strip.show(); // Update strip

  vol[volCount] = n;
  if (++volCount >= SAMPLES)
  {
    volCount = 0;
  }

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
    {
      minLvl = vol[i];
    }
    else if (vol[i] > maxLvl)
    {
      maxLvl = vol[i];
    }
  }

  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP)
  {
    maxLvl = minLvl + TOP;
  }
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

/// SI CAMBIA DE LUCES CON DESTELLOS BLANCOS 
void Vu9()
{

  EVERY_N_MILLISECONDS(1000)
  {
    peakspersec = peakcount; // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;           // Reset the counter every second.
  }

  soundmems();

  EVERY_N_MILLISECONDS(20)
  {
    ripple3();
  }

  show_at_max_brightness_for_power();

} // loop()


/// OK. CABIA DE COLOR CON ESPACIOS NEGROS EN EL MEDIO
void Vu10()
{
  int intensity = calculateIntensity();
  updateOrigin(intensity);
  assignDrawValues(intensity);
  writeSegmented();
  updateGlobals();
}

int calculateIntensity()
{
  int intensity;

  reading = analogRead(MIC_PIN);                        // Raw reading from mic
  reading = abs(reading - 512 - DC_OFFSET);             // Center on zero
  reading = (reading <= NOISE) ? 0 : (reading - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + reading) >> 3;                     // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  intensity = DRAW_MAX * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  return constrain(intensity, 0, DRAW_MAX - 1);
}

void updateOrigin(int intensity)
{
  // detect peak change and save origin at curve vertex
  if (growing && intensity < last_intensity)
  {
    growing = false;
    intensity_max = last_intensity;
    fall_from_left = !fall_from_left;
    origin_at_flip = origin;
  }
  else if (intensity > last_intensity)
  {
    growing = true;
    origin_at_flip = origin;
  }
  last_intensity = intensity;

  // adjust origin if falling
  if (!growing)
  {
    if (fall_from_left)
    {
      origin = origin_at_flip + ((intensity_max - intensity) / 2);
    }
    else
    {
      origin = origin_at_flip - ((intensity_max - intensity) / 2);
    }
    // correct for origin out of bounds
    if (origin < 0)
    {
      origin = DRAW_MAX - abs(origin);
    }
    else if (origin > DRAW_MAX - 1)
    {
      origin = origin - DRAW_MAX - 1;
    }
  }
}

void assignDrawValues(int intensity)
{
  // draw amplitue as 1/2 intensity both directions from origin
  int min_lit = origin - (intensity / 2);
  int max_lit = origin + (intensity / 2);
  if (min_lit < 0)
  {
    min_lit = min_lit + DRAW_MAX;
  }
  if (max_lit >= DRAW_MAX)
  {
    max_lit = max_lit - DRAW_MAX;
  }
  for (int i = 0; i < DRAW_MAX; i++)
  {
    // if i is within origin +/- 1/2 intensity
    if (
        (min_lit < max_lit && min_lit < i && i < max_lit)      // range is within bounds and i is within range
        || (min_lit > max_lit && (i > min_lit || i < max_lit)) // range wraps out of bounds and i is within that wrap
    )
    {
      draw[i] = Wheel(scroll_color);
    }
    else
    {
      draw[i] = 0;
    }
  }
}

void writeSegmented()
{
  int seg_len = N_PIXELS / SEGMENTS;

  for (int s = 0; s < SEGMENTS; s++)
  {
    for (int i = 0; i < seg_len; i++)
    {
      strip.setPixelColor(i + (s * seg_len), draw[map(i, 0, seg_len, 0, DRAW_MAX)]);
    }
  }
  strip.show();
}

uint32_t *segmentAndResize(uint32_t *draw)
{
  int seg_len = N_PIXELS / SEGMENTS;

  uint32_t segmented[N_PIXELS];
  for (int s = 0; s < SEGMENTS; s++)
  {
    for (int i = 0; i < seg_len; i++)
    {
      segmented[i + (s * seg_len)] = draw[map(i, 0, seg_len, 0, DRAW_MAX)];
    }
  }

  return segmented;
}

void writeToStrip(uint32_t *draw)
{
  for (int i = 0; i < N_PIXELS; i++)
  {
    strip.setPixelColor(i, draw[i]);
  }
  strip.show();
}

void updateGlobals()
{
  uint16_t minLvl, maxLvl;

  //advance color wheel
  color_wait_count++;
  if (color_wait_count > COLOR_WAIT_CYCLES)
  {
    color_wait_count = 0;
    scroll_color++;
    if (scroll_color > COLOR_MAX)
    {
      scroll_color = COLOR_MIN;
    }
  }

  vol[volCount] = reading; // Save sample for dynamic leveling
  if (++volCount >= SAMPLES)
    volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (uint8_t i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < N_PIXELS)
    maxLvl = minLvl + N_PIXELS;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

/// OK DEL MEDIO DISPERSA  LINEAS 
void Vu11()
{
  //currentBlending = LINEARBLEND;
  currentPalette = OceanColors_p; // Initial palette.
  currentBlending = LINEARBLEND;
  EVERY_N_SECONDS(5)
  { // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++)
    {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(100)
  { // AWESOME palette blending capability once they do change.
    uint8_t maxChanges = 24;
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }

  EVERY_N_MILLIS_I(thistimer, 20)
  {                                         // For fun, let's make the animation have a variable rate.
    uint8_t timeval = beatsin8(10, 20, 50); // Use a sinewave for the line below. Could also use peak/beat detection.
    thistimer.setPeriod(timeval);           // Allows you to change how often this routine runs.
    fadeToBlackBy(leds, N_PIXELS, 16);      // 1 = slow, 255 = fast fade. Depending on the faderate, the LED's further away will fade out.
    sndwave();
    soundble();
  }
  FastLED.setBrightness(max_bright);
  FastLED.show();

} // loop()

void soundble()
{ // Quick and dirty sampling of the microphone.

  int tmp = analogRead(MIC_PIN) - 512 - DC_OFFSET;
  sample = abs(tmp);

} // soundmems()

void sndwave()
{

  leds[N_PIXELS / 2] = ColorFromPalette(currentPalette, sample, sample * 2, currentBlending); // Put the sample into the center

  for (int i = N_PIXELS - 1; i > N_PIXELS / 2; i--)
  { //move to the left      // Copy to the left, and let the fade do the rest.
    leds[i] = leds[i - 1];
  }

  for (int i = 0; i < N_PIXELS / 2; i++)
  { // move to the right    // Copy to the right, and let the fade to the rest.
    leds[i] = leds[i + 1];
  }
  addGlitter(sampleavg);
}

/// OK DESTELLOS DE LUCES BLANCAS Y  DE COLORES 
void Vu12()
{

  EVERY_N_MILLISECONDS(1000)
  {
    peakspersec = peakcount; // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;           // Reset the counter every second.
  }

  soundripped();

  EVERY_N_MILLISECONDS(20)
  {
    rippvu();
  }

  FastLED.show();

} // loop()



/// OK BLANCO LOS LUCES DE COLORES PARA ATRAS 
void Vu13()
{ // The >>>>>>>>>> L-O-O-P <<<<<<<<<<<<<<<<<<<<<<<<<<<<  is buried here!!!11!1!

  EVERY_N_MILLISECONDS(1000)
  {
    peakspersec = peakcount; // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;           // Reset the counter every second.
  }

  soundripper();

  EVERY_N_MILLISECONDS(20)
  {
    jugglep();
  }

  FastLED.show();

} // loop()

void soundripper()
{ // Rolling average counter - means we don't have to go through an array each time.

  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60);

  samplesum = samplesum + sample - samplearray[samplecount]; // Add the new sample and remove the oldest sample in the array
  sampleavg = samplesum / NSAMPLES;                          // Get an average

  Serial.println(sampleavg);

  samplearray[samplecount] = sample;          // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES; // Update the counter for the array

  if (newtime > (oldtime + 200))
    digitalWrite(13, LOW); // Turn the LED off 200ms after the last peak.

  if ((sample > (sampleavg + potin)) && (newtime > (oldtime + 60)))
  { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    oldtime = newtime;
    // Change the current pattern function periodically.
    jugglep();
  }

} // loop()

void jugglep()
{ // Use the juggle routine, but adjust the timebase based on sampleavg for some randomness.

  // Persistent local variables
  static uint8_t thishue = 0;

  timeval = 40; // Our EVERY_N_MILLIS_I timer value.

  leds[0] = ColorFromPalette(currentPalette, thishue++, sampleavg, LINEARBLEND);

  for (int i = N_PIXELS - 1; i > 0; i--)
    leds[i] = leds[i - 1];

  addGlitter(sampleavg / 2); // Add glitter based on sampleavg. By Andrew Tuline.
}

void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256; j++)
  {
    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    // check if a button pressed
    if (digitalRead(buttonPin) != lastButtonState) // <------------- add this
      return;                                      // <------------ and this
    delay(wait);
  }
}


/// OK CAMBIA DE COLORES EL TUBO Y TIENE DESTELLOS DE COLORES 
void ripple()
{

  if (currentBg == nextBg)
  {
    nextBg = random(256);
  }
  else if (nextBg > currentBg)
  {
    currentBg++;
  }
  else
  {
    currentBg--;
  }
  for (uint16_t l = 0; l < N_PIXELS; l++)
  {
    leds[l] = CHSV(currentBg, 255, 50); // strip.setPixelColor(l, Wheel(currentBg, 0.1));
  }

  if (step == -1)
  {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }

  if (step == 0)
  {
    leds[center] = CHSV(color, 255, 255); // strip.setPixelColor(center, Wheel(color, 1));
    step++;
  }
  else
  {
    if (step < maxSteps)
    {
      Serial.println(pow(fadeRate, step));

      leds[wrap(center + step)] = CHSV(color, 255, pow(fadeRate, step) * 255); //   strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      leds[wrap(center - step)] = CHSV(color, 255, pow(fadeRate, step) * 255); //   strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3)
      {
        leds[wrap(center + step - 3)] = CHSV(color, 255, pow(fadeRate, step - 2) * 255); //   strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        leds[wrap(center - step + 3)] = CHSV(color, 255, pow(fadeRate, step - 2) * 255); //   strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step++;
    }
    else
    {
      step = -1;
    }
  }

  LEDS.show();
  delay(50);
}


/// OK  TIENE DESTELLOS DE COLORES 
void ripple2()
{
  if (BG)
  {
    if (currentBg == nextBg)
    {
      nextBg = random(256);
    }
    else if (nextBg > currentBg)
    {
      currentBg++;
    }
    else
    {
      currentBg--;
    }
    for (uint16_t l = 0; l < N_PIXELS; l++)
    {
      strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  }
  else
  {
    for (uint16_t l = 0; l < N_PIXELS; l++)
    {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }

  if (step == -1)
  {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }

  if (step == 0)
  {
    strip.setPixelColor(center, Wheel(color, 1));
    step++;
  }
  else
  {
    if (step < maxSteps)
    {
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3)
      {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step++;
    }
    else
    {
      step = -1;
    }
  }

  strip.show();
  delay(50);
}
//int wrap(int step) {
//  if(step < 0) return Pixels + step;
//  if(step > Pixels - 1) return step - Pixels;
//  return step;
//}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.


/// OK DDESTELLOS DE COLORES 
void Twinkle()
{
  if (random(25) == 1)
  {
    uint16_t i = random(N_PIXELS);
    if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1)
    {
      redStates[i] = random(256);
      greenStates[i] = random(256);
      blueStates[i] = random(256);
    }
  }

  for (uint16_t l = 0; l < N_PIXELS; l++)
  {
    if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1)
    {
      strip.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);

      if (redStates[l] > 1)
      {
        redStates[l] = redStates[l] * Fade;
      }
      else
      {
        redStates[l] = 0;
      }

      if (greenStates[l] > 1)
      {
        greenStates[l] = greenStates[l] * Fade;
      }
      else
      {
        greenStates[l] = 0;
      }

      if (blueStates[l] > 1)
      {
        blueStates[l] = blueStates[l] * Fade;
      }
      else
      {
        blueStates[l] = 0;
      }
    }
    else
    {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
  strip.show();
  delay(10);
}

// TOO HERE

// Pattern 3 - JUGGLE OK  LA LKUZ VA COMO VIVORITA PARA TODOS LAODS 
void pattern3()
{
  ChangeMe();
  juggle();
  show_at_max_brightness_for_power(); // Power managed display of LED's.
} // loop()

// OK 
void Balls()
{
  for (int i = 0; i < NUM_BALLS; i++)
  {
    tCycle[i] = millis() - tLast[i]; // Calculate the time since the last time the ball was on the ground

    // A little kinematics equation calculates positon as a function of time, acceleration (gravity) and intial velocity
    h[i] = 0.5 * GRAVITY * pow(tCycle[i] / 1000, 2.0) + vImpact[i] * tCycle[i] / 1000;

    if (h[i] < 0)
    {
      h[i] = 0;                         // If the ball crossed the threshold of the "ground," put it back on the ground
      vImpact[i] = COR[i] * vImpact[i]; // and recalculate its new upward velocity as it's old velocity * COR
      tLast[i] = millis();

      if (vImpact[i] < 0.01)
        vImpact[i] = vImpact0; // If the ball is barely moving, "pop" it back up at vImpact0
    }
    pos[i] = round(h[i] * (N_PIXELS - 1) / h0); // Map "h" to a "pos" integer index position on the LED strip
  }

  //Choose color of LEDs, then the "pos" LED on
  for (int i = 0; i < NUM_BALLS; i++)
    leds[pos[i]] = CHSV(uint8_t(i * 40), 255, 255);
  FastLED.show();
  //Then off for the next loop around
  for (int i = 0; i < NUM_BALLS; i++)
  {
    leds[pos[i]] = CRGB::Black;
  }
}

void colorWipe(uint32_t c, uint8_t wait)
{
  for (uint16_t i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
    if (digitalRead(buttonPin) != lastButtonState) // <------------- add this
      return;                                      // <------------ and this
    delay(wait);
  }
}

//// NO
void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, N_PIXELS, thisfade);
  int pos1 = beatsin16(thisbeat, 0, N_PIXELS);
  int pos2 = beatsin16(thatbeat, 0, N_PIXELS);
  leds[(pos1 + pos2) / 2] += CHSV(myhue++ / 64, thissat, thisbri);
}

void juggle()
{                   // Several colored dots, weaving in and out of sync with each other
  curhue = thishue; // Reset the hue values.
  fadeToBlackBy(leds, N_PIXELS, faderate);
  for (int i = 0; i < numdots; i++)
  {
    leds[beatsin16(basebeat + i + numdots, 0, N_PIXELS)] += CHSV(curhue, thissat, thisbright); //beat16 is a FastLED 3.1 function
    curhue += hueinc;
  }
} // juggle()

void ChangeMe()
{                                              // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 30; // IMPORTANT!!! Change '30' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;              // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand)
  { // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    if (secondHand == 0)
    {
      numdots = 1;
      faderate = 2;
    } // You can change values here, one at a time , or altogether.
    if (secondHand == 10)
    {
      numdots = 4;
      thishue = 128;
      faderate = 8;
    }
    if (secondHand == 20)
    {
      hueinc = 48;
      thishue = random8();
    } // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
  }
} // ChangeMe()

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++)
  { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    if (digitalRead(buttonPin) != lastButtonState) // <------------- add this
      return;                                      // <------------ and this
    delay(wait);
  }
}
// HERE

int wrap(int step)
{
  if (step < 0)
    return N_PIXELS + step;
  if (step > N_PIXELS - 1)
    return step - N_PIXELS;
  return step;
}

void one_color_allHSV(int ahue, int abright)
{ // SET ALL LEDS TO ONE COLOR (HSV)
  for (int i = 0; i < N_PIXELS; i++)
  {
    leds[i] = CHSV(ahue, 255, abright);
  }
}

uint32_t Wheel(byte WheelPos, float opacity)
{

  if (WheelPos < 85)
  {
    return strip.Color((WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity, 0);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color((255 - WheelPos * 3) * opacity, 0, (WheelPos * 3) * opacity);
  }
  else
  {
    WheelPos -= 170;
    return strip.Color(0, (WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity);
  }
}

void drawLine(uint8_t from, uint8_t to, uint32_t c)
{
  uint8_t fromTemp;
  if (from > to)
  {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for (int i = from; i <= to; i++)
  {
    strip.setPixelColor(i, c);
  }
}

void setPixel(int Pixel, byte red, byte green, byte blue)
{
  strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

void setAll(byte red, byte green, byte blue)
{

  for (int i = 0; i < N_PIXELS; i++)
  {

    setPixel(i, red, green, blue);
  }

  strip.show();
}

void soundmems()
{ // Rolling average counter - means we don't have to go through an array each time.
  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60);

  samplesum = samplesum + sample - samplearray[samplecount]; // Add the new sample and remove the oldest sample in the array
  sampleavg = samplesum / NSAMPLES;                          // Get an average
  samplearray[samplecount] = sample;                         // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                // Update the counter for the array

  if (newtime > (oldtime + 200))
    digitalWrite(13, LOW); // Turn the LED off 200ms after the last peak.

  if ((sample > (sampleavg + potin)) && (newtime > (oldtime + 60)))
  { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    digitalWrite(13, HIGH);
    oldtime = newtime;
  }
} // soundmems()

void ripple3()
{
  for (int i = 0; i < N_PIXELS; i++)
    leds[i] = CHSV(bgcol, 255, sampleavg * 2); // Set the background colour.

  switch (step)
  {

  case -1: // Initialize ripple variables.
    center = random(N_PIXELS);
    colour = (peakspersec * 10) % 255; // More peaks/s = higher the hue colour.
    step = 0;
    bgcol = bgcol + 8;
    break;

  case 0:
    leds[center] = CHSV(colour, 255, 255); // Display the first pixel of the ripple.
    step++;
    break;

  case maxsteps: // At the end of the ripples.
    // step = -1;
    break;

  default:                                                                               // Middle of the ripples.
    leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2); // Simple wrap from Marc Miller.
    leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2);
    step++; // Next step.
    break;
  } // switch step
} // ripple()

void addGlitter(fract8 chanceOfGlitter)
{ // Let's add some glitter, thanks to Mark

  if (random8() < chanceOfGlitter)
  {
    leds[random16(N_PIXELS)] += CRGB::White;
  }
}

void rippled()
{

  fadeToBlackBy(leds, N_PIXELS, 64); // 8 bit, 1 = slow, 255 = fast

  switch (step)
  {

  case -1: // Initialize ripple variables.
    center = random(N_PIXELS);
    colour = (peakspersec * 10) % 255; // More peaks/s = higher the hue colour.
    step = 0;
    break;

  case 0:
    leds[center] = CHSV(colour, 255, 255); // Display the first pixel of the ripple.
    step++;
    break;

  case maxsteps: // At the end of the ripples.
    // step = -1;
    break;

  default:                                                                               // Middle of the ripples.
    leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2); // Simple wrap from Marc Miller.
    leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2);
    step++; // Next step.
    break;
  } // switch step

} // ripple()

void soundripped()
{ // Rolling average counter - means we don't have to go through an array each time.

  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60);

  samplesum = samplesum + sample - samplearray[samplecount]; // Add the new sample and remove the oldest sample in the array
  sampleavg = samplesum / NSAMPLES;                          // Get an average

  Serial.println(sampleavg);

  samplearray[samplecount] = sample;          // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES; // Update the counter for the array

  if (newtime > (oldtime + 200))
    digitalWrite(13, LOW); // Turn the LED off 200ms after the last peak.

  if ((sample > (sampleavg + potin)) && (newtime > (oldtime + 60)))
  { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    oldtime = newtime;
  }

} // soundmems()

void rippvu()
{ // Display ripples triggered by peaks.

  fadeToBlackBy(leds, N_PIXELS, 64); // 8 bit, 1 = slow, 255 = fast

  switch (step)
  {

  case -1: // Initialize ripple variables.
    center = random(N_PIXELS);
    colour = (peakspersec * 10) % 255; // More peaks/s = higher the hue colour.
    step = 0;
    break;

  case 0:
    leds[center] = CHSV(colour, 255, 255); // Display the first pixel of the ripple.
    step++;
    break;

  case maxsteps: // At the end of the ripples.
    // step = -1;
    break;

  default:                                                                               // Middle of the ripples.
    leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2); // Simple wrap from Marc Miller.
    leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2);
    step++; // Next step.
    break;
  } // switch step
  addGlitter(sampleavg);
} // ripple()
