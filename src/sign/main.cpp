// This program doesn't account for millis overflow. Which means it may be temporarily wonky after running for 50 days
#include "commands.h"
#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <esp_now.h>

///////////////////
// Configuration //
///////////////////

constexpr uint32_t TIME_TO_AUTOSWITCH_IN_SECONDS = 60 * 2;
constexpr uint8_t MAX_BRIGHTNESS = 255;
constexpr uint32_t FRAMES_PER_SECOND = 120;

///////////////////

constexpr uint32_t TIME_TO_AUTOSWITCH_IN_MS = TIME_TO_AUTOSWITCH_IN_SECONDS * 1000;
uint8_t gBrightness = MAX_BRIGHTNESS;

// FastLED
constexpr uint16_t NUM_LEDS = 24 * 8 * 2;
constexpr uint8_t DATA1_PIN = 27;
constexpr uint8_t CLOCK1_PIN = 33;
constexpr uint8_t DATA2_PIN = 15;
constexpr uint8_t CLOCK2_PIN = 32;
CRGB leds[NUM_LEDS];

constexpr uint8_t MATRIX_WIDTH = 16;
constexpr uint8_t MATRIX_HEIGHT = 24;
constexpr uint8_t CentreX = (MATRIX_WIDTH / 2) - 1;
constexpr uint8_t CentreY = (MATRIX_HEIGHT / 2) - 1;

// ESP-now / WiFi
#define CHANNEL 1
#define NODE_NAME "CRC_Sign_1"
#define NODE_PASSWORD "CRC_Sign_1_Password"

constexpr uint8_t LED_PIN = 13;
bool ledOn = false;

/************/
/* Patterns */
/************/
CRGB gBaseColor = CRGB::White;
uint8_t gHue = 0;  // rotating "base color" used by many of the patterns

constexpr uint8_t PATTERN_COUNT = 11;
uint8_t gCurrentPatternNumber = 0;  // Index number of which pattern is current

void nextPattern(void);

void connected(void);
void rainbow(void);
void addGlitter(fract8 chanceOfGlitter, CRGB glitterColor);

uint16_t XY(uint8_t x, uint8_t y) {
  uint16_t i;
  if (x & 0x01) {
    // Odd columns run backwards
    uint8_t reverseY = (MATRIX_HEIGHT - 1) - y;
    i = (x * MATRIX_HEIGHT) + reverseY;
  } else {
    // Even columns run forwards
    i = (x * MATRIX_HEIGHT) + y;
  }
  return i;
}

void connected() {
  static unsigned long startTime = 0;
  static fract8 chanceOfGlitter = 0;

  rainbow();

  for (size_t i = 0; i < chanceOfGlitter; i++) {
    addGlitter(chanceOfGlitter, gHue);
  }

  chanceOfGlitter = qadd8(chanceOfGlitter, 1);

  if (chanceOfGlitter == 255) {
    if (startTime == 0) startTime = millis();
    if (millis() - startTime > 3000) {
      // reset and switch to next pattern
      startTime = 0;
      chanceOfGlitter = 0;
      nextPattern();
    }
  }
}

void adjust_gamma() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i].r = dim8_video(leds[i].r);
    leds[i].g = dim8_video(leds[i].g);
    leds[i].b = dim8_video(leds[i].b);
  }
}

DEFINE_GRADIENT_PALETTE(pit){
    0, 3, 3, 3,
    64, 13, 13, 255,  //blue
    128, 3, 3, 3,
    192, 255, 130, 3,  //orange
    255, 3, 3, 3};

void noise_noise1() {
  // https://gist.github.com/StefanPetrick/c856b6d681ec3122e5551403aabfcc68#file-noise_noise-ino-L27
  constexpr uint8_t NUM_LAYERS = 1;
  static uint32_t x[NUM_LAYERS];
  static uint32_t y[NUM_LAYERS];
  static uint32_t z[NUM_LAYERS];
  static uint32_t scale_x[NUM_LAYERS];
  static uint32_t scale_y[NUM_LAYERS];
  static uint8_t noise[1][MATRIX_WIDTH][MATRIX_HEIGHT];

  CRGBPalette16 Pal(pit);
  constexpr uint8_t XY_SPEED = 5;

  //modulate the position so that it increases/decreases x
  //(here based on the top left pixel - it could be any position else)
  //the factor "2" defines the max speed of the x movement
  //the "-255" defines the median moving direction
  x[0] = x[0] + (XY_SPEED * noise[0][0][0]) - 255;
  //modulate the position so that it increases/decreases y
  //(here based on the top right pixel - it could be any position else)
  y[0] = y[0] + (XY_SPEED * noise[0][MATRIX_WIDTH - 1][0]) - 255;
  //z just in one direction but with the additional "1" to make sure to never get stuck
  //in case the movement is stopped by a crazy parameter (noise data) combination
  //(here based on the down left pixel - it could be any position else)
  z[0] += 1 * ((noise[0][0][MATRIX_HEIGHT - 1]) / 4);
  //set the scaling based on left and right pixel of the middle line
  //here you can set the range of the zoom in both dimensions
  scale_x[0] = 8000 + (noise[0][0][CentreY] * MATRIX_WIDTH);
  scale_y[0] = 8000 + (noise[0][MATRIX_WIDTH - 1][CentreY] * MATRIX_HEIGHT);

  //calculate the noise data
  uint8_t layer = 0;
  for (uint8_t i = 0; i < MATRIX_WIDTH; i++) {
    uint32_t iOffset = scale_x[layer] * (i - CentreX);
    for (uint8_t j = 0; j < MATRIX_HEIGHT; j++) {
      uint32_t jOffset = scale_y[layer] * (j - CentreY);
      uint16_t data = inoise16(x[layer] + iOffset, y[layer] + jOffset, z[layer]);
      // limit the 16 bit results to the interesting range
      if (data < 11000) data = 11000;
      if (data > 51000) data = 51000;
      // normalize
      data = data - 11000;
      // scale down that the result fits into a byte
      data = data / 161;
      // store the result in the array
      noise[layer][i][j] = data;
    }
  }

  //map the colors
  for (uint8_t y = 0; y < MATRIX_HEIGHT; y++) {
    for (uint8_t x = 0; x < MATRIX_WIDTH; x++) {
      //I will add this overlay CRGB later for more colors
      //it´s basically a rainbow mapping with an inverted brightness mask
      CRGB overlay = CHSV(noise[0][map(y, 0, MATRIX_HEIGHT - 1, 0, MATRIX_WIDTH - 1)][map(x, 0, MATRIX_WIDTH - 1, 0, MATRIX_HEIGHT - 1)], 255, noise[0][x][y]);
      // CRGB overlay = CHSV(noise[0][y][x], 255, noise[0][x][y]);
      //here the actual colormapping happens - note the additional colorshift caused by the down right pixel noise[0][15][15]
      leds[XY(x, y)] = ColorFromPalette(Pal, noise[0][MATRIX_WIDTH - 1][MATRIX_HEIGHT - 1] + noise[0][x][y]) + overlay;
    }
  }

  //make it looking nice
  adjust_gamma();
}

void xyMatrixDrawOneFrame(byte startHue8, int8_t yHueDelta8, int8_t xHueDelta8) {
  byte lineStartHue = startHue8;
  for (byte y = 0; y < MATRIX_HEIGHT; y++) {
    lineStartHue += yHueDelta8;
    byte pixelHue = lineStartHue;
    for (byte x = 0; x < MATRIX_WIDTH; x++) {
      pixelHue += xHueDelta8;
      leds[XY(x, y)] = CHSV(pixelHue, 255, 255);
    }
  }
}

void xyMatrixPattern() {
  EVERY_N_MILLISECONDS(1000 / 15) {
    uint32_t ms = millis();
    int32_t yHueDelta32 = ((int32_t)cos16(ms * (27 / 1)) * (350 / MATRIX_WIDTH));
    int32_t xHueDelta32 = ((int32_t)cos16(ms * (39 / 1)) * (310 / MATRIX_HEIGHT));
    xyMatrixDrawOneFrame(ms / 65536, yHueDelta32 / 32768, xHueDelta32 / 32768);
    if (ms < 5000) {
      FastLED.setBrightness(scale8(gBrightness, (ms * 256) / 5000));
    } else {
      FastLED.setBrightness(gBrightness);
    }
  }
}

void Fire2018() {
  // https://gist.github.com/StefanPetrick/1ba4584e534ba99ca259c1103754e4c5
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t scale_x;
  uint32_t scale_y;

  constexpr uint8_t Width = MATRIX_WIDTH;
  constexpr uint8_t Height = MATRIX_HEIGHT;
  constexpr uint8_t CentreX = (Width / 2) - 1;
  constexpr uint8_t CentreY = (Height / 2) - 1;

  CRGBPalette16 pal = HeatColors_p;

  // storage for the noise data
  // adjust the size to suit your setup
  static uint8_t noise[Width][Height];

  // heatmap data with the size matrix width * height
  static uint8_t heat[Width * Height];

  // get one noise value out of a moving noise space
  uint16_t ctrl1 = inoise16(11 * millis(), 0, 0);
  // get another one
  uint16_t ctrl2 = inoise16(13 * millis(), 100000, 100000);
  // average of both to get a more unpredictable curve
  uint16_t ctrl = ((ctrl1 + ctrl2) / 2);

  // this factor defines the general speed of the heatmap movement
  // high value = high speed
  uint8_t speed = 27;

  // here we define the impact of the wind
  // high factor = a lot of movement to the sides
  x = 3 * ctrl * speed;

  // this is the speed of the upstream itself
  // high factor = fast movement
  y = 15 * millis() * speed;

  // just for ever changing patterns we move through z as well
  z = 3 * millis() * speed;

  // ...and dynamically scale the complete heatmap for some changes in the
  // size of the heatspots.
  // The speed of change is influenced by the factors in the calculation of ctrl1 & 2 above.
  // The divisor sets the impact of the size-scaling.
  scale_x = ctrl1 / 2;
  scale_y = ctrl2 / 2;

  // Calculate the noise array based on the control parameters.
  for (uint8_t i = 0; i < Width; i++) {
    uint32_t iOffset = scale_x * (i - CentreX);
    for (uint8_t j = 0; j < Height; j++) {
      uint32_t jOffset = scale_y * (j - CentreY);
      uint16_t data = ((inoise16(x + iOffset, y + jOffset, z)) + 1);
      noise[i][j] = data >> 8;
    }
  }

  // Draw the first (lowest) line - seed the fire.
  // It could be random pixels or anything else as well.
  for (uint8_t x = 0; x < Width; x++) {
    // draw
    leds[XY(x, Height - 1)] = ColorFromPalette(pal, noise[x][0]);
    // and fill the lowest line of the heatmap, too
    heat[XY(x, Height - 1)] = noise[x][0];
  }

  // Copy the heatmap one line up for the scrolling.
  for (uint8_t y = 0; y < Height - 1; y++) {
    for (uint8_t x = 0; x < Width; x++) {
      heat[XY(x, y)] = heat[XY(x, y + 1)];
    }
  }

  // Scale the heatmap values down based on the independent scrolling noise array.
  for (uint8_t y = 0; y < Height - 1; y++) {
    for (uint8_t x = 0; x < Width; x++) {
      // get data from the calculated noise field
      uint8_t dim = noise[x][y];

      // This number is critical
      // If it´s to low (like 1.1) the fire dosn´t go up far enough.
      // If it´s to high (like 3) the fire goes up too high.
      // It depends on the framerate which number is best.
      // If the number is not right you loose the uplifting fire clouds
      // which seperate themself while rising up.
      dim = dim / 1.4;

      dim = 255 - dim;

      // here happens the scaling of the heatmap
      heat[XY(x, y)] = scale8(heat[XY(x, y)], dim);
    }
  }

  // Now just map the colors based on the heatmap.
  for (uint8_t y = 0; y < Height - 1; y++) {
    for (uint8_t x = 0; x < Width; x++) {
      leds[XY(MATRIX_WIDTH - x, MATRIX_HEIGHT - y)] = ColorFromPalette(pal, heat[XY(x, y)]);
    }
  }
}

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void addGlitter(fract8 chanceOfGlitter, CRGB glitterColor) {
  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += glitterColor;
  }
}

void rainbowWithGlitter() {
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80, CRGB::White);
}

void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(gHue + random8(64), 200, 255);
}

void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  leds[pos] += CHSV(gHue, 255, 192);
}

void bpm() {
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for (int i = 0; i < NUM_LEDS; i++) {  //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void solid() {
  fill_solid(leds, NUM_LEDS, gBaseColor);
}

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {connected, Fire2018, solid, noise_noise1, xyMatrixPattern, rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm, rainbow, rainbowWithGlitter, confetti, sinelon, juggle};
// Fireplace, pattern not working

void nextPattern() {
  Serial.print("nextPattern ");
  Serial.print(gCurrentPatternNumber);
  Serial.print(" to ");
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % (PATTERN_COUNT - 1);
  if (gCurrentPatternNumber == 0) {
    //skip the `connected` pattern
    gCurrentPatternNumber++;
  }
  Serial.println(gCurrentPatternNumber);
}

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char* SSID = NODE_NAME;
  bool result = WiFi.softAP(SSID, NODE_PASSWORD, CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

unsigned long gLastMessageTime = 0;  // millis of last received message, we aren't handling overflow;

// callback when data is recv from Master
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
  gLastMessageTime = millis();

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  Serial.println(*data);
  Serial.println("");

  if (*data == REMOTE_CONNECTED) {
    Serial.println("REMOTE_CONNECTED");
    gCurrentPatternNumber = 0;
    Serial.println("PATTERN CHANGE RCVD");
  } else if (*data > 0 && *data <= PATTERN_COUNT) {
    gCurrentPatternNumber = *data;
  } else if (*data == 100) {
    // Do Nothing
  } else if (*data > 100) {
    Serial.println("COMMAND RCVD");
    switch (*data) {
      case BRIGHTNESS_UP:
        gBrightness = qadd8(gBrightness, 10);
        FastLED.setBrightness(gBrightness);
        break;
      case BRIGHTNESS_DOWN:
        gBrightness = qsub8(gBrightness, 10);
        FastLED.setBrightness(gBrightness);
        break;
    }
  }
  ledOn = !ledOn;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());

  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(LED_PIN, OUTPUT);

  FastLED.addLeds<APA102, DATA1_PIN, CLOCK1_PIN, BGR>(leds, NUM_LEDS / 2).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<APA102, DATA2_PIN, CLOCK2_PIN, BGR>(leds, NUM_LEDS / 2, NUM_LEDS / 2).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(gBrightness);
}

void loop() {
  EVERY_N_SECONDS(5) {
    Serial.println(gCurrentPatternNumber);

    if (millis() - gLastMessageTime > TIME_TO_AUTOSWITCH_IN_MS) {
      nextPattern();
    }
  }

  EVERY_N_MILLISECONDS(100) {
    if (ledOn) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }
  // fill_solid(leds, NUM_LEDS, CRGB::Green);
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);
  // delay(1000 / FRAMES_PER_SECOND);

  EVERY_N_MILLISECONDS(20) { gHue++; }  // slowly cycle the "base color" through the rainbow
}
