#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <esp_now.h>

// FastLED
#define NUM_LEDS 24 * 8 * 2
#define BRIGHTNESS 255
#define FRAMES_PER_SECOND 120
#define DATA_PIN 27
#define CLOCK_PIN 33
CRGB leds[NUM_LEDS];

// ESP-now / WiFi
#define CHANNEL 1
#define NODE_NAME "CRC_Sign_1"
#define NODE_PASSWORD "CRC_Sign_1_Password"

#define LED_PIN 13
bool ledOn = false;

/************/
/* Patterns */
/************/
uint8_t gHue = 0;  // rotating "base color" used by many of the patterns

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void addGlitter(fract8 chanceOfGlitter) {
  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void rainbowWithGlitter() {
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
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

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm, rainbow, rainbowWithGlitter, confetti, sinelon, juggle};
uint8_t gPatternMax = 10;
uint8_t gCurrentPatternNumber = 0;  // Index number of which pattern is current

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

// callback when data is recv from Master
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  Serial.println(*data);
  Serial.println("");

  if (*data > gPatternMax) {
    gCurrentPatternNumber = gPatternMax;
  } else {
    gCurrentPatternNumber = *data;
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

  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  if (ledOn) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);

  EVERY_N_MILLISECONDS(20) { gHue++; }  // slowly cycle the "base color" through the rainbow
}
