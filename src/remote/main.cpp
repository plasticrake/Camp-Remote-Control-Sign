// This program doesn't account for millis overflow. Which means it may be temporarily wonky after running for 50 days
#include "Adafruit_TLC5947.h"
#include <Arduino.h>
#include <FastLED.h>
#include <SimpleButton.h>
#include <WiFi.h>
#include <esp_now.h>

///////////////////
// Configuration //
///////////////////

constexpr uint32_t TIME_TO_IDLE_IN_SECONDS = 5;
constexpr uint32_t TIME_TO_AUTOSWITCH_IN_SECONDS = 10;
constexpr uint16_t MAX_BRIGHTNESS = 4095;

///////////////////

constexpr uint32_t TIME_TO_IDLE_IN_MS = TIME_TO_IDLE_IN_SECONDS * 1000;
constexpr uint32_t TIME_TO_AUTOSWITCH_IN_MS = TIME_TO_AUTOSWITCH_IN_SECONDS * 1000;

constexpr uint8_t BUTTON_COUNT = 11;
simplebutton::GPIOExpander* expander = NULL;
simplebutton::Button* buttonOne = NULL;
simplebutton::Button* buttonTwo = NULL;
simplebutton::Button* buttonThree = NULL;
simplebutton::Button* buttonFour = NULL;
simplebutton::Button* buttonFive = NULL;
simplebutton::Button* buttonSix = NULL;
simplebutton::Button* buttonSeven = NULL;
simplebutton::Button* buttonEight = NULL;
simplebutton::Button* buttonNine = NULL;
simplebutton::Button* buttonTen = NULL;
simplebutton::Button* buttonBonus = NULL;
simplebutton::Button* buttons[] = {buttonOne, buttonTwo, buttonThree, buttonFour, buttonFive, buttonSix, buttonSeven, buttonEight, buttonNine, buttonTen, buttonBonus};

enum ButtonId : uint8_t { NONE = 0,
                          ONE = 1,
                          TWO,
                          THREE,
                          FOUR,
                          FIVE,
                          SIX,
                          SEVEN,
                          EIGHT,
                          NINE,
                          TEN,
                          BONUS };

constexpr uint8_t buttonLedPin[] = {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};

// Button LEDs
constexpr uint8_t NUM_TLC5974 = 1;
constexpr uint8_t DATA_PIN = 27;
constexpr uint8_t CLOCK_PIN = 33;
constexpr uint8_t LATCH_PIN = 15;
Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5974, CLOCK_PIN, DATA_PIN, LATCH_PIN);

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#define SLAVE_PREFIX "CRC_Sign"

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counter and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

bool slaveFound = false;

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `CRC_Slave`
      if (SSID.indexOf(SLAVE_PREFIX) == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        slave.channel = CHANNEL;  // pick a channel
        slave.encrypt = 0;        // no encryption

        slaveFound = true;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

void deletePeer() {
  const esp_now_peer_info_t* peer = &slave;
  const uint8_t* peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t* peer = &slave;
    const uint8_t* peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if (exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void sendData(uint8_t data) {
  const uint8_t* peer_addr = slave.peer_addr;
  Serial.print("Sending: ");
  Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  expander = new simplebutton::MCP23017(0x20);

  tlc.begin();

  do {
    // check for errors
    if (!expander->connected()) {
      String errorMessage = expander->getError();
      Serial.println(errorMessage);
      Serial.println("Please check the wiring, the i2c address and restart the device!");
      delay(2000);
    }
  } while (!expander->connected());
  Serial.println("Port Expander Connected");

  buttonOne = new simplebutton::ButtonPullupGPIOExpander(expander, 0);
  buttonTwo = new simplebutton::ButtonPullupGPIOExpander(expander, 1);
  buttonThree = new simplebutton::ButtonPullupGPIOExpander(expander, 2);
  buttonFour = new simplebutton::ButtonPullupGPIOExpander(expander, 3);
  buttonFive = new simplebutton::ButtonPullupGPIOExpander(expander, 4);
  buttonSix = new simplebutton::ButtonPullupGPIOExpander(expander, 5);
  buttonSeven = new simplebutton::ButtonPullupGPIOExpander(expander, 6);
  buttonEight = new simplebutton::ButtonPullupGPIOExpander(expander, 7);
  buttonNine = new simplebutton::ButtonPullupGPIOExpander(expander, 8);
  buttonTen = new simplebutton::ButtonPullupGPIOExpander(expander, 9);
  buttonBonus = new simplebutton::ButtonPullupGPIOExpander(expander, 10);

  // for (int i = 0; i < BUTTON_COUNT; i++) {
  //   buttons[i]->setDefaultMinPushTime(100);
  //   buttons[i]->setDefaultMinReleaseTime(100);
  // }

  WiFi.mode(WIFI_STA);
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);
}

void updateButtons() {
  buttonOne->update();
  buttonTwo->update();
  buttonThree->update();
  buttonFour->update();
  buttonFive->update();
  buttonSix->update();
  buttonSeven->update();
  buttonEight->update();
  buttonNine->update();
  buttonTen->update();
  buttonBonus->update();
}

ButtonId getCurrentClickedButton() {
  if (buttonOne->clicked()) { return ONE; }
  if (buttonTwo->clicked()) { return TWO; }
  if (buttonThree->clicked()) { return THREE; }
  if (buttonFour->clicked()) { return FOUR; }
  if (buttonFive->clicked()) { return FIVE; }
  if (buttonSix->clicked()) { return SIX; }
  if (buttonSeven->clicked()) { return SEVEN; }
  if (buttonEight->clicked()) { return EIGHT; }
  if (buttonNine->clicked()) { return NINE; }
  if (buttonTen->clicked()) { return TEN; }
  if (buttonBonus->clicked()) { return BONUS; }
  return NONE;
}

ButtonId getCurrentHeldButton() {
  if (buttonOne->holding()) { return ONE; }
  if (buttonTwo->holding()) { return TWO; }
  if (buttonThree->holding()) { return THREE; }
  if (buttonFour->holding()) { return FOUR; }
  if (buttonFive->holding()) { return FIVE; }
  if (buttonSix->holding()) { return SIX; }
  if (buttonSeven->holding()) { return SEVEN; }
  if (buttonEight->holding()) { return EIGHT; }
  if (buttonNine->holding()) { return NINE; }
  if (buttonTen->holding()) { return TEN; }
  if (buttonBonus->holding()) { return BONUS; }
  return NONE;
}

bool getBonusModeStatus() {
  return (buttonBonus->holding());
}

uint8_t getPinForButton(uint8_t buttonId) {
  return buttonLedPin[buttonId - 1];
}

enum RemoteMode { NORMAL,
                  CHORD,  // Bonus held down, chord mode
                  IDLE
};

void updateButtonDisplay(uint8_t clickedButton, uint8_t activeButton, bool bonusHeld, RemoteMode mode, bool patternIsSticky) {
  if (mode == IDLE) {
    for (size_t i = 0; i < BUTTON_COUNT; i++) {
      tlc.setPWM(getPinForButton(i), beatsin16(100, 0, MAX_BRIGHTNESS, 0, 65536 / 2));
    }
  } else {
    for (size_t i = 0; i < BUTTON_COUNT; i++) {
      tlc.setPWM(getPinForButton(i), 0);
    }
  }

  if (activeButton != NONE) {
    tlc.setPWM(getPinForButton(activeButton), beatsin16(200, MAX_BRIGHTNESS / 2, MAX_BRIGHTNESS));
  }

  if (bonusHeld) {
    tlc.setPWM(getPinForButton(BONUS), beatsin16(300, MAX_BRIGHTNESS / 2, MAX_BRIGHTNESS));
  } else if (patternIsSticky) {
    tlc.setPWM(getPinForButton(BONUS), beatsin16(50, MAX_BRIGHTNESS / 2, MAX_BRIGHTNESS));
  }

  tlc.write();
}

bool getPatternIsSticky(bool patternIsSticky) {
  if (buttonBonus->doubleClicked()) {
    return !patternIsSticky;
  }
  return patternIsSticky;
}

bool getIsIdle(unsigned long timeOfLastAction) {
  return ((millis() - timeOfLastAction) > TIME_TO_IDLE_IN_MS);
}

bool getShouldAutoSwitchPattern(bool patternIsSticky, unsigned long timeOfLastAction, unsigned long timeOfLastPatternSwitch) {
  if (patternIsSticky) return false;
  return ((millis() - max(timeOfLastAction, timeOfLastPatternSwitch)) > TIME_TO_AUTOSWITCH_IN_MS);
}

ButtonId getNextPattern(uint8_t activeButton) {
  if (activeButton == TEN) {
    return ONE;
  }
  return static_cast<ButtonId>(activeButton + 1);
}

void loop() {
  static unsigned long timeOfLastAction = 0;         // millis of last user action, we aren't handling overflow;
  static unsigned long timeOfLastPatternSwitch = 0;  // millis of last pattern switch, we aren't handling overflow;
  static uint8_t activeButton = 0;                   // Active button / pattern
  static bool patternIsSticky = false;               // Remote in sticky mode, disable auto pattern switch

  static RemoteMode mode = NORMAL;

  updateButtons();

  patternIsSticky = getPatternIsSticky(patternIsSticky);

  bool bonusHeld = false;         // BONUS currently held
  ButtonId heldButton = NONE;     // Button currently held (excluding BONUS button)
  ButtonId clickedButton = NONE;  // Button currently clicked, or next pattern if during auto pattern switch;
  if (getShouldAutoSwitchPattern(patternIsSticky, timeOfLastAction, timeOfLastPatternSwitch)) {
    clickedButton = getNextPattern(activeButton);
    Serial.print("Auto Switching from ");
    Serial.print(activeButton);
    Serial.print(" to ");
    Serial.println(clickedButton);
  } else {
    bonusHeld = getBonusModeStatus();
    heldButton = getCurrentHeldButton();
    clickedButton = getCurrentClickedButton();
    if (clickedButton != NONE) {
      Serial.print("Button Clicked: ");
      Serial.println(clickedButton);
    }
    if (heldButton != NONE) {
      Serial.print("Button Held: ");
      Serial.println(heldButton);
    }
    if (bonusHeld) {
      Serial.println("BONUS Held");
    }
    if (clickedButton != NONE || heldButton != NONE || bonusHeld) {
      timeOfLastAction = millis();
    }
  }

  if (clickedButton > NONE) {
    timeOfLastAction = millis();
    if (clickedButton <= 10 && bonusHeld) {
      sendData(clickedButton + 100);
    } else {
      sendData(clickedButton);
      activeButton = clickedButton;
      timeOfLastPatternSwitch = millis();
    }
  } else if (bonusHeld && heldButton > NONE) {
    EVERY_N_MILLISECONDS(10) {
      timeOfLastAction = millis();
      sendData(clickedButton + 100);
    }
  }

  if (getIsIdle(timeOfLastAction)) {
    mode = IDLE;
  } else {
    mode = NORMAL;
  }

  updateButtonDisplay(clickedButton, activeButton, bonusHeld, mode, patternIsSticky);

  if (!slaveFound) {
    EVERY_N_SECONDS(5) {
      ScanForSlave();
      // If Slave is found, it would be populate in `slave` variable
      // We will check if `slave` is defined and then we proceed further
      if (slave.channel == CHANNEL) {  // check if slave channel is defined
        // `slave` is defined
        // Add slave as peer if it has not been added already
        bool isPaired = manageSlave();
        if (isPaired) {
          // pair success or already paired
          // Send data to device
          sendData(0);
        } else {
          Serial.println("Slave pair failed!");
        }
      } else {
        // No slave found to process
      }
    }
  }
}
