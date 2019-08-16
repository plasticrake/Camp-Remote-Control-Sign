#include <Arduino.h>
#include <FastLED.h>
#include <SimpleButton.h>
#include <WiFi.h>
#include <esp_now.h>

#define BUTTON_COUNT 10
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
simplebutton::Button* buttons[] = {buttonOne, buttonTwo, buttonThree, buttonFour, buttonFive, buttonSix, buttonSeven, buttonEight, buttonNine, buttonTen};

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

  for (int i = 0; i < BUTTON_COUNT; i++) {
    buttons[i]->setDefaultMinPushTime(100);
    buttons[i]->setDefaultMinReleaseTime(100);
  }

  WiFi.mode(WIFI_STA);
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    buttons[i]->update();
  }

  if (buttonOne->clicked()) {
    sendData(1);
  }
  if (buttonTwo->clicked()) {
    sendData(2);
  }
  if (buttonThree->clicked()) {
    sendData(3);
  }
  if (buttonFour->clicked()) {
    sendData(4);
  }
  if (buttonFive->clicked()) {
    sendData(5);
  }
  if (buttonSix->clicked()) {
    sendData(6);
  }
  if (buttonSeven->clicked()) {
    sendData(7);
  }
  if (buttonEight->clicked()) {
    sendData(8);
  }
  if (buttonNine->clicked()) {
    sendData(9);
  }
  if (buttonTen->clicked()) {
    sendData(10);
  }

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
