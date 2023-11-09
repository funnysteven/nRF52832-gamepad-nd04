/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This sketch demonstrate how to use BLEHidGamepad to send
 * button, dpad, and joystick report
 */
#define BOUNCE_WITH_PROMPT_DETECTION

#include <bluefruit.h>
#include <Bounce2.h>  // https://github.com/thomasfredericks/Bounce2
#include "pin_define.h"

BLEDis bledis;  // DIS (Device Information Service) helper class instance
BLEBas blebas;  // BAS (Battery Service) helper class instance
BLEHidGamepad blegamepad;

// defined in hid.h from Adafruit_TinyUSB_Arduino
hid_gamepad_report_t gp;

Bounce debouncers[numOfButtons];
int bootmode = BOOT_MODE_JOYSTICK;
int16_t currX = 0;
int16_t currY = 0;
signed char currDPAD = 0;
static bool sendReport;
long lastReportTime;
long lastBatteryCheckTime = -1000 * 60 * 5;
long lastblink = 0;
bool firstConnected;
bool ledon = false;

void setup() {
  Serial.begin(115200);

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while (!Serial) delay(10);
#endif

  Serial.println("Bluefruit52 HID Gamepad Example");
  Serial.println("-------------------------------\n");
  lastReportTime = millis();
  lastblink = millis();
  Serial.println("BOOTING...");
  boot_check_mode();
  Serial.println("INIT pins...");
  for (byte currentPinIndex = 0; currentPinIndex < numOfButtons; currentPinIndex++) {
    pinMode(buttonPins[currentPinIndex], INPUT_PULLUP);
    debouncers[currentPinIndex] = Bounce();
    debouncers[currentPinIndex].attach(buttonPins[currentPinIndex]);  // After setting up the button, setup the Bounce instance :
    debouncers[currentPinIndex].interval(5);
  }
  Serial.println("INIT pins done");
  Serial.println("Go to your phone's Bluetooth settings to pair your device");
  Serial.println("then open an application that accepts gamepad input");

  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setModel("nRF52 GAMEPAD",13);
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  /* Start BLE HID
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEHidAdafruit::begin() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blegamepad.begin();

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms
   */
  /* Bluefruit.Periph.setConnInterval(9, 12); */

  // Set up and start advertising
  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blegamepad);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

void loop() {
  delay(30);
  checkToSleep();
  checkBattery();
  // nothing to do if not connected or
  if (!Bluefruit.connected()) {
    delay(20);
    blink();
    firstConnected = true;
    return;
  }

  if (firstConnected) {
    firstConnected = false;
    noblink();
    Serial.println("CONNECTED.");
    gp.x = AXES_CENTER;
    gp.y = AXES_CENTER;
    gp.z = AXES_CENTER;
    gp.rz = AXES_CENTER;
    gp.rx = AXES_CENTER;
    gp.ry = AXES_CENTER;
    gp.hat = GAMEPAD_HAT_CENTERED;
    gp.buttons = 0;
    blegamepad.report(&gp);
    return;
  }
  // scan and debounce
  for (byte i = 0; i < numOfButtons; i++) {
    debouncers[i].update();
    if (debouncers[i].fell()) {
      buttonStatus[i] = 1;
    } else if (debouncers[i].rose()) {
      buttonStatus[i] = 0;
    }
  }
  sendReport = false;
  for (byte i = 0; i < numOfButtons; i++) {
    // key changed
    if (lastButtonStatus[i] != buttonStatus[i]) {
      sendReport = true;
      // copy value
      lastButtonStatus[i] = buttonStatus[i];
      if (i > 3) {
        if (buttonStatus[i] == 1) {
          bitSet(gp.buttons, i - 4);
        } else {
          bitClear(gp.buttons, i - 4);
        }
      }
    }
  }
  if (sendReport) {
    if (isJoystickMode()) {
      // axes:
      if (buttonStatus[0] == 0 && buttonStatus[1] == 0) {
        currY = AXES_CENTER;
      }
      if (buttonStatus[2] == 0 && buttonStatus[3] == 0) {
        currX = AXES_CENTER;
      }
      if (buttonStatus[0] == 1 && buttonStatus[1] == 0) {
        currY = AXES_MIN;
      }
      if (buttonStatus[0] == 0 && buttonStatus[1] == 1) {
        currY = AXES_MAX;
      }
      if (buttonStatus[2] == 1 && buttonStatus[3] == 0) {
        currX = AXES_MIN;
      }
      if (buttonStatus[2] == 0 && buttonStatus[3] == 1) {
        currX = AXES_MAX;
      }
      gp.x = currX;
      gp.y = currY;
    } else {
      // axes:
      if (buttonStatus[0] == 0 && buttonStatus[1] == 0 && buttonStatus[2] == 0 && buttonStatus[3] == 0) {
        currDPAD = GAMEPAD_HAT_CENTERED;
      }
      if (buttonStatus[0] == 1 && buttonStatus[1] == 0) {
        currDPAD = GAMEPAD_HAT_UP;
      }
      if (buttonStatus[0] == 0 && buttonStatus[1] == 1) {
        currDPAD = GAMEPAD_HAT_DOWN;
      }
      if (buttonStatus[2] == 1 && buttonStatus[3] == 0) {
        currDPAD = GAMEPAD_HAT_LEFT;
      }
      if (buttonStatus[2] == 0 && buttonStatus[3] == 1) {
        currDPAD = GAMEPAD_HAT_RIGHT;
      }
      if (buttonStatus[0] == 1 && buttonStatus[2] == 1) {
        currDPAD = GAMEPAD_HAT_UP_LEFT;
      }
      if (buttonStatus[0] == 1 && buttonStatus[3] == 1) {
        currDPAD = GAMEPAD_HAT_UP_RIGHT;
      }
      if (buttonStatus[1] == 1 && buttonStatus[2] == 1) {
        currDPAD = GAMEPAD_HAT_DOWN_LEFT;
      }
      if (buttonStatus[1] == 1 && buttonStatus[3] == 1) {
        currDPAD = GAMEPAD_HAT_DOWN_RIGHT;
      }
      gp.hat = currDPAD;
    }
  }

  if (sendReport) {
    lastReportTime = millis();
    blegamepad.report(&gp);
  }
}

void checkBattery() {
  if ((millis() - lastBatteryCheckTime) > 1000 * 60 * 1) {
    Serial.println("Checking battery...");
    lastBatteryCheckTime = millis();

    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);

    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12);  // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);

    // Get the raw 12-bit, 0..3000mV ADC value
    float raw = analogRead(PIN_BAT);

    // Set the ADC back to the default settings
    analogReference(AR_DEFAULT);
    analogReadResolution(10);

    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    float mvotes = raw * 2 * 3000.0f / 4096.0f;
    int percents = map(mvotes, 2000, 3700, 0, 100);
    if (percents > 100) percents = 100;
    Serial.print("BAT:[");
    Serial.print(mvotes);
    Serial.print("]mV, ");
    Serial.print(percents);
    Serial.println("%");
    blebas.write(percents);
  }
}


void checkToSleep() {
  if ((millis() - lastReportTime) > 1000 * 60 * 5) {
    Serial.println("SLEEPING...");
    lastReportTime = millis();
    nrf_gpio_cfg_sense_input(PIN_KEY_1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    sd_power_system_off();
  }
}


void blink() {
  if (millis() - lastblink >= 1000) {
    pinMode(PIN_LED, OUTPUT);
    digitalToggle(PIN_LED);
    lastblink = millis();
  }
}

void noblink() {
  analogWrite(PIN_LED, 10);
}


void boot_check_mode() {
  pinMode(BOOT_MODE_PIN, INPUT_PULLUP);
  int buttonState = digitalRead(BOOT_MODE_PIN);
  if (buttonState == LOW) {
    delay(5);
    buttonState = digitalRead(BOOT_MODE_PIN);
    if (buttonState == LOW) {
      bootmode = BOOT_MODE_DPAD;
      Serial.println("BOOT_MODE_DPAD");
    }
  } else {
    Serial.println("BOOT_MODE_JOYSTICK");
  }
}

bool isJoystickMode() {
  return bootmode == BOOT_MODE_JOYSTICK;
}