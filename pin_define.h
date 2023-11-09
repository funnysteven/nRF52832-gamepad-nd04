#define PIN_UP 22
#define PIN_DOWN 24
#define PIN_LEFT 23
#define PIN_RIGHT 25
#define PIN_L_THUMB 28
#define PIN_LED 30
#define PIN_BAT 29
#define PIN_KEY_1 4
#define PIN_KEY_2 5
#define PIN_KEY_3 10
#define PIN_KEY_4 9
#define PIN_KEY_5 7
#define PIN_KEY_6 13
#define PIN_KEY_7 11
#define PIN_KEY_8 12
#define PIN_KEY_9 20
#define PIN_KEY_10 19
#define PIN_KEY_11 18
#define PIN_KEY_12 17
#define PIN_KEY_13 16
#define PIN_KEY_14 15
#define PIN_KEY_15 14

#define AXES_MIN -127
#define AXES_MAX 127
#define AXES_CENTER 0


#define BOOT_MODE_JOYSTICK 0
#define BOOT_MODE_DPAD 1
//press UP key during boot, to enter dpad mode
#define BOOT_MODE_PIN 22


#define numOfButtons 20
byte buttonPins[numOfButtons] = { PIN_UP, PIN_DOWN, PIN_LEFT, PIN_RIGHT, PIN_KEY_1, PIN_KEY_2, PIN_KEY_3, PIN_KEY_4, PIN_KEY_5, PIN_KEY_6, PIN_KEY_7, PIN_KEY_8, PIN_KEY_9, PIN_KEY_10, PIN_KEY_11, PIN_KEY_12, PIN_KEY_13, PIN_KEY_14, PIN_KEY_15, PIN_L_THUMB };
byte buttonStatus[numOfButtons] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte lastButtonStatus[numOfButtons] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };