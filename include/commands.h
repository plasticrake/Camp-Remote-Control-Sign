#include <Arduino.h>

enum Commands : uint8_t { NONE = 0,
                          PATTERN_ONE = 1,
                          PATTERN_TWO = 2,
                          PATTERN_THREE = 3,
                          PATTERN_FOUR = 4,
                          PATTERN_FIVE = 5,
                          PATTERN_SIX = 6,
                          PATTERN_SEVEN = 7,
                          PATTERN_EIGHT = 8,
                          PATTERN_NINE = 9,
                          PATTERN_TEN = 10,
                          BRIGHTNESS_UP = 101,
                          BRIGHTNESS_DOWN = 104,
                          PALETTE_DOWN = 107,
                          PALETTE_UP = 109,
                          ADD_RED = 102,
                          ADD_GREEN = 105,
                          ADD_BLUE = 108,
                          SPEED_UP = 103,
                          SPEED_DOWN = 106,
                          RESET = 210,
                          REMOTE_CONNECTED = 254,
                          HEARTBEAT = 255 };

enum ButtonId : uint8_t { NO_BUTTON = 0,
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

constexpr uint8_t CLICKED = 0;
constexpr uint8_t HELD = 100;
constexpr uint8_t BONUS_CLICKED = 200;

uint8_t buttonToCommand[255];

void initializeButtonToCommand() {
  buttonToCommand[NONE] = NONE;
  buttonToCommand[ONE] = PATTERN_ONE;
  buttonToCommand[TWO] = PATTERN_TWO;
  buttonToCommand[THREE] = PATTERN_THREE;
  buttonToCommand[FOUR] = PATTERN_FOUR;
  buttonToCommand[FIVE] = PATTERN_FIVE;
  buttonToCommand[SIX] = PATTERN_SIX;
  buttonToCommand[SEVEN] = PATTERN_SEVEN;
  buttonToCommand[EIGHT] = PATTERN_EIGHT;
  buttonToCommand[NINE] = PATTERN_NINE;
  buttonToCommand[TEN] = PATTERN_TEN;
  buttonToCommand[ONE + BONUS_CLICKED] = BRIGHTNESS_UP;
  buttonToCommand[TWO + BONUS_CLICKED] = ADD_RED;
  buttonToCommand[THREE + BONUS_CLICKED] = SPEED_UP;
  buttonToCommand[FOUR + BONUS_CLICKED] = BRIGHTNESS_DOWN;
  buttonToCommand[FIVE + BONUS_CLICKED] = ADD_GREEN;
  buttonToCommand[SIX + BONUS_CLICKED] = SPEED_DOWN;
  buttonToCommand[SEVEN + BONUS_CLICKED] = PALETTE_DOWN;
  buttonToCommand[EIGHT + BONUS_CLICKED] = ADD_BLUE;
  buttonToCommand[NINE + BONUS_CLICKED] = PALETTE_UP;
  buttonToCommand[TEN + BONUS_CLICKED] = RESET;

  buttonToCommand[ONE + HELD] = BRIGHTNESS_UP;
  buttonToCommand[TWO + HELD] = ADD_RED;
  buttonToCommand[THREE + HELD] = SPEED_UP;
  buttonToCommand[FOUR + HELD] = BRIGHTNESS_DOWN;
  buttonToCommand[FIVE + HELD] = ADD_GREEN;
  buttonToCommand[SIX + HELD] = SPEED_DOWN;
  buttonToCommand[SEVEN + HELD] = PALETTE_DOWN;
  buttonToCommand[EIGHT + HELD] = ADD_BLUE;
  buttonToCommand[NINE + HELD] = PALETTE_UP;
  buttonToCommand[TEN + HELD] = RESET;
}

// const uint8_t buttonToCommand[] = {
//   [NONE] = NONE,
//   [ONE] = PATTERN_ONE,
//   [TWO] = PATTERN_TWO,
//   [THREE] = PATTERN_THREE,
//   [FOUR] = PATTERN_FOUR,
//   [FIVE] = PATTERN_FIVE,
//   [SIX] = PATTERN_SIX,
//   [SEVEN] = PATTERN_SEVEN,
//   [EIGHT] = PATTERN_EIGHT,
//   [NINE] = PATTERN_NINE,
//   [TEN] = PATTERN_TEN,

//   [ONE + BONUS_CLICKED] = BRIGHTNESS_UP,
//   [TWO + BONUS_CLICKED] = ADD_RED,
//   [THREE + BONUS_CLICKED] = BRIGHTNESS_UP,
//   [FOUR + BONUS_CLICKED] = BRIGHTNESS_DOWN,
//   [FIVE + BONUS_CLICKED] = ADD_GREEN,
//   [SIX + BONUS_CLICKED] = BRIGHTNESS_UP,
//   [SEVEN + BONUS_CLICKED] = PALETTE_DOWN,
//   [EIGHT + BONUS_CLICKED] = ADD_BLUE,
//   [NINE + BONUS_CLICKED] = PALETTE_UP,
//   [TEN + BONUS_CLICKED] = RESET,

//   [ONE + HELD] = BRIGHTNESS_UP,
//   [TWO + HELD] = ADD_RED,
//   [THREE + HELD] = BRIGHTNESS_UP,
//   [FOUR + HELD] = BRIGHTNESS_DOWN,
//   [FIVE + HELD] = ADD_GREEN,
//   [SIX + HELD] = BRIGHTNESS_UP,
//   [SEVEN + HELD] = PALETTE_DOWN,
//   [EIGHT + HELD] = ADD_BLUE,
//   [NINE + HELD] = PALETTE_UP,
//   [TEN + HELD] = RESET
//   };
