#include <Arduino.h>

// DIGITAL PIN

#define PIN_STEP_A       4      // PIN STEP FOR X
#define PIN_DIR_A        5      // PIN DIRECTION FOR X 
#define PIN_STEP_B       6      // PIN STEP FOR X
#define PIN_DIR_B        7      // PIN DIRECTION FOR X 
#define PIN_ENABLE_AB    9 

#define PIN_ENABLE_X     43
#define PIN_STEP_X       51      // PIN STEP FOR X        //26
#define PIN_DIRECTION_X  53      // PIN DIRECTION FOR X   //24 
#define PIN_X_MS_1       45
#define PIN_X_MS_2       47
#define PIN_X_MS_3       49

#define PIN_ENABLE_Y     31     
#define PIN_STEP_Y       39       // PIN STEP FOR Y       // 4
#define PIN_DIRECTION_Y  41       // PIN DIRECTION FOR Y  // 3
#define PIN_Y_MS_1       33
#define PIN_Y_MS_2       35
#define PIN_Y_MS_3       37

#define PIN_ENABLE_Z     23
#define PIN_STEP_Z       27      // PIN STEP FOR Y
#define PIN_DIRECTION_Z  29       // PIN DIRECTION FOR Y
#define PIN_Z_MS_1       25
#define Z_POSITION       22

#define BP_LEFT          52
#define BP_RIGH          50

#define LED_1            48
#define LED_2            46
#define LED_3            44
#define LED_4            42
#define LED_5            40
#define LED_6            38

#define TRIGGER_LIGHT    24      // PIN TRIGGER FOR START/STOP THE LIGHTING INSIDE THE BALL
#define LIGHTING         26      // PIN FOR FEED THE LIGHTING

#define POWER_1          28
#define POWER_2          30
#define AUX_1			 32
#define AUX_2            34
#define AUX_3            36

#define BUZZER           3

#define PING             13
#define PIN_RESET        12

// ANALOG PIN
#define CURRENT          0
#define VOLTAGE          1
#define BATT_1           2
#define BATT_2           3
#define BATT_3           4
#define BATT_4           5
#define TEMP_BALL        6
#define TEMP_A4988       15
