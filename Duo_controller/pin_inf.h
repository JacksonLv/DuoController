#ifndef PIN_INF_H_
#define PIN_INF_H_

#include "Arduino.h"
/******************************************************
 *                      Macros
 ******************************************************/

#define blue_led_pin        D7

#define PIN_CAPABILITY_NONE      0x00
#define PIN_CAPABILITY_DIGITAL   0x01
#define PIN_CAPABILITY_ANALOG    0x02
#define PIN_CAPABILITY_PWM       0x04
#define PIN_CAPABILITY_SERVO     0x08
#define PIN_CAPABILITY_I2C       0x10


// pin modes
//#define INPUT                 0x00 // defined in wiring.h
//#define OUTPUT                0x01 // defined in wiring.h
#define ANALOG                  0x02 // analog pin in analogInput mode
#define PWM                     0x03 // digital pin in PWM output mode
#define SERVO                   0x04 // digital pin in Servo output mode
#define DEFAULT_MODE            0xFF // Default is input

// Normally Servo.h must be included before Firmata.h (which then includes
// this file).  If Servo.h wasn't included, this allows the code to still
// compile, but without support for any Servos.  Hopefully that's what the
// user intended by not including Servo.h
#ifndef MAX_SERVOS
#define MAX_SERVOS 0
#endif

#ifndef digitalPinHasPWM
#define digitalPinHasPWM(p)     IS_PIN_DIGITAL(p)
#endif

#define TOTAL_PINS_NUM    18
#define VERSION_BLINK_PIN 7

#define IS_PIN_DIGITAL(p) ( (p) >= 0 && (p) < 18 )
#define IS_PIN_ANALOG(p)  ( (p) >= 8 && (p) < 16 )
#define IS_PIN_PWM(p)     ( ( (p) >= 0 && (p) < 5 ) || (p) == 8 || (p) == 9 || ( (p) >= 14 && (p) < 18 ) )
#define IS_PIN_SERVO(p)   ( (p) == 12 || (p) == 13 )

#define PIN_TO_DIGITAL(p) (p)
#define PIN_TO_ANALOG(p)  (p)
#define PIN_TO_PWM(p)     (p)
#define PIN_TO_SERVO(p)   (p)


/******************************************************
 *               Variable Definitions
 ******************************************************/
//for Controller
extern byte pins_mode[TOTAL_PINS_NUM];
extern int pins_state[TOTAL_PINS_NUM];
extern byte pins_pwm[TOTAL_PINS_NUM];
extern byte pins_servo[TOTAL_PINS_NUM];



#endif


