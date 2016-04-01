#ifndef WIFI_CTR_H_
#define WIFI_CTR_H_

#include "application.h"


/******************************************************
 *                      Macros
 ******************************************************/
#define HTTP_PORT 80

#define MAX_CLIENT_NUM          3


/******************************************************
 *               Variable Definitions
 ******************************************************/
// Server Port
extern TCPServer server;
extern TCPClient client[MAX_CLIENT_NUM];

extern Servo servos[TOTAL_PINS_NUM];

extern uint8_t duo_pin[TOTAL_PINS_NUM];

extern uint8_t wifi_status_check_flag;

/******************************************************
 *               function definitions
 ******************************************************/
extern void WiFireportPinServoData(byte pin);
extern void WiFireportPinDigitalData(byte pin);
extern void WiFireportPinPWMData(byte pin);
extern byte WiFireportDigitalInput();
extern void WiFireportPinAnalogData();

extern void wifiToDuo(uint8_t client_num);



#endif 
