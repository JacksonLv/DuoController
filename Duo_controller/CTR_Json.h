#ifndef CTR_JSON_H_
#define CTR_JSON_H_

#include "application.h"


/******************************************************
 *               Variable Definitions
 ******************************************************/


extern char json_rx_cmd[];
extern uint8_t json_rx_pin;
extern uint8_t json_rx_pinmode;
extern uint8_t json_rx_data;
extern uint8_t json_rx_len;
extern char json_rx_string[255];
extern char *json_tx_string;

extern char* parseJson(char *jsonString);

/******************************************************
 *               function definitions
 ******************************************************/
extern int creat_send_Json(byte pin,byte data);


#endif



