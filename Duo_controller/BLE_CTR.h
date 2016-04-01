#ifndef _BLE_CTR_H_
#define _BLE_CTR_H_

#include "application.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define TXRX_BUF_LEN               8     

#define DEVICE_NAME                "BLE Controller"

#define DEVICE_ID_ADDR (0x1FFF7A10)
#define DEVICE_ID_LEN  12

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern uint8_t ble_status_check_flag;
extern uint8_t ble_connected;
extern uint8_t ble_stop_adv;
extern uint8_t ble_start_adv;


/******************************************************
 *               function definitions
 ******************************************************/
extern void blereportPinServoData();
extern void blereportPinDigitalData();
extern byte blereportPinPWMData();
extern byte blereportPinAnalogData();

extern void pin_status_back_handle(btstack_timer_source_t *ts);
extern byte blereportDigitalInput();

extern void ble_set();


#endif

