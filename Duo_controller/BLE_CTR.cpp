#include "pin_inf.h"
#include <ArduinoJson.h>
#include "WiFi_CTR.h"
#include "BLE_CTR.h"

/******************************************************
 *               Variable Definitions
 ******************************************************/

btstack_timer_source_t pin_status_back;

uint8_t reback_pin  = 0;
uint8_t queryDone = false;
uint8_t ble_status_check_flag = 0;
uint8_t ble_connected = 0;

byte pins_mode[TOTAL_PINS_NUM];
int pins_state[TOTAL_PINS_NUM];
byte pins_pwm[TOTAL_PINS_NUM];
byte pins_servo[TOTAL_PINS_NUM];


/******************************************************
 *               BLE Variable Definitions
 ******************************************************/
static uint8_t service1_uuid[16]       ={0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e};
static uint8_t service1_tx_uuid[16]    ={0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e};
static uint8_t service1_rx_uuid[16]    ={0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e};

static uint8_t  appearance[2]    = {0x00, 0x02};
static uint8_t  change[2]        = {0x00, 0x00};
static uint8_t  conn_param[8]    = {0x28, 0x00, 0x90, 0x01, 0x00, 0x00, 0x90, 0x01};

static uint16_t character1_handle = 0x0000;
static uint16_t character2_handle = 0x0000;
static uint16_t character3_handle = 0x0000;

static uint8_t characteristic1_data[TXRX_BUF_LEN]={0x01};
static uint8_t characteristic2_data[TXRX_BUF_LEN]={0x00};

static advParams_t adv_params;

static uint8_t adv_data[]={0x02,0x01,0x06,0x08,0x08,'B','i','s','c','u','i','t',0x11,0x07,0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71};



void deviceConnectedCallback(BLEStatus_t status, uint16_t handle) {
    switch (status){
        case BLE_STATUS_OK:
            ble_connected = 1;
            Serial.println("BLE connected!");
            Serial.println("client stop");
            for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
            {
                client[MAX_CLIENT_NUM].stop();
            }
//            Serial.println("cloud disconnet");
//            Particle.disconnect();
            break;
        default:
            break;
    }
}

void deviceDisconnectedCallback(uint16_t handle){
    ble_connected = 0;
    ble_status_check_flag = 0;
    Serial.println("BLE Disconnected.");
    Serial.println("client creat");
    //client.connect();
    TCPClient client[MAX_CLIENT_NUM];
//    Serial.println("cloud connect");
//    Particle.connect();
}

void blereportPinDigitalData(byte pin)
{
    //Serial.println("reportPinDigitalData");
    byte state = pins_state[pin] ;
    byte mode = pins_mode[pin];
    byte buf[] = {'G', pin, mode, state};
    memcpy(characteristic2_data, buf, 4);
    ble.sendNotify(character2_handle, characteristic2_data, 4);
}

void blereportPinPWMData(byte pin)
{
    //Serial.println("reportPinPWMData");
    byte value = pins_pwm[pin];
    byte mode = pins_mode[pin];
    byte buf[] = {'G', pin, mode, value};
    memcpy(characteristic2_data, buf, min(4,TXRX_BUF_LEN));
    ble.sendNotify(character2_handle, characteristic2_data, min(4,TXRX_BUF_LEN));
}

void blereportPinServoData(byte pin)
{
    //Serial.println("reportPinServoData");
    byte value = pins_servo[pin];
    byte mode = pins_mode[pin];
    byte buf[] = {'G', pin, mode, value};
    memcpy(characteristic2_data, buf, min(4,TXRX_BUF_LEN));
    ble.sendNotify(character2_handle, characteristic2_data, min(4,TXRX_BUF_LEN));
}

void blereportPinCapability(byte pin)
{
    //Serial.println("reportPinCapability");
    byte buf[] = {'P', pin, 0x00};
    byte pin_cap = 0;

    if (IS_PIN_DIGITAL(pin))
        pin_cap |= PIN_CAPABILITY_DIGITAL;

    if (IS_PIN_ANALOG(pin))
        pin_cap |= PIN_CAPABILITY_ANALOG;

    if (IS_PIN_PWM(pin))
        pin_cap |= PIN_CAPABILITY_PWM;

    if (IS_PIN_SERVO(pin))
        pin_cap |= PIN_CAPABILITY_SERVO;

    buf[2] = pin_cap;
    //Serial.print("report: ");
    //for(uint8_t index=0; index<3; index++)
    //{
    //    Serial.print(buf[index], HEX);
    //    Serial.print(" ");
    //}
    //Serial.println(" ");
    memcpy(characteristic2_data, buf, 3);
    ble.sendNotify(character2_handle, characteristic2_data, 3);
}

void blesendCustomData(uint8_t *buf, uint8_t len)
{
    //Serial.println("sendCustomData");
    uint8_t data[20] = "Z";

    memcpy(&data[1], buf, len);
    memcpy(characteristic2_data, data, len+1 );
    ble.sendNotify(character2_handle, characteristic2_data, len+1);
}

byte blereportDigitalInput()
{
    static byte pin = 0;
    byte report = 0;

    if (!IS_PIN_DIGITAL(pin))
    {
        pin++;
        if (pin >= TOTAL_PINS_NUM)
            pin = 0;
        return 0;
    }

    if (pins_mode[pin] == INPUT)
    {
        byte current_state = digitalRead(duo_pin[pin]);

        if (pins_state[pin] != current_state)
        {
            pins_state[pin] = current_state;
            byte buf[] = {'G', pin, INPUT, current_state};
            memcpy(characteristic2_data, buf, 4);
            ble.sendNotify(character2_handle, characteristic2_data, 4);

            report = 1;
        }
    }
    pin++;
    if (pin >= TOTAL_PINS_NUM)
        pin = 0;

    return report;
}

byte blereportPinAnalogData()
{
    static byte pin = 0;
    byte report = 0;

    if (!IS_PIN_DIGITAL(pin))
    {
        pin++;
        if (pin >= TOTAL_PINS_NUM)
            pin = 0;
        return 0;
    }

    if (pins_mode[pin] == ANALOG)
    {
        uint16_t value = analogRead(duo_pin[pin]);
        byte value_lo = value;
        byte value_hi = value>>8;

        byte buf[] = {'G', pin, pins_mode[pin],value_hi, value_lo};
        memcpy(characteristic2_data, buf, 5);
        ble.sendNotify(character2_handle, characteristic2_data,5);

        report = 1;
    }

    pin++;
    if (pin >= TOTAL_PINS_NUM)
        pin = 0;

    return report;
}

void pin_status_back_handle(btstack_timer_source_t *ts)
{
    if(reback_pin < TOTAL_PINS_NUM)
    {
        blereportPinCapability(reback_pin);
        if ( (pins_mode[reback_pin] == INPUT) || (pins_mode[reback_pin] == OUTPUT) || (pins_mode[reback_pin] == DEFAULT_MODE))
            blereportPinDigitalData(reback_pin);
        else if (pins_mode[reback_pin] == PWM)
            blereportPinPWMData(reback_pin);
        else if (pins_mode[reback_pin] == SERVO)
            blereportPinServoData(reback_pin);

        reback_pin++;
        // reset
        ble.setTimer(ts, 100);
        ble.addTimer(ts);
    }
    else
    {
        queryDone = true;
        uint8_t str[] = "ABC";
        blesendCustomData(str, 3);
        
        pin_status_back.process = NULL;
        ble.removeTimer(&pin_status_back);
    }  
}



int gattWriteCallback(uint16_t value_handle, uint8_t *buf, uint16_t size)
{
    uint16_t index;
    
    if(character1_handle == value_handle)
    {
        memcpy(characteristic1_data, buf, size);
        Serial.print("value: ");
        for(uint8_t index=0; index<size; index++)
        {
            Serial.print(characteristic1_data[index], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");
        //Process the data
        switch(buf[0])
        {
            case 'V': //query protocol version
            {
                byte buf_tx[] = {'V', 0x00, 0x00, 0x01};
                memcpy(characteristic2_data, buf_tx, 4);
                ble.sendNotify(character2_handle, characteristic2_data, 4);   
                break;     
            }   
            case 'C': // query board total pin count
            {
                byte buf_tx[2] = {'C', TOTAL_PINS_NUM};
                memcpy(characteristic2_data, buf_tx, 2);
                ble.sendNotify(character2_handle, characteristic2_data, 2);   
                break;                  
            }
            case 'M': // query pin mode
            {
                byte buf_tx[] = {'M', buf[1], pins_mode[ buf[2] ]};
                memcpy(characteristic2_data, buf_tx, 3);
                ble.sendNotify(character2_handle, characteristic2_data, 3);  
                break;                  
            }
            case 'S': // query pin mode
            {
                byte pin = buf[1];
                byte mode = buf[2];
                if ( IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached() )
                    servos[PIN_TO_SERVO(pin)].detach();
                /* ToDo: check the mode is in its capability or not */
                /* assume always ok */
                if(mode != pins_mode[pin])
                {   
                    pins_mode[pin] = mode;
                    if (mode == OUTPUT)
                    {
                        pinMode(duo_pin[pin], OUTPUT);
                        digitalWrite(duo_pin[pin], LOW);
                        pins_state[pin] = LOW;
                    }
                    else if (mode == INPUT)
                    {
                        pinMode(duo_pin[pin], INPUT_PULLDOWN);
                        pins_state[pin] = LOW;
                    }
                    else if (mode == ANALOG)
                    {
                        if (IS_PIN_ANALOG(pin))
                        {
                            //pinMode(duo_pin[pin], AN_INPUT);
                        }
                    }
                    else if (mode == PWM)
                    {
                        if (IS_PIN_PWM(pin))
                        {
                            pinMode(duo_pin[PIN_TO_PWM(pin)], OUTPUT);
                            analogWrite(duo_pin[PIN_TO_PWM(pin)], 0);
                            pins_pwm[pin] = 0;
                            pins_mode[pin] = PWM;
                        }
                    }
                    else if (mode == SERVO)
                    {
                        if (IS_PIN_SERVO(pin))
                        {
                            pins_servo[pin] = 0;
                            pins_mode[pin] = SERVO;
                            if (!servos[PIN_TO_SERVO(pin)].attached())
                                servos[PIN_TO_SERVO(pin)].attach(duo_pin[PIN_TO_DIGITAL(pin)]);
                        }
                    }
                    else if(mode == DEFAULT_MODE)
                    {
                      pinMode(duo_pin[pin], INPUT_PULLDOWN);            
                      pins_state[pin] = LOW;
                    }
                }
                //if (mode == ANALOG)
                   //reportPinAnalogData(pin);
                if ( (mode == INPUT) || (mode == OUTPUT)|| (mode == DEFAULT_MODE))
                {
                    blereportPinDigitalData(pin);
                }
                else if (mode == PWM)
                {
                    blereportPinPWMData(pin);
                }
                else if (mode == SERVO)
                {
                    blereportPinServoData(pin);
                }
                break;                  
            }
            case 'G': // query pin data
            {
                byte pin = buf[1];
                blereportPinDigitalData(pin);
                break;              
            }
            case 'T': // set pin digital state
            {
                byte pin = buf[1];
                byte state = buf[2];
                digitalWrite(duo_pin[pin], state);
                pins_state[pin] = state;
                blereportPinDigitalData(pin);
                break;              
            }
            case 'N': // set PWM
            {
                byte pin = buf[1];
                byte value = buf[2];

                analogWrite(duo_pin[PIN_TO_PWM(pin)], value);
                pins_pwm[pin] = value;
                blereportPinPWMData(pin);
                break;                  
            }
            case 'O': // set Servo
            {
                byte pin = buf[1];
                byte value = buf[2];

                if (IS_PIN_SERVO(pin))
                    servos[PIN_TO_SERVO(pin)].write(value);
                pins_servo[pin] = value;
                blereportPinServoData(pin);
                break;                  
            }
            case 'A': // query all pin status
            {
                reback_pin = 0;
                // set one-shot timer
                pin_status_back.process = &pin_status_back_handle;
                ble.setTimer(&pin_status_back, 100);//2000ms
                ble.addTimer(&pin_status_back);
                break;
            }
            case 'P': // query pin capability
            {
                byte pin = buf[1];
                blereportPinCapability(pin);  
                break;
            }
            case 'Z':
            {
                byte len = buf[1];
                Serial.println("->");
                Serial.print("Received: ");
                Serial.print(len);
                Serial.println(" byte(s)");
                Serial.print(" Hex: ");
                for (int i=0;i<len;i++)
                  Serial.print(buf[i+2], HEX);
                Serial.println();
                break;
            }
        }
    }
    ble_status_check_flag = 1;
}



/*******************************************************************************
 * Function Name  : ble_set
 * Description    : setting the ble
 * Input          : None
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void ble_set()
{
    ble.init();

    ble.onConnectedCallback(deviceConnectedCallback);
    ble.onDisconnectedCallback(deviceDisconnectedCallback);
    ble.onDataWriteCallback(gattWriteCallback);

    ble.addService(0x1800);
    ble.addCharacteristic(0x2A00, ATT_PROPERTY_READ|ATT_PROPERTY_WRITE, (uint8_t*)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.addCharacteristic(0x2A01, ATT_PROPERTY_READ, appearance, sizeof(appearance));
    ble.addCharacteristic(0x2A04, ATT_PROPERTY_READ, conn_param, sizeof(conn_param));
    ble.addService(0x1801);
    ble.addCharacteristic(0x2A05, ATT_PROPERTY_INDICATE, change, sizeof(change));

    ble.addService(service1_uuid);
    character1_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, characteristic1_data, TXRX_BUF_LEN);
    character2_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, characteristic2_data, TXRX_BUF_LEN);

    le_connection_parameter_range_t range;
    range.le_conn_interval_min = 20;
    range.le_conn_interval_max = 100;
    range.le_conn_latency_min  = 0;
    range.le_conn_latency_max  = 0;
    range.le_supervision_timeout_min = 10;
    range.le_supervision_timeout_max = 3200;
    ble.setConnParams(range);
    
    adv_params.adv_int_min = 0x00A0;
    adv_params.adv_int_max = 0x01A0;
    adv_params.adv_type    = 0;
    adv_params.dir_addr_type = 0;
    memset(adv_params.dir_addr,0,6);
    adv_params.channel_map = 0x07;
    adv_params.filter_policy = 0x00;
    
    ble.setAdvParams(&adv_params);
    
    ble.setAdvData(sizeof(adv_data), adv_data);

    ble.startAdvertising();   

    
    
    Serial.println("BLE start advertising."); 
}


