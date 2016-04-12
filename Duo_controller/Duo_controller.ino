#include "pin_inf.h"
#include "MDNS.h"
#include "CTR_Json.h"
#include "WiFi_CTR.h"
#include "BLE_CTR.h"
#include "cloud_CTR.h"

/******************************************************
 *                      Macros
 ******************************************************/
 
#if defined(ARDUINO) 
SYSTEM_MODE(MANUAL);//do not connect to cloud
#else
SYSTEM_MODE(AUTOMATIC);//connect to cloud
#endif

// Modified the following for your AP/Router.
//#define AP "AP-02_2.4G"
//#define PIN "0098019777"

/******************************************************
 *               Variable Definitions
 ******************************************************/
btstack_timer_source_t status_check;
btstack_timer_source_t mode_check;

MDNS mdns;
String mdns_local_name;

static uint8_t data_temp[]={0};

uint8_t ble_stop_adv = 0;
uint8_t ble_start_adv = 1;


 /******************************************************
 *               Function Definitions
 ******************************************************/
void mdns_init();



static void mode_check_handle(btstack_timer_source_t *ts)
{
  if(ble_connected==0)
  {
      for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
      {
          if((client[i].connected()==true))
          {
            if(ble_stop_adv == 0)
            {
                Serial.println("ble stop adv");
                ble_start_adv = 0;
                ble.stopAdvertising();
            }
            else Serial.println("ble adv stoped");
            ble_stop_adv = 1;
            wifi_status_check_flag = 1;
        //    Serial.println("cloud disconnet");
        //    Particle.disconnect();
            // reset
            ble.setTimer(ts, 1000);
            ble.addTimer(ts);
            return; 
          }        
              
      }
      if(ble_stop_adv==1)
      {
        if(ble_start_adv==0)
        {
            Serial.println("ble start adv");
            wifi_status_check_flag = 0;
            ble.startAdvertising();
        }
        else Serial.println("ble adv started ");
        ble_stop_adv = 0;
        ble_start_adv = 1;
      }
  }
    // reset
    ble.setTimer(ts, 1000);
    ble.addTimer(ts);
}


static void status_check_handle(btstack_timer_source_t *ts)
{
    mdns.processQueries();
    if(ble_connected == 0)
    {
        for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
      {
            if (client[i].connected())
          {           
            //        Serial.println("Connected by TCP client.");
            //        client.println("Hello!");
            wifiToDuo(i);
          }
          else
          {                       
            client[i] = server.available();
          }
      }
    }
    
    if(ble_status_check_flag)
    {
        byte input_data_pending = blereportDigitalInput();
        if(input_data_pending)
        {
            // reset
            ble.setTimer(ts, 23);
            ble.addTimer(ts);
            return;
         }
         blereportPinAnalogData();
    } 
    
    if(wifi_status_check_flag)  
    {
        byte input_data_pending = WiFireportDigitalInput();
        if(input_data_pending)
        {
            // reset
            ble.setTimer(ts, 23);
            ble.addTimer(ts);
            return;
         }
         WiFireportPinAnalogData();
    }
        
    // reset
    ble.setTimer(ts, 23);
    ble.addTimer(ts);
}


/*******************************************************************************
 * Function Name  : mdns_init
 * Description    : init mdns service name,port
 * Input          : None
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void mdns_init()
{
    
    bool success = mdns.setHostname("duo");
     
    if (success) {
        success = mdns.setService("tcp", "duocontrol", 8888, mdns_local_name);
        Serial.println("setService");
    }

    if (success) {
        success = mdns.begin();
        Serial.println("mdns.begin");
    }
    
    if (success) {
        Spark.publish("mdns/setup", "success");
        Serial.println("mdns/setup success");
    } else {
        Spark.publish("mdns/setup", "error");
        Serial.println("mdns/setup error");
    }
}



void setup() {
  
    char addr[16];
    uint8_t dev_id[DEVICE_ID_LEN];
    Serial.begin(115200);
    delay(5000);
    
    ble_set();

    local_name_t local_name;
    HAL_Local_Name(&local_name);

    Serial.print("Local Name: ");
    for(uint8_t i=0; i<local_name.length; i++)
    {
      Serial.write(local_name.value[i]);
    }
    Serial.println("\n");

    
    memcpy(dev_id, (char*)DEVICE_ID_ADDR, DEVICE_ID_LEN);

    Serial.print("Device ID: ");
    for(uint8_t i=0; i<DEVICE_ID_LEN; i++)
    {
      uint8_t c;
      c = (dev_id[i]>>4) + 48;
      if(c>57) c += 39;
      Serial.write(c);
      c = (dev_id[i]&0x0F) + 48;
      if(c>57) c += 39;
      Serial.write(c);
    }
    Serial.println("\n");
    
    Serial.println("Note: If your Duo hasn't stored a valid WiFi profile, it will enter the listening mode for provisioning first.\n");
   
    WiFi.on();
    //WiFi.setCredentials(AP, PIN, WPA2);
    WiFi.connect();
    
    IPAddress localIP = WiFi.localIP();
    
    while (localIP[0] == 0)
    {
        localIP = WiFi.localIP();
        Serial.println("waiting for an IP address");
        delay(1000);
    }
  
    sprintf(addr, "%u.%u.%u.%u", localIP[0], localIP[1], localIP[2], localIP[3]);
    
    Serial.println(addr);
        
    server.begin();
    
    mdns_local_name= (char*)local_name.value;
    mdns_local_name.remove(local_name.length);
    Serial.print("mdns_local_name:  ");
    Serial.println(mdns_local_name);
    
    mdns_init(); 
          
      /* Default all to digital input */
    for (int pin = 0; pin < TOTAL_PINS_NUM; pin++)
    {
        // Set pin to input with internal pull up
        pinMode(duo_pin[pin], INPUT_PULLDOWN);

        // Save pin mode and state
        pins_mode[pin] = DEFAULT_MODE;
        pins_state[pin] = LOW;
    }

    pinMode(blue_led_pin,OUTPUT);
    // Save pin mode and state
    pins_mode[blue_led_pin] = OUTPUT;


    // set one-shot timer
    status_check.process = &status_check_handle;
    ble.setTimer(&status_check, 23);//100ms
    ble.addTimer(&status_check);

    // set mode check timer
    mode_check.process = &mode_check_handle;
    ble.setTimer(&mode_check, 1000);//500ms
    ble.addTimer(&mode_check);
    
    //Register all the cloud functions
    Particle.function("digitalread",tinkerDigitalRead);
    Particle.function("digitalwrite",tinkerDigitalWrite);
    Particle.function("analogread",tinkerAnalogRead);   
    Particle.function("analogwrite", particleToDuo);
    Particle.variable("mess",json_tx_string,STRING);
      
}

void loop() {        
    if((ble_connected==1)||(wifi_status_check_flag == 1))
    {
        digitalWrite(blue_led_pin, 0);
    }
    else 
    {
        digitalWrite(blue_led_pin, 1);
        delay(300);
        digitalWrite(blue_led_pin, 0);
        delay(300);
    }
}





