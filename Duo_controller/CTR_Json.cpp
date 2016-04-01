#include "pin_inf.h"
#include <ArduinoJson.h>
#include "CTR_Json.h"
#include "WiFi_CTR.h"


char json_rx_cmd[2] = "1";
uint8_t json_rx_pin = 255;
uint8_t json_rx_pinmode = 255;
uint8_t json_rx_data = 255;
uint8_t json_rx_len = 0;
uint8_t z_data[255] = {}; 
char json_rx_string[255];
char *json_tx_string = "1";


/**
 * Parse the JSON String. Uses aJson library
 *
 * Refer to http://hardwarefun.com/tutorials/parsing-json-in-arduino
 */
char* parseJson(char *jsonString)
{
//      if (len->valueint < 100000000)
//      {
//        memset(z_data, 0, sizeof(z_data));
//        json_rx_len = len->valueint;
//        Serial.println("Parsed successfully 3 " );
//        aJsonObject *data = aJson.getObjectItem(root, "data");
//        for (uint8_t i = 0; i < json_rx_len; i++)
//        {
//          aJsonObject *zdata_temp = aJson.getArrayItem(data, i);
//          //Serial.print("zdata_temp->valueint  ");
//          //Serial.println(zdata_temp->valueint);
//          z_data[i] = zdata_temp->valueint;
//        }
//
//      }
//    }
//  }
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(jsonString);
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return 0;
  }
  const char *cmd = root["cmd"];
  long pin = root["pin"];
  long pinmode = root["pinMode"];
  long data = root["data"];

  strcpy(json_rx_cmd,cmd);
  json_rx_pin = pin;
  json_rx_pinmode = pinmode;
  json_rx_data = data;
  
  Serial.print("json_rx_cmd  ");
  Serial.println(json_rx_cmd  );
  Serial.print("json_rx_pin  ");
  Serial.println(json_rx_pin);
  Serial.println("json_rx_pinmode  ");
  Serial.println(json_rx_pinmode);
  Serial.println("json_rx_data  ");
  Serial.println(json_rx_data);

}

/*******************************************************************************
 * Function Name  : creat_send_Json
 * Description    : creat a Json include cmd,pin,pinmode and data
 * Input          : Pin,data,client_num
 * Output         : Json to apps.
 * Return         : 1
 *******************************************************************************/
int creat_send_Json(byte pin,byte data)
{
  String cloud_json;
  char json_buffer[60];
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject(); 
  root["cmd"] = "G";
  root["pin"] = pin; 
  root["pinMode"] = pins_mode[pin];
  root["data"] = data;
  root.printTo(json_buffer,60);
  Serial.print("json_buffer:  ");
  //root.printTo(Serial);
  Serial.println(json_buffer);
  for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
  {
      if (client[i].connected())
      {         
         //root.printTo(client[i]); 
         client[i].print(json_buffer);
         client[i].flush();
         delay(2); 
         client[i].println();
      }
  }
  Particle.variable("mess",json_buffer,STRING);
  return 1;
}
