#include <ArduinoJson.h>
#include "pin_inf.h"
#include "CTR_Json.h"
#include "WiFi_CTR.h"


TCPClient client[MAX_CLIENT_NUM];
TCPServer server = TCPServer(8888);
Servo     servos[TOTAL_PINS_NUM];

void WiFireportPinServoData(byte pin);
void WiFireportPinDigitalData(byte pin);
void WiFireportPinPWMData(byte pin);
void WiFireportPinAnalogData();
void wifiToDuo(uint8_t client_num);


uint8_t wifi_status_check_flag = 0;


static uint8_t rx_data_len = 0;

uint8_t duo_pin[TOTAL_PINS_NUM] = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16, D17};

/*******************************************************************************
 * Function Name  : WiFireportPinServoData
 * Description    : creat a Json for servo and sent to apps
 * Input          : Pin
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void WiFireportPinServoData(byte pin)
{
  creat_send_Json(pin,pins_servo[pin]);
}
/*******************************************************************************
 * Function Name  : WiFireportPinAnalogData
 * Description    : creat a Json for Analog input and sent to apps
 * Input          : 
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void WiFireportPinAnalogData()
{
  static byte pin = 0;
  byte report = 0;
  int value; 
  
  if (!IS_PIN_DIGITAL(duo_pin[pin]))
  {
    pin++;
    if (pin >= TOTAL_PINS_NUM)
      pin = 0;
  }
  
  if (pins_mode[pin] == ANALOG)
  {
    value = analogRead(duo_pin[pin]);

    char json_buffer[60];
    StaticJsonBuffer<500> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject(); 
    root["cmd"] = "G";
    root["pin"] = pin; 
    root["pinMode"] = pins_mode[pin];
    root["data"] = value;
    root.printTo(json_buffer,60);
    Serial.print("Json_tx_string:  ");
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
    report = 1;
  }
  
  pin++;
  if (pin >= TOTAL_PINS_NUM)
    pin = 0;
      
  //Particle.variable("mess",json_tx_string,STRING);
}
/*******************************************************************************
 * Function Name  : WiFireportPinDigitalData
 * Description    : creat a Json for digital input and sent to apps
 * Input          : Pin,
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void WiFireportPinDigitalData(byte pin)
{ 
  creat_send_Json(pin,pins_state[pin]);
}
/*******************************************************************************
 * Function Name  : WiFireportDigitalInput
 * Description    : creat a Json for digital input and sent to apps
 * Input          : 
 * Output         : None.
 * Return         : None
 *******************************************************************************/
byte WiFireportDigitalInput()
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
            WiFireportPinDigitalData(pin);
            report = 1;
        }
    }
    pin++;
    if (pin >= TOTAL_PINS_NUM)
        pin = 0;

    return report;
}
/*******************************************************************************
 * Function Name  : WiFireportPinPWMData
 * Description    : creat a Json for pwm and sent to apps
 * Input          : Pin
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void WiFireportPinPWMData(byte pin)
{  
  creat_send_Json(pin,pins_pwm[pin]);

}


/*******************************************************************************
 * Function Name  : wifiToDuo
 * Description    : app send Json to duo by wifi and duo sent the Json back
 * Input          : client num
 * Output         : Json.
 * Return         : 1
 *******************************************************************************/
void wifiToDuo(uint8_t client_num)
{
  char c ;
  memset(json_rx_string, 0, sizeof(json_rx_string));
  memset(json_rx_cmd, 0, 2);
  json_rx_pin = 255;
  json_rx_pinmode = 255;
  json_rx_data = 255;
  json_rx_len = 0;
  if (client[client_num].available())              // if there's bytes to read from the client,
  {
    rx_data_len = client[client_num].available();
    uint8_t i = 0;

    //cmd = client.read();
    //Serial.write(cmd);
    while ( client[client_num].available())
    {
      c = client[client_num].read();             // read a byte, then
      json_rx_string[i] = c;
      //Serial.println(json_rx_string[i], HEX);
      i++;
    }
    Serial.print("json_rx_string");
    Serial.println(json_rx_string);
    parseJson(json_rx_string);
  }


  // Parse data here
  switch (json_rx_cmd[0])
  {

    case 'V': // query protocol version
      {

        char json_buffer[100];
        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject(); 
        root["cmd"] = "V";
        JsonArray& data = root.createNestedArray("data");
        data.add(0);
        data.add(0);
        data.add(2);
        root.printTo(json_buffer,100);
        Serial.print("Json_tx_string:  ");
        Serial.println(json_buffer);
        for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
        {
            if (client[i].connected())
            {
               client[i].print(json_buffer);
               client[i].flush();
               delay(2); 
               client[i].println();
            }
        }
      }
      break;

    case 'C': // query board total pin count
      {

        char json_buffer[50];
        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject(); 
        root["cmd"] = "V";
        root["data"] = TOTAL_PINS_NUM;
        Serial.print("Json_tx_string:  ");
        root.printTo(json_buffer,50);
        //root.printTo(Serial);
        Serial.println(json_buffer);
        for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
        {
            if (client[i].connected())
            {
               client[i].print(json_buffer);
               client[i].flush();
               delay(2); 
               client[i].println();
            }
        }
      }
      break;

    case 'M': // query pin mode
      {

          char json_buffer[100];
          StaticJsonBuffer<500> jsonBuffer;
          JsonObject& root = jsonBuffer.createObject(); 
          root["cmd"] = "M";
          root["pin"] = json_rx_pin; 
          root["pinMode"] = pins_mode[json_rx_pin];
          Serial.print("Json_tx_string:  ");
          root.printTo(json_buffer,100);
          Serial.println(json_buffer);
          for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
          {
              if (client[i].connected())
              {
                 
                 client[i].print(json_buffer);
                 client[i].flush();
                 delay(2); 
                 client[i].println();
              }
          }
      }
      break;

    case 'S': // set pin mode
      {

        if ( IS_PIN_SERVO(json_rx_pin) && json_rx_pinmode != SERVO && servos[PIN_TO_SERVO(json_rx_pin)].attached() )
           servos[PIN_TO_SERVO(json_rx_pin)].detach();

        /* ToDo: check the mode is in its capability or not */
        /* assume always ok */
        if (json_rx_pinmode != pins_mode[json_rx_pin])
        {

          pins_mode[json_rx_pin] = json_rx_pinmode;

          if (json_rx_pinmode == OUTPUT)
          {
            pinMode(duo_pin[json_rx_pin], OUTPUT);
            digitalWrite(duo_pin[json_rx_pin], LOW);
            pins_state[json_rx_pin] = LOW;
          }
          else if (json_rx_pinmode == INPUT)
          {
            pinMode(duo_pin[json_rx_pin], INPUT_PULLDOWN);
            pins_state[json_rx_pin] = LOW;
          }
          else if (json_rx_pinmode == ANALOG)
          {
              if (IS_PIN_ANALOG(json_rx_pinmode))
              {
                  //pinMode(duo_pin[pin], AN_INPUT);
              }
          }
          else if (json_rx_pinmode == PWM)
          {
            if (IS_PIN_PWM(json_rx_pin))
            {
              pinMode(duo_pin[json_rx_pin], OUTPUT);
              analogWrite(duo_pin[json_rx_pin], 0);
              pins_pwm[json_rx_pin] = 0;
              pins_mode[json_rx_pin] = PWM;
            }
          }
          else if (json_rx_pinmode == SERVO)
          {
            //pinMode(json_rx_pin, OUTPUT);
            if (IS_PIN_SERVO(json_rx_pin))
            {
                pins_servo[json_rx_pin] = 0;
                pins_mode[json_rx_pin] = SERVO;
                if (!servos[PIN_TO_SERVO(json_rx_pin)].attached())
                    servos[PIN_TO_SERVO(json_rx_pin)].attach(duo_pin[PIN_TO_DIGITAL(json_rx_pin)]);
            }
          }
          else if(json_rx_pinmode == DEFAULT_MODE)
          {
            pinMode(duo_pin[json_rx_pin], INPUT_PULLDOWN);            
            pins_state[json_rx_pin] = LOW;
          }
        }

        if ( (json_rx_pinmode == INPUT) || (json_rx_pinmode == OUTPUT) || (json_rx_pinmode == DEFAULT_MODE))
          WiFireportPinDigitalData(json_rx_pin);
        else if (json_rx_pinmode == PWM)
          WiFireportPinPWMData(json_rx_pin);
        else if (json_rx_pinmode == SERVO)
          WiFireportPinServoData(json_rx_pin);

      }
      break;

    case 'G': // query pin data
      {
        if ( (pins_mode[json_rx_pin] == INPUT) || (pins_mode[json_rx_pin] == OUTPUT) || (pins_mode[json_rx_pin] == DEFAULT_MODE))
          WiFireportPinDigitalData(json_rx_pin);
        else if (pins_mode[json_rx_pin] == PWM)
          WiFireportPinPWMData(json_rx_pin);
        else if (pins_mode[json_rx_pin] == SERVO)
          WiFireportPinServoData(json_rx_pin);
      }
      break;

    case 'T': // set pin digital state
      {

        digitalWrite(duo_pin[json_rx_pin], json_rx_data);
        pins_state[json_rx_pin] = json_rx_data;
        WiFireportPinDigitalData(json_rx_pin);
      }
      break;

    case 'N': // set PWM
      {

        analogWrite(duo_pin[json_rx_pin], json_rx_data);
        pins_pwm[json_rx_pin] = json_rx_data;
        WiFireportPinPWMData(json_rx_pin);
      }
      break;

    case 'O': // set Servo
      {
        Serial.println("set Servo");
         if (IS_PIN_SERVO(json_rx_pin))
              servos[PIN_TO_SERVO(json_rx_pin)].write(json_rx_data);
        Serial.println("Servo write");
        pins_servo[json_rx_pin] = json_rx_data;
        WiFireportPinServoData(json_rx_pin);
      }
      break;

    case 'A': // query all pin status
    
      for (int json_rx_pin = 0; json_rx_pin < TOTAL_PINS_NUM; json_rx_pin++)
      {
        //reportPinCapability(json_rx_pin);
        if ( (pins_mode[json_rx_pin] == INPUT) || (pins_mode[json_rx_pin] == OUTPUT) || (pins_mode[json_rx_pin] == DEFAULT_MODE))
          WiFireportPinDigitalData(json_rx_pin);
        else if (pins_mode[json_rx_pin] == PWM)
          WiFireportPinPWMData(json_rx_pin);
        else if (pins_mode[json_rx_pin] == SERVO)
          WiFireportPinServoData(json_rx_pin);
      }
      break;

    case 'R': // reset pin
      {
        Serial.println("Reset Pin");
        for(uint8_t i=0;i<TOTAL_PINS_NUM;i++)
        {
          pinMode(duo_pin[i], INPUT_PULLDOWN);            
          pins_state[i] = LOW;
          pins_mode[i] = DEFAULT_MODE;
          
        }

        char json_buffer[20];
        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject(); 
        root["cmd"] = "R";
        Serial.print("Json_tx_string:  ");
        root.printTo(json_buffer,20);
        //root.printTo(Serial);
        Serial.println(json_buffer);
        for(uint8_t i=0;i<MAX_CLIENT_NUM;i++)
        {
            if (client[i].connected())
            {
               
               client[i].print(json_buffer);
               client[i].flush();
               delay(2); 
               client[i].println();
            }
        }
      }
      break;
  }
}



