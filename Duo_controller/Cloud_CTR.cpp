#include "pin_inf.h"
#include <ArduinoJson.h>
#include "CTR_Json.h"
#include "WiFi_CTR.h"
#include "cloud_CTR.h"


 /******************************************************
 *               Function Definitions
 ******************************************************/

int tinkerAnalogWrite(String command);
int particleToDuo(String command);

/*******************************************************************************
 * Function Name  : cloudPinPWMData
 * Description    : creat a Json for pwm and sent to apps
 * Input          : Pin
 * Output         : None.
 * Return         : 1
 *******************************************************************************/
int cloudPinPWMData(byte pin)
{

  return creat_send_Json(pin,pins_pwm[pin]);

}

/*******************************************************************************
 * Function Name  : cloudPinServoData
 * Description    : creat a Json for servo and sent to apps
 * Input          : Pin
 * Output         : None.
 * Return         : 1
 *******************************************************************************/
int cloudPinServoData(byte pin)
{
  return creat_send_Json(pin,pins_servo[pin]);
}

/*******************************************************************************
 * Function Name  : cloudPinAnalogData
 * Description    : creat a Json for analog input and sent to apps
 * Input          : Pin
 * Output         : None.
 * Return         : 1
 *******************************************************************************/
byte cloudPinAnalogData(byte pin)
{
  uint16_t value;
  byte value_lo;
  byte value_hi;
  byte mode;
  
  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }

  if (pins_mode[pin] == ANALOG)
  {
    value = analogRead(duo_pin[pin]);
    value_lo = value;
    value_hi = value >> 8;

    mode = pins_mode[pin];
    //mode = (value_hi << 4) | mode;

  }

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
  Particle.variable("mess",json_buffer,STRING);
  return 1;

}

/*******************************************************************************
 * Function Name  : cloudPinServoData
 * Description    : creat a Json for digital input and sent to apps
 * Input          : Pin
 * Output         : None.
 * Return         : 1
 *******************************************************************************/
int cloudPinDigitalData(byte pin)
{
  return creat_send_Json(pin,digitalRead(duo_pin[pin]));
}
/*******************************************************************************
 * Function Name  : cloudPinResetData
 * Description    : creat a Json for digital input and sent to apps
 * Input          : void
 * Output         : None.
 * Return         : 1
 *******************************************************************************/
int cloudPinResetData()
{
    return 0;
}

/*******************************************************************************
 * Function Name  : tinkerDigitalRead
 * Description    : Reads the digital value of a given pin
 * Input          : Pin
 * Output         : None.
 * Return         : Value of the pin (0 or 1) in INT type
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerDigitalRead(String pin)
{
  return -2;
}

/*******************************************************************************
 * Function Name  : tinkerDigitalWrite
 * Description    : Sets the specified pin HIGH or LOW
 * Input          : Pin and value
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerDigitalWrite(String command)
{
  return -3;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogRead
 * Description    : Reads the analog value of a pin
 * Input          : Pin
 * Output         : None.
 * Return         : Returns the analog value in INT type (0 to 4095)
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerAnalogRead(String pin)
{
  return -2;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogWrite
 * Description    : Writes an analog value (PWM) to the specified pin
 * Input          : Pin and Value (0 to 255)
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerAnalogWrite(String command)
{
   return -2;
}

/*******************************************************************************
 * Function Name  : particleToDuo
 * Description    : send json to duo through particle cloud and creat the mess variable 
 * Input          : Json
 * Output         : Json variable mess.
 * Return         : 1 on success 
 *******************************************************************************/
int particleToDuo(String command)
{

  char json_rec_temp[255];
  memset(json_rx_cmd, 0, 2);
  Serial.print("length: ");
  Serial.println(command.length());
  command.toCharArray(json_rec_temp,command.length()+1);
  Serial.print("command: ");
  Serial.println(command);
  Serial.print("json: ");
  Serial.println(json_rec_temp);
  parseJson(json_rec_temp);
  //delay(3000);
  // Parse data here
  switch (json_rx_cmd[0])
  {
    case 'V': // query protocol version
      {
          int version_buf[3] = {0x00, 0x00, 0x02};

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
          Particle.variable("mess",json_buffer,STRING);
          return 1;
      }
      break;

    case 'C': // query board total pin count
      {

          char json_buffer[50];
          StaticJsonBuffer<500> jsonBuffer;
          JsonObject& root = jsonBuffer.createObject(); 
          root["cmd"] = "C";
          root["data"] = TOTAL_PINS_NUM;
          Serial.print("Json_tx_string:  ");
          root.printTo(json_buffer,50);
          //root.printTo(Serial);
          Serial.println(json_buffer);
          Particle.variable("mess",json_buffer,STRING);
          return 1;
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
          Particle.variable("mess",json_buffer,STRING);
          return 1;

      }
      break;

    case 'S': // set pin mode
      {
        if ( IS_PIN_SERVO(json_rx_pinmode) && json_rx_pinmode != SERVO && servos[PIN_TO_SERVO(json_rx_pinmode)].attached() )
                    servos[PIN_TO_SERVO(json_rx_pinmode)].detach();

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
            //pinMode(json_rx_pin, OUTPUT);
            if (IS_PIN_ANALOG(json_rx_pin)) {
//              if (IS_PIN_DIGITAL(json_rx_pin)) {
//                pinMode(PIN_TO_DIGITAL(json_rx_pin), OUTPUT);
//                digitalWrite(duo_pin[json_rx_pin], LOW);
//              }
            }
          }
          else if (json_rx_pinmode == PWM)
          {
            if (IS_PIN_PWM(json_rx_pin))
            {
              pinMode(duo_pin[json_rx_pin], OUTPUT);
              analogWrite(duo_pin[PIN_TO_PWM(json_rx_pin)], 0);
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
          else if (json_rx_pinmode == DEFAULT_MODE)
          {
            pinMode(duo_pin[json_rx_pin], INPUT_PULLDOWN);
            pins_state[json_rx_pin] = LOW;
          }
          
        }


        if ( (json_rx_pinmode == INPUT) || (json_rx_pinmode == OUTPUT) || (json_rx_pinmode == DEFAULT_MODE))
          return cloudPinDigitalData(json_rx_pin);
        else if (json_rx_pinmode == ANALOG)
          return cloudPinAnalogData(json_rx_pin);  
        else if (json_rx_pinmode == PWM)
          return cloudPinPWMData(json_rx_pin);
        else if (json_rx_pinmode == SERVO)
          return cloudPinServoData(json_rx_pin);
      }
      break;

    case 'G': // query pin data
      {

        if ( (pins_mode[json_rx_pin] == INPUT) || (pins_mode[json_rx_pin] == OUTPUT) || (json_rx_pinmode == DEFAULT_MODE))
          return cloudPinDigitalData(json_rx_pin);
        else if (pins_mode[json_rx_pin] == ANALOG)
          return cloudPinAnalogData(json_rx_pin);  
        else if (pins_mode[json_rx_pin] == PWM)
          return cloudPinPWMData(json_rx_pin);
        else if (pins_mode[json_rx_pin] == SERVO)
          return cloudPinServoData(json_rx_pin);

      }
      break;

    case 'T': // set pin digital state
      {

        digitalWrite(duo_pin[json_rx_pin], json_rx_data);
        return cloudPinDigitalData(json_rx_pin);
      }
      break;

    case 'N': // set PWM
      {

        analogWrite(duo_pin[json_rx_pin], json_rx_data);
        pins_pwm[json_rx_pin] = json_rx_data;
        return cloudPinPWMData(json_rx_pin);
      }
      break;

    case 'O': // set Servo
      {
        Serial.println("set Servo");
        servos[json_rx_pin].write(json_rx_data);
        Serial.println("Servo write");
        pins_servo[json_rx_pin] = json_rx_data;
        return cloudPinServoData(json_rx_pin);
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
        Particle.variable("mess",json_buffer,STRING);
        return 1;
      }
    break;
  }
  
  
}
