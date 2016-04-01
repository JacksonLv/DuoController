#ifndef _CLOUD_CTR_H_
#define _CLOUD_CTR_H_

 /******************************************************
 *               Function Definitions
 ******************************************************/
extern int tinkerDigitalRead(String pin);
extern int tinkerDigitalWrite(String command);
extern int tinkerAnalogRead(String pin);
extern int tinkerAnalogWrite(String command);
extern int particleToDuo(String command);

#endif
