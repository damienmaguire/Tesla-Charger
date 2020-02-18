/*
 * config.h
 *
 *  Created on: 16 Şub 2020
 *      Author: Koksal KURT
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define EEPROM_VERSION      11

uint16_t VirAdress = 20700;
typedef struct
{
    uint8_t  version; //eeprom version stored
    uint8_t  enabledChargers;
    uint8_t  mainsRelay; //which output is used to control the AC relay
    uint8_t  autoEnableCharger;
    uint8_t  canControl;
    uint8_t  type;
    uint8_t  phaseconfig;
    uint16_t voltSet;
    uint16_t tVolt;
    uint16_t currReq;
    uint32_t can0Speed;
    uint16_t dcdcsetpoint;
}   ChargerParams;

#endif /* INC_CONFIG_H_ */
