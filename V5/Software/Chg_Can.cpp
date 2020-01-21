#include <stdint.h>
#include <libopencm3/stm32/can.h>
#define SENDBUFFER_LEN        15
#include "Chg_Can.h"
#define NULL 0L




static int sendCnt;

typedef struct
{
   uint16_t id;
   uint32_t data[2];
} SENDBUFFER;


static SENDBUFFER sendBuffer[SENDBUFFER_LEN];

union bmw
{
   uint32_t value[2];
   char bytes[8];
};

uint16_t counter_329;

void SendBMW()
{
uint16_t outRPM = 3000;
uint16_t tempValue = 25;
if(outRPM<4800) outRPM=4800;  //set lowest rpm to 750 displayed on tach to keep car alive thinking engine is running.
if(outRPM>44800) outRPM=44800;  //DONT READ MORE THAN 7000RPM!

 char outRPMlo = outRPM & 0xFF;
 char outRPMhi = outRPM >> 8;

char ABSMsg;
bmw dme1Data;
dme1Data.bytes[0] = 0x05;
dme1Data.bytes[1] = 0x00;
dme1Data.bytes[2] = outRPMlo;  //RPM LSB
dme1Data.bytes[3] = outRPMhi;  //RPM MSB [RPM=(hex2dec("byte3"&"byte2"))/6.4]  0x12c0 should be 750rpm on tach
dme1Data.bytes[4] = 0x00;
dme1Data.bytes[5] = 0x00;
dme1Data.bytes[6] = 0x00;
dme1Data.bytes[7] = 0x00;



    if(counter_329 >= 22) counter_329 = 0;
    if(counter_329==0) ABSMsg=0x11;
    if(counter_329>=8 && counter_329<15) ABSMsg=0x86;
    if(counter_329>=15) ABSMsg=0xd9;
    counter_329++;

bmw dme2Data;
dme2Data.bytes[0] = ABSMsg;  //needs to cycle 11,86,d9
dme2Data.bytes[1] = tempValue; //temp bit tdata
dme2Data.bytes[2] = 0xc5;
dme2Data.bytes[3] = 0x00;
dme2Data.bytes[4] = 0x00;
dme2Data.bytes[5] = 0x00; //Throttle position currently just fixed value
dme2Data.bytes[6] = 0x00;
dme2Data.bytes[7] = 0x00;

bmw dme3Data;
dme3Data.bytes[0] = 0x00;  //2=check ewwngine on , 0=check engine off
dme3Data.bytes[1] = 0x00; //LSB fuel comp
dme3Data.bytes[2] = 0x00;  //MSB fuel comp
dme3Data.bytes[3] = 0x00;   // hex 08 = Overheat light on
dme3Data.bytes[4] = 0x7E;
dme3Data.bytes[5] = 0x10;
dme3Data.bytes[6] = 0x00;
dme3Data.bytes[7] = 0x18;



 uint32_t dataBMW316[2] = {dme1Data.value[0],dme1Data.value[1] };
 uint32_t dataBMW329[2] = {dme2Data.value[0],dme2Data.value[1] };
 uint32_t dataBMW545[2] = {dme3Data.value[0],dme3Data.value[1]};




Send(0x316,dataBMW316);
Send(0x329,dataBMW329);
Send(0x545,dataBMW545);

}

void Send(uint32_t canId, uint32_t data[2])
{
   can_disable_irq(CAN1, CAN_IER_TMEIE);

   if (can_transmit(CAN1, canId, false, false, 8, (uint8_t*)data) < 0 && sendCnt < SENDBUFFER_LEN)
   {
      /* enqueue in send buffer if all TX mailboxes are full */
      sendBuffer[sendCnt].id = canId;
      sendBuffer[sendCnt].data[0] = data[0];
      sendBuffer[sendCnt].data[1] = data[1];
      sendCnt++;
   }

   if (sendCnt > 0)
   {
      can_enable_irq(CAN1, CAN_IER_TMEIE);
   }
}

extern "C" void usb_hp_can_tx_isr()
{
   if (sendCnt > 0)
   {
      sendCnt--;
      can_transmit(CAN1, sendBuffer[sendCnt].id, false, false, 8, (uint8_t*)sendBuffer[sendCnt].data);
   }
   else
   {
      can_disable_irq(CAN1, CAN_IER_TMEIE);
   }
}


extern "C" void usb_lp_can_rx0_isr(void)
{
	uint32_t id;
	bool ext, rtr;
	uint8_t length, fmi;
	uint32_t data[2];

   for (int fifo = 0; fifo < 2; fifo++)
   {
      while (can_receive(CAN1, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, NULL) > 0)
      {

      }
   }
}
