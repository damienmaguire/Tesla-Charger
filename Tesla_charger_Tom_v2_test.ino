/*
Tesla Gen 2 Charger Phase 1 driver experimental code v1
2017
D.Maguire
Tweaks by T de Bree

Runs on OpenSource Logic board V1 in Gen2 charger. Commands all modules. 

"s" starts or stops charging
"v" sets voltage setpoint
"c" sets charge current. WARNING! this current will be pumped out by all modules equally. So if you set 5Amps you will get 5 amps from all modules (if they have mains) for a total of 15A into the battery.
"r" sets ramp time in milliseconds. r500 sets 500ms ramp time.

*/

#include <due_can.h>  
#include <due_wire.h> 
#include <DueTimer.h>  
#include <Wire_EEPROM.h> 


#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

 // Useful macros for setting and resetting bits
//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



//*********GENERAL VARIABLE   DATA ******************
int debug = 1; // 1 = show canbus feedback

uint16_t voltset = 0;
uint16_t curset = 0;
uint16_t curreq = 0;
uint16_t currampt = 500; //500ms ramptime as default
signed long curramp = 0;
int  setting = 1;
int incomingByte = 0;
int state,enab =0;
unsigned long tlast =0;



//*********Feedback from charge VARIABLE   DATA ******************
uint16_t dcvolt = 0;
uint16_t dccur = 0;

uint16_t acvolt = 0;
uint16_t accur = 0;

int newframe=0;

CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME incoming;    //structure to keep inbound frames

//setup bytes to contain CAN data
//bytes for 0x045c/////////////////////
byte test52;  //0x45c byte 2 test 2
byte test53;  //0x45c byte 3 test 3
/////////////////////////////////////////

//bytes for 0x042c/////////////////////
byte test20;  //0x42c byte 0 test 4
byte test21;  //0x42c byte 1 test 5
byte test24;  //0x42c byte 4 test 8


/////////////////////////////////////////




void setup() 
  {


    
    Serial.begin(9600);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

    Timer3.attachInterrupt(Charger_msgs).start(100000); // charger messages every 100ms

    

    

 
 // Initialize CAN ports 
      pinMode(48,OUTPUT);
      if (Can1.begin(500000,48)) //can1 external bus
        {
          Serial.println("Using CAN1 - initialization completed.\n");
           
        }
           else Serial.println("CAN1 initialization (sync) ERROR\n");


    // Initialize CAN0
     pinMode(50,OUTPUT);
     if (Can0.begin(500000,50)) //can0 charger modules
        {
          Serial.println("Using CAN0 - initialization completed.\n");
        }
        else Serial.println("CAN0 initialization (sync) ERROR\n");

int filter;
  //extended
  for (filter = 0; filter < 3; filter++) {
  Can0.setRXFilter(filter, 0, 0, true);
  Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
  Can0.setRXFilter(filter, 0, 0, false);
  Can1.setRXFilter(filter, 0, 0, false);
  } 
        
///////////////////CHARGER ENABLE AND ACTIVATE LINES///////////////////////////////////
    pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, OUTPUT); //CHG1 ENABLE
   pinMode(4, OUTPUT);  //CHG2 ENABLE
    pinMode(5, OUTPUT); //CHG3 ENABLE
     pinMode(62, OUTPUT); //CHG1 ACTIVATE
      pinMode(63, OUTPUT);  //CHG2 ACTIVATE
       pinMode(64, OUTPUT); //CHG3 ACTIVATE
//////////////////////////////////////////////////////////////////////////////////////       

//////////////DIGITAL OUTPUTS MAPPED TO X046. 10 PIN CONNECTOR ON LEFT//////////////////////////////////////////
  pinMode(48, OUTPUT); //OP1 - X046 PIN 6
  pinMode(49, OUTPUT); //OP2
  pinMode(50, OUTPUT); //OP2
  pinMode(51, OUTPUT); //OP3
///////////////////////////////////////////////////////////////////////////////////////

         
       delay(1000);                       // wait for a second
digitalWrite(3, HIGH);//enable phase 1 power module


       delay(1000);                       // wait for a second
digitalWrite(4, HIGH);//enable phase 2 power module
       delay(1000);                       // wait for a second
digitalWrite(5, HIGH);//enable phase 3 power module


/////////Setup initial state of 2 variable CAN messages///////////////////////////

     test52=0x15;
     test53=0x0e;

     test20=0x42;
     test21=0x60;
     test24=0x64;
///////////////////////////////////////////////////////////////////////////////////


  
}
   



void loop() {
  // put your main code here, to run repeatedly:
  /*
  if (Can0.available()) 
  {
    Can0.read(incoming); 
    candecode(incoming);
  } 
*/
 if (Serial.available() > 0)
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 118://v for voltage setting in whole numbers
         if (Serial.available() > 0)
         {
          voltset = (Serial.parseInt()*100);
          setting = 1;
         }
        break;
        
        case 116://t for current ramp time
         if (Serial.available() > 0)
         {
          currampt = Serial.parseInt();
          setting = 1;
         }
        break;
        
       case 115://s for start AND stop
         if (Serial.available() > 0)
         {
          state = !state;
          setting = 1;
         }
        break;

       case 101://e for enabling chargers followed by numbers to indicate which ones to run
         if (Serial.available() > 0)
         {
          enab = Serial.parseInt();
          setting = 1;
         }
        break;
        
      case 99: //c for current setting in whole numbers
         if (Serial.available() > 0)
         {
          curset = (Serial.parseInt()*1500);
          setting = 1;
         }
        break;

      default: 
      // if nothing else matches, do the default
      // default is optional
        break; 
      
    }
  }
 
  if (setting == 1) //display if any setting changed
    {
    Serial.println();
    if (state == 1)
    {
      Serial.print("Charger On   ");
    }
    else
    {
      Serial.print("Charger Off   ");
    }
    Serial.print("Enabled Phases : ");
    Serial.print(enab);    
    Serial.print("Set voltage : ");
    Serial.print(voltset*0.01,0);  
    Serial.print("V | Set current : ");
    Serial.print(curset*0.00066666,0);
    Serial.print(" A ");
    Serial.print("  ms | Set ramptime : ");
    Serial.print(currampt);

    Serial.print(" Ramp current : ");
    curramp = (curset-curreq)/500;
    
    Serial.print(curramp);    
     setting = 0;
    }

switch (state)
  {
    case 0: //Charger off
    
       test52=0x15;
       test53=0x0e;
       test21=0x60;
       test24=0x64;
       digitalWrite(48, LOW);//MAINS OFF
       delay(10);
       digitalWrite(62, LOW); //chargeph1 deactivate
       digitalWrite(63, LOW); //chargeph2 deactivate
       digitalWrite(64, LOW); //chargeph3 deactivate
      break;
  
    case 1://Charger on
    switch(enab)
    {
       case 1:
        digitalWrite(62, HIGH);       
       break;
       
       case 2:
        digitalWrite(63, HIGH);       
       break;
       
       case 3:
        digitalWrite(64, HIGH);       
       break;

       case 12:
        digitalWrite(62, HIGH);
        digitalWrite(63, HIGH);      
       break;
       
       case 13:
        digitalWrite(62, HIGH);
        digitalWrite(64, HIGH);       
       break;
       
       case 123:
        digitalWrite(62, HIGH);
        digitalWrite(63, HIGH);
        digitalWrite(64, HIGH);       
       break;

       case 23:
        digitalWrite(63, HIGH);
        digitalWrite(64, HIGH);       
       break;

      default: 
      // if nothing else matches, do the default
      // default is optional
        break; 
    }

    delay(10);
       test52=0x14;
       test53=0x2E;
       test21=0xc4;
       test24=0xfe;
       digitalWrite(48, HIGH);//MAINS ON
      break;
  
    default:
        // if nothing else matches, do the default
      break;
  }

if (curreq != curset)
  {
    if ((millis()- tlast) > 1)
      {
        tlast = millis();
        curreq = curreq + curramp;
      }
  }

if (debug != 0)
  {
    if (newframe & 3 != 0)
    {
     Serial.println();
     Serial.print(millis()); 
     Serial.print("  Charger Feebback //  AC voltage : ");
     Serial.print(acvolt);
     Serial.print("  AC current : ");
     Serial.print(accur/28);
     Serial.print("  DC voltage : ");
     Serial.print(dcvolt/100,2);
     Serial.print("  DC current : ");
     Serial.print(dccur/1000,2);
    }
  }

}


void candecode(CAN_FRAME &frame)
{
  switch(frame.id)
  {
    case 0x207:
      acvolt = frame.data.bytes[1];
      accur = ((frame.data.bytes[8]&=3)*256+frame.data.bytes[7]);
      newframe = newframe | 1;
      break;

    case 0x227: //dc feedback
      //dccur = frame.data.bytes[7]*256+frame.data.bytes[6];
      dccur = (frame.data.bytes[5]+frame.data.bytes[4])*0.000839233;
      dcvolt = frame.data.bytes[3]*256+frame.data.bytes[2];
      newframe = newframe | 2;
      break;

    default:
        // if nothing else matches, do the default
      break;
  }
}

void Charger_msgs()
{

/////////////////////This msg addresses all modules/////////////////////////////////////////////////  
        outframe.id = 0x045c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=lowByte(voltset);  //Voltage setpoint
        outframe.data.bytes[1]=highByte(voltset);//Voltage setpoint
        outframe.data.bytes[2]=test52;
        outframe.data.bytes[3]=test53;
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=0x00;
        outframe.data.bytes[6]=0x90;
        outframe.data.bytes[7]=0x8c;
        Can0.sendFrame(outframe); 
//////////////////////////////////////////////////////////////////////////////////////////////////////
        
//////////////////////////////////////Phase 1 command message////////////////////////////////////////
        outframe.id = 0x042c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=test20; 
        outframe.data.bytes[1]=test21; 
        outframe.data.bytes[2]=lowByte(curreq); //Current setpoint
        outframe.data.bytes[3]=highByte(curreq); //Current setpoint 
        outframe.data.bytes[4]=test24;
        outframe.data.bytes[5]=0x00;  
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
        Can0.sendFrame(outframe); 
////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////Phase 2 command message//////////////////////////////////////////////        
        outframe.id = 0x043c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=test20; 
        outframe.data.bytes[1]=test21; 
        outframe.data.bytes[2]=lowByte(curreq); //Current setpoint
        outframe.data.bytes[3]=highByte(curreq); //Current setpoint 
        outframe.data.bytes[4]=test24;
        outframe.data.bytes[5]=0x00;  
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
        Can0.sendFrame(outframe);
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////Phase 3 command message/////////////////////////////////////////////        
        outframe.id = 0x044c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=test20; 
        outframe.data.bytes[1]=test21; 
        outframe.data.bytes[2]=lowByte(curreq); //Current setpoint
        outframe.data.bytes[3]=highByte(curreq); //Current setpoint 
        outframe.data.bytes[4]=test24;
        outframe.data.bytes[5]=0x00;  
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
        Can0.sendFrame(outframe);
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////Static Frame every 100ms///////////////////////////////////////////////////////////////////
        outframe.id = 0x368;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x03;  
        outframe.data.bytes[1]=0x49;
        outframe.data.bytes[2]=0x29;
        outframe.data.bytes[3]=0x11;
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=0x0c;
        outframe.data.bytes[6]=0x40;
        outframe.data.bytes[7]=0xff;
       

                      

        Can0.sendFrame(outframe); 
////////////////////////////////////////////////////////////////////////////////////////////////////////

}
