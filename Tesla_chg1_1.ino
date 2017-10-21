/*
Tesla Gen 2 Charger Phase 1 driver experimental code v1
2017
D.Maguire
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


CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.


//setup bytes to contain CAN data
//bytes for 0x045c/////////////////////
byte test0;  //0x45c byte 0
byte test1;  //0x45c byte 1
byte test2;  //0x45c byte 2
byte test3;  //0x45c byte 3
/////////////////////////////////////////

//bytes for 0x042c/////////////////////
byte test4;  //0x45c byte 0
byte test5;  //0x45c byte 1
byte test6;  //0x45c byte 2
byte test7;  //0x45c byte 4
byte test8;  //0x45c byte 5

/////////////////////////////////////////




void setup() 
  {


    
    Serial.begin(9600);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

    Timer3.attachInterrupt(Charger_msgs).start(100000); // charger messages every 100ms

    

    

 
 // Initialize CAN ports 
      pinMode(48,OUTPUT);
      if (Can1.begin(500000,48)) 
        {
          Serial.println("Using CAN1 - initialization completed.\n");
           
        }
           else Serial.println("CAN1 initialization (sync) ERROR\n");


    // Initialize CAN0
     pinMode(50,OUTPUT);
     if (Can0.begin(500000,50)) 
        {
          Serial.println("Using CAN0 - initialization completed.\n");
        }
        else Serial.println("CAN0 initialization (sync) ERROR\n");


/////////Setup initial state of 2 variable CAN messages///////////////////////////

     test0=0x13;
     test1=0x8c;
     test2=0x15;
     test3=0x0e;

     test4=0x42;
     test5=0x60;
     test6=0x00;
     test7=0x00;
     test8=0x64;
///////////////////////////////////////////////////////////////////////////////////


  
}
   



void loop()
{ 
Serial.println("State 1: mains off. Wait 5 secs");
delay(5000);
Serial.println("State 2:");
     test4=0x42;
     test5=0x60;
     test6=0x00;
     test7=0x00;
     test8=0x68;
delay(500);  
Serial.println("State 3:");
     test4=0x42;
     test5=0x60;
     test6=0x00;
     test7=0x00;
     test8=0x69; 
     delay(500); 
     
Serial.println("State 4:");
     test4=0x42;
     test5=0x60;
     test6=0x00;
     test7=0x00;
     test8=0x6d; 
     delay(600); 
     
Serial.println("State 5:");
     test4=0x42;
     test5=0xc4;
     test6=0x00;
     test7=0x00;
     test8=0x6d; 
     delay(500); 
Serial.println("MAINS ON NOW!!!!!!");    
     delay(500);

Serial.println("State 6:");  
     test0=0x15;
     test1=0x8c;
     test2=0x15;
     test3=0x2e;
     delay(500);

Serial.println("State 7:");
     test4=0x42;
     test5=0xc4;
     test6=0x00;
     test7=0x00;
     test8=0x6c; 
     delay(500); 

Serial.println("State 8:368v target.");  
     test0=0xf8;
     test1=0x8f;
     test2=0x14;
     test3=0x2e;
     delay(500);
     
Serial.println("State 9");
     test4=0x42;
     test5=0xc4;
     test6=0x00;
     test7=0x00;
     test8=0xfc; 
     delay(5000); 
     
Serial.println("State 10:");
     test4=0x42;
     test5=0xc4;
     test6=0x00;
     test7=0x00;
     test8=0xfe; 
     delay(10000);

Serial.println("State 11:current ramp");
     test4=0x42;
     test5=0xc4;
     test6=0x5f;
     test7=0x00;
     test8=0xfe; 
     delay(2000);

     Serial.println("State 11:current ramp2");
     test4=0x42;
     test5=0xc4;
     test6=0x75;
     test7=0x02;
     test8=0xfe; 
     delay(2000);

     Serial.println("State 11:current ramp3");
     test4=0x42;
     test5=0xc4;
     test6=0x2c;
     test7=0x04;
     test8=0xfe; 
     delay(2000);

     Serial.println("State 11:current ramp4");
     test4=0x42;
     test5=0xc4;
     test6=0x79;
     test7=0x05;
     test8=0xfe; 
     delay(2000);

     Serial.println("State 11:current ramp5");
     test4=0x42;
     test5=0xc4;
     test6=0xb7;
     test7=0x08;
     test8=0xfe; 
     delay(2000);


     Serial.println("State 11:current ramp6");
     test4=0x42;
     test5=0xc4;
     test6=0xcd;
     test7=0x08;
     test8=0xfe; 
     delay(2000);

     Serial.println("State 11:current ramp6");
     test4=0x42;
     test5=0xc4;
     test6=0xb0;
     test7=0x0c;
     test8=0xfe; 
     delay(2000);

     Serial.println("State 11:current ramp7");
     test4=0x42;
     test5=0xc4;
     test6=0xa8;
     test7=0x14;
     test8=0xfe; 
     delay(2000);


while(1);
{
//wait here forever
}    
}



void Charger_msgs()
{
        outframe.id = 0x045c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=test0;  
        outframe.data.bytes[1]=test1;
        outframe.data.bytes[2]=test2;
        outframe.data.bytes[3]=test3;
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=0x00;
        outframe.data.bytes[6]=0x90;
        outframe.data.bytes[7]=0x8c;
        Can1.sendFrame(outframe); 

        outframe.id = 0x042c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=test4; 
        outframe.data.bytes[1]=test5; 
        outframe.data.bytes[2]=test6;  
        outframe.data.bytes[3]=test7;  
        outframe.data.bytes[4]=test8;
        outframe.data.bytes[5]=0x00;  
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
        Can1.sendFrame(outframe); 


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
       

                      

        Can1.sendFrame(outframe); 
////////////////////////////////////////////////////////////////////////////////////////////////////////

}




























