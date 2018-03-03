/*
  Tesla Gen 2 Charger Control Program
  2017-2018
  D.Maguire
  Tweaks by T de Bree
  Additional work by C. Kidder
  Runs on OpenSource Logic board V2 in Gen2 charger. Commands all modules.
  "s" starts or stops charging

  "v" sets voltage setpoint

  "c" sets charge current. WARNING! this current will be pumped out by all modules equally.
  So if you set 5Amps you will get 5 amps from all modules (if they have mains) for a total
  of 15A into the battery.

  "r" sets ramp time in milliseconds. r500 sets 500ms ramp time.
*/

#include <can_common.h>
#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include <DueTimer.h>
#include "config.h"

#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}

//*********GENERAL VARIABLE   DATA ******************
int debugevse = 0; // 1 = show Proximity status and Pilot current limmits
int debug = 1; // 1 = show phase module CAN feedback


uint16_t curset = 0;
signed long curramp = 0;
int  setting = 1;
int incomingByte = 0;
int state;
unsigned long tlast = 0;
bool bChargerEnabled;

//*********EVSE VARIABLE   DATA ******************
byte Proximity = 0;
int Type = 2;
//proximity status values for type 1
#define Unconnected 0 // 3.3V
#define Buttonpress 1 // 2.3V
#define Connected 2 // 1.35V

volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile int dutycycle = 0;

uint16_t cablelim = 0; // Type 2 cable current limit

//*********Single or Three Phase Config VARIABLE   DATA ******************
byte Config = 0;
uint16_t modulelimcur = 0;

//proximity status values
#define Singlephase 0 // all parrallel on one phase Type 1
#define Threephase 1 // one module per phase Type 2



//*********Feedback from charge VARIABLE   DATA ******************
uint16_t dcvolt[3] = {0, 0, 0};
uint16_t dccur[3] = {0, 0, 0};
uint16_t acvolt[3] = {0, 0, 0};
uint16_t accur[3] = {0, 0, 0};
byte inlettarg [3] = {0, 0, 0}; //inlet target temperatures, should be used to command cooling.
byte curtemplim [3] = {0, 0, 0};//current limit due to temperature
byte templeg[2][3] = {{0, 0, 0}, {0, 0, 0}}; //temperatures reported back
bool ACpres [3] = {0, 0, 0}; //AC present detection on the modules
int newframe = 0;

ChargerParams parameters;

void setup()
{
  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

  Timer3.attachInterrupt(Charger_msgs).start(90000); // charger messages every 100ms

  attachInterrupt(EVSE_PILOT, Pilotread , CHANGE);

  Wire.begin();
  EEPROM.read(0, parameters);
  if (parameters.version != EEPROM_VERSION)
  {
    parameters.version = EEPROM_VERSION;
    parameters.can0Speed = 500000;
    parameters.can1Speed = 500000;
    parameters.currRampTime = 500;
    parameters.currReq = 0; //max input limit per module 1500 = 1A
    parameters.enabledChargers = 123;
    parameters.mainsRelay = 48;
    parameters.voltSet = 0;
    parameters.autoEnableCharger = 0; //don't auto enable it by default
    EEPROM.write(0, parameters);
  }

  // Initialize CAN ports
  if (Can1.begin(parameters.can1Speed, 255)) //can1 external bus
  {
    Serial.println("Using CAN1 - initialization completed.\n");
  }
  else Serial.println("CAN1 initialization (sync) ERROR\n");


  // Initialize CAN0
  if (Can0.begin(parameters.can0Speed, 255)) //can0 charger modules
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

  //////////////////////////////////////////////////////////////////////////////////////

  ///////////////////CHARGER ENABLE AND ACTIVATE LINES///////////////////////////////////
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CHARGER1_ENABLE, OUTPUT); //CHG1 ENABLE
  pinMode(CHARGER2_ENABLE, OUTPUT);  //CHG2 ENABLE
  pinMode(CHARGER3_ENABLE, OUTPUT); //CHG3 ENABLE
  pinMode(CHARGER1_ACTIVATE, OUTPUT); //CHG1 ACTIVATE
  pinMode(CHARGER2_ACTIVATE, OUTPUT);  //CHG2 ACTIVATE
  pinMode(CHARGER3_ACTIVATE, OUTPUT); //CHG3 ACTIVATE
  //////////////////////////////////////////////////////////////////////////////////////

  //////////////DIGITAL OUTPUTS MAPPED TO X046. 10 PIN CONNECTOR ON LEFT//////////////////////////////////////////
  pinMode(DIG_OUT_1, OUTPUT); //OP1 - X046 PIN 6
  pinMode(DIG_OUT_2, OUTPUT); //OP2
  pinMode(DIG_OUT_3, OUTPUT); //OP2
  pinMode(DIG_OUT_4, OUTPUT); //OP3
  pinMode(EVSE_ACTIVATE, OUTPUT); //pull Pilot to 6V
  ///////////////////////////////////////////////////////////////////////////////////////


  delay(1000);                       // wait for a second
  digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
  delay(1000);                       // wait for a second
  digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
  delay(1000);                       // wait for a second
  digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module

  bChargerEnabled = false; //are we supposed to command the charger to charge?
}

void loop()
{
  CAN_FRAME incoming;

  if (Can0.available())
  {
    Can0.read(incoming);
    candecode(incoming);
  }

  if (Serial.available())
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 'a'://v for voltage setting in whole numbers
        if (Serial.available() > 0)
        {
          parameters.autoEnableCharger = Serial.parseInt();
          if (parameters.autoEnableCharger > 1)
          {
            parameters.autoEnableCharger = 0;
          }
          setting = 1;
        }
        break;
      case 'v'://v for voltage setting in whole numbers
        if (Serial.available() > 0)
        {
          parameters.voltSet = (Serial.parseInt() * 100);
          setting = 1;
        }
        break;
      case 't'://t for current ramp time
        if (Serial.available() > 0)
        {
          parameters.currRampTime = Serial.parseInt();
          setting = 1;
        }
        break;
      case 's'://s for start AND stop
        if (Serial.available() > 0)
        {
          state = !state;
          setting = 1;
          digitalWrite(LED_BUILTIN, HIGH);
        }
        break;
      case 'e'://e for enabling chargers followed by numbers to indicate which ones to run
        if (Serial.available() > 0)
        {
          parameters.enabledChargers = Serial.parseInt();
          setting = 1;
        }
        break;
      case 'c': //c for current setting in whole numbers
        if (Serial.available() > 0)
        {
          parameters.currReq = (Serial.parseInt() * 1500);
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
    EEPROM.write(0, parameters);
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
    Serial.print(parameters.enabledChargers);
    Serial.print("Set voltage : ");
    Serial.print(parameters.voltSet * 0.01f, 0);
    Serial.print("V | Set current : ");
    Serial.print(parameters.currReq * 0.00066666, 0);
    Serial.print(" A ");
    //Serial.print("  ms | Set ramptime : ");
    //Serial.print(parameters.currRampTime);

    //Serial.print(" Ramp current : ");
    //curramp = (curset - parameters.currReq) / 500;

    //Serial.print(curramp);

    if (parameters.autoEnableCharger == 1)
    {
      Serial.print(" Autostart On   ");
    }
    else
    {
      Serial.print(" Autostart Off   ");
    }
    setting = 0;
  }

  switch (state)
  {
    case 0: //Charger off
      bChargerEnabled = false;
      digitalWrite(DIG_OUT_1, LOW);//MAINS OFF
      delay(10);
      digitalWrite(CHARGER1_ACTIVATE, LOW); //chargeph1 deactivate
      digitalWrite(CHARGER1_ACTIVATE, LOW); //chargeph2 deactivate
      digitalWrite(CHARGER1_ACTIVATE, LOW); //chargeph3 deactivate
      break;
    case 1://Charger on
      bChargerEnabled = true;
      switch (parameters.enabledChargers)
      {
        case 1:
          digitalWrite(CHARGER1_ACTIVATE, HIGH);
          break;
        case 2:
          digitalWrite(CHARGER2_ACTIVATE, HIGH);
          break;
        case 3:
          digitalWrite(CHARGER3_ACTIVATE, HIGH);
          break;
        case 12:
          digitalWrite(CHARGER1_ACTIVATE, HIGH);
          digitalWrite(CHARGER2_ACTIVATE, HIGH);
          break;
        case 13:
          digitalWrite(CHARGER1_ACTIVATE, HIGH);
          digitalWrite(CHARGER3_ACTIVATE, HIGH);
          break;
        case 123:
          digitalWrite(CHARGER1_ACTIVATE, HIGH);
          digitalWrite(CHARGER2_ACTIVATE, HIGH);
          digitalWrite(CHARGER3_ACTIVATE, HIGH);
          break;
        case 23:
          digitalWrite(CHARGER2_ACTIVATE, HIGH);
          digitalWrite(CHARGER3_ACTIVATE, HIGH);
          break;
        default:
          // if nothing else matches, do the default
          // default is optional
          break;
      }
      delay(100);
      digitalWrite(DIG_OUT_1, HIGH);//MAINS ON
      break;
    default:
      // if nothing else matches, do the default
      break;
  }

  if (debug != 0)
  {
    if (newframe & 3 != 0)
    {
      Serial.println();
      Serial.print(millis());
      for (int x = 0; x < 3; x++)
      {
        Serial.print("  Phase ");
        Serial.print(x + 1);
        Serial.print(" Feebback //  AC present: ");
        Serial.print(ACpres[x]);
        Serial.print("  AC volt: ");
        Serial.print(acvolt[x]);
        Serial.print("  AC cur: ");
        Serial.print(accur[x] / 28);
        Serial.print("  DC volt: ");
        Serial.print(dcvolt[x]);
        Serial.print("  DC cur: ");
        Serial.print(dccur[x] / 1000, 2);
        Serial.print("  Inlet Targ: ");
        Serial.print(inlettarg[x]);
        Serial.print("  Temp Lim Cur: ");
        Serial.print(curtemplim[x]);
        Serial.print("  ");
        Serial.print(templeg[0][x]);
        Serial.print("  ");
        Serial.print(templeg[1][x]);
        Serial.println();
      }
    }
  }

  evseread();
  if (parameters.autoEnableCharger == 1)
  {
    if (Proximity == Connected) //check if plugged in
    {
      digitalWrite(EVSE_ACTIVATE, HIGH);//pull pilot low to indicate ready - NOT WORKING freezes PWM reading
      if (debugevse != 0)
      {
        Serial.println();
        Serial.print(millis());
        Serial.print(" AC limit : ");
        Serial.print(accurlim);
        Serial.print("  ");
        Serial.print(dutycycle);
      }
      ACcurrentlimit();
      if (debugevse != 0)
      {
        Serial.print(" Phase limit : ");
        Serial.print(modulelimcur * 0.00066666, 1);
      }
      if (modulelimcur > 1500) //above one amp active modules
      {
        state = 1;
      }
    }
    else // unplugged or buton pressed stop charging
    {
      state = 0;
      digitalWrite(EVSE_ACTIVATE, LOW);
    }
  }
}


void candecode(CAN_FRAME &frame)
{
  int x = 0;
  switch (frame.id)
  {
    case 0x24B: //phase 3 temp message 2
      curtemplim[2] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x23B: //phase 3 temp message 1
      templeg[0][2] = frame.data.bytes[0] - 40;
      templeg[1][2] = frame.data.bytes[1] - 40;
      inlettarg[2] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;

    case 0x239: //phase 2 temp message 1
      templeg[0][1] = frame.data.bytes[0] - 40;
      templeg[1][1] = frame.data.bytes[1] - 40;
      inlettarg[1] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x249: //phase 2 temp message 2
      curtemplim[1] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x237: //phase 1 temp message 1
      templeg[0][0] = frame.data.bytes[0] - 40;
      templeg[1][0] = frame.data.bytes[1] - 40;
      inlettarg[0] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x247: //phase 2 temp message 2
      curtemplim[0] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x207: //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[0] = frame.data.bytes[1];
      accur[0] = (((frame.data.bytes[6] & 3) << 7) + (frame.data.bytes[5] & 01111111));
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[0] = true;
      }
      else
      {
        ACpres[0] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x209: //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[1] = frame.data.bytes[1];
      accur[1] = (((frame.data.bytes[6] & 3) << 7) + (frame.data.bytes[5] & 01111111));
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[1] = true;
      }
      else
      {
        ACpres[1] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x20B: //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[2] = frame.data.bytes[1];
      accur[2] = (((frame.data.bytes[6] & 3) << 7) + (frame.data.bytes[5] & 01111111));
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[2] = true;
      }
      else
      {
        ACpres[2] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x227: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      //dccur = frame.data.bytes[7]*256+frame.data.bytes[6];
      dccur[0] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) * 0.000839233;
      dcvolt[0] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052864; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;
    case 0x229: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      //dccur = frame.data.bytes[7]*256+frame.data.bytes[6];
      dccur[1] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) * 0.000839233;
      dcvolt[1] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052864; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;
    case 0x22B: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      //dccur = frame.data.bytes[7]*256+frame.data.bytes[6];
      dccur[2] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) * 0.000839233;
      dcvolt[2] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.010528564; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;

    default:
      // if nothing else matches, do the default
      break;
  }
}

void Charger_msgs()
{
  CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.
  /////////////////////This msg addresses all modules/////////////////////////////////////////////////
  outframe.id = 0x045c;            // Set our transmission address ID
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = lowByte(parameters.voltSet);  //Voltage setpoint
  outframe.data.bytes[1] = highByte(parameters.voltSet);//Voltage setpoint
  outframe.data.bytes[2] = 0x14;
  if (bChargerEnabled) outframe.data.bytes[3] = 0x2e;
  else outframe.data.bytes[3] = 0x0e;
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x90;
  outframe.data.bytes[7] = 0x8c;
  Can0.sendFrame(outframe);
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////Phase 1 command message////////////////////////////////////////
  outframe.id = 0x042c;            // Set our transmission address ID
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = 0x42;
  outframe.data.bytes[2] = lowByte(modulelimcur); //AC Current setpoint
  outframe.data.bytes[3] = highByte(modulelimcur); //AC Current setpoint
  if (bChargerEnabled)
  {
    outframe.data.bytes[1] = 0xBB;
    outframe.data.bytes[4] = 0xFE;
  }
  else
  {
    outframe.data.bytes[1] = 0x60;
    outframe.data.bytes[4] = 0x64;
  }
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x00;
  outframe.data.bytes[7] = 0x00;
  Can0.sendFrame(outframe);
  //////////////////////////////Phase 2 command message//////////////////////////////////////////////
  outframe.id = 0x43c;        //phase 2 and 3 are copies of phase 1 so no need to set them up again
  Can0.sendFrame(outframe);
  ///////////////////////////////Phase 3 command message/////////////////////////////////////////////
  outframe.id = 0x44c;
  Can0.sendFrame(outframe);

  ///////////Static Frame every 100ms///////////////////////////////////////////////////////////////////
  outframe.id = 0x368;            // Set our transmission address ID
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = 0x03;
  outframe.data.bytes[1] = 0x49;
  outframe.data.bytes[2] = 0x29;
  outframe.data.bytes[3] = 0x11;
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x0c;
  outframe.data.bytes[6] = 0x40;
  outframe.data.bytes[7] = 0xff;
  Can0.sendFrame(outframe);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void evseread()
{
  uint16_t val = 0;
  val = analogRead(EVSE_PROX);     // read the input pin
  if ( Type == 2)
  {
    if ( val > 950)
    {
      Proximity = Unconnected;
    }
    else
    {
      Proximity = Connected;
      if ( val < 950 && val > 800)
      {
        cablelim = 13000;
      }
      if ( val < 800 && val > 700)
      {
        cablelim = 20000;
      }
      if ( val < 600 && val > 450)
      {
        cablelim = 32000;
      }
      if ( val < 400 && val > 250)
      {
        cablelim = 63000;
      }
    }
  }

  if ( Type == 1)
  {
    if ( val > 800)
    {
      Proximity = Unconnected;
    }
    else
    {
      if ( val > 550)
      {
        Proximity = Buttonpress;
      }
      else
      {
        Proximity = Connected;
      }
    }
  }
  if (debugevse != 0)
  {
    Serial.println();
    Serial.print(val );
    Serial.print("  Proximity Status : ");
    switch (Proximity)
    {
      case Unconnected:
        Serial.print("Unconnected");
        break;
      case Buttonpress:
        Serial.print("Button Pressed");
        break;
      case Connected:
        Serial.print("Connected");
        break;

    }
  }
}

void Pilotread()
{
  Pilotcalc();
}

void Pilotcalc()
{
  if (  digitalRead(EVSE_PILOT ) == HIGH)
  {
    duration = micros() - pilottimer;
    pilottimer = micros();
  }
  else
  {
    accurlim = (micros() - pilottimer) * 100 / duration * 600; //Calculate the duty cycle then multiply by 600 to get mA current limit
  }
}

void ACcurrentlimit()
{
  if (micros() - pilottimer > 1200) //too big a gap in pilot signal kills means signal error or disconnected so no current allowed.
  {
    accurlim = 0;
  }
  if (Config == Singlephase)
  {
    modulelimcur = (accurlim / 3) * 1.5 ; // all module parallel, sharing AC input current
  }
  else
  {
    modulelimcur = accurlim * 1.5; // one module per phase, EVSE current limit is per phase
  }
  if (modulelimcur > parameters.currReq) //if evse allows more current then set in parameters limit it
  {
    modulelimcur = parameters.currReq;
  }
  if (Type == 2)
  {
    if (modulelimcur > cablelim)
    {
      modulelimcur = cablelim;
    }
  }
}

