/*
  Tesla Gen 2 Charger Control Program
  2017-2018
  T de Bree
  D.Maguire
  Additional work by C. Kidder
  Runs on OpenSource Logic board V2 in Gen2 charger. Commands all modules.
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
int debugevse = 1; // 1 = show Proximity status and Pilot current limmits
int debug = 1; // 1 = show phase module CAN feedback


uint16_t curset = 0;
int  setting = 1;
int incomingByte = 0;
int state;
unsigned long tlast, tcan, tboot = 0;
bool bChargerEnabled;

//*********EVSE VARIABLE   DATA ******************
byte Proximity = 0;
uint16_t ACvoltIN = 240; // AC input voltage 240VAC for EU/UK and 110VAC for US
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

//proximity status values
#define Singlephase 0 // all parrallel on one phase Type 1
#define Threephase 1 // one module per phase Type 2

//*********Charger Control VARIABLE   DATA ******************
bool Vlimmode = true; // Set charges to voltage limit mode
uint16_t modulelimcur, dcaclim = 0;
uint16_t maxaccur = 16000; //maximum AC current in mA
uint16_t maxdccur = 45000; //max DC current outputin mA
int activemodules, slavechargerenable = 0;



//*********Feedback from charge VARIABLE   DATA ******************
uint16_t dcvolt[3] = {0, 0, 0};//1 = 1V
uint16_t dccur[3] = {0, 0, 0};
uint16_t totdccur = 0;//1 = 0.005Amp
uint16_t acvolt[3] = {0, 0, 0};//1 = 1V
uint16_t accur[3] = {0, 0, 0};//1 = 0.06666 Amp
long acpower = 0;
byte inlettarg [3] = {0, 0, 0}; //inlet target temperatures, should be used to command cooling.
byte curtemplim [3] = {0, 0, 0};//current limit due to temperature
byte templeg[2][3] = {{0, 0, 0}, {0, 0, 0}}; //temperatures reported back
bool ACpres [3] = {0, 0, 0}; //AC present detection on the modules
bool ModEn [3] = {0, 0, 0}; //Module enable feedback on the modules
bool ModFlt [3] = {0, 0, 0}; //module fault feedback
byte ModStat [3] = {0, 0, 0};//Module Status
int newframe = 0;

ChargerParams parameters;

//*********DCDC Messages VARIABLE   DATA ******************
bool dcdcenable = 1; // turn on can messages for the DCDC.

//*********Charger Messages VARIABLE   DATA ******************
int ControlID = 0x300;
int StatusID = 0x410;
unsigned long ElconID = 0x18FF50E5;
unsigned long ElconControlID = 0x1806E5F4;

int candebug = 1;


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
    parameters.currReq = 0; //max input limit per module 1500 = 1A
    parameters.enabledChargers = 123; // enable per phase - 123 is all phases - 3 is just phase 3
    parameters.mainsRelay = 48;
    parameters.voltSet = 32000; //1 = 0.01V
    parameters.autoEnableCharger = 0; //disable auto start, proximity and pilot control
    parameters.canControl = 0; //0 disabled can control, 1 master, 3 slave
    parameters.dcdcsetpoint = 14000; //voltage setpoint for dcdc in mv
    parameters.phaseconfig = Threephase; //AC input configuration
    parameters.type = 2; //Socket type1 or 2
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
  for (filter = 0; filter < 3; filter++)
  {
    Can0.setRXFilter(filter, 0, 0, true);
    Can1.setRXFilter(filter, 0, 0, true);
  }
  //standard
  for (int filter = 3; filter < 7; filter++)
  {
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


  pinMode(DIG_IN_1, INPUT); //IP1
  pinMode(DIG_IN_2, INPUT); //IP2
  //////////////////////////////////////////////////////////////////////////////////////

  //////////////DIGITAL OUTPUTS MAPPED TO X046. 10 PIN CONNECTOR ON LEFT//////////////////////////////////////////
  pinMode(DIG_OUT_1, OUTPUT); //OP1 - X046 PIN 6
  pinMode(DIG_OUT_2, OUTPUT); //OP2
  pinMode(DIG_OUT_3, OUTPUT); //OP2
  pinMode(DIG_OUT_4, OUTPUT); //OP3
  pinMode(EVSE_ACTIVATE, OUTPUT); //pull Pilot to 6V
  ///////////////////////////////////////////////////////////////////////////////////////

  dcaclim = maxaccur;

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

  if (Can1.available())
  {
    Can1.read(incoming);
    canextdecode(incoming);
  }

  if (Serial.available())
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 'a'://a for auto enable
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

      case 'p'://a for can control enable
        if (Serial.available() > 0)
        {
          parameters.phaseconfig = Serial.parseInt() - 1;
          if ( parameters.phaseconfig == 2)
          {
            parameters.phaseconfig = Threephase;
          }
          if (parameters.phaseconfig == 0)
          {
            parameters.phaseconfig = Singlephase;
          }
          setting = 1;
        }
        break;

      case 't'://t for type
        if (Serial.available() > 0)
        {
          parameters.type = Serial.parseInt();
          if (parameters.type > 2)
          {
            parameters.type = 2;
          }
          if (parameters.type == 0)
          {
            parameters.type = 2;
          }
          setting = 1;
        }
        break;

      case 'x'://a for can control enable
        if (Serial.available() > 0)
        {
          parameters.canControl = Serial.parseInt();
          if (parameters.canControl > 3)
          {
            parameters.canControl = 0;
          }
          setting = 1;
        }
        break;

      case 'm'://m for dc current setting in whole numbers
        if (Serial.available() > 0)
        {
          maxdccur = (Serial.parseInt() * 1000);
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

      case 's'://s for start AND stop
        if (Serial.available() > 0)
        {
          setting = 1;
          digitalWrite(LED_BUILTIN, HIGH);
          if (state == 0)
          {
            state = 2;// initialize modules
            tboot = millis();
          }
          if (state == 1)
          {
            state = 0;// initialize modules
          }
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
    Serial.print("V | Set current lim AC : ");
    Serial.print(parameters.currReq * 0.00066666, 0);
    Serial.print(" A DC :");
    Serial.print(maxdccur * 0.001, 1);
    Serial.print(" A ");
    if (parameters.autoEnableCharger == 1)
    {
      Serial.print(" Autostart On   ");
    }
    else
    {
      Serial.print(" Autostart Off   ");
    }
    if (parameters.canControl == 1)
    {
      Serial.print(" Can Mode: Master ");
    }
    if (parameters.canControl == 2)
    {
      Serial.print(" Can Mode: Master Elcon ");
    }
    if (parameters.canControl == 3)
    {
      Serial.print(" Can Mode: Slave ");
    }
    if (parameters.phaseconfig == Singlephase)
    {
      Serial.print(" Single Phase ");
    }
    if (parameters.phaseconfig == Threephase)
    {
      Serial.print(" Three Phase ");
    }
    if (parameters.type == 1)
    {
      Serial.print(" Type 1 ");
    }
    if (parameters.type == 2)
    {
      Serial.print(" Type 2 ");
    }
    setting = 0;
    Serial.println();
    Serial.println();
  }
  if (parameters.canControl > 1)
  {
    /*if (millis() - tcan > 500)
      {
      state = 0;
      Serial.println();
      Serial.println("CAN time-out");
      }
    */
  }

  switch (state)
  {
    case 0: //Charger off
      if (bChargerEnabled == true)
      {
        bChargerEnabled = false;
      }
      digitalWrite(DIG_OUT_1, LOW);//MAINS OFF
      digitalWrite(EVSE_ACTIVATE, LOW);
      digitalWrite(CHARGER1_ACTIVATE, LOW); //chargeph1 deactivate
      digitalWrite(CHARGER2_ACTIVATE, LOW); //chargeph2 deactivate
      digitalWrite(CHARGER3_ACTIVATE, LOW); //chargeph3 deactivate
      digitalWrite(CHARGER1_ENABLE, LOW);//disable phase 1 power module
      digitalWrite(CHARGER2_ENABLE, LOW);//disable phase 2 power module
      digitalWrite(CHARGER3_ENABLE, LOW);//disable phase 3 power module
      break;

    case 1://Charger on
      if (digitalRead(DIG_IN_1) == HIGH)
      {
        if (bChargerEnabled == false)
        {
          bChargerEnabled = true;
          switch (parameters.enabledChargers)
          {
            case 1:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              activemodules = 1;
              break;
            case 2:
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              activemodules = 1;
              break;
            case 3:
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 1;
              break;
            case 12:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              activemodules = 2;
              break;
            case 13:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 2;
              break;
            case 123:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 3;
              break;
            case 23:
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 2;
              break;
            default:
              // if nothing else matches, do the default
              // default is optional
              break;
          }
          delay(100);
          digitalWrite(DIG_OUT_1, HIGH);//MAINS ON
          digitalWrite(EVSE_ACTIVATE, HIGH);
        }
      }
      else
      {
        bChargerEnabled == false;
        state = 0;
      }
      break;

    case 2:
      switch (parameters.enabledChargers)
      {
        case 1:
          digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
          break;
        case 2:
          digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
          break;
        case 3:
          digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
          break;
        case 12:
          digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
          digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
          break;
        case 13:
          digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
          digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
          break;
        case 123:
          digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
          digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
          digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
          break;
        case 23:
          digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
          digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
          break;

        default:
          // if nothing else matches, do the default
          break;
      }
      if (tboot <  (millis() - 500))
      {
        state = 1;
      }

    default:
      // if nothing else matches, do the default
      break;
  }
  if (tlast <  (millis() - 500))
  {
    tlast = millis();
    if (debug != 0)
    {
      Serial.println();
      Serial.print(millis());
      Serial.print(" State: ");
      Serial.print(state);
      if (bChargerEnabled)
      {
        Serial.print(" ON  ");
      }
      else
      {
        Serial.print(" OFF ");
      }
      if (digitalRead(DIG_IN_1) == HIGH)
      {
        Serial.print(" D1 H");
      }
      else
      {
        Serial.print(" D1 L");
      }


      if (bChargerEnabled)
      {
        Serial.println();
        for (int x = 0; x < 3; x++)
        {
          Serial.print("  Phase ");
          Serial.print(x + 1);
          Serial.print(" Feebback //  AC present: ");
          Serial.print(ACpres[x]);
          Serial.print("  AC volt: ");
          Serial.print(acvolt[x]);
          Serial.print("  AC cur: ");
          Serial.print((accur[x] * 0.06666), 2);
          Serial.print("  ");
          Serial.print(accur[x]);
          Serial.print("  DC volt: ");
          Serial.print(dcvolt[x]);
          Serial.print("  DC cur: ");
          Serial.print(dccur[x] * 0.000839233, 2);
          Serial.print("  Inlet Targ: ");
          Serial.print(inlettarg[x]);
          Serial.print("  Temp Lim Cur: ");
          Serial.print(curtemplim[x]);
          Serial.print("  ");
          Serial.print(templeg[0][x]);
          Serial.print("  ");
          Serial.print(templeg[1][x]);
          Serial.print(" EN:");
          Serial.print(ModEn[x]);
          Serial.print(" Flt:");
          Serial.print(ModFlt[x]);
          Serial.print(" Stat:");
          Serial.print(ModStat[x], BIN);
          Serial.println();
        }
      }
      else
      {
        Serial.println();
        Serial.print("Modules Turned OFF");
        Serial.println();
      }
    }
    if (debugevse != 0)
    {
      Serial.println();
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
      /*
        Serial.print(" AC limit : ");
        Serial.print(accurlim);
      */
      Serial.print(" Cable Limit: ");
      Serial.print(cablelim);
      Serial.print(" Module Cur Request: ");
      Serial.print(modulelimcur / 1.5, 0);
      /*
        Serial.print(" DC AC Cur Lim: ");
        Serial.print(dcaclim);
        Serial.print(" Active: ");
        Serial.print(activemodules);
      */
      Serial.print(" DC total Cur:");
      Serial.print(totdccur * 0.005, 2);
      Serial.print(" DC Setpoint:");
      Serial.print(parameters.voltSet * 0.01, 0);
    }

  }
  DCcurrentlimit();
  ACcurrentlimit();

  //EVSE automatic control

  evseread();
  if (Proximity == Connected) //check if plugged in
  {
    //digitalWrite(EVSE_ACTIVATE, HIGH);//pull pilot low to indicate ready - NOT WORKING freezes PWM reading
    if (modulelimcur > 1400) // one amp or more active modules
    {
      if (parameters.autoEnableCharger == 1)
      {
        if (state == 0)
        {
          if (digitalRead(DIG_IN_1) == HIGH)
          {
            state = 2;// initialize modules
            tboot = millis();
          }
        }
      }
    }
    digitalWrite(DIG_OUT_2, HIGH); //enable AC present indication
  }
  else // unplugged or buton pressed stop charging
  {
    state = 0;
    digitalWrite(DIG_OUT_2, LOW); //disable AC present indication
    digitalWrite(EVSE_ACTIVATE, LOW);
  }
}



void candecode(CAN_FRAME & frame)
{
  int x = 0;
  switch (frame.id)
  {
    case 0x217: //phase 1 Status message
      ModStat[0] = frame.data.bytes[0];
      break;

    case 0x219: //phase 2 Status message
      ModStat[1] = frame.data.bytes[0];
      break;

    case 0x21B: //phase 3 Status message
      ModStat[2] = frame.data.bytes[0];
      break;

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
      accur[0] = (uint16_t((frame.data.bytes[5] & 0x7F) << 2) | uint16_t(frame.data.bytes[6] >> 6)) ;
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[0] = true;
      }
      else
      {
        ACpres[0] = false;
      }
      x = frame.data.bytes[2] & 0x40;
      if (x != 0)
      {
        ModEn[0] = true;
      }
      else
      {
        ModEn[0] = false;
      }
      x = frame.data.bytes[2] & 0x20;
      if (x != 0)
      {
        ModFlt[0] = true;
      }
      else
      {
        ModFlt[0] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x209: //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[1] = frame.data.bytes[1];
      accur[1] = (uint16_t((frame.data.bytes[5] & 0x7F) << 2) | uint16_t(frame.data.bytes[6] >> 6)) ;
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[1] = true;
      }
      else
      {
        ACpres[1] = false;
      }
      x = frame.data.bytes[2] & 0x40;
      if (x != 0)
      {
        ModEn[1] = true;
      }
      else
      {
        ModEn[1] = false;
      }
      x = frame.data.bytes[2] & 0x20;
      if (x != 0)
      {
        ModFlt[1] = true;
      }
      else
      {
        ModFlt[1] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x20B: //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[2] = frame.data.bytes[1];
      accur[2] = (uint16_t((frame.data.bytes[5] & 0x7F) << 2) | uint16_t(frame.data.bytes[6] >> 6)) ;
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[2] = true;
      }
      else
      {
        ACpres[2] = false;
      }
      x = frame.data.bytes[2] & 0x40;
      if (x != 0)
      {
        ModEn[2] = true;
      }
      else
      {
        ModEn[2] = false;
      }
      x = frame.data.bytes[2] & 0x20;
      if (x != 0)
      {
        ModFlt[2] = true;
      }
      else
      {
        ModFlt[2] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x227: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      dccur[0] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) ;//* 0.000839233 convert in rest of code
      dcvolt[0] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052864; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;
    case 0x229: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      dccur[1] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) ;//* 0.000839233 convert in rest of code
      dcvolt[1] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052864; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;
    case 0x22B: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      dccur[2] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) ;//* 0.000839233 convert in rest of code
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
  if (bChargerEnabled)
  {
    outframe.data.bytes[3] = 0x2e;
  }
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
    outframe.data.bytes[1] = lowByte(uint16_t(ACvoltIN / 1.2));
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
  /*////////////////////////////////////////////////////////////////////////////////////////////////////////
    External CAN
    ////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  uint16_t y, z = 0;
  outframe.id = StatusID;
  if (parameters.canControl == 3)
  {
    outframe.id = StatusID + 1;
  }
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = 0x00;
  for (int x = 0; x < 3; x++)
  {
    y = y +  dcvolt[x] ;
  }
  outframe.data.bytes[0] = y / 3;

  if (parameters.phaseconfig == Singlephase)
  {
    for (int x = 0; x < 3; x++)
    {
      z = z + (accur[x] * 66.66) ;
    }
  }
  else
  {
    z = accur[2] * 66.66;
  }

  outframe.data.bytes[1] = lowByte (z);
  outframe.data.bytes[2] = highByte (z);

  outframe.data.bytes[3] = lowByte (uint16_t (totdccur)); //0.005Amp
  outframe.data.bytes[4] = highByte (uint16_t (totdccur));  //0.005Amp
  outframe.data.bytes[5] = lowByte (uint16_t (modulelimcur * 0.66666));
  outframe.data.bytes[6] = highByte (uint16_t (modulelimcur * 0.66666));
  outframe.data.bytes[7] = 0x00;
  outframe.data.bytes[7] = Proximity << 6;
  outframe.data.bytes[7] = outframe.data.bytes[7] || (parameters.type << 4);
  Can1.sendFrame(outframe);

  /////////Elcon Message////////////

  outframe.id = ElconID;
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 1;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request


  outframe.data.bytes[0] = highByte (y * 10 / 3);
  outframe.data.bytes[1] = lowByte (y * 10 / 3);
  outframe.data.bytes[2] = highByte (uint16_t (totdccur * 20)); //0.005Amp conv to 0.1
  outframe.data.bytes[3] = lowByte (uint16_t (totdccur * 20)); //0.005Amp conv to 0.1
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x00;
  outframe.data.bytes[7] = 0x00;
  Can1.sendFrame(outframe);


  ///DCDC CAN//////////////////////////////////////////////////////////////////////
  if (dcdcenable)
  {
    outframe.id = 0x3D8;
    outframe.length = 3;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    outframe.data.bytes[0] = highByte (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);
    outframe.data.bytes[1] = lowByte (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);

    outframe.data.bytes[1] = outframe.data.bytes[1] | 0x20;
    outframe.data.bytes[2] = 0x00;
    Can1.sendFrame(outframe);
  }

  ////////////////////////////////////////////////////////////////////

  if (parameters.canControl == 1)
  {
    outframe.id = ControlID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    outframe.data.bytes[0] = 0;

    if (state != 0)
    {
      if ( slavechargerenable == 1)
      {
        outframe.data.bytes[0] = 0x01;
      }
    }

    outframe.data.bytes[1] = highByte(parameters.voltSet);
    outframe.data.bytes[2] = lowByte(parameters.voltSet);
    outframe.data.bytes[3] = highByte(maxdccur);
    outframe.data.bytes[4] = lowByte(maxdccur);
    outframe.data.bytes[5] = highByte(modulelimcur);
    outframe.data.bytes[6] = lowByte(modulelimcur);
    outframe.data.bytes[7] = 0;

    Can1.sendFrame(outframe);
  }
}

void evseread()
{
  uint16_t val = 0;
  val = analogRead(EVSE_PROX);     // read the input pin
  if ( parameters.type == 2)
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

  if ( parameters.type == 1)
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
  if (parameters.autoEnableCharger == 1)
  {
    if (micros() - pilottimer > 1200) //too big a gap in pilot signal kills means signal error or disconnected so no current allowed.
    {
      accurlim = 0;
    }
    if (parameters.phaseconfig == Singlephase)
    {
      modulelimcur = (accurlim / 3) * 1.5 ; // all module parallel, sharing AC input current
    }
    else
    {
      modulelimcur = accurlim * 1.5; // one module per phase, EVSE current limit is per phase
    }
    if (parameters.type == 2)
    {
      if (modulelimcur > (cablelim * 1.5))
      {
        modulelimcur = cablelim * 1.5;
      }
    }
  }
  else
  {
    if (parameters.phaseconfig == Singlephase)
    {
      modulelimcur = (parameters.currReq / 3); // all module parallel, sharing AC input current
    }
  }
  if (parameters.canControl == 1)
  {
    if (modulelimcur > (15000 * 1.5)) //enable second charger if current available >15A
    {
      modulelimcur = modulelimcur * 0.5;
      slavechargerenable = 1;

    }
    else
    {
      slavechargerenable = 0;
    }
  }
  if (parameters.phaseconfig == Threephase)
  {
    if (modulelimcur > parameters.currReq) //if evse allows more current then set in parameters limit it
    {
      modulelimcur = parameters.currReq;
    }
  }
  else
  {
    if (modulelimcur > (parameters.currReq / activemodules)) //if evse allows more current then set in parameters limit it
    {
      modulelimcur = (parameters.currReq / activemodules);
    }
  }
  /*
    if (modulelimcur > (dcaclim * 1.5)) //if more current then max per module or limited by DC output current
    {
    modulelimcur = (dcaclim * 1.5);
    }
  */
}

void DCcurrentlimit()
{
  totdccur = 1; // 0.005Amp
  activemodules = 0;
  for (int x = 0; x < 3; x++)
  {
    totdccur = totdccur + (dccur[x] * 0.1678466) ;
    if (acvolt[x] > 50 && dcvolt[x] > 50)
    {
      activemodules++;
    }
  }
  dcaclim = 0;
  int x = 2;
  dcaclim = ((dcvolt[x] * (maxdccur + 400)) / acvolt[x]) / activemodules;
}

void canextdecode(CAN_FRAME & frame)
{
  int x = 0;
  if (parameters.canControl == 2)
  {
    if (ElconControlID == frame.id) //Charge Control message
    {
      parameters.voltSet = ((frame.data.bytes[0] << 8) + frame.data.bytes[1]) * 0.1;
      maxdccur = (frame.data.bytes[2] << 8) + frame.data.bytes[3];

      if (frame.data.bytes[4] & 0x01 == 1)
      {
        if (state == 0)
        {
          state = 2;
          tboot = millis();
        }
      }
      else
      {
        state = 0;
      }
      if (candebug == 1)
      {
        Serial.println();
        Serial.print( state);
        Serial.print(" ");
        Serial.print(parameters.voltSet);
        Serial.print(" ");
        Serial.print(modulelimcur);
        Serial.println();
      }
      tcan = millis();
    }
  }

  if (parameters.canControl == 3)
  {
    if (ControlID == frame.id) //Charge Control message
    {
      if (frame.data.bytes[0] & 0x01 == 1)
      {
        if (state == 0)
        {
          state = 2;
          tboot = millis();
        }
      }
      else
      {
        state = 0;
      }
      parameters.voltSet = (frame.data.bytes[1] << 8) + frame.data.bytes[2];
      maxdccur = (frame.data.bytes[3] << 8) + frame.data.bytes[4];
      modulelimcur  = (frame.data.bytes[5] << 8) + frame.data.bytes[6];
      if (candebug == 1)
      {
        Serial.println();
        Serial.print( state);
        Serial.print(" ");
        Serial.print(parameters.voltSet);
        Serial.print(" ");
        Serial.print(modulelimcur);
        Serial.println();
      }

      tcan = millis();
    }
  }

}

