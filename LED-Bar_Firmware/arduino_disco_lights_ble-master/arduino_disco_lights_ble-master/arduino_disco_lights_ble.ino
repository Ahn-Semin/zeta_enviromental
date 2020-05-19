/*********************************************************************

This is an example for music controlled Arduino NEOPIXEL strip using
"Disco Lights BLE" Android application.

It utilizes

 1.) Arduino UNO

 2.) nRF8001 Bluetooth Low Energy Breakout from the adafruit http://www.adafruit.com/products/1697
     and library written by Kevin Townsend/KTOWN https://github.com/adafruit/Adafruit_nRF8001.

     A guide for setup at https://learn.adafruit.com/getting-started-with-the-nrf8001-bluefruit-le-breakout. 

 3.) FastLED library for controlling NEOPIXEL strip https://github.com/FastLED/FastLED

 4.) Two effects written by Mark Kriegsman and ported to the VisualizationScheduler environment:

     a. Twinkling 'holiday' lights
     b. DiscoStrobe
     
 5.) Added sound ripple effect.

 6.) VisualizationScheduler for managing different effects in time or space.

 7.) "Disco Lights BLE" Application Communication Protocol.

 8.) "Disco Lights BLE" Android application which provides beat and other information 
     for music controlled visualization.

Steps:

1.) Download nRF8001 BLE Breakout library and copy in to Arduino folder

2.) Download 'FastLed3.x' and copy in to Arduino folder

3.) Copy this folder in to Arduino folder

4.) Adjust HW setting according the your HW setup
    
    The sketch HW configuration is

    a. Arduino Uno

    b. nRF8001 BLE Breakout is connected same way as described in guide above 
       (https://learn.adafruit.com/getting-started-with-the-nrf8001-bluefruit-le-breakout/hooking-everything-up).

    c. NEOPIXEL controller is connected to pin 6.

MIT license, check LICENSE for more information
All text above must be included in any redistribution
*********************************************************************/

///// BLE UART LIB ///////////////////////////////////////////////////////

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2  // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

///// "DISCO LIGHTS BLE" COMMUNICATION PROTOCOL

#include "DiscoLightsBleProtocol.h"

struct Notification
{
  void onGeneralInfo();
  void onRhythmInfo();
};

DiscoLightsBleProtocolI2<Notification> ProtocolLayer;

///// FASTLED LIB AND VISUALIZATION SCHEDULER WITH EFFECTS LIB

#include <FastLED.h>
#include "VisualizationScheduler.h"
#include "EffectTwinklingHolidayLights.h"
#include "EffectSoundRippleLights.h"
 
struct NeopixelHardwareConfiguration
{
  enum { NumofLeds      = 150   };
  enum { ControllerPin  = 6     };
  enum { MaxCurrentInmA = 10000 };
  enum { TickTimeInMs   = 0     }; // NO WAIT INSIDE SINCE THE BLE HW MUST BE POLLED //
};
 
struct EffectsConfiguration
{
  enum { NumofEffects        = 2  };
  enum { EffectTimeoutInSecs = 30 };
  enum { ExecutionOrder      = EffectOrderIncreasing };
  
  static void addEffects();
}; 

NEOPIXEL_STRIP_DEFINE_EFFECT_SCHEDULER(EffectsConfiguration);
NEOPIXEL_STRIP_DEFINE_VISUALIZATION_IFACE(NeopixelHardwareConfiguration);

// ADDING EFFECTS ON THE STRIP //

EffectTwinklingHolidayLights < 0, NeopixelHardwareConfiguration::NumofLeds > gHolidayLights;
EffectSoundRippleLights      < 0, NeopixelHardwareConfiguration::NumofLeds > gSoundRippleLights;

void EffectsConfiguration::addEffects()
{
  NEOPIXEL_STRIP_EFFECT_INSERT( &gHolidayLights,     0 );
  NEOPIXEL_STRIP_EFFECT_INSERT( &gSoundRippleLights, 1 );
}

///// STATES AND STATUSES

bool      RhythmState;
uint8_t   IntensityRaw;
uint16_t  IntensityEma;
int       CountInMs;

inline
void CalculateIntensityEma()
{
  enum { EmaAlpha   = 64           };
  enum { EmaAlphaM1 = 255-EmaAlpha };
  
  IntensityRaw = ProtocolLayer.PopIntensityC1();
  IntensityEma = (IntensityEma>>8) * (uint16_t)EmaAlphaM1 
               + (uint16_t)IntensityRaw * (uint16_t)EmaAlpha;
}

inline
bool UpdateIntensityData ( )
{
  int  ms = millis();
  int  de = CountInMs - ms;
  bool do_update = false;

  if(ProtocolLayer.TickInMs()!=0 )
  {
    if(de<0)
    {
      do_update = true;
      CountInMs = ms;
     
      CalculateIntensityEma ( );
    }
    else
    {
      while(de>=ProtocolLayer.TickInMs())
      {
        de       -= ProtocolLayer.TickInMs();
        do_update = true;
  
        CalculateIntensityEma ( );
      }
      
      CountInMs = ms - de;
    }
  }
  return do_update;
}

inline
unsigned char GetIntensityEma()
{
  uint8_t intensity = IntensityEma>>8;

  return intensity;
}

inline
void ResetLocalVars ( )
{
  IntensityEma = 0;
  IntensityRaw = 0;
  CountInMs    = 0;
}

/// ARDUINO SETUP ///

inline
void DisconnectProtocol()
{
  ProtocolLayer.Init();
  ResetLocalVars();

  NEOPIXEL_STRIP_EFFECT_SET_ENABLE(&gHolidayLights,     true);
  NEOPIXEL_STRIP_EFFECT_SET_ENABLE(&gSoundRippleLights, false);

  NEOPIXEL_STRIP_GLOBAL_BRIGHTNESS(255);
}

void setup(void)
{ 
  // DEBUGGING //
  
  Serial.begin(115200);
  while(!Serial); // Leonardo/Micro should wait for serial init

  Serial.write("Arduino BLE started\n");

  // BLE UART //
  
  BTLEserial.begin();

  // VISUALIZATION SCHEDULER & PROTOCOL LAYER //
  
  NEOPIXEL_STRIP_ARDUINO_SETUP();
  
  // switching to manual mode: controlling effects using music recovered context ... //
  
  NEOPIXEL_STRIP_MANUAL_MODE(true);
  
  DisconnectProtocol();
}

/// ARDUINO LOOP ///

inline 
void UpdateBleUartState()
{
  static aci_evt_opcode_t last_status = ACI_EVT_DISCONNECTED;
  
  BTLEserial.pollACI();

  aci_evt_opcode_t status = BTLEserial.getState();
  if (status != last_status)
  { 
    if( status == ACI_EVT_DEVICE_STARTED || status == ACI_EVT_DISCONNECTED)
    {
	    DisconnectProtocol();

      Serial.write('DISCONNECTED\n');
    }
    
    last_status = status;
  }

  if (status == ACI_EVT_CONNECTED) 
  {
    while (BTLEserial.available()) 
    {
      ProtocolLayer.FrameDecode(BTLEserial.read());
    }
  }
}

inline
void ProtocolLayerSendsDevNotif( uint8_t effect_index )
{
  if(ProtocolLayer.FillBufferWithNotification(effect_index)==true )
  {
    uint8_t length = ProtocolLayer.Encode();
    BTLEserial.write(ProtocolLayer.Buffer(),length+1);
  }
}

inline
void UpdateProtocol()
{
  bool stop_read_cmd = false;

  while(stop_read_cmd==false)
  {
    unsigned short cmd   = ProtocolLayer.CommandQueueRead();
    unsigned char  param = cmd>>8;

    switch( (cmd&0xff) ) 
    {
    case ProtoID_Unknown:
      stop_read_cmd = true;
      break;
    case ProtoID_DeviceCapabilityI2:
      if( ProtocolLayer.FillBufferWithDeviceCapabilityI2Frame( eLedTypeStrip, 0/*high byte*/, NeopixelHardwareConfiguration::NumofLeds/*low byte*/, 2, eFeatureSupportEffectInfo )==true )
      {
        uint8_t length = ProtocolLayer.Encode();
        BTLEserial.write(ProtocolLayer.Buffer(),length+1);
      }
      break;
    case ProtoID_EffectInfo:
      if(param==1) 
      {
      	if( ProtocolLayer.FillBufferWithEffectInfo( param, 0, gHolidayLights.Name() )==true )
      	{
      	  uint8_t length = ProtocolLayer.Encode();
      	  BTLEserial.write(ProtocolLayer.Buffer(),length+1);
      	}
      }
      else
      if(param==2)
      {
      	if( ProtocolLayer.FillBufferWithEffectInfo( param, 0, gSoundRippleLights.Name() )==true )
      	{
      	  uint8_t length = ProtocolLayer.Encode();
      	  BTLEserial.write(ProtocolLayer.Buffer(),length+1);
      	}
      }
      break;
    case ProtoID_GetDevStatus:
	    ProtocolLayerSendsDevNotif(RhythmState==true?2:1);
      break;
    }
  }  
}

void UpdateSchedulerInManualMode()
{
  if( RhythmState==false )
  {
    if( ProtocolLayer.IsBpmValid()==true && GetIntensityEma() > 60 )
    {
      RhythmState = true;
      
      NEOPIXEL_STRIP_EFFECT_SET_ENABLE(&gHolidayLights, false);
    
      NEOPIXEL_STRIP_EFFECT_SET_ENABLE(&gSoundRippleLights, true);
      gSoundRippleLights.UpdateFs(ProtocolLayer.TickInMs());

      NEOPIXEL_STRIP_GLOBAL_BRIGHTNESS(255);

      ProtocolLayerSendsDevNotif(2);
    }
  }
  else
  if ( RhythmState==true )
  {
    if ( ProtocolLayer.IsRhythmTracked()==false || GetIntensityEma() < 30 )
    {
      RhythmState = false;

      NEOPIXEL_STRIP_EFFECT_SET_ENABLE(&gHolidayLights,     true);
      NEOPIXEL_STRIP_EFFECT_SET_ENABLE(&gSoundRippleLights, false);
    
      if(ProtocolLayer.IsReady()==true)
      {
        // It connected to "Disco Lights BLE". //
        
  	    ProtocolLayerSendsDevNotif(1);
      }
    }
  }
}

void Notification::onGeneralInfo()
{
  // updating effects with emotion information //
  gSoundRippleLights.UpdateEmotion(ProtocolLayer.BasicEmotion());
}

void Notification::onRhythmInfo()
{
  // updating effects with rhythm information //  
  gSoundRippleLights.UpdateRhythm( ProtocolLayer.GetRhythmCursorInSamples(), ProtocolLayer.GetRhythmEndInSamples());
}

void loop()
{
  // BLE HW AND PROTOCOL LAYER UPDATE //
  
  UpdateBleUartState();

  UpdateProtocol();

  // VISUALIZATION SCHEDULER IN MANUAL MODE //
  
  UpdateSchedulerInManualMode();

  if(ProtocolLayer.IsReady()==true)
  {
    if(UpdateIntensityData()==true)
    {
      // updating effects with intensity information //
      if(IntensityRaw > 30)
      {
        gSoundRippleLights.UpdateIntensity(IntensityRaw);

        if(RhythmState == false)
        {
          NEOPIXEL_STRIP_GLOBAL_BRIGHTNESS(IntensityRaw);       
        }
      }
    }
  }
  
  NEOPIXEL_STRIP_ARDUINO_LOOP();
}


