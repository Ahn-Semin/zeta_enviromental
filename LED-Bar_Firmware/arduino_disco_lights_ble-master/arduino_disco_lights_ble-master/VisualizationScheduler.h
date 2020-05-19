/********************************************************************************
This is the VisualizationScheduler for FastLED 3.+ library.
It manages effects in time or space.

Copyright (c) 2015-2016 Irina Riegert

MIT license, check LICENSE for more information
All text above must be included in any redistribution.
********************************************************************************/

#if !defined(__LED_VISUALIZATION_SCHEDULER_H__)
#define __LED_VISUALIZATION_SCHEDULER_H__

// Example of Neopixel strip hardware configuration:
//
// struct HardwareConfiguration
// {
//   enum { NumofLeds      = 60  };
//   enum { ControllerPin  = 6   };
//   enum { MaxCurrentInmA = 500 };
//   enum { TickTimeInMs   = 10  };
// };
//
// struct NeopixelVisualizationIface<HardwareConfiguration,EffectIface>

template< typename Configuration, class EffectIface >
struct NeopixelVisualizationIface
{
  static struct CRGB mLeds[Configuration::NumofLeds];
 
  inline 
  static void setup()
  {
    // reset all LEDs
    
    FastLED.addLeds<NEOPIXEL,Configuration::ControllerPin>(mLeds, Configuration::NumofLeds);
    FastLED.clearData();
    FastLED.show();
    
    // Initializing effects ( effects may use not all LEDS )

    EffectIface::Init();
    
    FastLED.addLeds<NEOPIXEL,Configuration::ControllerPin>(mLeds, EffectIface::TotalLength()).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(EffectIface::GetBrightness());
    set_max_power_in_volts_and_milliamps(5, Configuration::MaxCurrentInmA);
  }

  inline
  static void loop()
  {
    if( EffectIface::Update()==true )
      show_at_max_brightness_for_power();
    if(Configuration::TickTimeInMs!=0)
      delay_at_max_brightness_for_power(Configuration::TickTimeInMs);
  }

  inline
  static void set_brightness( uint8_t brightness )
  {
     FastLED.setBrightness(brightness);
  }
};

template< typename Configuration,  class EffectIface >
struct CRGB NeopixelVisualizationIface<Configuration,EffectIface>::mLeds[Configuration::NumofLeds];

//
// class Fading

struct Fading
{
  inline
  static uint8_t fade ( uint8_t val, uint8_t fadeTime )
  {
    if (val == 0)
      return val;
      
    uint8_t subAmt = val >> fadeTime;
    if (subAmt < 1)
      subAmt = 1;
    val -= subAmt;
    
    return val;
  }

  inline
  static void Update ( CRGB *leds, int numof_leds, uint8_t fadeTime )
  {
    for(int i = 0; i < numof_leds; i++)
    {
      if (leds[i].r > 0)
      {
        leds[i].r = fade(leds[i].r, fadeTime);
        
        if (leds[i].r % 2)
        {
          (leds[i].r)--;
        }
      }
      
      if (leds[i].g > 0)
      {
        leds[i].g = fade(leds[i].g, fadeTime);
        
        if (leds[i].g % 2)
        {
          leds[i].g--;
        }
      }

      if (leds[i].b > 0)
      {
        leds[i].b = fade(leds[i].b, fadeTime);
        
        if (leds[i].b % 2)
        {
          leds[i].b--;
        }
      }
    }
  }
};

//
// class EffectIface

class EffectIface
{
protected:
  EffectIface() 
    : mEnabled         (false)
    , mIndexStart      (0)
    , mNumofLeds       (0)
    , mUpdatePeriodInMs(0)
    , mBrightness      (0) 
  {}

public:
  virtual ~EffectIface() {}

  virtual void Init() = 0;
  virtual void Processing(int delta) = 0;

  bool    mEnabled;
  int     mIndexStart;
  int     mNumofLeds;
  int     mUpdatePeriodInMs;
  uint8_t mBrightness;
};


enum { EffectOrderRandomized = 0
     , EffectOrderIncreasing = 1
     , EffectOrderDecreasing = 2 };

// Example of effects scheduler configuration:
//
// struct ScheduleConfiguration
// {
//   enum { NumofEffects        = 3  };
//   enum { EffectTimeoutInSecs = 0  };
//   enum { ExecutionOrder      = EffectOrderRandomized };
//
//   static void addEffects();
// };
//
// struct VisualizationEffectSchedule<ScheduleConfiguration>

template<typename Configuration>
struct VisualizationEffectSchedule
{
  enum { StateIdle         = 0
       , StateSingleEffect = 1
       , StateRun          = 2
       , StateFade         = 3 };

  enum { OneSecondTimeout  = 1000 /*ms*/ };

  enum { FadePeriod        = 1000/40 }; // 25 frame per second
  enum { FadeTime          = 62 };      // ~2.5 seconds
  
  static uint8_t      mState;
  static int          mUsedStripLength;
  static int          mLastUpdateTimeInMs;
  static int          mOneSecondTimer;
  static int          mEffectSwitchTimer;
  static int          mTimeQueue[Configuration::NumofEffects];
  static EffectIface *mList[Configuration::NumofEffects];

  inline 
  static bool SetEffect( EffectIface *ptr, uint8_t index )
  {
    if(index>=Configuration::NumofEffects)
      return false;
    mList[index] = ptr;
    return true;
  }

  inline 
  static int TotalLength()
  {
    return mUsedStripLength;
  }

  static void Clear()
  {
    mState              = StateIdle;
    mUsedStripLength    = 0;
    mLastUpdateTimeInMs = 0;
    mOneSecondTimer     = 0;
    mEffectSwitchTimer  = 0;

    memset(&mList,NULL,sizeof(mList));
  }

  static uint8_t GetBrightness()
  {
    // case A: look for maximal brightness
    //uint8_t br = 0;
    //for(int ii=0;ii<NumofEffects;++ii)
    //{
    //  if(mList[ii]==NULL)
    //    continue;
    //  if( mList[idx]->mEnabled == false )
    //    continue;
    //  if(br<mList[ii]->mBrightness)
    //    br=mList[ii]->mBrightness;
    //}

    // case B: look for minimal brightness
    uint8_t br = 0xff;
    for(int ii=0;ii<Configuration::NumofEffects;++ii)
    {
      if( mList[ii] == NULL )
        continue;
      if( mList[ii]->mEnabled == false )
        continue;
      if( br > mList[ii]->mBrightness )
        br = mList[ii]->mBrightness;
    }

    return br;
  }

  static void InitializeEffects()
  {
    int effects = 0;

    mUsedStripLength = 0;

    for(int ii=0;ii<Configuration::NumofEffects;++ii)
    {
      if(mList[ii]==NULL)
        continue;

      ++effects;

      mList[ii]->Init();

      int end = mList[ii]->mIndexStart + mList[ii]->mNumofLeds;
      if( mUsedStripLength < end )
        mUsedStripLength = end;

      mTimeQueue[ii] = mList[ii]->mUpdatePeriodInMs;
    }

    ++mUsedStripLength;

    if( effects!=0 )
      mState = StateRun;
    else
      mState = StateIdle;

    if( mState == StateRun )
    {
      // 1st correction: if only one algorithm inserted and random selection feature is enabled
      if(Configuration::EffectTimeoutInSecs > 0 && Configuration::NumofEffects == 1)
      {
        mList[0]->mEnabled = true;
        FastLED.setBrightness(mList[0]->mBrightness);
        mState = StateSingleEffect;
      }

      // 2nd correction: multiple effects at same time on the strip.
      if( Configuration::EffectTimeoutInSecs==0 )
      {
        // remark: a sketch should enable effects which should be shown.
        mState = StateSingleEffect;
        FastLED.setBrightness(GetBrightness());
      }

      if( mState==StateRun )
        SelectEffect();
    }

    mLastUpdateTimeInMs = millis();
  }

  static void SelectEffect()
  {
      uint8_t idx;
      
      switch(Configuration::ExecutionOrder)
      {
      case EffectOrderRandomized:
      default:
        idx = random8() % Configuration::NumofEffects;
        for( uint8_t ii = 0; ii < Configuration::NumofEffects; ++ii )
        {
          if( mList[idx]!=NULL )
            break;
          if( ++idx >= Configuration::NumofEffects )
            idx = 0;
        }
        break;
      case EffectOrderIncreasing:
        for( idx = 0; idx < Configuration::NumofEffects; ++idx )
        {
          if( mList[idx] == NULL )
            continue;
          if( mList[idx]->mEnabled == false )
            continue;
          break;
        }
        for( uint8_t ii = 0; ii < Configuration::NumofEffects; ++ii )
        {
          if( ++idx >= Configuration::NumofEffects )
            idx = 0;
          if( mList[idx] == NULL )
            continue;
          break;
        }
        break;
      case EffectOrderDecreasing:
        for( idx = 0; idx < Configuration::NumofEffects; ++idx )
        {
          if( mList[idx] == NULL )
            continue;
          if( mList[idx]->mEnabled == false )
            continue;
          break;
        }
        for( uint8_t ii = 0; ii < Configuration::NumofEffects; ++ii )
        {
          if( idx == 0 )
            idx = Configuration::NumofEffects - 1;
          else
            --idx;
          if( mList[idx] ==NULL )
            continue;
          break;
        }
        break;
      }
      
      for(uint8_t ii = 0; ii < Configuration::NumofEffects; ++ii )
      {
        if ( mList[ii] == NULL )
          continue;
          
        mList[ii]->mEnabled = false;
      }
          
      FastLED.setBrightness(mList[idx]->mBrightness);
      FastLED.setDither(BINARY_DITHER);
      
      mList[idx]->mEnabled  = true;
      
      mEffectSwitchTimer    = Configuration::EffectTimeoutInSecs;
  }

  inline
  static bool ScheduleEffects ( int delta )
  {
    bool do_update = false;

    for( uint8_t ii=0;ii<Configuration::NumofEffects;++ii )
    {
      if( mList[ii] == NULL )
        continue;

      if( mList[ii]->mEnabled == false )
        continue;

      if( mTimeQueue[ii] > delta )
      {
        mTimeQueue[ii] -= delta;
      }
      else
      {
        mTimeQueue[ii] = mList[ii]->mUpdatePeriodInMs;
        do_update      = true;
        
        mList[ii]->Processing(mList[ii]->mUpdatePeriodInMs);
      }
    }

    return do_update;
  }

  inline
  static bool FadeEffect ( int delta )
  {
    bool    do_update = false;
    uint8_t ii;

    for( ii=0; ii < Configuration::NumofEffects; ++ii )
    {
      if( mList[ii] == NULL )
        continue;

      if( mList[ii]->mEnabled == false )
        continue;

      if( mTimeQueue[ii] > delta )
      {
        mTimeQueue[ii] -= delta;
      }
      else
      {
        mTimeQueue[ii] = FadePeriod;
        do_update      = true;
      }

      break;
    }

    if( do_update == true && ii < Configuration::NumofEffects )
    {
      const CRGB speed(1,1,1);
      CRGB *led_ptr = FastLED.leds() + mList[ii]->mIndexStart;
      int   zeroed  = 0;
      
      Fading::Update( led_ptr, mList[ii]->mNumofLeds, 4 );
           
      if( --mEffectSwitchTimer<=0 )
      {
        mState = StateRun;
        
        SelectEffect();
      }
    }

    return do_update;
  }

  inline 
  static void Init()
  {
    Clear();
    Configuration::addEffects();
    InitializeEffects();
  }

  inline 
  static bool Update()
  {
    bool do_update = false;
    int  delta;

    // calculating time elapsed since last access time

    {
      int time = millis();
      delta               = time - mLastUpdateTimeInMs;
      mLastUpdateTimeInMs = time;
    }

    switch(mState)
    {
    case StateIdle:
      break;

    case StateSingleEffect:
      do_update = ScheduleEffects(delta);
      break;

    case StateRun:
      if(mOneSecondTimer>delta)
      {
        mOneSecondTimer-=delta;
      }
      else
      {
        mOneSecondTimer = OneSecondTimeout;

        if(mEffectSwitchTimer>0)
          --mEffectSwitchTimer;

        if( mEffectSwitchTimer == 0 )
        {
          mState             = StateFade;
          mEffectSwitchTimer = FadeTime;
          
          FastLED.setDither(DISABLE_DITHER);
          
          break;
        }
      }
      do_update = ScheduleEffects(delta);
      break;

    case StateFade:
      do_update = FadeEffect(delta);
      break;
    }

    return do_update;
  }

  inline
  static void ManualMode( const bool enable )
  {
    if (enable == true)
    {
      mState = StateSingleEffect;

      for (uint8_t idx = 0; idx < Configuration::NumofEffects; ++idx)
      {
        if (mList[idx] == NULL)
          continue;
        mList[idx]->mEnabled = false;
      }
      return;
    }

    InitializeEffects();
  }
};

template< typename Configuration >
uint8_t VisualizationEffectSchedule<Configuration>::mState          = 0;
template< typename Configuration >
int VisualizationEffectSchedule<Configuration>::mUsedStripLength    = 0;
template< typename Configuration >
int VisualizationEffectSchedule<Configuration>::mLastUpdateTimeInMs = 0;
template< typename Configuration >
int VisualizationEffectSchedule<Configuration>::mOneSecondTimer     = 0;
template< typename Configuration >
int VisualizationEffectSchedule<Configuration>::mEffectSwitchTimer  = 0;

template< typename Configuration >
EffectIface* VisualizationEffectSchedule<Configuration>::mList[Configuration::NumofEffects];
template< typename Configuration >
int VisualizationEffectSchedule<Configuration>::mTimeQueue[Configuration::NumofEffects];

//////////////////////////////////
// MACRO USED IN ARDUINO SKETCH //
//////////////////////////////////

#define NEOPIXEL_STRIP_DEFINE_EFFECT_SCHEDULER( scheduler_confuguration )    typedef VisualizationEffectSchedule < scheduler_confuguration >                      GlobalEffectScheduler
#define NEOPIXEL_STRIP_DEFINE_VISUALIZATION_IFACE( hardware_configuration )  typedef NeopixelVisualizationIface < hardware_configuration, GlobalEffectScheduler > GlobalVisualizationIface

#define NEOPIXEL_STRIP_ARDUINO_SETUP()                  GlobalVisualizationIface::setup()
#define NEOPIXEL_STRIP_ARDUINO_LOOP()                   GlobalVisualizationIface::loop()
#define NEOPIXEL_STRIP_GLOBAL_BRIGHTNESS(brightness)    GlobalVisualizationIface::set_brightness(brightness)

#define NEOPIXEL_STRIP_EFFECT_INSERT(effect_ptr, index) GlobalEffectScheduler::SetEffect(effect_ptr,index)
#define NEOPIXEL_STRIP_MANUAL_MODE(enable)              GlobalEffectScheduler::ManualMode(enable)

////////////////////////////////
// MACRO USED IN EFFECT CLASS //
////////////////////////////////

#define NEOPIXEL_STRIP_EFFECT_LEDS(effect_ptr)                                   (FastLED.leds()+(effect_ptr)->mIndexStart)
#define NEOPIXEL_STRIP_EFFECT_LENGTH(effect_ptr)                                 ((effect_ptr)->mNumofLeds)

#define NEOPIXEL_STRIP_EFFECT_SET_LEDS_POSITION(effect_ptr, start_index, length) {(effect_ptr)->mIndexStart=start_index;(effect_ptr)->mNumofLeds=length;}
#define NEOPIXEL_STRIP_EFFECT_SET_BRIGHTNESS(effect_ptr, brightness)             {(effect_ptr)->mBrightness=brightness;}
#define NEOPIXEL_STRIP_EFFECT_SET_ENABLE(effect_ptr, flag)                       {(effect_ptr)->mEnabled=flag;}
#define NEOPIXEL_STRIP_EFFECT_SET_PERIOD_MS(effect_ptr, period_ms)               {(effect_ptr)->mUpdatePeriodInMs=period_ms;}

#endif 


