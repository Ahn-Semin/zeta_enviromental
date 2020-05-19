#if !defined(__EFFECT_SOUND_RIPPLE_LIGHTS_H__)
#define __EFFECT_SOUND_RIPPLE_LIGHTS_H__

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

template < int StartLedIndex, int NumofLeds >
class EffectSoundRippleLights : public EffectIface
{
  enum { BRIGHTNESS               = 255 };
  enum { FRAMES_PER_SECOND        = 100 };
  enum { TICK_IN_MS               = 1000 / FRAMES_PER_SECOND };
  enum { FADING                   = ((NumofLeds+1)>>1)/15 };
  
private:

  uint8_t  mFsInMs;
  uint8_t  mTimerCountInMs;
  uint8_t  mBeatPosInSamples;
  uint8_t  mBeatEndInSamples;
  uint8_t  mIntensityRaw;
  uint8_t  mEmotionState;

public:
  virtual ~EffectSoundRippleLights() {}
  EffectSoundRippleLights() {}

  const char* Name() const 
  {
    return "SOUNDRIPPLE";
  }

  void Init()
  {
    NEOPIXEL_STRIP_EFFECT_SET_LEDS_POSITION(this, StartLedIndex, NumofLeds);
    NEOPIXEL_STRIP_EFFECT_SET_PERIOD_MS(this,  TICK_IN_MS);
    NEOPIXEL_STRIP_EFFECT_SET_BRIGHTNESS(this, BRIGHTNESS)

    mFsInMs           =
    mTimerCountInMs   = 0;
    mBeatPosInSamples =
    mBeatEndInSamples = 0;
    mIntensityRaw     = 0;
  }

  void Processing(int delta)
  {
    mTimerCountInMs += (uint8_t)delta;

    while (mTimerCountInMs >= mFsInMs)
    {
      mTimerCountInMs -= mFsInMs;

      if (++mBeatPosInSamples >= mBeatEndInSamples)
      {
        mBeatPosInSamples = 0;        
      }

      Shift();
      Animate();
      EmittingBeat();
    }
  }

  void UpdateFs( uint8_t fs_in_ms  )
  {
    mFsInMs = fs_in_ms;
  }

  void UpdateRhythm( uint8_t c, uint8_t e)
  {  
    if(e)
    {      
      mBeatPosInSamples = c;
      mBeatEndInSamples = e;
    }    
  }

  void UpdateIntensity( uint8_t i)
  {
    mIntensityRaw = i;
  }

  void UpdateEmotion( uint8_t e)
  {
    mEmotionState  = e;
  }
 
private:

  void Shift()
  {
      CRGB* leds_ptr  = NEOPIXEL_STRIP_EFFECT_LEDS(this);
      int   led_count = NEOPIXEL_STRIP_EFFECT_LENGTH(this);
      int   left_0, right_0;

      if (led_count & 1)
      {
        left_0 = right_0 = led_count >> 1;
      }
      else
      {
        right_0 = led_count >> 1;
        left_0  = right_0 - 1;
      }
  
      // move to the right
      for (int i = 0; i < right_0; i++)
      {
        leds_ptr[i] = leds_ptr[i + 1];
      }

      //move to the left
      for (int i = led_count - 1; i > left_0; i--)
      {
        leds_ptr[i] = leds_ptr[i - 1];
      }
    
  }

  void Animate()
  {
    CRGB* leds_ptr  = NEOPIXEL_STRIP_EFFECT_LEDS(this);
    int   led_count = NEOPIXEL_STRIP_EFFECT_LENGTH(this);

    // fade_down
    for (int i = 0; i < led_count; i++)
    {
      leds_ptr[i].fadeToBlackBy(FADING);
    }
  }

  void EmittingBeat()
  {
    CRGB*   leds_ptr  = NEOPIXEL_STRIP_EFFECT_LEDS(this);
    int     led_count = NEOPIXEL_STRIP_EFFECT_LENGTH(this);
    uint8_t thresh    = (mBeatEndInSamples + 1 )>>1;
    CRGB    color;

    switch(mEmotionState)
    {
    case eGeneralInfoI2BasicEmotionNeutral:
    default:
      color = CHSV(HUE_AQUA, 255, mIntensityRaw);
      break;
    case eGeneralInfoI2BasicEmotionHappiness:
      switch(random8(3))
      {
        case 0:
          color = CHSV(HUE_ORANGE, 255, mIntensityRaw);  
          break;
        case 1:
          color = CHSV(HUE_GREEN, 255, mIntensityRaw);  
          break;
        case 2:
          color = CHSV(HUE_YELLOW, 255, mIntensityRaw);
          break;
      }
      break;
    case eGeneralInfoI2BasicEmotionSadness:
      if(random8(2)==0)
      {
        color = CHSV(HUE_BLUE, 255, mIntensityRaw);  
      }
      else
      {
        color = CHSV(HUE_PURPLE, 255, mIntensityRaw);  
      }
      break;
    case eGeneralInfoI2BasicEmotionAnger:
      if(random8(2)==0)
      {
        color = CHSV(HUE_RED, 255, mIntensityRaw);
      }
      else
      {
        color = CHSV(HUE_PINK, 255, mIntensityRaw);
      }
      break;      
    }

    if (led_count & 1)
    {
      led_count >>= 1;

      if(mBeatPosInSamples < thresh )
      {
        leds_ptr[led_count] = color;
        leds_ptr[led_count].fadeToBlackBy(FADING);
      }
      else
      {
        leds_ptr[led_count] = CRGB(CRGB::Black);
      }
    }
    else
    {
      led_count >>= 1;

      if(mBeatPosInSamples < thresh )
      {
        leds_ptr[led_count] = color;
        leds_ptr[led_count].fadeToBlackBy(FADING);
        
        leds_ptr[--led_count] = color;
        leds_ptr[led_count].fadeToBlackBy(FADING);
      }
      else
      {
        leds_ptr[led_count]   = CRGB(CRGB::Black);
        leds_ptr[--led_count] = CRGB(CRGB::Black);
      }
    }
  }
};

#endif

