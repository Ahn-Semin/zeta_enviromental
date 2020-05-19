
/********************************************************************************

  "Disco Lights BLE" Communication Protocol

  Copyright (c) 2015-2016 Irina Riegert
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

********************************************************************************/

#if !defined( __DISCO_LIGHT_BLE_PROTOCOL_H__ )
#define  __DISCO_LIGHT_BLE_PROTOCOL_H__

# if defined(__cplusplus)
extern "C" {
# endif

  typedef enum tag_of_eProtocolID
  { ProtoID_Unknown            = 0
  , ProtoID_GeneralInfo        = 1
  , ProtoID_BeatChannel        = 2
  , ProtoID_FiveFreqBands      = 3
  , ProtoID_GeneralInfoI2      = 4
  , ProtoID_RhythmChannel      = 5
  // 5-16: Allocated for future use //
  // 17-199: Reserved //
  , ProtoID_DeviceCapability   = 200
  , ProtoID_EffectInfo         = 201
  , ProtoID_GetDevStatus       = 202
  // 203: Allocated for future use //
  , ProtoID_DeviceCapabilityI2 = 204
  // 205-215: Allocated for future use //
  // 216-254: Reserved //
  , ProtoID_DevNotification    = 255
  } eProtocolID;

  typedef enum tag_of_eExtFrameLength
  { eExtPacketLengthMinimumInBytes  = 2
  , eExtFrameLengthMinimumInBytes   = 3
  , eExtFrameBufferSzMinimumInBytes = 20
  , eExtFrameBufferSzMaximumInBytes = 40
  } eExtFrameLength;

  typedef enum tag_of_eUQ88BigEndianFormat
  { eUQ88IntegerPartIndex    = 0
  , eUQ88FractionalPartIndex = 1
  , eUQ88NumofBytes          = 2
  } eUQ88BigEndianFormat;

  # if defined(WIN32) && (WIN32==1)
  #  pragma pack(push)
  #  pragma pack(1)
  # endif

  // DEFINITION: struct CmuExtPacketHeader_t

  typedef struct tag_of_CmuExtPacketHeader_t
  {
    unsigned char mProtocolID;   // [1,255] //
    unsigned char mFrameCount;   // [1,255] //
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketHeader_t
  , *PCMUEXTPKTHDR;

  // DEFINITION: struct CmuExtPacketGeneralInfo_t

 typedef enum tag_of_eGeneralInfoBitmap
 { eGeneralInfoInputSignalPresenceBit = 0x80
 } eGeneralInfoBitmap;

 typedef struct tag_of_CmuExtPacketGeneralInfo_t
 {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mFsInHz[2];         // byte 0       : bit 7      InputSignalPresenceBit         //
                                             //              : bits [0,6] integer part of UQ7.8  [0,100] //
                                             // byte 1       : bits [0,7] fraction part of UQ7.8 [1,255] //
    unsigned char        mTotalLogIntensity; // [0,255]  0   :-90dB                                      //
                                             //          255 : 0dB                                       //
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketGeneralInfo_t
  , *PCMUEXTPKTGENERAL;

  // DEFINITION: struct CmuExtPacketBeatInfo_t

  typedef enum tag_of_eBeatInfoBitmap
  { eBeatInfoBeatTrackingBit = 0x80
  } eBeatlInfoBitmap;

  typedef struct tag_of_CmuExtPacketBeatInfo_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mBeatPeriodInSamples[2]; // byte 0 : bit 7      BeatTrackingBit        //
                                                  //        : bits [0,6] integer part of UQ7.8  //
                                                  // byte 1 : bits [0,7] fraction part of UQ7.8 //
    unsigned char        mBeatStateInSamples[2];  // UQ8.0 0: beat cursor                       // 
                                                  // UQ8.0 1: beat end                          //
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketBeatInfo_t
  , *PCMUEXTPKTBEATINFO;

  // DEFINITION: struct CmuExtPacket5FreqBands_t

  typedef enum tag_of_e5FreqBands
  {
    e5FreqBandsLength = 5

  } e5FreqBands;

  typedef struct tag_of_CmuExtPacket5FreqBands_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mFreqBandLogIntensity[e5FreqBandsLength]; // [0-255] where 0:OFF 255:MAX_BRIGHTNESS //
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacket5FreqBands_t
  , *PCMUEXTPKT5FREQBANDS;

  // DEFINITION: struct CmuExtPacketGeneralInfoI2_t

  typedef enum tag_of_eGeneralInfoI2Bitmap
  { eGeneralInfoI2InputSignalPresenceBit = 0x80
  , eGeneralInfoI2VoiceDetectedBit       = 0x40
  , eGeneralInfoI2ReservedMask           = 0x30
  , eGeneralInfoI2EmotionMask            = 0x0f
  } eAdditionalInfoBitmap;
 
  typedef enum tag_of_eGeneralInfoI2BasicEmotions
  { eGeneralInfoI2BasicEmotionNeutral    = 0x0
  , eGeneralInfoI2BasicEmotionHappiness  = 0x1
  , eGeneralInfoI2BasicEmotionSadness    = 0x2
  , eGeneralInfoI2BasicEmotionAnger      = 0x3
  } tag_of_eGeneralInfoI2BasicEmotions;

  typedef enum tag_of_eGeneralInfoI2Intensity
  {
    eGeneralInfoI2IntensityEntries = 6

  } eGeneralInfoI2Intensity;

  typedef struct tag_of_CmuExtPacketGeneralInfoI2_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mBitmap;
    unsigned char        mIntensityC1[eGeneralInfoI2IntensityEntries];
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketGeneralInfoI2_t
  ,*PCMUEXTPKTGENERALI2;

  // DEFINITION: struct CmuExtPacketRhythmInfo_t

  typedef enum tag_of_eRhythmInfoBitmap
  {
    eRhythmInfoTrackingBit = 0x80
  , eRhythmInfoReserved    = 0x7f
  } eRhythmInfoBitmap;

  typedef enum tag_of_eRhythmInfoParamIndex
  {
    eRhythmInfoIndexCursor = 0
  , eRhythmInfoIndexEnd    = 1
  , eRhythmInfoIndexOnTime = 2
  , eRhythmInfoNumofIndices
  } eRhythmInfoParamIndex;

  typedef struct tag_of_CmuExtPacketRhythmInfo_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mBitmap;                                        // bit 7  : RhythmTrackingBit                 //
                                                                         //                                            //
    unsigned char        mBeatPeriodInSamples[eUQ88NumofBytes];          // rhythm period                              //
                                                                         //        : bits [0,7] integer part of UQ7.8  //
                                                                         // byte 1 : bits [0,7] fraction part of UQ7.8 //
    unsigned char        mBeatStatusInSamples[eRhythmInfoNumofIndices];  // UQ8.0 x 3                                  //
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketRhythmInfo_t
  ,*PCMUEXTPKTRHYTHMINFO;

  // DEFINITION: struct CmuExtPacketReqDevCap_t

  typedef struct tag_of_CmuExtPacketReqDevCap_t
  {
    CmuExtPacketHeader_t mHeader;
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketReqDevCap_t
  , *PCMUEXTPKTREQDEVCAP;

  // DEFINITION: struct CmuExtPacketRspDevCap_t

  typedef enum tag_of_eLedType
  { eLedTypeUnknown = 0
  , eLedTypeSingle  = 1
  , eLedTypeStick   = 2
  , eLedTypeStrip   = 3
  , eLedTypeRing    = 4
  , eLedTypeMatrix  = 5
  , eNumofLedTypes
  } eLedType;

  typedef enum tag_of_eFeatureBitmap
  {
    eFeatureSupportEffectInfo = 0x1

  } eFeatureBitmap;

  typedef enum tag_of_eCapabilityVersion
  {
    eCapabilityVersionNumber = 1

  } eCapabilityVersion;

  typedef struct tag_of_CmuExtPacketRspDevCap_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mLedDevice;
    unsigned char        mSize[2];  // 0:Rows 1:Columns //
    unsigned char        mNumofEffects;
    unsigned char        mFeatureBitmap;
    unsigned char        mCapabilityVersion;
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketRspDevCap_t
  , *PCMUEXTPKTRSPDEVCAP;

  // DEFINITION: struct CmuExtPacketReqEffectInfo_t

  typedef struct tag_of_CmuExtPacketReqEffectInfo_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mIndex;

  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketReqEffectInfo_t
  , *PCMUEXTPKTREQEFFECTINFO;

  // DEFINITION: struct CmuExtPacketRspEffectInfo_t

  typedef enum tag_of_eEffectProperty
  {
    eEffectMaxNameLength = 12
    
  } eEffectProperty;

  typedef struct tag_of_CmuExtPacketRspEffectInfo_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mIndex;
    unsigned char        mBitmap;
    unsigned char        mNameBytes;
    unsigned char        mName[eEffectMaxNameLength];
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketRspEffectInfo_t
  , *PCMUEXTPKTRSPEFFECTINFO;

  // DEFINITION: struct CmuExtPacketGetDevStatus_t

  typedef struct tag_of_CmuExtPacketGetDevStatus_t
  {
    CmuExtPacketHeader_t mHeader;
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketGetDevStatus_t
  , *PCMUEXTPKTGETDEVSTATUS;

  // DEFINITION: struct CmuExtPacketReqDevCapI2_t

  typedef struct tag_of_CmuExtPacketReqDevCapI2_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mFsDspInHz[eUQ88NumofBytes]; // byte 0       : bits [0,7] integer part of UQ8.8  [0,255] //
                                                      // byte 1       : bits [0,7] fraction part of UQ8.8 [0,255] //
    unsigned char        mFsTxtInHz[eUQ88NumofBytes]; // byte 0       : bits [0,7] integer part of UQ8.8  [0,255] //
                                                      // byte 1       : bits [0,7] fraction part of UQ8.8 [0,255] //
  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketReqDevCapI2_t
  ,*PCMUEXTPKTREQDEVCAPI2;

  // DEFINITION: struct CmuExtPacketDevNotif_t

  typedef struct tag_of_CmuExtPacketDevNotif_t
  {
    CmuExtPacketHeader_t mHeader;
    unsigned char        mActiveEffectIndex;

  }
  # if !defined(WIN32) || (WIN32==0)
  __attribute__((aligned(1), packed))
  # endif
    CmuExtPacketDevNotif_t
  , *PCMUEXTPKTDEVNOTIF;

  # if defined(WIN32) && (WIN32==1)
  #  pragma pack(pop)
  #endif

# ifdef __cplusplus
}
# endif

// DEFINITION: template < struct Notification > class DiscoLightsBleProtocolI2

// example of 'struct Notification'
// 
//struct Notification
//{
//  void onGeneralInfo() { /* TODO */ }
//  void onRhythmInfo()  { /* TODO */ }
//};

template < class Notification >
class DiscoLightsBleProtocolI2
{
private:

  enum { eCobsIdleByte = 0x00 };

  unsigned char mBuffer[eExtFrameBufferSzMinimumInBytes];

  unsigned int  mRcvCursor;
  unsigned int  mRcvCobsCode;
  unsigned char mRcvCobsState;

  unsigned char mRcvFrmState;
  unsigned char mRcvFrmId;
  unsigned char mRcvEopIndex;
  unsigned char mRcvSopIndex;

private:

  inline
  void CobsDecodeReset()
  {
    mRcvCursor = 0;
    mRcvCobsCode = 0;
    mRcvCobsState = 0;
  }

  inline
  int CobsDecode(unsigned char rcv_byte)
  {
    if (mRcvCursor > eExtFrameBufferSzMinimumInBytes)
    {
      return -1;
    }

    if (mRcvCobsState == 0)
    {
      if (rcv_byte == 1)
      {
        mBuffer[mRcvCursor++] = 0;
        return 2;
      }

      mRcvCobsCode = rcv_byte - 1;
      mRcvCobsState = 1;
      mBuffer[mRcvCursor++] = 0;

      return 1;
    }

    mBuffer[mRcvCursor++] = rcv_byte;

    if (--mRcvCobsCode <= 0)
    {
      mRcvCobsState = 0;

      return 2;
    }

    return 0;
  }

  inline
  void FrameDecodeReset()
  {
    CobsDecodeReset();

    mRcvFrmState =
    mRcvEopIndex =
    mRcvSopIndex = 0;
  }

  inline
  void FrameDecodeInit()
  {
    mRcvFrmId = 0;
   
    FrameDecodeReset();
    CommandQueueReset();
    DecodedDataReset();
  }

  inline
  void FrameDecodeUpdateState(unsigned char current_frm_id)
  {
    if (mRcvFrmId == current_frm_id)
    {
      mRcvFrmState = 1;
    }
    else
    {
      mRcvFrmState = 0;
    }

    mRcvFrmId = current_frm_id;
    if (mRcvFrmId == 255)
    {
      mRcvFrmId = 1;
    }
    else
    {
      ++mRcvFrmId;
    }
  }

  inline
  void FrameDecodeShiftCobs()
  {
    mRcvCursor = 0;
    mRcvEopIndex = 0;
    mRcvSopIndex = 0;
  }

public:

  // --- API --- //

  DiscoLightsBleProtocolI2()
  {
    FrameDecodeInit();
  }

  void Init()
  {
    FrameDecodeInit();
  }

  inline
  unsigned char BufferLength() const
  {
    return eExtFrameBufferSzMinimumInBytes;
  }

  inline
  unsigned char* Buffer()
  {
    return mBuffer;
  }

  // --- Frame Encoding --- // 

  static unsigned char Encode_I(unsigned char *frm_buffer_ptr)
  {
    unsigned char frm_length = frm_buffer_ptr[0];
    unsigned char cursor_idx = 1;
    unsigned char code_idx = 0;
    unsigned char code = 1;

    if (frm_length == 0 || frm_length > eExtFrameBufferSzMaximumInBytes)
      return 0;

    while (cursor_idx < frm_length)
    {
      if (frm_buffer_ptr[cursor_idx] != eCobsIdleByte)
      {
        ++cursor_idx;
        ++code;
        continue;
      }

      frm_buffer_ptr[code_idx] = code;
      code = 1;
      code_idx = cursor_idx++;
    }

    frm_buffer_ptr[code_idx] = code;

    return frm_length;
  }

  inline
  unsigned char Encode()
  {
    return Encode_I(mBuffer);
  }

  inline
  bool FillBufferWithDeviceCapabilityI2Frame( unsigned char device_type
                                            , unsigned char rows
                                            , unsigned char columns
                                            , unsigned char numof_effects
                                            , unsigned char feature_bitmap )
  {
    if ( eLedTypeUnknown == device_type 
      || device_type > eNumofLedTypes 
      || ( rows == 0 && eLedTypeMatrix == device_type ) 
      || columns == 0 
      || numof_effects == 0 )
    {
      return false;
    }

    mBuffer[0] = sizeof(CmuExtPacketRspDevCap_t) + 1;
    mBuffer[1] = ProtoID_DeviceCapabilityI2;
    mBuffer[2] = mRcvFrmId;
    mBuffer[3] = device_type;
    mBuffer[4] = rows;
    mBuffer[5] = columns;
    mBuffer[6] = numof_effects;
    mBuffer[7] = feature_bitmap;
    mBuffer[8] = eCapabilityVersionNumber;
    mBuffer[9] = 0;

    return true;
  }

  inline
  bool FillBufferWithEffectInfo( unsigned char effect_index
                               , unsigned char bitmap
                               , const char   *name_string_ptr)
  {
    if ( effect_index == 0 
      || name_string_ptr == NULL 
      || name_string_ptr[0] == '\0' )
    {
      return false;
    }

    int length = strlen(name_string_ptr);

    if (length > eEffectMaxNameLength)
      return false;

    mBuffer[1] = ProtoID_EffectInfo;
    mBuffer[2] = mRcvFrmId;
    mBuffer[3] = effect_index;
    mBuffer[4] = bitmap;
    mBuffer[5] = length;

    memcpy(&mBuffer[6], name_string_ptr, length);

    length += 6;

    mBuffer[0]      = length;
    mBuffer[length] = 0;

    return true;
  }

  bool FillBufferWithNotification(unsigned char active_effect_index)
  {
    if (active_effect_index == 0)
      return false;

    mBuffer[0] = sizeof(CmuExtPacketDevNotif_t) + 1;
    mBuffer[1] = ProtoID_DevNotification;
    mBuffer[2] = mRcvFrmId;
    mBuffer[3] = active_effect_index;
    mBuffer[4] = 0;

    return true;
  }

  // --- Frame Decoding --- //

private:

  enum { QCmdBufferSize = 2 };
  enum { QCmdBufferMask = QCmdBufferSize - 1 };
  unsigned short QCmdBuffer[QCmdBufferSize];
  unsigned char  QCmdRead;
  unsigned char  QCmdWrite;
  unsigned char  QCmdElements;

  inline
  void CommandQueueWrite(unsigned char cmd, unsigned char param = 0)
  {
    unsigned short val;

    if (QCmdElements >= QCmdBufferSize)
    {
      val = CommandQueueRead();
    }

    val = param;
    val <<= 8;
    val |= cmd;

    QCmdBuffer[QCmdWrite++] = val;

    QCmdWrite &= QCmdBufferMask;
    ++QCmdElements;
  }

public:

  inline
  void CommandQueueReset()
  {
    QCmdRead = QCmdWrite = QCmdElements = 0;
  }

  inline
  unsigned short CommandQueueRead()
  {
    unsigned short val = 0;

    if (QCmdElements != 0)
    {
      val = QCmdBuffer[QCmdRead++];

      QCmdRead &= QCmdBufferMask;
      --QCmdElements;
    }

    return val;
  }

public:

  void InitDecoder(unsigned short fs_dsp_in_hz, unsigned short fs_txt_in_hz)
  {
    mQ8p8FsDspInHz = fs_dsp_in_hz;
    mQ8p8FsTxtInHz = fs_txt_in_hz;
    
    calculateIntensityQueueParamsI2();
  }

  void FrameDecode(unsigned char rcv_byte)
  {
    if (rcv_byte == eCobsIdleByte)
    {
      goto FrameResetAndFnExit;
    }

    CobsDecode(rcv_byte);

    if (mRcvEopIndex == 0)
    {
      unsigned char sop_set  = 0;
      unsigned char proto_id = ProtoID_Unknown;

      if (mRcvCursor >= eExtFrameLengthMinimumInBytes)
      {
        if (mBuffer[0] == 0 && mBuffer[1] != 0 && mBuffer[2] != 0)
        {
          proto_id     = mBuffer[1];
          sop_set      = 1;
          mRcvSopIndex = 1;

          FrameDecodeUpdateState(mBuffer[2]);
        }
      }
      else
      if (mRcvCursor >= eExtPacketLengthMinimumInBytes)
      {
        if (mBuffer[0] != 0 && mBuffer[1] != 0)
        {
          proto_id = mBuffer[0];
          sop_set = 1;
          mRcvSopIndex = 0;
        }
      }

      if (sop_set != 0)
      {
        switch (proto_id)
        {
        case ProtoID_Unknown:
        default:
          goto FrameResetAndFnExit;
        case ProtoID_GeneralInfoI2:
          mRcvEopIndex = sizeof(CmuExtPacketGeneralInfoI2_t) + mRcvSopIndex;
          break;
        case ProtoID_RhythmChannel:
          mRcvEopIndex = sizeof(CmuExtPacketRhythmInfo_t) + mRcvSopIndex;
          break;
        case ProtoID_EffectInfo:
          mRcvEopIndex = sizeof(CmuExtPacketReqEffectInfo_t) + mRcvSopIndex;
          break;
        case ProtoID_GetDevStatus:
          mRcvEopIndex = sizeof(CmuExtPacketGetDevStatus_t) + mRcvSopIndex;
          break;
        case ProtoID_DeviceCapabilityI2:
          mRcvEopIndex = sizeof(CmuExtPacketReqDevCapI2_t) + mRcvSopIndex;
          break;
        }
      }
    }

    if (mRcvEopIndex != 0 && mRcvCursor >= mRcvEopIndex)
    {
      unsigned char proto_id = mBuffer[mRcvSopIndex];

      if ( mRcvFrmState == 1
        || proto_id == ProtoID_DeviceCapabilityI2
        || proto_id == ProtoID_EffectInfo
        || proto_id == ProtoID_GetDevStatus)
      {
        switch (proto_id)
        {

        case ProtoID_GeneralInfoI2:
          updateGeneralInfoI2((PCMUEXTPKTGENERALI2)&mBuffer[mRcvSopIndex]);
          break;

        case ProtoID_RhythmChannel:
          updateRhythmChannel((PCMUEXTPKTRHYTHMINFO)&mBuffer[mRcvSopIndex]);
          break;
          
        case ProtoID_EffectInfo:
        {
          PCMUEXTPKTREQEFFECTINFO pkt = (PCMUEXTPKTREQEFFECTINFO)&mBuffer[mRcvSopIndex];
          CommandQueueWrite(ProtoID_EffectInfo, pkt->mIndex);
        }
          break;

        case ProtoID_GetDevStatus:
          CommandQueueWrite(ProtoID_GetDevStatus);
          break;

        case ProtoID_DeviceCapabilityI2:
          updateSystemParamsI2((PCMUEXTPKTREQDEVCAPI2)&mBuffer[mRcvSopIndex]);
          CommandQueueWrite(ProtoID_DeviceCapabilityI2);
          break;          
        }
      }

      FrameDecodeShiftCobs();
    }

    return;

  FrameResetAndFnExit:
    FrameDecodeReset();
  }

private: // Data Packets Decoding //

  enum { eInputSignalPresenceBit = 0x01
       , eVoiceDetectedBit       = 0x02
       , eRhythmTrackingBit      = 0x04 };

  enum { NumofIntensityC1Entres  = eGeneralInfoI2IntensityEntries };

  unsigned char         mBitmap;
  unsigned char         mEmotion;

  unsigned short        mQ8p8FsDspInHz;
  unsigned short        mQ8p8FsTxtInHz;

  unsigned short        mQ8p8BeatPeriodInSamples;
  unsigned char         mRhythmCursorInSamples;
  unsigned char         mRhythmOnTimeInSamples;
  unsigned char         mRhythmEndInSamples;

  unsigned char         mDspTickInMs;

  unsigned char         mIntensityIdxMax;
  mutable unsigned char mIntensityIdxCursor;
  unsigned char         mIntensityC1[NumofIntensityC1Entres];

  unsigned char         mRhythmBpm;
  Notification          mNotifier;

private:

  inline
  void calculateIntensityQueueParamsI2()
  {
    mIntensityIdxMax = 0;
    mDspTickInMs     = 0;

    if (mQ8p8FsTxtInHz > 256 && mQ8p8FsDspInHz > 256)
    {
      // Transmition Tick in milliseconds
      uint32_t n = 1000;
      n <<= 8;        
      n  += (mQ8p8FsTxtInHz >> 1);
      n  /= mQ8p8FsTxtInHz;
      
      // Update Tick in milliseconds
      uint32_t m = 1000;
      m <<= 8;
      m  += (mQ8p8FsDspInHz >> 1);
      m  /=  mQ8p8FsDspInHz;
      
      mDspTickInMs  =(unsigned char)m;
      
      // Intensity Queue Size
      n /= m;
      if (n > NumofIntensityC1Entres)
      {
        mIntensityIdxMax = NumofIntensityC1Entres;
      }
      else
      {
        mIntensityIdxMax = (unsigned char)n;
      }
    }
  }

  inline
  void updateSystemParamsI2(PCMUEXTPKTREQDEVCAPI2 pkt)
  {
    mQ8p8FsDspInHz   = pkt->mFsDspInHz[0];
    mQ8p8FsDspInHz <<= 8;
    mQ8p8FsDspInHz  |= pkt->mFsDspInHz[1];
    
    mQ8p8FsTxtInHz   = pkt->mFsTxtInHz[0];
    mQ8p8FsTxtInHz <<= 8;
    mQ8p8FsTxtInHz  |= pkt->mFsTxtInHz[1];
    
    calculateIntensityQueueParamsI2();
  }

  inline
  void updateGeneralInfoI2(PCMUEXTPKTGENERALI2 pkt)
  {
    mBitmap &=~(eInputSignalPresenceBit | eVoiceDetectedBit | eRhythmTrackingBit);
    
    mBitmap |= (pkt->mBitmap & eGeneralInfoI2InputSignalPresenceBit) ? eInputSignalPresenceBit : 0;
    mBitmap |= (pkt->mBitmap & eGeneralInfoI2VoiceDetectedBit)       ? eVoiceDetectedBit       : 0;

    mEmotion            = pkt->mBitmap & eGeneralInfoI2EmotionMask;
    mIntensityIdxCursor = 0;

    memcpy(mIntensityC1, pkt->mIntensityC1, mIntensityIdxMax);

    mNotifier.onGeneralInfo();

    if(mQ8p8BeatPeriodInSamples==0 && mRhythmBpm!=0)
    {
      mRhythmBpm = 0;
      mNotifier.onRhythmInfo();
    }
    
    mQ8p8BeatPeriodInSamples = 0;
    mRhythmCursorInSamples   = 0;
    mRhythmOnTimeInSamples   = 0;
    mRhythmEndInSamples      = 0;
  }

  inline
  void CalculateBpm()
  {
    if (mQ8p8FsDspInHz == 0)
    {
      mRhythmBpm = 0;
      return;
    }
     
    uint32_t a = 1000;
    uint32_t b = 60000;
                
    a <<= 8;
    a  +=(mQ8p8FsDspInHz>>1);
    a  /= mQ8p8FsDspInHz;
    a  *= mQ8p8BeatPeriodInSamples;
      
    if(a==0)
    {
      mRhythmBpm = 0;
      return;
    }

    b <<= 8;
    b  += (a>>1);
    b  /= a;
    
    mRhythmBpm = (unsigned char)b;
  }

  inline
  void updateRhythmChannel(PCMUEXTPKTRHYTHMINFO pkt)
  {
    mBitmap                    |= (pkt->mBitmap & eRhythmInfoTrackingBit) ? eRhythmTrackingBit : 0;

    mQ8p8BeatPeriodInSamples   = pkt->mBeatPeriodInSamples[eUQ88IntegerPartIndex];
    mQ8p8BeatPeriodInSamples <<= 8;
    mQ8p8BeatPeriodInSamples  |= pkt->mBeatPeriodInSamples[eUQ88FractionalPartIndex];

    mRhythmCursorInSamples     = pkt->mBeatStatusInSamples[eRhythmInfoIndexCursor];
    mRhythmEndInSamples        = pkt->mBeatStatusInSamples[eRhythmInfoIndexEnd];
    mRhythmOnTimeInSamples     = pkt->mBeatStatusInSamples[eRhythmInfoIndexOnTime];

    CalculateBpm();

    mNotifier.onRhythmInfo();
  }

  inline
  void DecodedDataReset()
  {
    mBitmap                  = 0;
    mEmotion                 = eGeneralInfoI2BasicEmotionNeutral;
    mQ8p8FsDspInHz           = 0;
    mQ8p8FsTxtInHz           = 0;
    mQ8p8BeatPeriodInSamples = 0;
    mRhythmCursorInSamples   = 0;
    mRhythmOnTimeInSamples   = 0;
    mRhythmEndInSamples      = 0;
    mDspTickInMs             = 0;
    mIntensityIdxMax         = 0;
    mIntensityIdxCursor      = 0;

    mRhythmBpm               = 0;
    
    memset(mIntensityC1, 0, sizeof(mIntensityC1));
  }

public: // Received Streaming Data //

  inline
  bool IsReady() 
  {
    return mQ8p8FsDspInHz!=0 && mQ8p8FsTxtInHz!=0;
  }
  
  inline
  unsigned char GetBpm() const
  {
    return mRhythmBpm;
  }

  inline
  unsigned char GetRhythmCursorInSamples() const
  {
    return mRhythmCursorInSamples;
  }

  inline
  unsigned char GetRhythmEndInSamples() const
  {
    return mRhythmEndInSamples;
  }

  inline
  unsigned char GetRhythmOnTimeInSamples() const
  {
    return mRhythmOnTimeInSamples;
  }

  inline
  bool IsInputSignalPresent() const
  {
    return mBitmap&eInputSignalPresenceBit ? true : false;
  }

  inline
  bool IsVoiceDetected() const
  {
    return mBitmap&eVoiceDetectedBit ? true : false;
  }

  inline
  bool IsRhythmTracked() const
  {
    return mBitmap&eRhythmTrackingBit ? true : false;
  }

  inline
  bool IsBpmValid()
  {
    return mRhythmBpm!=0 && (mBitmap&(eInputSignalPresenceBit|eRhythmTrackingBit))==(eInputSignalPresenceBit|eRhythmTrackingBit);
  }

  inline
  unsigned char BasicEmotion() const
  {
    return mEmotion;
  }

  inline
  unsigned char PopIntensityC1() const
  {
    unsigned char val = mIntensityC1[mIntensityIdxCursor];

    if (++mIntensityIdxCursor >= mIntensityIdxMax)
    {
      mIntensityIdxCursor = mIntensityIdxMax - 1;
    }
    
    return val;
  }

  inline
  unsigned char TickInMs() const
  {
    return mDspTickInMs;
  }

};

#endif


