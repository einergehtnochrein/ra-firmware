/* Copyright (c) 2011-2013, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __USBAUDIO_H
#define __USBAUDIO_H

#include "usbd.h"
#include "usbd_rom_api.h"


/* USB Class Codes and applicable sub classes and protocols */
#define USBC_AUDIO                                      (0x01)
#define   USBCS_AUDIO_SUBCLASS_UNDEFINED                    (0x00)
#define   USBCS_AUDIO_AUDIOCONTROL                          (0x01)
#define   USBCS_AUDIO_AUDIOSTREAMING                        (0x02)
#define   USBCS_AUDIO_MIDISTREAMING                         (0x03)
#define     USBCP_AUDIO_PR_PROTOCOL_UNDEFINED                   (0x00)

/* Class-specific Descriptors (and their length) */
#define USBD_AUDIO_CS_UNDEFINED                         (0x20)
#define USBD_SIZE_AUDIO_CS_UNDEFINED                    (5)
#define USBD_AUDIO_CS_DEVICE                            (0x21)
#define USBD_AUDIO_CS_CONFIGURATION                     (0x22)
#define USBD_AUDIO_CS_STRING                            (0x23)
#define USBD_AUDIO_CS_INTERFACE                         (0x24)
#define USBD_AUDIO_CS_ENDPOINT                          (0x25)


/* Subtype of descriptors */
#define USBD_AUDIO_ST_HEADER                            (1)
#define USBD_AUDIO_ST_INPUT_TERMINAL                    (2)
#define USBD_AUDIO_ST_OUTPUT_TERMINAL                   (3)
#define USBD_AUDIO_ST_MIXER_UNIT                        (4)
#define USBD_AUDIO_ST_SELECTOR_UNIT                     (5)
#define USBD_AUDIO_ST_FEATURE_UNIT                      (6)
#define USBD_AUDIO_ST_PROCESSING_UNIT                   (7)
#define USBD_AUDIO_ST_EXTENSION_UNIT                    (8)


/* MIDI Endpoint Type */
#define USBD_AUDIO_MIDI_ENDPOINT_UNDEFINED              0x00
#define USBD_AUDIO_MIDI_ENDPOINT_MS_GENERAL             0x01

/* MIDI Jack Type */
#define USBD_AUDIO_MIDI_JACK_TYPE_UNDEFINED             0x00
#define USBD_AUDIO_MIDI_JACK_TYPE_EMBEDDED              0x01
#define USBD_AUDIO_MIDI_JACK_TYPE_EXTERNAL              0x02



/* Feature Unit Control Selectors */
enum {
    USBAC_CS_FU_MUTE_CONTROL                    = 0x01,
    USBAC_CS_FU_VOLUME_CONTROL                  = 0x02,
    USBAC_CS_FU_BASS_CONTROL                    = 0x03,
    USBAC_CS_FU_MID_CONTROL                     = 0x04,
    USBAC_CS_FU_TREBLE_CONTROL                  = 0x05,
    USBAC_CS_FU_GRAPHIC_EQUALIZER_CONTROL       = 0x06,
    USBAC_CS_FU_AUTOMATIC_GAIN_CONTROL          = 0x07,
    USBAC_CS_FU_DELAY_CONTROL                   = 0x08,
    USBAC_CS_FU_BASS_BOOST_CONTROL              = 0x09,
    USBAC_CS_FU_LOUDNESS_CONTROL                = 0x0A,
};

/* Processing Unit Process Types */
enum {
    USBAC_PU_UPDOWNMIX_PROCESS                  = 1,
    USBAC_PU_DOLBY_PROLOGIC_PROCESS             = 2,
    USBAC_PU_3D_STEREO_EXTENDER_PROCESS         = 3,
    USBAC_PU_REVERBERATION_PROCESS              = 4,
    USBAC_PU_CHORUS_PROCESS                     = 5,
    USBAC_PU_DYN_RANGE_COMP_PROCESS             = 6,
};

/* Processing Unit (Dynamic Range Compressor) Control Selectors */
enum {
    USBAC_CS_PU_DR_ENABLE_CONTROL               = 1,
    USBAC_CS_PU_COMPRESSION_RATIO_CONTROL       = 2,
    USBAC_CS_PU_MAXAMPL_CONTROL                 = 3,
    USBAC_CS_PU_THRESHOLD_CONTROL               = 4,
    USBAC_CS_PU_ATTACK_TIME                     = 5,
    USBAC_CS_PU_RELEASE_TIME                    = 6,
};


#define USBAC_CS_EP_SAMPLING_FREQ_CONTROL               0x01
#define USBAC_CS_EP_PITCH_CONTROL                       0x02



#define SET_CUR                                         0x01
#define GET_CUR                                         0x81
#define SET_MIN                                         0x02
#define GET_MIN                                         0x82
#define SET_MAX                                         0x03
#define GET_MAX                                         0x83
#define SET_RES                                         0x04
#define GET_RES                                         0x84
#define SET_MEM                                         0x05
#define GET_MEM                                         0x85
#define GET_STAT                                        0xFF






/** Minimum, maximum and resolution for a control in a unit. */
typedef struct USBAUDIO_ControlRange {
    uint8_t     unit_id;
    uint8_t     control_id;
    int16_t     min;
    int16_t     max;
    int16_t     current;
    uint16_t    resolution;
    uint8_t     length;
    LPCLIB_Callback callback;
} USBAUDIO_ControlRange;


/** USB Class-Specific AC Interface descriptor. */
#define _USBAUDIO_ControlInterfaceHeaderDescriptorDef(name, n)\
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint16_t    bcdADC;                             \
        uint16_t    wTotalLength;                       \
        uint8_t     bInCollection;                      \
        uint8_t     baInterfaceNr[n];                   \
    }) name


/** USB Audio Input Terminal Descriptor. */
typedef __PACKED(struct USBAUDIO_InputTerminalDescriptor {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bTerminalID;
    uint16_t    wTerminalType;
    uint8_t     bAssocTerminal;
    uint8_t     bNrChannels;
    uint16_t    wChannelConfig;
    uint8_t     iChannelNames;
    uint8_t     iTerminal;
}) USBAUDIO_InputTerminalDescriptor;


/** USB Audio Output Terminal Descriptor. */
typedef __PACKED(struct USBAUDIO_OutputTerminalDescriptor {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bTerminalID;
    uint16_t    wTerminalType;
    uint8_t     bAssocTerminal;
    uint8_t     bSourceID;
    uint8_t     iTerminal;
}) USBAUDIO_OutputTerminalDescriptor;


/** USB Feature Unit Descriptor. */
#define _USBAUDIO_FeatureUnitDescriptorDef(name, ch,bmaControls_type) \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bUnitID;                            \
        uint8_t     bSourceID;                          \
        uint8_t     bControlSize;                       \
        bmaControls_type bmaControls[1 + (ch)];         \
        uint8_t     iFeature;                           \
    }) name


/** USB Processing Unit Descriptor. */
#define _USBAUDIO_ProcessingUnitDescriptorDef(name, p,n) \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bUnitID;                            \
        uint16_t    wProcessType;                       \
        uint8_t     bNrInPins;                          \
        uint8_t     baSourceID[p];                      \
        uint8_t     bNrChannels;                        \
        uint16_t    wChannelConfig;                     \
        uint8_t     iChannelNames;                      \
        uint8_t     bControlSize;                       \
        uint8_t     bmControls[n];                      \
        uint8_t     iProcessing;                        \
    }) name


/** USB Mixer Unit Descriptor. */
#define _USBAUDIO_MixerUnitDescriptorDef(name, p,n,m) \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bUnitID;                            \
        uint8_t     bNrInPins;                          \
        uint8_t     baSourceID[p];                      \
        uint8_t     bNrChannels;                        \
        uint16_t    wChannelConfig;                     \
        uint8_t     iChannelNames;                      \
        uint8_t     bmControls[(n * m + 7) / 8];        \
        uint8_t     iMixer;                             \
    }) name


/** USB Selector Unit Descriptor. */
#define _USBAUDIO_SelectorUnitDescriptorDef(name, p)    \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bUnitID;                            \
        uint8_t     bNrInPins;                          \
        uint8_t     baSourceID[p];                      \
        uint8_t     iSelector;                          \
    }) name


/** USB Audio Streaming Interface Descriptor. */
typedef __PACKED(struct USBAUDIO_StreamingInterfaceDescriptor {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bTerminalLink;
    uint8_t     bDelay;
    uint16_t    wFormatTag;
}) USBAUDIO_StreamingInterfaceDescriptor;


/** USB Audio Data Endpoint Descriptor. */
typedef __PACKED(struct USBAUDIO_IsoDataEndpointDescriptor {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bmAttributes;
    uint8_t     bLockDelayUnits;
    uint16_t    wLockDelay;
}) USBAUDIO_IsoDataEndpointDescriptor;


/** Isochronous endpoint descriptor. */
typedef __PACKED(struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bEndpointAddress;
    uint8_t     bmAttributes;
    uint16_t    wMaxPacketSize;
    uint8_t     bInterval;
    uint8_t     bRefresh;
    uint8_t     bSyncAddress;
}) USB_EndpointDescriptorIso;


/**********************************************************************************************/



/* Terminal Types.
 * See "Universal Serial Bus, Device Class Definition for Terminal Types, Release 1.0".
 */

enum {
    USBAUDIO_TERMINAL_USB_UNDEFINED                                 = 0x0100,
    USBAUDIO_TERMINAL_USB_STREAMING                                 = 0x0101,
    USBAUDIO_TERMINAL_USB_VENDOR_SPECIFIC                           = 0x01FF,

    USBAUDIO_TERMINAL_INPUT_UNDEFINED                               = 0x0200,
    USBAUDIO_TERMINAL_INPUT_MICROPHONE                              = 0x0201,
    USBAUDIO_TERMINAL_INPUT_DESKTOP_MICROPHONE                      = 0x0202,
    USBAUDIO_TERMINAL_INPUT_PERSONAL_MICROPHONE                     = 0x0203,
    USBAUDIO_TERMINAL_INPUT_OMNI_DIRECTIONAL_MICROPHONE             = 0x0204,
    USBAUDIO_TERMINAL_INPUT_MICROPHONE_ARRAY                        = 0x0205,
    USBAUDIO_TERMINAL_INPUT_PROCESSING_MICROPHONE_ARRAY             = 0x0206,

    USBAUDIO_TERMINAL_OUTPUT_UNDEFINED                              = 0x0300,
    USBAUDIO_TERMINAL_OUTPUT_SPEAKER                                = 0x0301,
    USBAUDIO_TERMINAL_OUTPUT_HEADPHONES                             = 0x0302,
    USBAUDIO_TERMINAL_OUTPUT_HEAD_MOUNTED_DISPLAY_AUDIO             = 0x0303,
    USBAUDIO_TERMINAL_OUTPUT_DESKTOP_SPEAKER                        = 0x0304,
    USBAUDIO_TERMINAL_OUTPUT_ROOM_SPEAKER                           = 0x0305,
    USBAUDIO_TERMINAL_OUTPUT_COMMUNICATION_SPEAKER                  = 0x0306,
    USBAUDIO_TERMINAL_OUTPUT_LOW_FREQUENCY_EFFECTS_SPEAKER          = 0x0307,

    USBAUDIO_TERMINAL_BIDIRECTIONAL_UNDEFINED                       = 0x0400,
    USBAUDIO_TERMINAL_BIDIRECTIONAL_HANDSET                         = 0x0401,
    USBAUDIO_TERMINAL_BIDIRECTIONAL_HEADSET                         = 0x0402,
    USBAUDIO_TERMINAL_BIDIRECTIONAL_SPEAKERPHONE                    = 0x0403,
    USBAUDIO_TERMINAL_BIDIRECTIONAL_ECHO_SPPRESSING_SPEAKERPHONE    = 0x0404,
    USBAUDIO_TERMINAL_BIDIRECTIONAL_ECHO_CANCELING_SPEAKERPHONE     = 0x0405,

    USBAUDIO_TERMINAL_TELEPHONY_UNDEFINED                           = 0x0500,
    USBAUDIO_TERMINAL_TELEPHONY_PHONE_LINE                          = 0x0501,
    USBAUDIO_TERMINAL_TELEPHONY_TELEPHONE                           = 0x0502,
    USBAUDIO_TERMINAL_TELEPHONY_DOWN_LINE_PHONE                     = 0x0503,

    USBAUDIO_TERMINAL_EXTERNAL_UNDEFINED                            = 0x0600,
    USBAUDIO_TERMINAL_EXTERNAL_ANALOG_CONNECTOR                     = 0x0601,
    USBAUDIO_TERMINAL_EXTERNAL_DIGITAL_AUDIO_INTERFACE              = 0x0602,
    USBAUDIO_TERMINAL_EXTERNAL_LINE_CONNECTOR                       = 0x0603,
    USBAUDIO_TERMINAL_EXTERNAL_LEGACY_AUDIO_CONNECTOR               = 0x0604,
    USBAUDIO_TERMINAL_EXTERNAL_SPDIF_INTERFACE                      = 0x0605,
    USBAUDIO_TERMINAL_EXTERNAL_1394_DA_STREAM                       = 0x0606,
    USBAUDIO_TERMINAL_EXTERNAL_1394_DV_STREAM_SOUNDTRACK            = 0x0607,
    USBAUDIO_TERMINAL_EXTERNAL_ADAT_LIGHTPIPE                       = 0x0608,
    USBAUDIO_TERMINAL_EXTERNAL_TDIF                                 = 0x0609,
    USBAUDIO_TERMINAL_EXTERNAL_MADI                                 = 0x060A,

    USBAUDIO_TERMINAL_EMBEDDED_UNDEFINED                            = 0x0700,
    USBAUDIO_TERMINAL_EMBEDDED_LEVEL_CALIBRATION_NOISE_SOURCE       = 0x0701,
    USBAUDIO_TERMINAL_EMBEDDED_EQUALIZATION_NOISE                   = 0x0702,
    USBAUDIO_TERMINAL_EMBEDDED_CD_PLAYER                            = 0x0703,
    USBAUDIO_TERMINAL_EMBEDDED_DAT                                  = 0x0704,
    USBAUDIO_TERMINAL_EMBEDDED_DCC                                  = 0x0705,
    USBAUDIO_TERMINAL_EMBEDDED_COMPRESSED_AUDIO_PLAYER              = 0x0706,
    USBAUDIO_TERMINAL_EMBEDDED_ANALOG_TAPE                          = 0x0707,
    USBAUDIO_TERMINAL_EMBEDDED_PHONOGRAPH                           = 0x0708,
    USBAUDIO_TERMINAL_EMBEDDED_VCR_AUDIO                            = 0x0709,
    USBAUDIO_TERMINAL_EMBEDDED_VIDEO_DISC_AUDIO                     = 0x070A,
    USBAUDIO_TERMINAL_EMBEDDED_DVD_AUDIO                            = 0x070B,
    USBAUDIO_TERMINAL_EMBEDDED_TV_TUNER_AUDIO                       = 0x070C,
    USBAUDIO_TERMINAL_EMBEDDED_SATELLITE_RECEIVER_AUDIO             = 0x070D,
    USBAUDIO_TERMINAL_EMBEDDED_CABLE_TUNER_AUDIO                    = 0x070E,
    USBAUDIO_TERMINAL_EMBEDDED_DSS_AUDIO                            = 0x070F,
    USBAUDIO_TERMINAL_EMBEDDED_RADIO_RECEIVER                       = 0x0710,
    USBAUDIO_TERMINAL_EMBEDDED_RADIO_TRANSMITTER                    = 0x0711,
    USBAUDIO_TERMINAL_EMBEDDED_MULTI_TRACK_RECORDER                 = 0x0712,
    USBAUDIO_TERMINAL_EMBEDDED_SYNTHESIZER                          = 0x0713,
    USBAUDIO_TERMINAL_EMBEDDED_PIANO                                = 0x0714,
    USBAUDIO_TERMINAL_EMBEDDED_GUITAR                               = 0x0715,
    USBAUDIO_TERMINAL_EMBEDDED_DRUMS_RHYTHM                         = 0x0716,
    USBAUDIO_TERMINAL_EMBEDDED_OTHER_MUSICAL_INSTRUMENT             = 0x0717,
};



/**********************************************************************************************/



/* Audio Data Formats.
 * See "Universal Serial Bus, Device Class Definition for Audio Data Formats, Release 1.0".
 */

enum {
    USBAUDIO_FORMATTAG_TYPE_I_UNDEFINED                             = 0x0000,
    USBAUDIO_FORMATTAG_PCM                                          = 0x0001,
    USBAUDIO_FORMATTAG_PCM8                                         = 0x0002,
    USBAUDIO_FORMATTAG_IEEE_FLOAT                                   = 0x0003,
    USBAUDIO_FORMATTAG_ALAW                                         = 0x0004,
    USBAUDIO_FORMATTAG_MULAW                                        = 0x0005,

    USBAUDIO_FORMATTAG_TYPE_II_UNDEFINED                            = 0x1000,
    USBAUDIO_FORMATTAG_MPEG                                         = 0x1001,
    USBAUDIO_FORMATTAG_AC3                                          = 0x1002,

    USBAUDIO_FORMATTAG_TYPE_III_UNDEFINED                           = 0x2000,
    USBAUDIO_FORMATTAG_IEC1937_AC3                                  = 0x2001,
    USBAUDIO_FORMATTAG_IEC1937_MPEG1_LAYER1                         = 0x2002,
    USBAUDIO_FORMATTAG_IEC1937_MPEG1_LAYER23                        = 0x2003,
    USBAUDIO_FORMATTAG_IEC1937_MPEG2_NOEXT = USBAUDIO_FORMATTAG_IEC1937_MPEG1_LAYER23,
    USBAUDIO_FORMATTAG_IEC1937_MPEG2_EXT                            = 0x2004,
    USBAUDIO_FORMATTAG_IEC1937_MPEG2_LAYER1_LS                      = 0x2005,
    USBAUDIO_FORMATTAG_IEC1937_MPEG2_LAYER23_LS                     = 0x2006,
};

enum {
    USBAUDIO_FORMAT_TYPE_UNDEFINED                                  = 0x00,
    USBAUDIO_FORMAT_TYPE_I                                          = 0x01,
    USBAUDIO_FORMAT_TYPE_II                                         = 0x02,
    USBAUDIO_FORMAT_TYPE_III                                        = 0x03,
};

enum {
    USBAUDIO_FORMAT_MPEG_CONTROL_UNDEFINED                          = 0x00,
    USBAUDIO_FORMAT_MP_DUAL_CHANNEL_CONTROL                         = 0x01,
    USBAUDIO_FORMAT_MP_SECOND_STEREO_CONTROL                        = 0x02,
    USBAUDIO_FORMAT_MP_MULTILINGUAL_CONTROL                         = 0x03,
    USBAUDIO_FORMAT_MP_DYN_RANGE_CONTROL                            = 0x04,
    USBAUDIO_FORMAT_MP_SCALING_CONTROL                              = 0x05,
    USBAUDIO_FORMAT_MP_HILO_SCALING_CONTROL                         = 0x06,
};

enum {
    USBAUDIO_FORMAT_AC_CONTROL_UNDEFINED                            = 0x00,
    USBAUDIO_FORMAT_AC_MODE_CONTROL                                 = 0x01,
    USBAUDIO_FORMAT_AC_DYN_RANGE_CONTROL                            = 0x02,
    USBAUDIO_FORMAT_AC_SCALING_CONTROL                              = 0x03,
    USBAUDIO_FORMAT_AC_HILO_SCLAING_CONTROL                         = 0x04,
};

/** USB Microphone Type I Format Type Descriptor. */
#define _USBAUDIO_TypeIFormatDescriptorDef(name, n)     \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bFormatType;                        \
        uint8_t     bNrChannels;                        \
        uint8_t     bSubFrameSize;                      \
        uint8_t     bBitResolution;                     \
        uint8_t     bSamFreqType;                       \
        uint8_t     tSamFreq[n][3];                     \
    }) name



/**********************************************************************************************/



/* MIDI.
 * See "Universal Serial Bus, Device Class Definition for MIDI Devices, Release 1.0".
 */


enum {
    USBAUDIO_MIDI_INTERFACE_MS_DESCRIPTOR_UNDEFINED                 = 0x00,
    USBAUDIO_MIDI_INTERFACE_MS_HEADER                               = 0x01,
    USBAUDIO_MIDI_INTERFACE_MIDI_IN_JACK                            = 0x02,
    USBAUDIO_MIDI_INTERFACE_MIDI_OUT_JACK                           = 0x03,
    USBAUDIO_MIDI_INTERFACE_ELEMENT                                 = 0x04,
};

enum {
    USBAUDIO_MIDI_ENDPOINT_DESCRIPTOR_UNDEFINED                     = 0x00,
    USBAUDIO_MIDI_ENDPOINT_MS_GENERAL                               = 0x01,
};

enum {
    USBAUDIO_MIDI_JACKTYPE_JACK_TYPE_UNDEFINED                      = 0x00,
    USBAUDIO_MIDI_JACKTYPE_EMBEDDED                                 = 0x01,
    USBAUDIO_MIDI_JACKTYPE_EXTERNAL                                 = 0x02,
};

enum {
    USBAUDIO_MIDI_CONTROLSELECTOR_EP_CONTROL_UNDEFINED              = 0x00,
    USBAUDIO_MIDI_CONTROLSELECTOR_ASSOCIATION_CONTROL               = 0x01,
};


/** USB Audio MIDI Adapter Class-specific MS Interface Descriptor. */
typedef __PACKED(struct USBAUDIO_MidiStreamingInterfaceDescriptor {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint16_t    bcdADC;
    uint16_t    wTotalLength;
}) USBAUDIO_MidiStreamingInterfaceDescriptor;


/** USB Audio MIDI IN Jack Descriptor. */
typedef __PACKED(struct USBAUDIO_MidiInJackDescriptor {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bJackType;
    uint8_t     bJackID;
    uint8_t     iJack;
}) USBAUDIO_MidiInJackDescriptor;


/** USB Audio MIDI OUT Jack Descriptor. */
#define _USBAUDIO_MidiOutJackDescriptorDef(name, p)     \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bJackType;                          \
        uint8_t     bJackID;                            \
        uint8_t     bNrInputPins;                       \
        __PACKED(struct {                               \
            uint8_t ID;                                 \
            uint8_t Pin;                                \
        }) baSource[p];                                 \
        uint8_t     iJack;                              \
    }) name


/** USB Audio class-specific MIDI Streaming Bulk Endpoint Descriptor. */
#define _USBAUDIO_MidiBulkEndpointDescriptorDef(name, n) \
    typedef __PACKED(struct _##name {                   \
        uint8_t     bLength;                            \
        uint8_t     bDescriptorType;                    \
        uint8_t     bDescriptorSubtype;                 \
        uint8_t     bNumEmbMIDIJack;                    \
        uint8_t     baAssocJackID[n];                   \
    }) name



/**********************************************************************************************/


/** Audio control interface */
typedef struct {
    uint8_t     interfaceNumber;
    bool        usesInterrupt;
    uint8_t     endpointNumberInterrupt;
    uint8_t     numControls;
    USBAUDIO_ControlRange *controls;
} USBAUDIO_ControlInterface;

/** One alternative for an audio streaming interface. */
typedef struct {
    uint8_t     numEndpoints;
    uint8_t     linkedTerminal;
    uint8_t     endpointNumber;
    uint16_t    packetSize;
    uint16_t    supportedControls;
    uint8_t     numSamplingFrequencies;
    const uint32_t  *samplingFrequencies;
} USBAUDIO_StreamingInterfaceParams;

/** Complete audio streaming interface. */
typedef struct {
    uint8_t     interfaceNumber;
    uint8_t     numAltSettings;
    uint8_t     activeSetting;
    bool        isMuted;
    uint32_t    currentSamplerate;
    const USBAUDIO_StreamingInterfaceParams *params;
} USBAUDIO_StreamingInterface;


/** Complete description of an Audio Class function. */
typedef struct {
    const USBAUDIO_ControlInterface *controlInterface;
    uint8_t     numStreamingInterfaces;
    USBAUDIO_StreamingInterface *streamingInterfaces;
    LPCLIB_Callback callback;
    int numEndpoints;
    const uint8_t *pEndpointList;
} USBAUDIO_FunctionDeclaration;




/**********************************************************************************************/

/** Handle for a USB audio class instance. */
typedef struct _USBAUDIO_Context *USBAUDIO_Handle;


/** Opcodes to specify the configuration command in a call to \ref USBAUDIO_ioctl. */
typedef enum USBAUDIO_Opcode {
    USBAUDIO_OPCODE_xxx,                    /**< Config action: ... */
} USBAUDIO_Opcode;



/** Descriptor to specify the configuration in a call to \ref USBAUDIO_ioctl. */
typedef struct USBAUDIO_Config {
    USBAUDIO_Opcode opcode;                 /**< Config action opcode */
    LPCLIB_Switch continued;                /**< Set if further config struct's follow in an array */

    union {
        int dummy;
    };
} USBAUDIO_Config;


/* Event opcodes for audio class */
typedef enum USBAUDIO_CallbackEvent {
    USBAUDIO_EVENT_INVALID = 0,
    USBAUDIO_EVENT_INTERFACE_CHANGE,        /**< Interface selection (alternate setting) */
    USBAUDIO_EVENT_CONFIGURATION_CHANGE,    /**< Configuration selection */
    USBAUDIO_EVENT_SET_CONTROL,             /**< Set value of a control */
    USBAUDIO_EVENT_GET_CONTROL,             /**< Get value of a control */
    USBAUDIO_EVENT_ENDPOINT,                /**< Set an endpoint parameter */
} USBAUDIO_CallbackEvent;


typedef struct {
    void (*init) (void *context);
    void (*getNextBuffer) (void *context, uint32_t *address, uint32_t *length);

} USBAUDIO_MemoryModel;


void USBAUDIO_frameHandler (USBAUDIO_Handle handle, int frameNumber);

/* Open a USB Audio Class instance. */
ErrorCode_t USBAUDIO_init(USBD_HANDLE_T hUsb,
                          const USBAUDIO_FunctionDeclaration *pFunction,
                          const USBAUDIO_MemoryModel *pMemModel,
                          void *memModelData,
                          USBAUDIO_Handle *pHandle);

#endif
