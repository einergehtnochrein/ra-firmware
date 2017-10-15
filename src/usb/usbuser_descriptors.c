/* Copyright (c) 2015, DF9DQ
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


#include <stdlib.h>

#include "usbuser_config.h"
#include "usbd_strings.h"


    /* --- String Descriptors --- */

/** Enumerate all string descriptors.
 *
 *  Use these names in the string descriptor table and the device/configuration descriptors
 *  to achieve an automatic numbering of all string descriptors.
 *
 *  The first entry must be assigned the index 1.
 *  The last entry must be USB_NUMBER_OF_STRING_DESCRIPTORS.
 */
enum {
    USBSTR_IDVENDOR = 1,
    USBSTR_IDPRODUCT,
    USBSTR_SERIALNUMBER,

    USBSTR_SERIAL_IAD,
    USBSTR_SERIAL_CIF,
    USBSTR_SERIAL_DIF,

    USBSTR_AUDIO_CONTROL,
    USBSTR_AUDIO_INPUT_TERMINAL,
    USBSTR_AUDIO_FEATURE,
    USBSTR_AUDIO_OUTPUT_TERMINAL,

    USBSTR_AUDIO_STREAM,
};

/* NOTE: Make sure there is no comma after the last entry! */
static const DECLARE_USBD_STRINGS (
    theUSB_StringDescriptor,
    0x0409,
    L"radiosonde.leckasemmel.de",
    L"Ra Sondengott",
    L"*",

    L"Serial IAD",
    L"Serial CIF",
    L"Serial DIF",

    L"Audio Control",
    L"Audio Input Terminal",
    L"Audio Feature",
    L"Audio Output Terminal",

    L"Audio Stream"
);


/** Descriptor structures of application-specific size. */

_USBAUDIO_ControlInterfaceHeaderDescriptorDef(USBAPP_AudioControlInterfaceHeaderDescriptor, 1); /* n = 1 streaming interface */
_USBAUDIO_FeatureUnitDescriptorDef(USBAPP_AudioFeatureUnitDescriptor, 2, uint8_t);              /* ch = 2 */
_USBAUDIO_TypeIFormatDescriptorDef(USBAPP_FormatDescriptor1, 1);                                /* n = 1 sample frequency */


/* For convenience */
#define LE24(w) ((w) % 256UL), (((w) % 65536UL) / 256UL), ((w) / 65536UL)

/** Device descriptor. */
ALIGNED(4) const USB_DEVICE_DESCRIPTOR appDeviceDescriptor = {
    .bLength                    = USB_DEVICE_DESC_SIZE,
    .bDescriptorType            = USB_DEVICE_DESCRIPTOR_TYPE,
    .bcdUSB                     = 0x0200,
    .bDeviceClass               = USB_DEVICE_CLASS_MISCELLANEOUS,
    .bDeviceSubClass            = 2,
    .bDeviceProtocol            = 1,
    .bMaxPacketSize0            = 64,
    .idVendor                   = 0x16C0,
    .idProduct                  = 0x05DC,
    .bcdDevice                  = 0x0100,
    .iManufacturer              = USBSTR_IDVENDOR,
    .iProduct                   = USBSTR_IDPRODUCT,
    .iSerialNumber              = USBSTR_SERIALNUMBER,
    .bNumConfigurations         = USBCONFIG_NUM_CONFIGURATIONS,
};


/** Configuration 1 */
ALIGNED(4) const __PACKED(struct {
    USB_CONFIGURATION_DESCRIPTOR                    config;

    USB_INTERFACE_ASSOCIATION_DESCRIPTOR            serialIAD;
    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
        __PACKED(struct _serialControlInterfaceSpec {
            CDC_HEADER_DESCRIPTOR                       functionalHeader;
            CDC_ABSTRACT_CONTROL_MANAGEMENT_DESCRIPTOR  acm;
            CDC_UNION_DESCRIPTOR                        unionInterface;
            CDC_CALL_MANAGEMENT_DESCRIPTOR              callManagement;
            USB_ENDPOINT_DESCRIPTOR                     endpoint_int;
        }) specification;
    }) serialCIF;
    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
        USB_ENDPOINT_DESCRIPTOR                         endpoint_out;
        USB_ENDPOINT_DESCRIPTOR                         endpoint_in;
    }) serialDIF;

    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
        __PACKED(struct _audioControlInterfaceSpec {
            USBAPP_AudioControlInterfaceHeaderDescriptor interfaceHeader;
            USBAUDIO_InputTerminalDescriptor            inputTerminal;
            USBAPP_AudioFeatureUnitDescriptor           featureUnit;
            USBAUDIO_OutputTerminalDescriptor           outputTerminal;
        }) specification;
    }) audioControl;

    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
    }) audioStream_alt0;

    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor1                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    }) audioStream_alt1;

    uint8_t                                         terminator;

}) appConfiguration1 = {

    .config = {
        .bLength                = USB_CONFIGURATION_DESC_SIZE,
        .bDescriptorType        = USB_CONFIGURATION_DESCRIPTOR_TYPE,
        .wTotalLength           = sizeof(appConfiguration1) - 1,
        .bNumInterfaces         = USBCONFIG_NUM_INTERFACES,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80,     /* bus-powered */
        .bMaxPower              = 250/2,
    },

    .serialIAD = {
        .bLength                = USB_INTERFACE_ASSOCIATION_DESC_SIZE,
        .bDescriptorType        = USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,
        .bFirstInterface        = USBCONFIG_INTERFACE_SERIAL_CIF,
        .bInterfaceCount        = 2,
        .bFunctionClass         = CDC_COMMUNICATION_INTERFACE_CLASS,
        .bFunctionSubClass      = CDC_ABSTRACT_CONTROL_MODEL,
        .bFunctionProtocol      = CDC_PROTOCOL_COMMON_AT_COMMANDS,
        .iFunction              = 0,
    },

    .serialCIF = {
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_SERIAL_CIF,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = CDC_COMMUNICATION_INTERFACE_CLASS,
            .bInterfaceSubClass     = CDC_ABSTRACT_CONTROL_MODEL,
            .bInterfaceProtocol     = CDC_PROTOCOL_COMMON_AT_COMMANDS,
            .iInterface             = USBSTR_SERIAL_CIF,
        },

        .specification = {
            .functionalHeader = {
                .bLength                = CDC_HEADER_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_HEADER,
                .bcdCDC                 = 0x0110,   /* 1.10 */
            },

            .acm = {
                .bLength                = CDC_ABSTRACT_CONTROL_MANAGEMENT_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_ABSTRACT_CONTROL_MANAGEMENT,
                .bmCapabilities         = 0x02,
            },

            .unionInterface = {
                .bLength                = CDC_UNION_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_UNION,
                .bMasterInterface       = USBCONFIG_INTERFACE_SERIAL_CIF,
                .bSlaveInterface0       = USBCONFIG_INTERFACE_SERIAL_DIF,
            },

            .callManagement = {
                .bLength                = CDC_CALL_MANAGEMENT_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_CALL_MANAGEMENT,
                .bmCapabilities         = 0x01,
                .bDataInterface         = 0,
            },

            .endpoint_int = {
                .bLength                = USB_ENDPOINT_DESC_SIZE,
                .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
                .bEndpointAddress       = USBCONFIG_SERIAL_CIF_EP_INT,
                .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,
                .wMaxPacketSize         = USBCONFIG_SERIAL_CIF_EP_INT_SIZE,
                .bInterval              = 10,
            },
        },
    },

    .serialDIF = {
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_SERIAL_DIF,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 2,
            .bInterfaceClass        = CDC_DATA_INTERFACE_CLASS,
            .bInterfaceSubClass     = 0,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_SERIAL_DIF,
        },

        /* DIF OUT endpoint */
        .endpoint_out = {
            .bLength                = USB_ENDPOINT_DESC_SIZE,
            .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress       = USBCONFIG_SERIAL_DIF_EP_OUT,
            .bmAttributes           = USB_ENDPOINT_TYPE_BULK,
            .wMaxPacketSize         = USBCONFIG_SERIAL_DIF_EP_OUT_SIZE,
            .bInterval              = 0,
        },

        /* DIF IN endpoint */
        .endpoint_in = {
            .bLength                = USB_ENDPOINT_DESC_SIZE,
            .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress       = USBCONFIG_SERIAL_DIF_EP_IN,
            .bmAttributes           = USB_ENDPOINT_TYPE_BULK,
            .wMaxPacketSize         = USBCONFIG_SERIAL_DIF_EP_IN_SIZE,
            .bInterval              = 0,
        },
    },

    .audioControl = {
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_CONTROL,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOCONTROL,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_AUDIO_CONTROL,
        },

        .specification = {
            /* Class-Specific Audio Control Interface */
            .interfaceHeader = {
                .bLength                = sizeof(USBAPP_AudioControlInterfaceHeaderDescriptor),
                .bDescriptorType        = USBD_AUDIO_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_HEADER,
                .bcdADC                 = 0x0100,   /* 1.0 */
                .wTotalLength           = sizeof(struct _audioControlInterfaceSpec),
                .bInCollection          = 1,
                .baInterfaceNr          = {
                    USBCONFIG_INTERFACE_AUDIO_STREAM,
                },
            },

            /* Input Terminal */
            .inputTerminal = {
                .bLength                = sizeof(USBAUDIO_InputTerminalDescriptor),
                .bDescriptorType        = USBD_AUDIO_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN,
                .wTerminalType          = USBAUDIO_TERMINAL_EXTERNAL_ANALOG_CONNECTOR,
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 2,
                .wChannelConfig         = 0x0003,
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_AUDIO_INPUT_TERMINAL,
            },

            /* Feature Unit (UDA) */
            .featureUnit = {
                .bLength                = sizeof(USBAPP_AudioFeatureUnitDescriptor),
                .bDescriptorType        = USBD_AUDIO_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x03,               /* bmaControls(0)       Master: Mute+Volume */
                    0x00,               /* bmaControls(1)       CH1                 */
                    0x00,               /* bmaControls(1)       CH2                 */
                },
                .iFeature               = USBSTR_AUDIO_FEATURE,
            },

            /* Output Terminal I/Q */
            .outputTerminal = {
                .bLength                = sizeof(USBAUDIO_OutputTerminalDescriptor),
                .bDescriptorType        = USBD_AUDIO_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_OUTPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_OUT,
                .wTerminalType          = USBAUDIO_TERMINAL_USB_STREAMING,
                .bAssocTerminal         = 0,        /* no association */
                .bSourceID              = USBCONFIG_UNIT_FEATURE,
                .iTerminal              = USBSTR_AUDIO_OUTPUT_TERMINAL,
            },
        },
    },

    .audioStream_alt0 = {
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAM,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_AUDIO_STREAM,
        },
    },

    .audioStream_alt1 = {
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAM,
            .bAlternateSetting      = 1,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_AUDIO_STREAM,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = sizeof(USBAUDIO_StreamingInterfaceDescriptor),
            .bDescriptorType        = USBD_AUDIO_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = sizeof(USBAPP_FormatDescriptor1),
            .bDescriptorType        = USBD_AUDIO_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 2,
            .bSubFrameSize          = 2,
            .bBitResolution         = 16,
            .bSamFreqType           = 1,        /* 1 frequency follows */
            .tSamFreq               = {
                { LE24(16000UL), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = sizeof(USB_EndpointDescriptorIso),
            .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress       = USBCONFIG_AUDIO_IN_EP,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_AUDIO_IN_EP_SIZE,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = sizeof(USBAUDIO_IsoDataEndpointDescriptor),
            .bDescriptorType        = USBD_AUDIO_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },

    .terminator = 0,
};

