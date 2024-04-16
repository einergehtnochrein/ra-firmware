
#include <stdbool.h>

#include "CppUTest/TestHarness.h"
#include <iostream>
#include <iomanip>

#include "meisei/meiseiprivate.h"


MEISEI_InstanceData _ims100 = {
    .next = NULL,
    .id = 1,
    .serialSonde = 100350,
    .serialPcb = 609614,
    .serialSensorBoom = 308203,
    .model = MEISEI_MODEL_IMS100,
    .rxFrequencyMHz = 405.1,
    .rxOffset = 0,
    .lastUpdated = 0,
    .frameCounter = 42,
    .rssi = -91.2,
    .realTime = 1234,

    /* Config data is arbitrary (but from a real sonde).
     * It's used to verify the PTU algorithms.
     */
    .configValidFlags = 0xFFFFFFFFFFFFFFFFLL,
    .config = {
        100350.0,   0,          609614.0,   222.0,      3082203.0,  0,          36400.0,    0,
        0,          0,          0,          0,          0,          0,          1012509.0,  41.0,
        100350.0,   60.0,       39.99,      34.99,      29.97,      24.93,      20.05,      -10.0,
        -19.98,     -30.05,     -40.04,     -70.03,     -85.07,     0,          0,          0,
        100350.0,   0.658795,   1.25913,    1.49993,    1.79661,    2.1631,     2.60689,    9.28088,
        14.9521,    24.9554,    42.8286,    268.863,    724.167,    0,          0,          0,
        100350.0,   -5081.23,   7091.44,    -3115.84,   439.502,    -36.453,    99.7144,    0.59756,
        -2.87194,   3.93014e-7, 2.82543e-4, 3.33698e-3, 0,          0,          3.0,        0,
        },

    .refFreq = 0,

    /* The BCH codewords in these raw packets are extracted from the raw RX frames
     * by _ims100_helper_split_raw().
     */
    .configPacketEven = {},
    .configPacketOdd = {},
    .gpsPacketEven = {},
    .gpsPacketOdd = {},

    .frameCounterEven = 0,

    .gps = {},
    .metro = {},
};


/* Raw frames from receiver.
 * These are two consecutive frames (even and odd frame numbers 7584/7585 belonging together).
 * They each contain the payload (276 bit) of both the config and GPS blocks,
 * i.e. 552 bits (69 bytes) total. MSB of bytes is received first.
 */
static uint8_t _ims100_raw_rx_frame[2][69] = {
    {   0x1d, 0xa0, 0xc6, 0x46, 0xae, 0x50, 0x36, 0x02, 0x8f, 0x88, 0x6b, 0x00, 0x00, 0x0b, 0xae, 0x77,
        0x64, 0x16, 0x6c, 0xc6, 0x18, 0x23, 0x9b, 0x00, 0x00, 0x80, 0x4e, 0xb0, 0x01, 0x77, 0x00, 0x24,
        0x16, 0xa2, 0x53, 0x32, 0x48, 0x18, 0xdf, 0xee, 0xc0, 0x79, 0x00, 0x1b, 0x11, 0x7e, 0xea, 0xe4,
        0x03, 0xdd, 0x5c, 0x10, 0x2c, 0x6a, 0x41, 0x13, 0x61, 0x47, 0x0d, 0x09, 0x79, 0x80, 0xe3, 0x05,
        0x7d, 0x2f, 0x9f, 0x7d, 0x25, },
    {   0x1d, 0xa1, 0x00, 0x00, 0x44, 0x39, 0xc4, 0xec, 0x7e, 0x52, 0x56, 0xa0, 0x00, 0x0b, 0xae, 0x78,
        0xb6, 0xd6, 0x6c, 0xa6, 0x38, 0x3e, 0xb1, 0x98, 0x3b, 0x80, 0x00, 0x42, 0xb9, 0x74, 0x36, 0x10,
        0x00, 0x54, 0x00, 0x00, 0x08, 0x02, 0xa3, 0x0e, 0xc0, 0xc0, 0x44, 0x61, 0x8f, 0x2b, 0x0a, 0x52,
        0x1f, 0x18, 0x1c, 0x64, 0x24, 0x10, 0x12, 0x19, 0x9f, 0x10, 0x93, 0x99, 0x60, 0x00, 0x32, 0x80,
        0x00, 0x23, 0x67, 0x39, 0x1f, },
};

static MEISEI_RawPacket _ims100_raw_config_even;
static MEISEI_RawPacket _ims100_raw_gps_even;
static MEISEI_RawPacket _ims100_raw_config_odd;
static MEISEI_RawPacket _ims100_raw_gps_odd;


/* Helper function:
 *
 * A radio frame consists of two 276-bit blocks (we call it the 'config' and the 'gps' nlock), each
 * preceeded by a different 48-symbol sync word. Raw frames above contain the 276+276=552bit=69byte
 * total of both payloads.
 * These 69 bytes must be divided into 12 BCH codewords of 46 bits each. The M0+ coprocessor in this
 * application automatically splits the incoming bitstream into chunks of 46 bits, and stores them
 * into 64-bit variables. This helper function has the task to emulate the M0+ functionality and
 * split up the raw data provided in this test file.
 */
void _ims100_helper_split_raw (const uint8_t raw[69], MEISEI_RawPacket *rawConf, MEISEI_RawPacket *rawGps)
{
    for (int i = 0; i < 6; i++) {
        uint64_t x = 0;
        for (int j = 0; j < 46; j++) {
            int b = raw[(i * 46 + j) / 8];
            uint64_t bit = (b >> (7 - (i * 46 + j) % 8)) & 1;
            x |= bit << j;
        }
        rawConf->fields[i] = x;
    }

    for (int i = 6; i < 12; i++) {
        uint64_t x = 0;
        for (int j = 0; j < 46; j++) {
            int b = raw[(i * 46 + j) / 8];
            uint64_t bit = (b >> (7 - (i * 46 + j) % 8)) & 1;
            x |= bit << j;
        }
        rawGps->fields[i - 6] = x;
    }
}



TEST_GROUP(ims100)
{
    void setup()
    {
        _ims100_helper_split_raw(_ims100_raw_rx_frame[0], &_ims100_raw_config_even, &_ims100_raw_gps_even);
        _ims100_helper_split_raw(_ims100_raw_rx_frame[1], &_ims100_raw_config_odd, &_ims100_raw_gps_odd);

        _MEISEI_extractDataFromCodewords(&_ims100_raw_config_even, &_ims100.configPacketEven);
        _MEISEI_extractDataFromCodewords(&_ims100_raw_gps_even, &_ims100.gpsPacketEven);
        _MEISEI_extractDataFromCodewords(&_ims100_raw_config_odd, &_ims100.configPacketOdd);
        _MEISEI_extractDataFromCodewords(&_ims100_raw_gps_odd, &_ims100.gpsPacketOdd);

        _ims100.frameCounterEven = 7584;    //TODO take from raw RX frame
        _ims100.frameCounter = 7585;
    }

    void teardown()
    {
    }
};


TEST(ims100, metro_calc)
{
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _MEISEI_processMetrology(&_ims100));
    DOUBLES_EQUAL_TEXT(-9.037, _ims100.metro.temperature, 1e-3, "Main temperature");
    DOUBLES_EQUAL_TEXT(28.11, _ims100.metro.humidity, 1e-2, "Relative Humidity");
}


/* Test of the iMS-100 utility function to decode BCH codewords */
TEST(ims100, utils_bch)
{
    int numErrors;
    int totalErrors = 0;

    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _MEISEI_checkBCH(&_ims100_raw_config_even, &numErrors));
    totalErrors += numErrors;
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _MEISEI_checkBCH(&_ims100_raw_gps_even, &numErrors));
    totalErrors += numErrors;
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _MEISEI_checkBCH(&_ims100_raw_config_odd, &numErrors));
    totalErrors += numErrors;
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _MEISEI_checkBCH(&_ims100_raw_gps_odd, &numErrors));
    totalErrors += numErrors;

    LONGS_EQUAL_TEXT(0, totalErrors, "Total # bit errors in BCH codewords");
}
