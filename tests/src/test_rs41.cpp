
#include <stdbool.h>

#include "CppUTest/TestHarness.h"
#include <iostream>
#include <iomanip>

#include "rs41/rs41private.h"


#define U24(x) (x % 256), ((x / 256) % 256), (x / 65536)


const RS41_SubFrameMetrology sfMet = {
    .adc = {
        {.current = {U24(170206)}, .refmin = {U24(132352)}, .refmax = {U24(191917)}},
        {.current = {U24(556984)}, .refmin = {U24(489593)}, .refmax = {U24(557883)}},
        {.current = {U24(166311)}, .refmin = {U24(132351)}, .refmax = {U24(191918)}},
        {.current = {U24(329520)}, .refmin = {U24(287401)}, .refmax = {U24(414470)}},
    },
    .val12_16 = 0,
    .temperaturePressureSensor = 3479,
    .val14_16 = 0,
};



RS41_InstanceData _rs41 = {
    .next = NULL,
    .id = 1,
    .name = {'S','0','1','0','0','1','2','3'},
    .fragmentValidFlags = 0x0007FFFFFFFFFFFF,
    .lastUpdated = 0,
    .rxFrequencyMHz = 405.1,
    .batteryVoltage = 3.0,
    .temperatureTx = 25.0,
    .temperatureRef = 20.0,
    .frameCounter = 1234,
    .txPower_dBm = 13,
    .encrypted = false,
    .onDescent = false,
    .killCounterRefFrame = 1,
    .killCounterRefCount = 10000,
    .gps = {},
    .metro = {},
    .logMode = RS41_LOGMODE_NONE,
    .params = {
        .crc = 0, //TODO
        .frequency = 32640,
        .startupTxPower = 7,
        .reserved005 = 0,
        .optionFlags = 0x0F,
        .reserved009 = 0,
        .reserved00B = 0,
        .serial = {'S','0','1','0','0','1','2','3'},
        .firmwareVersion = 21500,
        .minHeight4Flight = 600,
        .lowBatVoltageThreshold = 18,
        .nfcDetectorThreshold = 5,
        .reserved01D = 0xB4,
        .reserved01E = 0,
        .reserved01F = 0,
        .refTemperatureTarget = 20,
        .lowBatCapacityThreshold = 0,
        .reserved023 = 0,
        .reserved025 = 0,
        .flightKillFrames = -1,
        .reserved029 = 0,
        .burstKill = 0,
        .reserved02C = 0,
        .reserved02D = 0,
        .freshBatteryCapacity = 9089,
        .factor1000 = 1000,
        .allowXdata = 1,
        .ubloxHwVersionHigh = 4,
        .ubloxHwVersionLow = 7,
        .ubloxSwVersion = 703,
        .ubloxSwBuild = 0,
        .ubloxConfigErrors = 0,
        .radioVersionCode = 6,
        .refResistorLow = 750.0,
        .refResistorHigh = 1100.0,
        .refCapLow = 0,
        .refCapHigh = 47.0,
        .taylorT = {-243.9108, 0.187654, 8.2e-6},
        .calT = 1.2502499,
        .polyT = {-0.21249175, 0.010971329, 0, 0, 0, 0},
        .calibU = {44.2928, 5.02358},
        .matrixU = {
            {-0.002586, -2.2437, 9.9229,  -3.6191, 54.355,  -93.301},
            {51.706,    38.871,  209.44,  -378.44, 9.1733,  19.53},
            {150.26,    -150.91, -280.31, 182.29,  3247.4,  4083.7},
            {-233.57,   345.37,  200.22,  -388.25, -3617.7, 0},
            {225.84,    -233.05, 0,       0,       0,       0},
            {-93.064,   0,       0,       0,       0,       0},
            {0,         0,       0,       0,       0,       0}
        },
        .taylorTU = {-243.9108, 0.187654, 8.2e-6},
        .calTU = 1.3070685,
        .polyTrh = {-0.0613, 0.0074332, 0, 0, 0, 0},
        .reserved14D = 0,
        .reserved14E = 0,
        .f152 = 0,
        .u156 = 0,
        .f157 = 0,
        .reserved15B = 0,
        .reserved15C = 0,
        .f160 = {0},
        .startIWDG = 0,
        .parameterSetupDone = 1,
        .enableTestMode = 0,
        .enableTX = 1,
        .f1F0 = {0},
        .pressureLaunchSite = {970.0,970.0},
        .names = {
            .variant = {'R','S','4','1','-','S','G','P'},
            .mainboard = {'R','S','M','4','1','2'},
        },
        .serials = {
            .mainboard = {'L','1','1','2','3','5','5','3'},
            .text235 = {'0','0','0','0','0','0','0','0','0','0'},
            .reserved241 = 0,
            .pressureSensor = {'N','1','3','1','0','4','8','7'},
            .reserved24B = 0,
        },
        .reserved24D = 0,
        .reserved24F = 0,
        .reserved251 = 0,
        .xdataUartBaud = 1,
        .reserved254 = 0,
        .cpuTempSensorVoltageAt25deg = 1.4,
        .reserved259 = 0,
        .reserved25A= {0},
        .matrixP = {
            -388.45581,    -79.18203,   243.82368,     -58.065853,    8.5971889,    -0.50057161,
            1.0,           0.058872867, -8.3246361e-4, -9.1532292e-3, 1.207599e-3,  3.7496924e-4,
            -3.0500942e-4, 0,           0,             -3.7212999e-6, 1.3885001e-6, 0
        },
        .vectorBp = {0.080465, 0.000219, 0.000155},
        .reserved2B2= {0},
        .matrixBt = {
            -0.02246, 0.44842, -0.6039, 0.42744,
            -14.532,  53.833,  -21.347, -24.952,
            11.921,   -51.022, 17.465,  30.525
        },
        .reserved2EA = {0},
        .halfword2FA = {0},
        .reserved30C = 0,
        .reserved310 = 0,
        .reserved314 = 0,
        .reserved315 = 0,
        .burstKillFrames = 10000,
        .reserved318 = {0},

        .killCountdown = -1,
        .launchAltitude = -4000,
        .heightOfFlightStart = 600,
        .lastTxPowerLevel = 7,
        .numSoftwareResets = 0,
        .intTemperatureCpu = 25,
        .intTemperatureRadio = 30,
        .remainingBatteryCapacity = 1000,
        .numUbxDiscarded = 0,
        .numUbxStall = 0
    },
};


/* Raw frame from receiver. Still needs dewhitening and bit reordering.
 * (Ra receiver part assumes MSB first, but RS41 sends LSB first).
 */
static uint8_t _rs41_raw_rx_frame[312] = {
    0x68, 0x0b, 0xb0, 0x85, 0x50, 0x99, 0x9c, 0x71, 0xae, 0xcc, 0x43, 0x4f, 0xc6, 0x99, 0x26, 0x0a,
    0x39, 0xbb, 0x9d, 0x0a, 0x7c, 0xfb, 0x6f, 0x28, 0xcd, 0x59, 0x3c, 0x0f, 0xa1, 0x3b, 0xa0, 0x84,
    0x2a, 0xd0, 0x8a, 0xa4, 0xbd, 0x00, 0x20, 0xce, 0x3b, 0x1c, 0x1c, 0xe4, 0x39, 0x7e, 0x9a, 0x67,
    0xee, 0xe8, 0xc8, 0x6a, 0xb5, 0xf4, 0xbe, 0xcf, 0xc5, 0xcd, 0x50, 0xa6, 0x91, 0xca, 0x10, 0x19,
    0xcc, 0xa0, 0x9a, 0x58, 0x9f, 0x22, 0xd9, 0x64, 0x64, 0x4a, 0x13, 0x95, 0x91, 0x56, 0x0b, 0x46,
    0x3f, 0x35, 0xaa, 0xb2, 0xf2, 0xab, 0x03, 0x0d, 0x38, 0xec, 0x29, 0x93, 0x32, 0xc7, 0x11, 0x63,
    0x56, 0xbe, 0x94, 0x46, 0xef, 0x04, 0xe3, 0xc6, 0x10, 0x1b, 0x3d, 0x7b, 0x36, 0xb5, 0x8b, 0x8a,
    0x0e, 0x68, 0x34, 0x35, 0xa5, 0x5e, 0x72, 0x0f, 0xfe, 0x81, 0xd6, 0x7c, 0x2d, 0x80, 0xe6, 0x39,
    0xc0, 0xae, 0xfa, 0x70, 0x9f, 0x04, 0xdc, 0x64, 0x84, 0xdc, 0xa7, 0x69, 0xe6, 0x64, 0xa6, 0x92,
    0xce, 0x4a, 0xb2, 0x60, 0xf4, 0xdf, 0xd5, 0xbf, 0x40, 0x86, 0xca, 0x76, 0x8f, 0x29, 0xaa, 0x30,
    0x02, 0xce, 0x63, 0xde, 0x30, 0x90, 0x07, 0x06, 0x7a, 0x05, 0x62, 0x0a, 0xe1, 0xeb, 0xb5, 0xc4,
    0xb8, 0x3a, 0x5c, 0x8a, 0x13, 0x2b, 0xd9, 0x6b, 0x4e, 0x17, 0x7c, 0xb8, 0x84, 0x68, 0x00, 0x9d,
    0x9c, 0xa0, 0x3f, 0xd0, 0x62, 0xb2, 0x7d, 0x1e, 0x7b, 0x43, 0x86, 0x7b, 0x06, 0x84, 0x21, 0x85,
    0x48, 0x95, 0xf6, 0x38, 0xd2, 0x32, 0x3a, 0x05, 0x56, 0x04, 0xe9, 0x3a, 0x9c, 0x19, 0xe3, 0xc9,
    0xe8, 0x46, 0x85, 0x22, 0x50, 0xab, 0x96, 0xf7, 0xbb, 0x24, 0x35, 0xed, 0xc0, 0x55, 0x2f, 0xc4,
    0xbc, 0xb6, 0xdc, 0x07, 0x9d, 0x59, 0xaa, 0x19, 0x6d, 0x3e, 0x1a, 0x41, 0x23, 0x2a, 0x6f, 0x3f,
    0x4c, 0x33, 0x38, 0x32, 0xb7, 0x70, 0xdc, 0x64, 0xdd, 0xf5, 0x9d, 0xff, 0x92, 0xf7, 0xa5, 0x9d,
    0x99, 0x0e, 0x7c, 0x90, 0xa2, 0x1e, 0x20, 0xb7, 0x4e, 0x91, 0x7f, 0x5e, 0x1f, 0xd9, 0x95, 0x74,
    0xb9, 0xb6, 0x8e, 0xa8, 0x27, 0x08, 0x74, 0x86, 0x0b, 0x3d, 0x2d, 0x6d, 0x60, 0x55, 0x2f, 0xc4,
    0x1e, 0x76, 0xdc, 0x75, 0xfd, 0xde, 0x05, 0x60, 
};


static RS41_SubFrameCalibConfig _rs41_configBlock = {
    .frameCounter = 0x0544,
    .name = {'N','3','4','2','0','1','0','0'},
    .batteryVoltage100mV = 29,
    .reserved00B = 0,
    .flags = 0,
    .cryptoMode = 0,
    .temperatureRef = 34,
    .errorLog = 0,
    .humidityHeatingPwm = 49,
    .txPower = 3,
    .maxCalibIndex = 50,
    .thisCalibIndex = 21,
    .calibFragment = {0x00,0x00,0xFF,0xFF,0xFF,0xC6,0x00,0x4E,0x64,0x2E,0x42,0x00,0x00,0x00,0x00,0x00},
    .crc = 0x387E,
};



TEST_GROUP(rs41)
{
    void setup()
    {
    }

    void teardown()
    {
    }
};


TEST(rs41, metro_calc)
{
    ENUMS_EQUAL_INT(LPCLIB_ILLEGAL_PARAMETER, _RS41_processMetrologyBlock(&sfMet, NULL));
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _RS41_processMetrologyBlock(&sfMet, &_rs41));
    DOUBLES_EQUAL_TEXT(-3.897, _rs41.metro.T, 1e-3, "Main temperature");
    DOUBLES_EQUAL_TEXT(1.620, _rs41.metro.TU, 1e-3, "U sensor temperature");
    DOUBLES_EQUAL_TEXT(584.11, _rs41.metro.pressure, 1e-2, "Air pressure");
    DOUBLES_EQUAL_TEXT(27.55, _rs41.metro.RH, 1.2e-2, "Humidity");
}


/* Test of the RS41 utility function to decode and detect RS41 short/long frames */
TEST(rs41, utils_reedsolomon)
{
    int numErrors;
    bool longFrame;

    /* Change bitorder and apply dewhitening pattern. */
    _RS41_removeWhitening(_rs41_raw_rx_frame, sizeof(_rs41_raw_rx_frame));
    /* First byte after parity block(s) must be 0x0F (short frame indicator) */
    BYTES_EQUAL(0x0F, _rs41_raw_rx_frame[48]);

    /* Must decode (both!) codewords as a short frame without errors */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _RS41_checkReedSolomon(_rs41_raw_rx_frame, &numErrors, &longFrame));
    LONGS_EQUAL_TEXT(0, numErrors, "Total number of errors detected in short RS41 frame");
    CHECK_FALSE(longFrame);

    /* TODO introduce errors */
}


/* Basic test of the block CRC check */
TEST(rs41, utils_crc16)
{
    CHECK_TRUE(_RS41_checkCRC((uint8_t *)&_rs41_configBlock, sizeof(_rs41_configBlock) - sizeof(_rs41_configBlock.crc), _rs41_configBlock.crc));
}
