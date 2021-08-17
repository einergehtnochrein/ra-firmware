
#include "CppUTest/TestHarness.h"

#include "src/rs41/rs41private.h"


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
        .reserved030 = 0,
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
    ENUMS_EQUAL_INT(LPCLIB_ILLEGAL_PARAMETER, _RS41_processMetrologyBlock(&sfMet, NULL, NULL));
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, _RS41_processMetrologyBlock(&sfMet, &_rs41.metro, &_rs41));
    DOUBLES_EQUAL_TEXT(-3.897, _rs41.metro.T, 1e-3, "Main temperature");
    DOUBLES_EQUAL_TEXT(1.620, _rs41.metro.temperatureUSensor, 1e-3, "U sensor temperature");
    DOUBLES_EQUAL_TEXT(584.11, _rs41.metro.pressure, 1e-2, "Air pressure");
    DOUBLES_EQUAL_TEXT(27.55, _rs41.metro.RH, 1.2e-2, "Humidity");
}

