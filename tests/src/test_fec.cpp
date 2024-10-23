
#include "CppUTest/TestHarness.h"
#include <iostream>
#include <iomanip>

#include "reedsolomon/reedsolomon.h"


/* RS41 raw frame after dewhitening and with correct bit order */
static uint8_t _rs41_raw_dewhitened[312] = {
    0x24, 0xd5, 0x54, 0xaf, 0xf3, 0xdd, 0xff, 0xa8, 0x54, 0x53, 0x00, 0x18, 0x1a, 0xc4, 0x09, 0xf1,
    0xc8, 0xb4, 0xfe, 0x5c, 0xe2, 0x37, 0xaa, 0xe5, 0x44, 0xec, 0xbe, 0x8f, 0x82, 0x45, 0xa7, 0x0d,
    0xc7, 0x77, 0x61, 0x46, 0x48, 0x10, 0x2a, 0x12, 0x0c, 0x84, 0x8c, 0x91, 0x9a, 0xd4, 0xad, 0xc5,
    0x0f, 0x79, 0x28, 0xf8, 0x12, 0x54, 0x31, 0x32, 0x35, 0x30, 0x34, 0x34, 0x38, 0x1a, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x14, 0x00, 0x00, 0x5d, 0x00, 0x07, 0x32, 0x0a, 0x43, 0xf0, 0x37, 0xbd, 0xc3,
    0xa8, 0xc5, 0x12, 0x41, 0x93, 0x3d, 0x9c, 0x41, 0xeb, 0x41, 0x16, 0xb6, 0x4b, 0x7a, 0x2a, 0xea,
    0xf9, 0x01, 0x19, 0x01, 0x02, 0x30, 0xe9, 0x02, 0xd8, 0x64, 0x08, 0x68, 0x6a, 0x07, 0x25, 0x72,
    0x08, 0x78, 0x17, 0x02, 0x1a, 0x01, 0x02, 0x31, 0xe9, 0x02, 0x55, 0x6f, 0x05, 0x48, 0x6f, 0x04,
    0x31, 0x70, 0x06, 0x00, 0x00, 0x64, 0xfd, 0x00, 0x00, 0x5b, 0x27, 0x7c, 0x1e, 0x7b, 0x08, 0xe8,
    0x27, 0x3b, 0x0a, 0x0a, 0xf3, 0x13, 0xf7, 0x0c, 0xf5, 0x17, 0xd1, 0x11, 0xf6, 0x0d, 0xf7, 0x20,
    0xd3, 0x0f, 0xf6, 0x18, 0xf9, 0x19, 0xce, 0x01, 0x8e, 0x1c, 0xf2, 0xe6, 0x81, 0x7d, 0x59, 0x00,
    0x65, 0x32, 0x01, 0xff, 0x77, 0xaf, 0xd7, 0x17, 0xe4, 0x6b, 0x00, 0x4c, 0x90, 0x5f, 0x08, 0x21,
    0x0b, 0x00, 0xa5, 0x05, 0xbf, 0x09, 0x78, 0x5e, 0xff, 0xa2, 0xa3, 0x34, 0x19, 0x7c, 0xe9, 0x00,
    0x46, 0xc0, 0x28, 0x10, 0x97, 0xa4, 0x00, 0x51, 0x9d, 0x56, 0x15, 0x23, 0x3e, 0x01, 0x65, 0xbf,
    0x84, 0x1e, 0x91, 0x27, 0xff, 0xc5, 0x47, 0x8e, 0x0d, 0x98, 0x18, 0x01, 0x05, 0x00, 0x00, 0x00,
    0x45, 0x03, 0x00, 0x4e, 0x06, 0xe1, 0x19, 0x59, 0x20, 0xff, 0x66, 0xd3, 0x75, 0x1d, 0xfe, 0x64,
    0x00, 0xc9, 0x45, 0x42, 0x14, 0x4a, 0xfd, 0x00, 0x9a, 0xcf, 0x7b, 0x15, 0x30, 0xb2, 0xc8, 0x18,
    0xcd, 0x19, 0x79, 0x05, 0x99, 0x90, 0x58, 0x1c, 0x85, 0xff, 0x7c, 0x05, 0xff, 0x02, 0x0b, 0x02,
    0x0e, 0x11, 0x41, 0x76, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xec, 0xc7
};

/* CF-06-AH raw frame with correct bit order */
static uint8_t _cf06_raw[99] = {
    0xff, 0xaa, 0xaa, 0x15, 0x2f, 0x03, 0x09, 0x00, 0xed, 0x3a, 0x0f, 0x8d, 0xf9, 0xbc, 0x08, 0x90,
    0x2d, 0x15, 0x1f, 0xfc, 0x49, 0x00, 0x01, 0x3a, 0x04, 0xeb, 0xff, 0x4a, 0xfe, 0x0f, 0x0c, 0x03,
    0xeb, 0xe0, 0xfd, 0x00, 0x00, 0xff, 0x3a, 0xeb, 0x35, 0x15, 0x78, 0xa6, 0xb8, 0x48, 0xf5, 0x26,
    0xa4, 0x0a, 0x56, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x03, 0xeb, 0xe0,
    0xfd, 0x00, 0x00, 0x3a, 0xeb, 0xa7, 0xff, 0x3b, 0x05, 0x29, 0xfe, 0x03, 0xeb, 0xd6, 0xfd, 0x00,
    0x00, 0x36, 0xeb, 0xa0, 0xff, 0xba, 0x05, 0x50, 0xfe, 0xab, 0xed, 0x40, 0xe5, 0x8d, 0x64, 0x01,
    0xb0, 0x30, 0x00
};

/* LMS-6 RS(255,223) codeword */
static uint8_t _lms6_raw[255] = {
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x24, 0x54, 0x00, 0x00, 0x00, 0x12, 0xd6, 0x87, 0x00, 0x03, 0x00, 0x00, 0x0f, 0xa0,
    0x00, 0x00, 0x00, 0x00, 0x1b, 0xc3, 0xd7, 0x12, 0xbd, 0x5b, 0x71, 0x18, 0x00, 0x03, 0x7c, 0xf8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44,
    0x4a, 0xb0, 0xc5, 0x76, 0x99, 0x46, 0x65, 0xf1, 0x74, 0x00, 0x57, 0xa0, 0x19, 0x4e, 0xec, 0x24,
    0xf2, 0xa7, 0xa2, 0x16, 0xe2, 0x35, 0xe0, 0xa1, 0xd1, 0x94, 0x5d, 0xe4, 0xa8, 0x85, 0x07
};



static uint8_t _rs41_null;

static uint8_t * _rs41_getDataAddressShort1 (int index)
{
    if (index < 132) {
        return &_rs41_raw_dewhitened[48 + 2 * index];
    }
    else if (index >= 231) {
        return &_rs41_raw_dewhitened[index - 231];
    }
    else {
        _rs41_null = 0;
        return &_rs41_null;
    }
}

uint8_t * _rs41_getDataAddressShort2 (int index)
{
    if (index < 132) {
        return &_rs41_raw_dewhitened[49 + 2 * index];
    }
    else if (index >= 231) {
        return &_rs41_raw_dewhitened[index - 207];
    }
    else {
        _rs41_null = 0;
        return &_rs41_null;
    }
}

static uint8_t _cf06_null;

static uint8_t * _cf06_getData1 (int index)
{
    if (index < 42) {
        return &_cf06_raw[44 - index];
    }
    else if (index >= 249) {
        return &_cf06_raw[299 - index];
    }
    else {
        _cf06_null = 0;
        return &_cf06_null;
    }
}

uint8_t * _cf06_getData2 (int index)
{
    if (index < 89) {
        return &_cf06_raw[91 - index];
    }
    else if (index >= 249) {
        return &_cf06_raw[346 - index];
    }
    else {
        _cf06_null = 0;
        return &_cf06_null;
    }
}

uint8_t * _lms6_getData (int index)
{
    return &_lms6_raw[index];
}


TEST_GROUP(fec)
{
    void setup()
    {
    }

    void teardown()
    {
    }
};


/* Test of the "general" Reed Solomon decoder, but uses RS41 frame as an example */
TEST(fec, reedsolomon_rs41)
{
    int numErrors;
    int i;

    /* Prepare Galois field */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_makeGaloisField(0x11D));

    /* Begin with a correct codeword. Should be accepted, and no errors detected.
     * Actually a RS41 frame consists of two interleaved codewords, so do two tests!
     */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(24, 0, 1, 1, _rs41_getDataAddressShort1, &numErrors));
    LONGS_EQUAL_TEXT(0, numErrors, "No corrections in valid CW1");
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(24, 0, 1, 1, _rs41_getDataAddressShort2, &numErrors));
    LONGS_EQUAL_TEXT(0, numErrors, "No corrections in valid CW2");

    /* Add some errors, max. 12 in both codewords. This is still correctable. */
    int errorLocations1[] = {0, 23, 62, 64, 66, 110, 120, 130, 200, 292, 296, 300}; /* 0,23 in parity block */
    int nErrors1 = sizeof(errorLocations1) / sizeof(errorLocations1[0]);
    for (i = 0; i < nErrors1; i++) {
        _rs41_raw_dewhitened[errorLocations1[i]] ^= 0x42;
    }
    int errorLocations2[] = {24, 29, 43, 67, 91, 93, 95, 97, 121, 199, 223, 307};   /* 24,29,43 in parity block */
    int nErrors2 = sizeof(errorLocations2) / sizeof(errorLocations2[0]);
    for (i = 0; i < nErrors2; i++) {
        _rs41_raw_dewhitened[errorLocations2[i]] ^= 0xFF;
    }

    /* Codewords must be corrected, and correct number of errors detected */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(24, 0, 1, 1, _rs41_getDataAddressShort1, &numErrors));
    LONGS_EQUAL_TEXT(nErrors1, numErrors, "Number of errors injected in CW1");
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(24, 0, 1, 1, _rs41_getDataAddressShort2, &numErrors));
    LONGS_EQUAL_TEXT(nErrors2, numErrors, "Number of errors injected in CW2");

    /* TODO validate corrected frames! */
}


/* Test of the "general" Reed Solomon decoder, but uses CF-06-AH frame as an example */
TEST(fec, reedsolomon_cf06)
{
    int numErrors;
    int i;

    /* Prepare Galois field */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_makeGaloisField(0x11D));

    /* Begin with a correct codeword. Should be accepted, and no errors detected.
     * Actually a CF06 frame consists of two codewords, so do two tests!
     */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(6, 1, 1, 1, _cf06_getData1, &numErrors));
    LONGS_EQUAL_TEXT(0, numErrors, "No corrections in valid CW1");
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(6, 1, 1, 1, _cf06_getData2, &numErrors));
    LONGS_EQUAL_TEXT(0, numErrors, "No corrections in valid CW2");

    /* Add some errors, max. 3 in both codewords. This is still correctable.
     * Note that codeword 1 is a subset of codeword 2! We add three errors to CW1 and three errors
     * to the codeword 2 only part, so at the beginning there are three errors in CW1 and six errors
     * in CW2. We correct CW1 first, so CW2 becomes correctable.
     */
    int errorLocations1[] = {4, 39, 45}; /* 45 in parity block */
    int nErrors1 = sizeof(errorLocations1) / sizeof(errorLocations1[0]);
    for (i = 0; i < nErrors1; i++) {
        _cf06_raw[errorLocations1[i]] ^= 0x42;
    }
    int errorLocations2[] = {58, 69, 97};   /* 97 in parity block */
    int nErrors2 = sizeof(errorLocations2) / sizeof(errorLocations2[0]);
    for (i = 0; i < nErrors2; i++) {
        _cf06_raw[errorLocations2[i]] ^= 0xFF;
    }

    /* Codewords must be corrected, and correct number of errors detected */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(6, 1, 1, 1, _cf06_getData1, &numErrors));
    LONGS_EQUAL_TEXT(nErrors1, numErrors, "Number of errors injected in CW1");
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(6, 1, 1, 1, _cf06_getData2, &numErrors));
    LONGS_EQUAL_TEXT(nErrors2, numErrors, "Number of errors injected in CW2");

    /* TODO validate corrected frames! */
}


/* Test of the "general" Reed Solomon decoder, uses LMS-6 frame as an example */
TEST(fec, reedsolomon_lms6)
{
    int numErrors;
    int i;

    /* Prepare Galois field */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_makeGaloisField(0x187));

    /* Begin with a correct codeword. Should be accepted, and no errors detected.
     */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(32, 112, 11, 116, _lms6_getData, &numErrors));
    LONGS_EQUAL_TEXT(0, numErrors, "No corrections in valid CW");

    /* Add some errors, max. 16. This is still correctable.
     */
    int errorLocations[] = {4, 13, 14, 39, 71, 72, 73, 111, 117, 132, 133, 134, 135, 240}; /* 240 in parity block */
    int nErrors = sizeof(errorLocations) / sizeof(errorLocations[0]);
    for (i = 0; i < nErrors; i++) {
        _lms6_raw[errorLocations[i]] ^= 0x42;
    }

    /* Codeword must be corrected, and correct number of errors detected */
    ENUMS_EQUAL_INT(LPCLIB_SUCCESS, REEDSOLOMON_process(32, 112, 11, 116, _lms6_getData, &numErrors));
    LONGS_EQUAL_TEXT(nErrors, numErrors, "Number of errors injected in CW");

    /* TODO validate corrected frame! */
}
