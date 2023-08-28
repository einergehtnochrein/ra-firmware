
#include "CppUTest/TestHarness.h"
#include <iostream>
#include <iomanip>

#include "monitor/monprivate.h"

/*
 * NOTE: This test does not cover the target assembler implementation
 *       of parts of the encoder.
 *       C implementations of these functions exist in 'stubs/adpcm.c'
 */

TEST_GROUP(adpcm)
{
    void setup()
    {
    }

    void teardown()
    {
    }
};


/* Test of the 16-kbit/s encoder using official "normal" test pattern. */
TEST(adpcm, nrm16)
{
    int numErrors;
    int i;
    adpcm_t adpcm;


    ADPCM_init(&adpcm);

    /* Read PCM test pattern */
    FILE *pcmfile = fopen("src/data/nrm.pcm","rb");
    CHECK(0 != pcmfile);
    fseek(pcmfile, 0, SEEK_END);
    int pcmsize = 2 * (ftell(pcmfile) / 2);
    int numSamples = pcmsize / 2;
    fseek(pcmfile, 0, SEEK_SET);
    int16_t *pcm = (int16_t *)malloc(pcmsize);
    CHECK(0 != pcm);
    fread(pcm, pcmsize, 1, pcmfile);
    fclose(pcmfile);

    /* Store ADPCM codes one byte per sample. */
    uint8_t *codes = (uint8_t *)malloc(numSamples);
    CHECK(0 != codes);

    /* Process PCM samples. Store ADPCM output codes. */
    for (i = 0; i < numSamples; i++) {
        int32_t sl = (int16_t)(floor(pcm[i] / 5.0));
        codes[i] = ADPCM_processSample(&adpcm, sl);

#if 1
        printf("%6d,%6d,%6d,%2X,%6d,"
               "%6d,%6d,%6d,%6d,%6d,%6d,%6d,"
               "%6d,%6d,%6d,%6d,"
               "%6d,%6d,"
               "%6d,%6d,%6d,%6d,%6d,%6d,"
               "%6d,%6d,%6d,%6d,%6d,%6d,"
               "%3d\n",
                    sl, adpcm.y, adpcm.d, codes[i], adpcm._dq,
                    adpcm.sez, adpcm.se, adpcm.yu, adpcm.yl, adpcm.ap, adpcm.dms, adpcm.dml,
                    adpcm._sr, adpcm.sr[0], adpcm.sr[1], adpcm.dqsez,
                    adpcm.a[0], adpcm.a[1],
                    adpcm.b[0], adpcm.b[1], adpcm.b[2], adpcm.b[3], adpcm.b[4], adpcm.b[5],
                    adpcm.dq[0], adpcm.dq[1], adpcm.dq[2], adpcm.dq[3], adpcm.dq[4], adpcm.dq[5],
                    adpcm.td
                    );
#endif
    }

#if 0
    for (i = 0; i < numSamples; i++) {
        if (i % 16 == 0) {
            printf("\n");
        }
        printf(" %02X", codes[i]);
    }
    printf("\n");
#endif

    free(codes);
    free(pcm);
}
