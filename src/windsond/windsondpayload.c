
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "windsond.h"
#include "windsondprivate.h"


static uint32_t _readbits (const unsigned char rxbuf[], int *startpos, int n)
{
    uint32_t data = 0;
    for (int i = 0; i < n; i++) {
        uint32_t by = rxbuf[3 + ((*startpos + i) / 8)];
        uint32_t bi = (by >> (7 - (*startpos + i) % 8)) & 1;
        data = 2 * data + bi;
    }

    *startpos += n;

    return data;
}


/* Decode frame payload */
LPCLIB_Result _WINDSOND_processPayload (WINDSOND_InstanceData **instancePointer, const uint8_t *payload, int length, float frequencyMHz)
{
    (void)length;  //TODO should match end of frame after reading payload bits

    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    uint8_t opcode = payload[1] & 0x0F;

    /* METEO frame? */
    if (opcode == 0x0C) {
        uint16_t id = 0;
        uint8_t sid = 0;
        float temperature = NAN;
        float humidity = NAN;
        float pressure = NAN;
        float illuminance = NAN;
        float latitude = NAN;
        float longitude = NAN;
        float altitude = NAN;
        float speed_kmh = NAN;
        float direction = NAN;
        int startpos = 8;

        int nbits = payload[2];
        _Bool haveID = (payload[1] & 0xF0) == 0;
        uint8_t flags = payload[3];

        /* Sonde ID: Either the 16-bit true ID, or the 4-bit SID */
        //TODO
        if (haveID) {
            id = _readbits(payload, &startpos, 16); // id
        }
        else {
            sid = _readbits(payload, &startpos, 4); // sid
        }

        WINDSOND_InstanceData *instance = _WINDSOND_getInstanceDataStructure(frequencyMHz, id, sid);
        if (!instance) {
            return LPCLIB_ERROR;
        }
        *instancePointer = instance;

        _Bool haveHUM = false;
        if ((payload[0] == 0x04) || (payload[0] == 0x1B) || (payload[0] == 0x3E) || (payload[0] == 0x45)) {  //TODO ???
            haveHUM = true;
        }
        _Bool haveBASE1 = flags != 0;
        _Bool haveBASE2 = (flags & 0xF0) != 0;
        _Bool haveGPS = (flags >> 7) & 1;
        _Bool haveLUX = (flags >> 6) & 1;
        _Bool haveHIST = (flags >> 5) & 1;
        _Bool haveEXTRA = (flags >> 1) & 1;

       _readbits(payload, &startpos, 4);  // probably "md"

       if (haveBASE1) {
            int32_t tebits = _readbits(payload, &startpos, 14);
            temperature = -10.0f + (tebits - 8192) / 100.0f;

            if (_readbits(payload, &startpos, 2) == 1) {
                if (_readbits(payload, &startpos, 1) == 1) {
                    int tei = _readbits(payload, &startpos, 8);
                } else {
                    //????
                }
            }

            if (haveHUM) {
                humidity = 0.05f * _readbits(payload, &startpos, 11);
            }
        }

        if (haveBASE2) {
            pressure = _readbits(payload, &startpos, 16) * 0.02f;
        }

        if (haveGPS) {
            /* Flag: speed and direction follow the altitude field */
            int haveSPDANG = _readbits(payload, &startpos, 1);

            /* Flag: altitude field present */
            if (_readbits(payload, &startpos, 1) == 1) {
                if (_readbits(payload, &startpos, 1) == 0) {
                    /* Bit field contains altitude in meters */
                    altitude = _readbits(payload, &startpos, 14);

                    /* Update ref_temperature for coming altitude estimates */
#if 0
                    if (!isnan(pc->ground_altitude) && !isnan(pc->ground_pressure)) {
                        if (pressure < pc->ground_pressure - 8) {  //TODO...
                            pc->ref_temperature = (0.0065f * (altitude - pc->ground_altitude))
                                                / (1.0f - pow(pressure / pc->ground_pressure, 1.0f / 5.255f));
                        }
                    }
#endif
                }
                else {
                    /* Don't know how to interpret */
                    return LPCLIB_ERROR;
                }
            }

            /* Can/should we estimate the altitude? */
#if 0
            if (isnan(altitude)) {
                if (!isnan(pc->ground_altitude) && !isnan(pc->ground_pressure) && !isnan(pc->ref_temperature)) {
                    alt = pc->ground_altitude
                        + ((1.0 - pow(pressure / pc->ground_pressure, 1 / 5.255)) * pc->ref_temperature) / 0.0065;
                }
            }
#endif

            if (haveSPDANG) {
                speed_kmh = _readbits(payload, &startpos, 11) * 0.18f;
                direction = _readbits(payload, &startpos, 12) * 0.01f;
            }

            /* The next bit indicates the presence of a yet unknown eight bit field (some flags?) */
            if (_readbits(payload, &startpos, 1) == 1) {
                _readbits(payload, &startpos, 8);
            }

            int havePOS = 0;

            /* Latitude/Longitude comes in different formats (20...57 bits) */
            switch (_readbits(payload, &startpos, 2)) {
                case 0:     /* TODO: unknown format. 20 bits? */
                    _readbits(payload, &startpos, 20);
                    break;

                case 1:     /* 14-bit delta to last known reference position */
                    {
                        int32_t lat_delta14 = _readbits(payload, &startpos, 14);
                        int32_t lon_delta14 = _readbits(payload, &startpos, 14);

                        if (os_time - instance->timestamp_reference < 2000) { /* Insist on new reference after 20s */
                            if ((lat_delta14 < 10000) && (lon_delta14 < 10000)) {
                                int32_t lat_delta_ref = instance->latitude_reference % 10000;
                                if (abs(lat_delta14 - lat_delta_ref) >= 5000) {
                                    lat_delta14 += (lat_delta14 < lat_delta_ref) ? 10000 : -10000;
                                }
                                instance->latitude_reference = 10000 * (instance->latitude_reference / 10000) + lat_delta14;

                                int32_t lon_delta_ref = instance->longitude_reference % 10000;
                                if (abs(lon_delta14 - lon_delta_ref) >= 5000) {
                                    lon_delta14 += (lon_delta14 < lon_delta_ref) ? 10000 : -10000;
                                }
                                instance->longitude_reference = 10000 * (instance->longitude_reference / 10000) + lon_delta14;

                                havePOS = 1;
                            }
                        }
                    }
                    break;

                case 2:     /* 20-bit delta to last known reference position */
                    {
                        int32_t lat_delta20 = _readbits(payload, &startpos, 20);
                        int32_t lon_delta20 = _readbits(payload, &startpos, 20);

                        if (os_time - instance->timestamp_reference < 5000) { /* Insist on new reference after 50s */
                            if ((lat_delta20 < 1000000) && (lon_delta20 < 1000000)) {
                                int32_t lat_delta_ref = instance->latitude_reference % 1000000;
                                if (abs(lat_delta20 - lat_delta_ref) >= 500000) {
                                    lat_delta20 += (lat_delta20 < lat_delta_ref) ? 1000000 : -1000000;
                                }
                                instance->latitude_reference = 1000000 * (instance->latitude_reference / 1000000) + lat_delta20;

                                int32_t lon_delta_ref = instance->longitude_reference % 1000000;
                                if (abs(lon_delta20 - lon_delta_ref) >= 500000) {
                                    lon_delta20 += (lon_delta20 < lon_delta_ref) ? 1000000 : -1000000;
                                }
                                instance->longitude_reference = 1000000 * (instance->longitude_reference / 1000000) + lon_delta20;

                                havePOS = 1;
                            }
                        }
                    }
                    break;

                case 3:     /* Reference position (absolute value) */
                    instance->latitude_reference = _readbits(payload, &startpos, 28);
                    instance->longitude_reference = _readbits(payload, &startpos, 29);
                    instance->timestamp_reference = os_time;

                    havePOS = 1;
                    break;
            }

            if (havePOS) {
                int32_t latdeg = (instance->latitude_reference - 90000000l) / 1000000l;
                float latmin = (instance->latitude_reference - 90000000l - 1e6f*latdeg) / 10000.0f;
                latitude = (latdeg + latmin / 60.0f) * (M_PI / 180.0f);

                int32_t londeg = (instance->longitude_reference - 180000000l) / 1000000l;
                double lonmin = (instance->longitude_reference - 180000000l - 1e6f*londeg) / 10000.0f;
                longitude = (londeg + lonmin / 60.0f) * (M_PI / 180.0f);
            }
        }

        if (haveLUX) {
            illuminance = _readbits(payload, &startpos, 12);
        }

        if (haveHIST) {
            /* Read PTU history (but don't store, just ignore...) */
            int idx = 1;
            while (idx < 10) {  // TODO: Expected maximum number of history elements
            if (haveGPS) {
                /* spdX, angX */
                _readbits(payload, &startpos, 8);
                _readbits(payload, &startpos, 12);
            }

            /* teX, (huX) */
            _readbits(payload, &startpos, 8);
            if (haveHUM) {
                _readbits(payload, &startpos, 8);
            }

            /* CONTINUE bit */
            if (_readbits(payload, &startpos, 1) == 0) {
                /* End of list */
                break;
            }

            ++idx;
            }
        }

        instance->metro.temperature = temperature;
        instance->metro.humidity = humidity;
        instance->metro.pressure = pressure;
        instance->metro.illuminance = illuminance;

        instance->gps.observerLLA.lat = latitude;
        instance->gps.observerLLA.lon = longitude;
        instance->gps.observerLLA.alt = altitude;
        instance->gps.observerLLA.velocity = speed_kmh;
        instance->gps.observerLLA.direction = direction;
        GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
    }

    return LPCLIB_SUCCESS;
}

