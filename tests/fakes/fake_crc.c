
#include <stdio.h>

#include "lpclib.h"



/** Local context of CRC block. */
static struct CRC_Context {
    uint32_t crc;
    uint32_t mask;
    int poly;
    uint32_t xorout;
    int data_invers;
    int data_reverse;
} _crc;


LPCLIB_Result CRC_open (CRC_Mode mode, CRC_Handle *pHandle)
{
    CRC_Handle handle = &_crc;

    handle->mask = 0xFFFF;
    handle->poly = ((mode >> 0) & 0x03);
    if (handle->poly == 2) {
        handle->mask = 0xFFFFFFFF;
    }
    
    handle->xorout = (mode & (1 << 5)) ? (0xFFFFFFFF & handle->mask) : 0;
    handle->data_invers = (mode & (1 << 3)) ? 1 : 0;
    handle->data_reverse = (mode & (1 << 2)) ? 1 : 0;
#if 0
        (sumBitOrder << 4)      |
#endif
    *pHandle = handle;
    return LPCLIB_SUCCESS;
}


LPCLIB_Result CRC_close (CRC_Handle *pHandle)
{
    *pHandle = LPCLIB_INVALID_HANDLE;
    return LPCLIB_SUCCESS;
}


LPCLIB_Result CRC_write (CRC_Handle handle,
                         void *pData,
                         uint32_t numBytes,
                         LPCLIB_Callback callback,
                         struct DMA_ChannelContext *dma)
{
    uint32_t n;
    int i;

    for (n = 0; n < numBytes; n++) {
        uint8_t b = ((uint8_t *)pData)[n];
        if (handle->data_invers) {
            b ^= 0xFF;
        }

        switch (handle->poly) {
            case 0:
                if (handle->data_reverse) {
                }
                else {
                    handle->crc ^= (b << 8);
                    for (i = 0; i < 8; i++) {
                        if (handle->crc & 0x8000) {
                            handle->crc = ((handle->crc << 1) ^ 0x1021) & handle->mask;
                        }
                        else {
                            handle->crc = (handle->crc << 1) & handle->mask;
                        }
                    }
                }
                handle->crc &= handle->mask;
                break;
            case 1:
                if (handle->data_reverse) {
                }
                else {
                    handle->crc ^= (b << 8);
                    for (i = 0; i < 8; i++) {
                        if (handle->crc & 0x8000) {
                            handle->crc = ((handle->crc << 1) ^ 0x8005) & handle->mask;
                        }
                        else {
                            handle->crc = (handle->crc << 1) & handle->mask;
                        }
                    }
                }
                handle->crc &= handle->mask;
                break;
            case 2:
                break;
        }
    }

    return LPCLIB_SUCCESS;                          /* No callback! */
}


uint32_t CRC_read (CRC_Handle handle)
{
    return ((handle->crc ^ handle->xorout) & handle->mask);
}


CRC_Mode CRC_makeMode (
        CRC_Polynomial polynomial,
        CRC_DataBitOrder dataBitOrder,
        CRC_SumBitOrder sumBitOrder,
        CRC_DataPolarity dataPolarity,
        CRC_SumPolarity sumPolarity)
{
    return (CRC_Mode)(
        (polynomial << 0)       |
        (dataBitOrder << 2)     |
        (sumBitOrder << 4)      |
        (dataPolarity << 3)     |
        (sumPolarity << 5)
        );
}


void CRC_seed (CRC_Handle handle, uint32_t seed)
{
    handle->crc = seed & handle->mask;
}

