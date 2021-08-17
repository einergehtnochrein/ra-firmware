
#ifndef __LPC54XXX_CRC_H__
#define __LPC54XXX_CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct CRC_Context *CRC_Handle;


typedef enum CRC_Polynomial {
    CRC_POLY_CRCCCITT = 0,                  /**< CRC-CCITT */
    CRC_POLY_CRC16 = 1,                     /**< CRC-16 */
    CRC_POLY_CRC32 = 2,                     /**< CRC-32 */
} CRC_Polynomial;


typedef enum CRC_DataBitOrder {
    CRC_DATAORDER_NORMAL = 0,               /**< Normal bit order */
    CRC_DATAORDER_REVERSE = 1,              /**< Reverse bit order */
} CRC_DataBitOrder;


typedef enum CRC_SumBitOrder {
    CRC_SUMORDER_NORMAL = 0,                /**< Normal bit order */
    CRC_SUMORDER_REVERSE = 1,               /**< Reverse bit order */
} CRC_SumBitOrder;


typedef enum CRC_DataPolarity {
    CRC_DATAPOLARITY_NORMAL = 0,            /**< Data not inverted */
    CRC_DATAPOLARITY_INVERSE = 1,           /**< Data inverted (one's complement) */
} CRC_DataPolarity;


typedef enum CRC_SumPolarity {
    CRC_SUMPOLARITY_NORMAL = 0,             /**< Sum not inverted */
    CRC_SUMPOLARITY_INVERSE = 1,            /**< Sum inverted (one's complement) */
} CRC_SumPolarity;


typedef uint32_t CRC_Mode;


typedef enum CRC_CallbackEvent {
    CRC_EVENT_COMPLETE = 0,                 /**< Transaction completed. */
} CRC_CallbackEvent;


/** Predefined CRC modes. */
#define CRC_MODE_HDLC \
    CRC_makeMode(CRC_POLY_CRC16, \
                 CRC_DATAORDER_NORMAL, CRC_SUMORDER_NORMAL, \
                 CRC_DATAPOLARITY_NORMAL, CRC_SUMPOLARITY_INVERSE)


LPCLIB_Result CRC_open (CRC_Mode mode, CRC_Handle *pHandle);

LPCLIB_Result CRC_close (CRC_Handle *pHandle);

uint32_t CRC_read (CRC_Handle handle);

void CRC_seed (CRC_Handle handle, uint32_t seed);


struct DMA_ChannelContext;
LPCLIB_Result CRC_write (CRC_Handle handle,
                         void *pData,
                         uint32_t numBytes,
                         LPCLIB_Callback callback,
                         struct DMA_ChannelContext *dma);


uint32_t CRC_read (CRC_Handle handle);

CRC_Mode CRC_makeMode (
        CRC_Polynomial polynomial,
        CRC_DataBitOrder dataBitOrder,
        CRC_SumBitOrder sumBitOrder,
        CRC_DataPolarity dataPolarity,
        CRC_SumPolarity sumPolarity);

void CRC_seed (CRC_Handle handle, uint32_t seed);


#ifdef __cplusplus
}
#endif

#endif

