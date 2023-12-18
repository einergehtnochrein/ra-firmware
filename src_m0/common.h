#ifndef __COMMON_H
#define __COMMON_H

/* Include this header in both projects (M4 and M0+ core).
 * Contains definitions for inter-processor communication.
 * TODO: For now this is M0+ only.
 */

#define IPC_S2M_DATA_SIZE                   1024
#define IPC_S2M_NUM_BUFFERS                 4

typedef struct {
    volatile uint8_t valid;
    uint8_t opcode;
    uint8_t param;
    uint8_t reserved;
    uint16_t numBits;
    uint16_t rxTime;

    uint8_t data8[IPC_S2M_DATA_SIZE];
} IPC_S2M;

typedef enum {
    IPC_PACKET_TYPE_VAISALA_RS92 = 0,
    IPC_PACKET_TYPE_VAISALA_RS41 = 1,
    IPC_PACKET_TYPE_GRAW_INVERTED = 2,
    IPC_PACKET_TYPE_GRAW_NORMAL = 3,
    IPC_PACKET_TYPE_MODEM = 255,
    IPC_PACKET_TYPE_MODEM_M10 = 4,
    IPC_PACKET_TYPE_MODEM_M20 = 5,
    IPC_PACKET_TYPE_MODEM_PILOT = 7,
    IPC_PACKET_TYPE_MEISEI_CONFIG = 8,
    IPC_PACKET_TYPE_MEISEI_GPS = 9,
    IPC_PACKET_TYPE_JINYANG_RSG20 = 10,
    IPC_PACKET_TYPE_WINDSOND_S1 = 11,
    IPC_PACKET_TYPE_MRZ = 12,
    IPC_PACKET_TYPE_HT03G_CF06AH = 13,
    IPC_PACKET_TYPE_VIKRAM_PSB3 = 14,
    IPC_PACKET_TYPE_IMET54 = 15,
} IPC_PACKET_TYPE;

#endif
