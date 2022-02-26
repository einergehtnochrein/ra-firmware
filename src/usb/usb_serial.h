
#ifndef __USB_SERIAL_H
#define __USB_SERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

ErrorCode_t USBSerial_init(
        USBD_HANDLE_T hUsb,
        const USB_INTERFACE_DESCRIPTOR *pCifIntfDesc,
        const USB_INTERFACE_DESCRIPTOR *pDifIntfDesc,
        uint32_t *mem_base,
        uint32_t *mem_size);

int USBSerial_read (void *message, int maxLen);
int USBSERIAL_readLine (void *buffer, int nbytes);
void USBSerial_write (const void *message, int len);
void USBSerial_sendNotification(uint8_t type, uint16_t data);
void USBSERIAL_worker (void);


#ifdef __cplusplus
}
#endif

#endif /* __USB_SERIAL_H */
