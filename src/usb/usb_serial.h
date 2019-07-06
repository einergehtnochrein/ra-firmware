
#ifndef __USB_SERIAL_H
#define __USB_SERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

extern ErrorCode_t USBSerial_init(USBD_HANDLE_T hUsb,
                                  const USB_INTERFACE_DESCRIPTOR *pCifIntfDesc,
                                  const USB_INTERFACE_DESCRIPTOR *pDifIntfDesc,
                                  uint32_t *mem_base,
                                  uint32_t *mem_size);

int USBSerial_read (void *message, int maxLen);
extern void USBSerial_write (const void *message, int len);
void USBSERIAL_worker (void);


#ifdef __cplusplus
}
#endif

#endif /* __USB_SERIAL_H */
