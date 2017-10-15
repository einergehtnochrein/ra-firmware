
#ifndef __USB_SERIAL_H
#define __USB_SERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#define CONSUMER_REPORT_SIZE                1

/**
 * @brief	HID consumer interface init routine.
 * @param	hUsb		: Handle to USB device stack
 * @param	pIntfDesc	: Pointer to HID interface descriptor
 * @param	mem_base	: Pointer to memory address which can be used by HID driver
 * @param	mem_size	: Size of the memory passed
 * @return	On success returns LPC_OK. Params mem_base and mem_size are updated
 *			to point to new base and available size.
 */
extern ErrorCode_t USBSerial_init(int port,
                                  USBD_HANDLE_T hUsb,
                                  const USB_INTERFACE_DESCRIPTOR *pCifIntfDesc,
                                  const USB_INTERFACE_DESCRIPTOR *pDifIntfDesc,
                                  uint32_t *mem_base,
                                  uint32_t *mem_size);

extern void USBSerial_write (int port, const void *message, int len);


#ifdef __cplusplus
}
#endif

#endif /* __USB_SERIAL_H */
