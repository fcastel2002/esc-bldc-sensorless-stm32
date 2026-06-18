#include "usb_device.h"

#include "usbd_core.h"
#include "usbd_customhid.h"
#include "usbd_customhid_if.h"
#include "usbd_desc.h"

USBD_HandleTypeDef USBD_Device;

void MX_USB_DEVICE_Init(void)
{
  (void)USBD_Init(&USBD_Device, &HID_Desc, 0);
  (void)USBD_RegisterClass(&USBD_Device, USBD_CUSTOM_HID_CLASS);
  (void)USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_fops);
  (void)USBD_Start(&USBD_Device);
}

