#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#include "usbd_def.h"

#define DEVICE_ID1 0x1FFFF7E8U
#define DEVICE_ID2 0x1FFFF7ECU
#define DEVICE_ID3 0x1FFFF7F0U

#define USB_SIZ_STRING_SERIAL 0x1AU

extern USBD_DescriptorsTypeDef HID_Desc;

#endif /* __USBD_DESC_H */

