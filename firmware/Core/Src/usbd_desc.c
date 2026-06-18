#include "usbd_desc.h"

#include "usbd_core.h"
#include "usbd_ctlreq.h"

/* TODO: Replace these placeholder VID/PID/string values before product use. */
#define USBD_VID 0x3232U
#define USBD_PID 0xEC32U
#define USBD_LANGID_STRING 0x0409U
#define USBD_MANUFACTURER_STRING "Francisco Castel / Uncuyo"
#define USBD_PRODUCT_FS_STRING "ESC BLDC STM32"
#define USBD_CONFIGURATION_FS_STRING "ESC BLDC HID Config"
#define USBD_INTERFACE_FS_STRING "ESC BLDC HID Control Interface"

static uint8_t* USBD_HID_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static uint8_t* USBD_HID_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static uint8_t* USBD_HID_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static uint8_t* USBD_HID_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static uint8_t* USBD_HID_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static uint8_t* USBD_HID_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static uint8_t* USBD_HID_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
static void IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len);
static void Get_SerialNum(void);

USBD_DescriptorsTypeDef HID_Desc = {
  USBD_HID_DeviceDescriptor,
  USBD_HID_LangIDStrDescriptor,
  USBD_HID_ManufacturerStrDescriptor,
  USBD_HID_ProductStrDescriptor,
  USBD_HID_SerialStrDescriptor,
  USBD_HID_ConfigStrDescriptor,
  USBD_HID_InterfaceStrDescriptor,
};

__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,
  USB_DESC_TYPE_DEVICE,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  USB_MAX_EP0_SIZE,
  LOBYTE(USBD_VID),
  HIBYTE(USBD_VID),
  LOBYTE(USBD_PID),
  HIBYTE(USBD_PID),
  0x00,
  0x02,
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION,
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING),
};

__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t* USBD_HID_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

static uint8_t* USBD_HID_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

static uint8_t* USBD_HID_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  USBD_GetString((uint8_t*)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t* USBD_HID_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  USBD_GetString((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t* USBD_HID_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  *length = USB_SIZ_STRING_SERIAL;
  Get_SerialNum();
  return USBD_StringSerial;
}

static uint8_t* USBD_HID_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  USBD_GetString((uint8_t*)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t* USBD_HID_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length)
{
  (void)speed;
  USBD_GetString((uint8_t*)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static void Get_SerialNum(void)
{
  uint32_t deviceserial0 = *(uint32_t*)DEVICE_ID1;
  uint32_t deviceserial1 = *(uint32_t*)DEVICE_ID2;
  uint32_t deviceserial2 = *(uint32_t*)DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0U) {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

static void IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len)
{
  for (uint8_t idx = 0; idx < len; idx++) {
    uint8_t digit = (uint8_t)(value >> 28);
    pbuf[2U * idx] = (digit < 0x0AU) ? (uint8_t)(digit + '0') : (uint8_t)(digit + 'A' - 10U);
    value <<= 4;
    pbuf[(2U * idx) + 1U] = 0U;
  }
}
