#include "usbd_customhid_if.h"

#include "comm_transport.h"

static int8_t CustomHID_Init(void);
static int8_t CustomHID_DeInit(void);
static int8_t CustomHID_OutReport(uint8_t* report, uint16_t len);

__ALIGN_BEGIN static uint8_t CustomHID_ReportDesc[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END = {
  0x06, 0x00, 0xFF, /* Usage Page (Vendor Defined 0xFF00) */
  0x09, 0x01,       /* Usage (0x01) */
  0xA1, 0x01,       /* Collection (Application) */
  0x15, 0x00,       /* Logical Minimum (0) */
  0x26, 0xFF, 0x00, /* Logical Maximum (255) */
  0x75, 0x08,       /* Report Size (8) */
  0x95, 0x40,       /* Report Count (64) */
  0x09, 0x01,       /* Usage (Input Report) */
  0x81, 0x02,       /* Input (Data,Var,Abs) */
  0x95, 0x40,       /* Report Count (64) */
  0x09, 0x02,       /* Usage (Output Report) */
  0x91, 0x02,       /* Output (Data,Var,Abs) */
  0xC0,             /* End Collection */
};

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops = {
  CustomHID_ReportDesc,
  CustomHID_Init,
  CustomHID_DeInit,
  CustomHID_OutReport,
};

static int8_t CustomHID_Init(void)
{
  return 0;
}

static int8_t CustomHID_DeInit(void)
{
  return 0;
}

static int8_t CustomHID_OutReport(uint8_t* report, uint16_t len)
{
  comm_transport_receive_usb_report(report, len);
  return 0;
}

