#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define USBD_MAX_NUM_INTERFACES 1U
#define USBD_MAX_NUM_CONFIGURATION 1U
#define USBD_MAX_STR_DESC_SIZ 0x100U
#define USBD_SUPPORT_USER_STRING_DESC 0U
#define USBD_SELF_POWERED 1U
#define USBD_DEBUG_LEVEL 0U

#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE 64U
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE 27U

#define MAX_STATIC_ALLOC_SIZE 128U

void* USBD_static_malloc(uint32_t size);
void USBD_static_free(void* p);

#define USBD_malloc USBD_static_malloc
#define USBD_free USBD_static_free

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define USBD_ErrLog(...)
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)
#else
#define USBD_DbgLog(...)
#endif

#endif /* __USBD_CONF_H */

