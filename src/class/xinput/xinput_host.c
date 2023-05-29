/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if (CFG_TUH_ENABLED && CFG_TUH_XINPUT)

#include "host/usbh.h"
#include "host/usbh_classdriver.h"

#include "xinput_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

#define XINPUT_DESC_TYPE_XINPUT 0x21

/// USB XINPUT Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength;         /**< Numeric expression that is the total size of the XINPUT descriptor */
  uint8_t  bDescriptorType; /**< Constant name specifying type of XINPUT descriptor. */
} tusb_xinput_descriptor_xinput_t;

typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  uint16_t epin_size;
  uint16_t epout_size;

  uint8_t epin_buf[CFG_TUH_XINPUT_EPIN_BUFSIZE];
  uint8_t epout_buf[CFG_TUH_XINPUT_EPOUT_BUFSIZE];
} xinputh_interface_t;

typedef struct
{
  uint8_t inst_count;
  xinputh_interface_t instances[CFG_TUH_XINPUT];
} xinputh_device_t;

CFG_TUSB_MEM_SECTION
static xinputh_device_t _xinputh_dev[CFG_TUH_DEVICE_MAX];

//------------- Internal prototypes -------------//

// Get XINPUT device & interface
TU_ATTR_ALWAYS_INLINE static inline xinputh_device_t* get_dev(uint8_t dev_addr);
TU_ATTR_ALWAYS_INLINE static inline xinputh_interface_t* get_instance(uint8_t dev_addr, uint8_t instance);
static uint8_t get_instance_id_by_itfnum(uint8_t dev_addr, uint8_t itf);
static uint8_t get_instance_id_by_epaddr(uint8_t dev_addr, uint8_t ep_addr);

//--------------------------------------------------------------------+
// Interface API
//--------------------------------------------------------------------+

uint8_t tuh_xinput_instance_count(uint8_t dev_addr)
{
  return get_dev(dev_addr)->inst_count;
}

bool tuh_xinput_mounted(uint8_t dev_addr, uint8_t instance)
{
  xinputh_interface_t* xinput_itf = get_instance(dev_addr, instance);
  return (xinput_itf->ep_in != 0) || (xinput_itf->ep_out != 0);
}

//--------------------------------------------------------------------+
// Interrupt Endpoint API
//--------------------------------------------------------------------+

bool tuh_xinput_receive_report(uint8_t dev_addr, uint8_t instance)
{
  xinputh_interface_t* xinput_itf = get_instance(dev_addr, instance);

  // claim endpoint
  TU_VERIFY( usbh_edpt_claim(dev_addr, xinput_itf->ep_in) );

  if ( !usbh_edpt_xfer(dev_addr, xinput_itf->ep_in, xinput_itf->epin_buf, xinput_itf->epin_size) )
  {
    usbh_edpt_release(dev_addr, xinput_itf->ep_in);
    return false;
  }

  return true;
}

//--------------------------------------------------------------------+
// USBH API
//--------------------------------------------------------------------+
void xinputh_init(void)
{
  tu_memclr(_xinputh_dev, sizeof(_xinputh_dev));
}

bool xinputh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) result;

  uint8_t const dir = tu_edpt_dir(ep_addr);
  uint8_t const instance = get_instance_id_by_epaddr(dev_addr, ep_addr);
  xinputh_interface_t* xinput_itf = get_instance(dev_addr, instance);

  if ( dir == TUSB_DIR_IN )
  {
    TU_LOG2("  Get Report callback (%u, %u)\r\n", dev_addr, instance);
    TU_LOG3_MEM(xinput_itf->epin_buf, xferred_bytes, 2);
    tuh_xinput_report_received_cb(dev_addr, instance, xinput_itf->epin_buf, (uint16_t) xferred_bytes);
  }else
  {
    if (tuh_xinput_report_sent_cb) tuh_xinput_report_sent_cb(dev_addr, instance, xinput_itf->epout_buf, (uint16_t) xferred_bytes);
  }

  return true;
}

void xinputh_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );

  xinputh_device_t* xinput_dev = get_dev(dev_addr);

  if (tuh_xinput_umount_cb)
  {
    for (uint8_t inst = 0; inst < xinput_dev->inst_count; inst++ ) tuh_xinput_umount_cb(dev_addr, inst);
  }

  tu_memclr(xinput_dev, sizeof(xinputh_device_t));
}

//--------------------------------------------------------------------+
// Enumeration
//--------------------------------------------------------------------+

bool xinputh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  (void) rhport;
  (void) max_len;

  TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_itf->bInterfaceClass);
  TU_VERIFY(93 == desc_itf->bInterfaceSubClass);
  TU_VERIFY(1 == desc_itf->bInterfaceProtocol);

  TU_LOG2("[%u] XINPUT opening Interface %u\r\n", dev_addr, desc_itf->bInterfaceNumber);

  // len = interface + xinput + n*endpoints
  uint16_t const drv_len = (uint16_t) (sizeof(tusb_desc_interface_t) + sizeof(tusb_xinput_descriptor_xinput_t) +
                                       desc_itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
  TU_ASSERT(max_len >= drv_len);

  uint8_t const *p_desc = (uint8_t const *) desc_itf;

  //------------- XINPUT descriptor -------------//
  p_desc = tu_desc_next(p_desc);
  tusb_xinput_descriptor_xinput_t const *desc_xinput = (tusb_xinput_descriptor_xinput_t const *) p_desc;
  TU_ASSERT(XINPUT_DESC_TYPE_XINPUT == desc_xinput->bDescriptorType);

  // not enough interface, try to increase CFG_TUH_XINPUT
  // TODO multiple devices
  xinputh_device_t* xinput_dev = get_dev(dev_addr);
  TU_ASSERT(xinput_dev->inst_count < CFG_TUH_XINPUT, 0);

  xinputh_interface_t* xinput_itf = get_instance(dev_addr, xinput_dev->inst_count);  

  //------------- Endpoint Descriptors -------------//
  p_desc = tu_desc_next(p_desc);
  tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;

  for(int i = 0; i < desc_itf->bNumEndpoints; i++)
  {
    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType);
    TU_ASSERT( tuh_edpt_open(dev_addr, desc_ep) );

    if(tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN)
    {
      xinput_itf->ep_in     = desc_ep->bEndpointAddress;
      xinput_itf->epin_size = tu_edpt_packet_size(desc_ep);
    }
    else
    {
      xinput_itf->ep_out     = desc_ep->bEndpointAddress;
      xinput_itf->epout_size = tu_edpt_packet_size(desc_ep);
    }

    p_desc = tu_desc_next(p_desc);
    desc_ep = (tusb_desc_endpoint_t const *) p_desc;
  }

  xinput_dev->inst_count++;

  xinput_itf->itf_num   = desc_itf->bInterfaceNumber;

  return true;
}

//--------------------------------------------------------------------+
// Set Configure
//--------------------------------------------------------------------+

enum {
  CONFIG_COMPLETE
};

static void config_driver_mount_complete(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len);
static void process_set_config(tuh_xfer_t* xfer);

bool xinputh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  tusb_control_request_t request;
  request.wIndex = tu_htole16((uint16_t) itf_num);

  tuh_xfer_t xfer;
  xfer.daddr     = dev_addr;
  xfer.result    = XFER_RESULT_SUCCESS;
  xfer.setup     = &request;
  xfer.user_data = CONFIG_COMPLETE;

  // fake request to kick-off the set config process
  process_set_config(&xfer);

  return true;
}

static void process_set_config(tuh_xfer_t* xfer)
{
  uintptr_t const state = xfer->user_data;
  uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
  uint8_t const daddr   = xfer->daddr;

  uint8_t const instance    = get_instance_id_by_itfnum(daddr, itf_num);

  switch(state)
  {
    case CONFIG_COMPLETE:
    {
      uint8_t const* desc_report = usbh_get_enum_buf();
      uint16_t const desc_len    = tu_le16toh(xfer->setup->wLength);

      config_driver_mount_complete(daddr, instance, desc_report, desc_len);
    }
    break;

    default: break;
  }
}

static void config_driver_mount_complete(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  xinputh_interface_t* xinput_itf = get_instance(dev_addr, instance);

  // enumeration is complete
  tuh_xinput_mount_cb(dev_addr, instance, desc_report, desc_len);

  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(dev_addr, xinput_itf->itf_num);
}

//--------------------------------------------------------------------+
// Helper
//--------------------------------------------------------------------+

// Get Device by address
TU_ATTR_ALWAYS_INLINE static inline xinputh_device_t* get_dev(uint8_t dev_addr)
{
  return &_xinputh_dev[dev_addr-1];
}

// Get Interface by instance number
TU_ATTR_ALWAYS_INLINE static inline xinputh_interface_t* get_instance(uint8_t dev_addr, uint8_t instance)
{
  return &_xinputh_dev[dev_addr-1].instances[instance];
}

// Get instance ID by interface number
static uint8_t get_instance_id_by_itfnum(uint8_t dev_addr, uint8_t itf)
{
  for ( uint8_t inst = 0; inst < CFG_TUH_XINPUT; inst++ )
  {
    xinputh_interface_t *xinput = get_instance(dev_addr, inst);

    if ( (xinput->itf_num == itf) && (xinput->ep_in || xinput->ep_out) ) return inst;
  }

  return 0xff;
}

// Get instance ID by endpoint address
static uint8_t get_instance_id_by_epaddr(uint8_t dev_addr, uint8_t ep_addr)
{
  for ( uint8_t inst = 0; inst < CFG_TUH_XINPUT; inst++ )
  {
    xinputh_interface_t *xinput = get_instance(dev_addr, inst);

    if ( (ep_addr == xinput->ep_in) || ( ep_addr == xinput->ep_out) ) return inst;
  }

  return 0xff;
}

#endif
