/************************************************************************************
 * configs/stm32f4discovery/src/stm32_usb.c
 *
 *   Copyright (C) 2012-2013, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_otgfs.h"
#include "stm32f4discovery.h"

#ifdef CONFIG_STM32_OTGFS

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning "CONFIG_STM32_OTGFS is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

#ifndef CONFIG_STM32F4DISCO_USBHOST_PRIO
#  define CONFIG_STM32F4DISCO_USBHOST_PRIO 100
#endif

#ifndef CONFIG_STM32F4DISCO_USBHOST_STACKSIZE
#  define CONFIG_STM32F4DISCO_USBHOST_STACKSIZE 1024
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uinfo("Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          (void)CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-related
 *   GPIO pins for the STM32F4Discovery board.
 *
 ************************************************************************************/

void stm32_usbinitialize(void)
{
  /* The OTG FS has an internal soft pull-up.  No GPIO configuration is required */
}

/***********************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ***********************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_usbhost_initialize(void)
{
  int pid;
#if defined(CONFIG_USBHOST_HUB)    || defined(CONFIG_USBHOST_MSC) || \
    defined(CONFIG_USBHOST_HIDKBD) || defined(CONFIG_USBHOST_HIDMOUSE) || \
    defined(CONFIG_USBHOST_XBOXCONTROLLER)
  int ret;
#endif

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub class support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      uerr("ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB mass storage class class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Initialize the HID keyboard class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID mouse class\n");
    }
#endif

#ifdef CONFIG_USBHOST_XBOXCONTROLLER
  /* Initialize the HID mouse class */

  ret = usbhost_xboxcontroller_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the XBox Controller class\n");
    }
#endif

  /* Then get an instance of the USB host interface */

  uinfo("Initialize USB host\n");
  g_usbconn = stm32_otgfshost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      uinfo("Start usbhost_waiter\n");

      pid = task_create("usbhost", CONFIG_STM32F4DISCO_USBHOST_PRIO,
                        CONFIG_STM32F4DISCO_USBHOST_STACKSIZE,
                        (main_t)usbhost_waiter, (FAR char * const *)NULL);
      return pid < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#endif

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

#ifdef CONFIG_USBDEV
void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_STM32_OTGFS */
