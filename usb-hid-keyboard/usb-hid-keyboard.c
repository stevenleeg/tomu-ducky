/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \addtogroup Examples
 *
 * This example implements a USB Human Interface Device (HID)
 * to demonstrate the use of the USB device stack.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <toboot.h>
#include "file.h"

TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN);

/* Systick interrupt frequency, Hz */
#define SYSTICK_FREQUENCY 100

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY 14000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0101    /* Program version */


// Declare injectkeys function
void injkeys(char *source, uint8_t mod);
void execute_frame();

bool g_usbd_is_connected = false;
bool once = true;
usbd_device *g_usbd_dev = 0;

typedef enum injectState {
	state_IDLE,
	state_START_INJECT,
	state_INJECTING,
	state_KEY_DOWN,
	state_KEY_UP,
	state_MOD_DOWN,
	state_MOD_KEY_DOWN,
	state_MOD_KEY_UP,
	state_MOD_UP,
	state_WAIT
} injectState_t;

static const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        0x09, 0x06, // USAGE (Keyboard)
        0xa1, 0x01, // COLLECTION (Application)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x01, // LOGICAL_MAXIMUM (1)
        0x75, 0x01, // REPORT_SIZE (1)
        0x95, 0x08, // REPORT_COUNT (8)
        0x81, 0x02, // INPUT (Data,Var,Abs) //1 byte

        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x08, // REPORT_SIZE (8)
        0x81, 0x03, // INPUT (Cnst,Var,Abs) //1 byte

        0x95, 0x06, // REPORT_COUNT (6)
        0x75, 0x08, // REPORT_SIZE (8)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x65, // LOGICAL_MAXIMUM (101)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
        0x81, 0x00, // INPUT (Data,Ary,Abs) //6 bytes

        0xc0, // END_COLLECTION
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 8,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 1, // 1=keyboard, 2=mouse
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Tomu",
	"HID keyboard Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

    /* Dirty way to know if we're connected */
    g_usbd_is_connected = true;

	return 1;
}

void hid_keydown(char key)
{
	static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    buf[2] = key;
    usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8);
}

void hid_keyup(char key)
{
    (void) key;
    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8);
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

    usbd_register_control_callback(
            dev,
            USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
            USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
            hid_control_request);
}

void usb_isr(void)
{
    usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
    while(1);
}

void sys_tick_handler(void)
{
    if (!g_usbd_is_connected || !once) {
        return;
    }
    for(int i = 0; i != 3500000; ++i) __asm__("nop");  // wait before keys injection

    // TODO: Add button catch before starting the script

    /*injkeys("this is an example of key injection 0123456789",0);*/
    /*injkeys(" . . ", 0); // Can't send two consecutive identical caracters ...*/
    /*injkeys("this is an example of key injection 0123456789",2);*/

    once = false;

    while (!file_eof()) {
        execute_frame();
    }
}

void execute_frame()
{
	static injectState_t state = state_START_INJECT;
	static uint8_t wait = 0;
	static uint16_t injectToken = 0x0000;
    int i = 0;
	
	switch(state) {
		case state_IDLE:
            // TODO: Blink the LED a couple of times
			break;		
			
		case state_START_INJECT:
			file_open();
			state = state_INJECTING;
			break;
			
		case state_INJECTING:				
			if (file_eof()) {
				file_close();	
				state = state_IDLE;
				break;
			}
			
			injectToken = (file_getc() | (file_getc() << 8));			
						
			if ((injectToken & 0xff) == 0x00) {				
				wait = injectToken>>8;
				state = state_WAIT;
			} else if ((injectToken >> 8) == 0x00) {
				state = state_KEY_DOWN;
			} else {
				state = state_MOD_DOWN;					
			}					
			break;
			
		case state_KEY_DOWN:
			hid_keydown(injectToken & 0xff);

            for(i = 0; i != 150; ++i) __asm__("nop");
            hid_keyup(injectToken & 0xff);

            for(i = 0; i != 1500000; ++i)
                __asm__("nop");
			state = state_INJECTING;
			break;

		case state_KEY_UP:
			hid_keyup(injectToken & 0xff);
			state = state_INJECTING;

            // Spin for a few cycles
            for(i = 0; i != 1500000; ++i)
                __asm__("nop");
			break;			
			
		/*case state_MOD_DOWN:*/
			/*udi_hid_kbd_modifier_down(injectToken>>8);*/
			/*state = state_MOD_KEY_DOWN;*/
			/*break;*/

		/*case state_MOD_KEY_DOWN:*/
			/*udi_hid_kbd_down(injectToken&0xff);*/
			/*state = state_MOD_KEY_UP;*/
			/*break;*/

		/*case state_MOD_KEY_UP:*/
			/*udi_hid_kbd_up(injectToken&0xff);*/
			/*state = state_MOD_UP;		*/
			/*break;*/
			
		/*case state_MOD_UP:*/
			/*udi_hid_kbd_modifier_up(injectToken>>8);*/
			/*state = state_INJECTING;*/
			/*break;	*/
			
		case state_WAIT:
			if( --wait == 0 ) {
				state = state_INJECTING;
			}
			break;
			
		default:
			state = state_IDLE;
	}
}

// HID Usage Tables
// https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf

void injkeys(char *source, uint8_t mod)
{
	static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // key pressed
	static uint8_t buf2[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Key released

    if (!g_usbd_is_connected) {
        return;
    }

	int i;
    buf[0] = mod; // Key modifier, 2=LeftShift

    int lstr = strlen(source);
    // change ascii to keyboard map
    for (int j = 0; j < lstr; j++) {
        if(source[j] > 48 && source[j] < 58) // numbers 1-9
            buf[2] = source[j] -19;
        else if (source[j] == 48) // number 0
            buf[2] = 39;
        else if (source[j] == 32) // space bar
            buf[2] = 44;
        else if(source[j] == 46) // .
            buf[2] = 55;
        else
            buf[2] = source[j] - 93; // lowercase letters

        usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8);
        for(i = 0; i != 150; ++i) __asm__("nop");
        usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8);
        for(i = 0; i != 150000; ++i) //Wait a little
            __asm__("nop");

    }
    usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8); // Be sure key is released
    for(i = 0; i != 150000; ++i) //Wait a little
        __asm__("nop");
}

int main(void)
{
    int i;

    /* Make sure the vector table is relocated correctly (after the Tomu bootloader) */
    SCB_VTOR = 0x4000;

    /* Disable the watchdog that the bootloader started. */
    WDOG_CTRL = 0;

    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set up both LEDs as outputs */
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    /* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, hid_set_config);

    /* Enable USB IRQs */
    nvic_set_priority(NVIC_USB_IRQ, 0x40);
	nvic_enable_irq(NVIC_USB_IRQ);

    /* Configure the system tick, at lower priority than USB IRQ */
    systick_set_frequency(SYSTICK_FREQUENCY, AHB_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0x10);

    gpio_set(LED_RED_PORT, LED_RED_PIN);
    while(1) {

    gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
    //    gpio_toggle(LED_RED_PORT, LED_RED_PIN);
        for(i = 0; i != 500000; ++i)
			__asm__("nop");
    }
}

