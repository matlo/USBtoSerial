/*
             LUFA Library
     Copyright (C) Dean Camera, 2016.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Mathieu Laurendeau (mat.lau@laposte.net)
    - adaptation for low latency requirements

  Copyright 2016  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "USBtoSerial.h"

// Bootloader related fields
// Old Caterian bootloader places the MAGIC key into unsafe RAM locations (it can be rewritten
// by the running sketch before to actual reboot).
// Newer bootloaders, recognizable by the LUFA "signature" at the end of the flash, can handle both
// the usafe and the safe location. Check once if the bootloader in new, then set the
// _updatedLUFAbootloader variable to true/false and place the magic key consequently
#define MAGIC_KEY 0x7777
#define MAGIC_KEY_POS 0x0800
#define NEW_LUFA_SIGNATURE 0xDCFB

static bool _updatedLUFAbootloader = false;

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[1024];

static volatile uint8_t USART_Timeout = 0;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		while (1)
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
			if (ReceivedByte < 0)
			{
				break;
			}
			while (!Serial_IsSendReady()) ;
			Serial_SendByte(ReceivedByte);
		}

		uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
		if ((BufferCount && TCNT1 >= USART_Timeout) // there is something to send and reception timeout fired
			|| BufferCount > sizeof(USARTtoUSB_Buffer_Data) / 2) // also send when buffer is half-full
		{
			Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

			if (Endpoint_IsINReady())
			{
				/* Read bytes from the USART receive buffer into the USB IN endpoint */
				while (BufferCount--)
				{
					/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
					if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
											RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
					{
						break;
					}
					RingBuffer_Remove(&USARTtoUSB_Buffer);
				}
			}
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at FCPU /64

#if MAGIC_KEY_POS != (RAMEND-1)
	if (pgm_read_word(FLASHEND - 1) == NEW_LUFA_SIGNATURE) {
		_updatedLUFAbootloader = true;
	}
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (USB_DeviceState != DEVICE_STATE_Configured || RingBuffer_IsFull(&USARTtoUSB_Buffer))
	{
		return;
	}

	if (RingBuffer_GetCount(&USARTtoUSB_Buffer))
	{
		// the timeout is twice the byte reception interval
		USART_Timeout = TCNT1 << 2;
	}

	TCNT1 = 0;

	RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/* Borrowed from the Arduino source code:
 * https://github.com/arduino/Arduino/blob/2bfe164b9a5835e8cb6e194b928538a9093be333/hardware/arduino/avr/cores/arduino/CDC.cpp#L97
 */
static void handleResetToBootloader(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	// auto-reset into the bootloader is triggered when the port, already
	// open at 1200 bps, is closed.  this is the signal to start the watchdog
	// with a relatively long period so it can finish housekeeping tasks
	// like servicing endpoints before the sketch ends

	uint16_t magic_key_pos = MAGIC_KEY_POS;

// If we don't use the new RAMEND directly, check manually if we have a newer bootloader.
// This is used to keep compatible with the old leonardo bootloaders.
// You are still able to set the magic key position manually to RAMEND-1 to save a few bytes for this check.
#if MAGIC_KEY_POS != (RAMEND-1)
	// For future boards save the key in the inproblematic RAMEND
	// Which is reserved for the main() return value (which will never return)
	if (_updatedLUFAbootloader) {
		// horray, we got a new bootloader!
		magic_key_pos = (RAMEND-1);
	}
#endif

	// We check DTR state to determine if host port is open (bit 0 of lineState).
	if (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 1200 && (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) == 0)
	{
#if MAGIC_KEY_POS != (RAMEND-1)
		// Backup ram value if its not a newer bootloader.
		// This should avoid memory corruption at least a bit, not fully
		if (magic_key_pos != (RAMEND-1)) {
			*(uint16_t *)(RAMEND-1) = *(uint16_t *)magic_key_pos;
		}
#endif
		// Store boot key
		*(uint16_t *)magic_key_pos = MAGIC_KEY;
		wdt_enable(WDTO_120MS);
	}
	else
	{
		// Most OSs do some intermediate steps when configuring ports and DTR can
		// twiggle more than once before stabilizing.
		// To avoid spurious resets we set the watchdog to 250ms and eventually
		// cancel if DTR goes back high.

		wdt_disable();
		wdt_reset();
#if MAGIC_KEY_POS != (RAMEND-1)
		// Restore backed up (old bootloader) magic key data
		if (magic_key_pos != (RAMEND-1)) {
			*(uint16_t *)magic_key_pos = *(uint16_t *)(RAMEND-1);
		} else
#endif
		{
			// Clean up RAMEND key
			*(uint16_t *)magic_key_pos = 0x0000;
		}
	}
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	handleResetToBootloader(CDCInterfaceInfo);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	handleResetToBootloader(CDCInterfaceInfo);

	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Set the new baud rate before configuring the USART */
	UBRR1  = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	/* Reconfigure the USART in double speed mode for a wider baud rate range at the expense of accuracy */
	UCSR1C = ConfigMask;
	UCSR1A = (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);
}

