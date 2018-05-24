/*
             LUFA Library
     Copyright (C) Dean Camera, 2018.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2018  Dean Camera (dean [at] fourwalledcubicle [dot] com)

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
 *  Main source file for the CDC class bootloader. This file contains the complete bootloader logic.
 */

#define  INCLUDE_FROM_BOOTLOADERCDC_C
#include "BootloaderCDC.h"
#include "stk500.h"
#include "write.h"

/** Contains the current baud rate and other settings of the first virtual serial port. This must be retained as some
 *  operating systems will not open the port unless the settings can be set successfully.
 */
static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
                                           .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                           .ParityType  = CDC_PARITY_None,
                                           .DataBits    = 8                            };


/** Flag to indicate if the bootloader should be running, or should exit and allow the application code to run
 *  via a watchdog reset. When cleared the bootloader will exit, starting the watchdog and entering an infinite
 *  loop until the AVR restarts and the application runs.
 */
static bool RunBootloader = true;

/** Magic lock for forced application start. If the HWBE fuse is programmed and BOOTRST is unprogrammed, the bootloader
 *  will start if the /HWB line of the AVR is held low and the system is reset. However, if the /HWB line is still held
 *  low when the application attempts to start via a watchdog reset, the bootloader will re-start. If set to the value
 *  \ref MAGIC_BOOT_KEY the special init function \ref Application_Jump_Check() will force the application to start.
 */
uint16_t MagicBootKey ATTR_NO_INIT;


/** Special startup routine to check if the bootloader was started via a watchdog reset, and if the magic application
 *  start key has been loaded into \ref MagicBootKey. If the bootloader started via the watchdog and the key is valid,
 *  this will force the user application to start via a software jump.
 */
void Application_Jump_Check(void)
{
	bool JumpToApplication = false;

		/* Check if the device's BOOTRST fuse is set */
		if (!(BootloaderAPI_ReadFuse(GET_HIGH_FUSE_BITS) & ~FUSE_BOOTRST))
		{
			/* If the reset source was not an external reset or the key is correct, clear it and jump to the application */
			if (!(MCUSR & (1 << EXTRF)) || (MagicBootKey == MAGIC_BOOT_KEY))
			  JumpToApplication = true;

			/* Clear reset source */
			MCUSR &= ~(1 << EXTRF);
		}
		else
		{
			/* If the reset source was the bootloader and the key is correct, clear it and jump to the application;
			 * this can happen in the HWBE fuse is set, and the HBE pin is low during the watchdog reset */
			if ((MCUSR & (1 << WDRF)) && (MagicBootKey == MAGIC_BOOT_KEY))
				JumpToApplication = true;

			/* Clear reset source */
			MCUSR &= ~(1 << WDRF);
		}

	/* Don't run the user application if the reset vector is blank (no app loaded) */
	bool ApplicationValid = (pgm_read_word_near(0) != 0xFFFF);

	/* If a request has been made to jump to the user application, honor it */
	if (JumpToApplication && ApplicationValid)
	{
		/* Turn off the watchdog */
		MCUSR &= ~(1 << WDRF);
		wdt_disable();

		/* Clear the boot key and jump to the user application */
		MagicBootKey = 0;

		// cppcheck-suppress constStatement
		((void (*)(void))0x0000)();
	}
}

/** Main program entry point. This routine configures the hardware required by the bootloader, then continuously
 *  runs the bootloader processing routine until instructed to soft-exit, or hard-reset via the watchdog to start
 *  the loaded application code.
 */
int main(void)
{
	/* Setup hardware required for the bootloader */
	SetupHardware();

	/* Enable global interrupts so that the USB stack can function */
	GlobalInterruptEnable();

	while (RunBootloader)
	{
		CDC_Task();
		USB_USBTask();
	}

	/* Wait a short time to end all USB transactions and then disconnect */
	_delay_us(1000);

	/* Disconnect from the host - USB interface will be reset later along with the AVR */
	USB_Detach();

	/* Unlock the forced application start mode of the bootloader if it is restarted */
	MagicBootKey = MAGIC_BOOT_KEY;

	/* Enable the watchdog and force a timeout to reset the AVR */
	wdt_enable(WDTO_250MS);

	for (;;);
}

/** Configures all hardware required for the bootloader. */
static void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Relocate the interrupt vector table to the bootloader section */
	MCUCR = (1 << IVCE);
	MCUCR = (1 << IVSEL);

	/* Initialize the USB*/
	USB_Init();
}


/** Event handler for the USB_ConfigurationChanged event. This configures the device's endpoints ready
 *  to relay data to and from the attached USB host.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	/* Setup CDC Notification, Rx and Tx Endpoints */
	Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPADDR, EP_TYPE_INTERRUPT,
	                           CDC_NOTIFICATION_EPSIZE, 1);

	Endpoint_ConfigureEndpoint(CDC_TX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);

	Endpoint_ConfigureEndpoint(CDC_RX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Ignore any requests that aren't directed to the CDC interface */
	if ((USB_ControlRequest.bmRequestType & (CONTROL_REQTYPE_TYPE | CONTROL_REQTYPE_RECIPIENT)) !=
	    (REQTYPE_CLASS | REQREC_INTERFACE))
	{
		return;
	}

	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearOUT();
			}

			break;
		case CDC_REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearIN();
			}

			break;
        case CDC_REQ_SetControlLineState:
	        if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
	        {
	            Endpoint_ClearSETUP();
	            Endpoint_ClearStatusStage();
	        }

	        break;
	}
}


/** Retrieves the next byte from the host in the CDC data OUT endpoint, and clears the endpoint bank if needed
 *  to allow reception of the next data packet from the host.
 *
 *  \return Next received byte from the host in the CDC data OUT endpoint
 */
static uint8_t FetchNextCommandByte(void)
{
	/* Select the OUT endpoint so that the next data byte can be read */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* If OUT endpoint empty, clear it and wait for the next packet from the host */
	while (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearOUT();

		while (!(Endpoint_IsOUTReceived()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			  return 0;
		}
	}

	/* Fetch the next byte from the OUT endpoint */
	return Endpoint_Read_8();
}

/** Writes the next response byte to the CDC data IN endpoint, and sends the endpoint back if needed to free up the
 *  bank when full ready for the next byte in the packet to the host.
 *
 *  \param[in] Response  Next response byte to send to the host
 */
void WriteNextResponseByte(const uint8_t Response)
{
	/* Select the IN endpoint so that the next data byte can be written */
	Endpoint_SelectEndpoint(CDC_TX_EPADDR);

	/* If IN endpoint full, clear it and wait until ready for the next packet to the host */
	if (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearIN();

		while (!(Endpoint_IsINReady()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			  return;
		}
	}

	/* Write the next byte to the IN endpoint */
	Endpoint_Write_8(Response);
}



/** Task to read in AVR109 commands from the CDC data OUT endpoint, process them, perform the required actions
 *  and send the appropriate response back to the host.
 */
static void CDC_Task(void)
{
	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Check if endpoint has a command in it sent from the host */
	if (!(Endpoint_IsOUTReceived()))
	  return;

	if (ST_PROCESS != parseMsg(FetchNextCommandByte())) return;
	processCommand();
	replyMsg();

	/* Select the IN endpoint */
	Endpoint_SelectEndpoint(CDC_TX_EPADDR);

	/* Remember if the endpoint is completely full before clearing it */
	bool IsEndpointFull = !(Endpoint_IsReadWriteAllowed());

	/* Send the endpoint data to the host */
	Endpoint_ClearIN();

	/* If a full endpoint's worth of data was sent, we need to send an empty packet afterwards to signal end of transfer */
	if (IsEndpointFull)
	{
		while (!(Endpoint_IsINReady()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			  return;
		}

		Endpoint_ClearIN();
	}

	/* Wait until the data has been sent to the host */
	while (!(Endpoint_IsINReady()))
	{
		if (USB_DeviceState == DEVICE_STATE_Unattached)
		  return;
	}

	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Acknowledge the command from the host */
	Endpoint_ClearOUT();
}
