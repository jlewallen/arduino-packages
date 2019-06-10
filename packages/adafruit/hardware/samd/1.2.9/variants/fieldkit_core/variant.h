/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_FIELDKIT_CORE_
#define _VARIANT_FIELDKIT_CORE_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		 (32768ul)

/** Master clock frequency */
#define VARIANT_MCK			   (120000000ul)

#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT                  (94u)
#define NUM_DIGITAL_PINS            (53u)
#define NUM_ANALOG_INPUTS           (16u)
#define NUM_ANALOG_OUTPUTS          (2u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

#define digitalPinToPort(P)         (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)      (1 << g_APinDescription[P].ulPin)
#define portOutputRegister(port)    (&(port->OUT.reg))
#define portInputRegister(port)     (&(port->IN.reg))
#define portModeRegister(port)      (&(port->DIR.reg))
#define digitalPinHasPWM(P)         (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

/*
 * Analog pins
 */
#define PIN_A0               (uint8_t)(-1)
#define PIN_A1               (uint8_t)(-1)

static const uint8_t A0  =   (uint8_t)(-1);
static const uint8_t A1  =   (uint8_t)(-1);

#define ADC_RESOLUTION		12

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

// PORTB15 38  Pad3 MISO
// PORTB12 18  Pad0 MOSI
// PORTB13 19  Pad1 SCK
// PORTB14 39  Pad2
#define PIN_SPI_MISO         (38u)
#define PIN_SPI_MOSI         (18u)
#define PIN_SPI_SCK          (19u)
#define PERIPH_SPI           sercom0
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3

// SPI_PAD_0_SCK_1 = 0,
// SPI_PAD_2_SCK_3,
// SPI_PAD_3_SCK_1,
// SPI_PAD_0_SCK_3

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 0

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (77ul)
#define PIN_USB_DM          (78ul)
#define PIN_USB_DP          (79ul)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

/*
 * QSPI Interfaces
 */
#define PIN_QSPI_SCK	(89u)
#define PIN_QSPI_CS		(90u)
#define PIN_QSPI_IO0	(91u)
#define PIN_QSPI_IO1	(92u)
#define PIN_QSPI_IO2	(93u)
#define PIN_QSPI_IO3	(94u)

/*
 * PCC
 */
#define PIN_PCC_DEN1    (26u)
#define PIN_PCC_DEN2    (27u)
#define PIN_PCC_CLK     (28u)
#define PIN_PCC_XCLK	(29u)
#define PIN_PCC_D0      (37u)
#define PIN_PCC_D1      (36u)
#define PIN_PCC_D2      (35u)
#define PIN_PCC_D3      (34u)
#define PIN_PCC_D4      (33u)
#define PIN_PCC_D5      (32u)
#define PIN_PCC_D6      (31u)
#define PIN_PCC_D7      (30u)
#define PIN_PCC_D8      (39u)
#define PIN_PCC_D9      (38u)
#define PIN_PCC_D10     (41u)
#define PIN_PCC_D11     (40u)
#define PIN_PCC_D12     (43u)
#define PIN_PCC_D13     (42u)

// TODO: Meaningful value for this?
#define VARIANT_QSPI_BAUD_DEFAULT 5000000

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern SERCOM sercom6;
extern SERCOM sercom7;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial

#endif /* _VARIANT_FIELDKIT_CORE_ */

