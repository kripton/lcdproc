/** \file server/drivers/hd44780-i2c-adafruit.c
 * \c i2c-adafruit connection type of \c hd44780 driver for Hitachi HD44780 based LCD displays.
 *
 * The LCD is operated in its 4 bit-mode to be connected to a single MCP23017
 * I2C port expander that is accessed by the server via the i2c device interface.
 * 
 * The backlight can be single or RGB.
 * Five buttons (Up, Down, Left, Right, Select) are also connected to the IO expander.
 * 
 * For more info about the boards, please see:
 * 	https://learn.adafruit.com/adafruit-16x2-character-lcd-plus-keypad-for-raspberry-pi
 * 	https://learn.adafruit.com/rgb-lcd-shield
 */

/* Copyright
 *		  2020 Jannis Achstetter <kripton@kripserver.net>
 *		  2005 Matthias Goebl <matthias.goebl@goebl.net>
 *		  2000, 1999, 1995 Benjamin Tse <blt@Comports.com>
 *		  2001 Joris Robijn <joris@robijn.net>
 *		  1999 Andrew McMeikan <andrewm@engineer.com>
 *		  1998 Richard Rognlie <rrognlie@gamerz.net>
 *		  1997 Matthias Prinke <m.prinke@trashcan.mcnet.de>
 *
 * The connections are:
 * MCP23017	  LCD
 * B4 (5)	  D4 (11)
 * B3 (4)	  D5 (12)
 * B2 (3)	  D6 (13)
 * B1 (2)	  D7 (14)
 * B7 (8)	  RS (4)
 * B6 (7)	  RW (5)
 * B5 (6)	  EN (6)
 * A6 (27)	  backlight (optional, single or RED)
 * A7 (28)	  backlight (optional, GREEN)
 * B0 (1)	  backlight (optional, BLUE)
 * A0 (21)	  Button "SELECT"
 * A1 (22)	  Button "RIGHT"
 * A2 (23)	  Button "DOWN"
 * A3 (24)	  Button "UP"
 * A4 (25)	  Button "LEFT"
 *
 *
 * Configuration:
 * device=/dev/i2c-0   # the device file of the i2c bus
 * port=0x20   # the i2c address of the i2c port expander
 *
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

/* Backlight states (unsigned char)
 *
 * Bit 0 = RED
 * Bit 1 = GREEN
 * Bit 2 = BLUE
 * 
 * 00	Backlight OFF
 * 01	Backlight RED
 * 02	Backlight GREEN
 * 03	Backlight RED + GREEN
 * 04	Backlight BLUE
 * 05	Backlight RED + BLUE
 * 06	Backlight GREEN + BLUE
 * 07	Backlight RED + GREEN + BLUE
 */

#include "hd44780-i2c-adafruit.h"
#include "hd44780-low.h"

#include "shared/report.h"
#ifdef HAVE_CONFIG_H
# include "config.h"
#endif
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "i2c.h"

// Generally, any function that accesses the LCD control lines needs to be
// implemented separately for each HW design. This is typically (but not
// restricted to):
// HD44780_senddata
// HD44780_readkeypad

void i2c_adafruit_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch);
void i2c_adafruit_HD44780_backlight(PrivateData *p, unsigned char state);
void i2c_adafruit_HD44780_close(PrivateData *p);

// Lower 8 bits = Port A
// Upper 8 bits = Port B
// Outputs
#define RS	0x8000
#define RW	0x4000
#define EN	0x2000
#define BLR	0x0040
#define BLG	0x0080
#define BLB	0x0100
#define D4	0x1000
#define D5	0x0800
#define D6	0x0400
#define D7	0x0200
// Buttons
#define BS	0x0001
#define BR	0x0002
#define BD	0x0004
#define BU	0x0008
#define BL	0x0010

// Inversions
#define BL_INVERT_R 0
#define BL_INVERT_G 0
#define BL_INVERT_B 0

#define I2C_ADDR_MASK 0x7f

// Set the outputs to drive LCD pins
// The lowest 16 bit of val are decisive:
// the lower 8 bits are port A,
// the upper 8 bits are port B
static void
i2c_out(PrivateData *p, unsigned int val)
{
	unsigned char data[3];
	static int no_more_errormsgs=0;

	// We merge the backlight right int val here
	// Otherwise we would unintentionally change it here
	if (p->backlight_bit & 0x01) {
		val = val | p->i2c_line_BLR;
	}
	if (p->backlight_bit & 0x02) {
		val = val | p->i2c_line_BLG;
	}
	if (p->backlight_bit & 0x04) {
		val = val | p->i2c_line_BLB;
	}

	data[0] = 0x12; // Register GPIOA
	data[1] = val & 0xFF;
	data[2] = (val >> 8) & 0xFF;

	if (i2c_write(p->i2c, data, 3) < 0) {
		p->hd44780_functions->drv_report(no_more_errormsgs?RPT_DEBUG:RPT_ERR, "HD44780: I2C-ADAFRUIT: i2c write data %04x failed: %s",
			val, strerror(errno));
		no_more_errormsgs=1;
	}
}


/**
 * Initialize the driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0       Success.
 * \retval -1      Error.
 */
int
hd_init_i2c(Driver *drvthis)
{
	PrivateData *p = (PrivateData*) drvthis->private_data;
	HD44780_functions *hd44780_functions = p->hd44780_functions;
	char device[256] = I2C_DEFAULT_DEVICE;

	p->i2c_backlight_invert_R = drvthis->config_get_bool(drvthis->name, "BacklightInvert_R", 0, BL_INVERT_R);
	p->i2c_backlight_invert_G = drvthis->config_get_bool(drvthis->name, "BacklightInvert_G", 0, BL_INVERT_G);
	p->i2c_backlight_invert_B = drvthis->config_get_bool(drvthis->name, "BacklightInvert_B", 0, BL_INVERT_B);
	p->i2c_line_RS = drvthis->config_get_int(drvthis->name, "i2c_line_RS", 0, RS);
	p->i2c_line_RW = drvthis->config_get_int(drvthis->name, "i2c_line_RW", 0, RW);
	p->i2c_line_EN = drvthis->config_get_int(drvthis->name, "i2c_line_EN", 0, EN);
	p->i2c_line_BLR = drvthis->config_get_int(drvthis->name, "i2c_line_BLR", 0, BLR);
	p->i2c_line_BLG = drvthis->config_get_int(drvthis->name, "i2c_line_BLG", 0, BLG);
	p->i2c_line_BLB = drvthis->config_get_int(drvthis->name, "i2c_line_BLB", 0, BLB);
	p->i2c_line_D4 = drvthis->config_get_int(drvthis->name, "i2c_line_D4", 0, D4);
	p->i2c_line_D5 = drvthis->config_get_int(drvthis->name, "i2c_line_D5", 0, D5);
	p->i2c_line_D6 = drvthis->config_get_int(drvthis->name, "i2c_line_D6", 0, D6);
	p->i2c_line_D7 = drvthis->config_get_int(drvthis->name, "i2c_line_D7", 0, D7);
	p->i2c_line_BS = drvthis->config_get_int(drvthis->name, "i2c_line_BS", 0, BS);
	p->i2c_line_BR = drvthis->config_get_int(drvthis->name, "i2c_line_BS", 0, BR);
	p->i2c_line_BD = drvthis->config_get_int(drvthis->name, "i2c_line_BS", 0, BD);
	p->i2c_line_BU = drvthis->config_get_int(drvthis->name, "i2c_line_BS", 0, BU);
	p->i2c_line_BL = drvthis->config_get_int(drvthis->name, "i2c_line_BS", 0, BL);

	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin RS mapped to 0x%04X", p->i2c_line_RS);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin RW mapped to 0x%04X", p->i2c_line_RW);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin EN mapped to 0x%04X", p->i2c_line_EN);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin BLR mapped to 0x%04X", p->i2c_line_BLR);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin BLG mapped to 0x%04X", p->i2c_line_BLG);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin BLB mapped to 0x%04X", p->i2c_line_BLB);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin D4 mapped to 0x%04X", p->i2c_line_D4);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin D5 mapped to 0x%04X", p->i2c_line_D5);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin D6 mapped to 0x%04X", p->i2c_line_D6);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Pin D7 mapped to 0x%04X", p->i2c_line_D7);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Button BS mapped to 0x%04X", p->i2c_line_BS);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Button BR mapped to 0x%04X", p->i2c_line_BR);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Button BD mapped to 0x%04X", p->i2c_line_BD);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Button BU mapped to 0x%04X", p->i2c_line_BU);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Button BL mapped to 0x%04X", p->i2c_line_BL);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Invert Backlight R %d", p->i2c_backlight_invert_R);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Invert Backlight G %d", p->i2c_backlight_invert_G);
	report(RPT_INFO, "HD44780: I2C-ADAFRUIT: Invert Backlight B %d", p->i2c_backlight_invert_B);

	p->backlight_bit = p->i2c_line_BLR | p->i2c_line_BLG | p->i2c_line_BLB;

	/* READ CONFIG FILE */

	/* Get serial device to use */
	strncpy(device, drvthis->config_get_string(drvthis->name, "Device", 0, I2C_DEFAULT_DEVICE), sizeof(device));
	device[sizeof(device)-1] = '\0';
	report(RPT_INFO,"HD44780: I2C-ADAFRUIT: Using device '%s' and address 0x%02X for a MCP23017",
		device, p->port & I2C_ADDR_MASK);

	p->i2c = i2c_open(device, p->port & I2C_ADDR_MASK);
	if (!p->i2c) {
		report(RPT_ERR, "HD44780: I2C-ADAFRUIT: connecting to device '%s' slave 0x%02X failed:", device, p->port & I2C_ADDR_MASK, strerror(errno));
		return(-1);
	}

	// What needs to be changed from defaults:
	// Enable Pull-Ups on button inputs
	// Switch output pins to output mode

	unsigned char data[3];

	// Pull-UPs on any line that has a button connected
	data[0] = 0x0c; // Register GPPUA
	data[1] = (p->i2c_line_BS & 0xFF) | // Port A
	          (p->i2c_line_BR & 0xFF) |
	          (p->i2c_line_BD & 0xFF) |
	          (p->i2c_line_BU & 0xFF) |
	          (p->i2c_line_BL & 0xFF);
	data[2] = ((p->i2c_line_BS >> 8) & 0xFF) | // Port B
	          ((p->i2c_line_BR >> 8) & 0xFF) |
	          ((p->i2c_line_BD >> 8) & 0xFF) |
	          ((p->i2c_line_BU >> 8) & 0xFF) |
	          ((p->i2c_line_BL >> 8) & 0xFF);

	if (i2c_write(p->i2c, data, 3) < 0) {
		report(RPT_ERR, "HD44780: I2C-ADAFRUIT: i2c set pull-ups failed: %s", strerror(errno));
	}

	// Output directions
	data[0] = 0x00; // Register IODIRA
	data[1] = 0xFF && // Port A
	          !(p->i2c_line_RS & 0xFF) &&
	          !(p->i2c_line_RW & 0xFF) &&
	          !(p->i2c_line_EN & 0xFF) &&
	          !(p->i2c_line_D4 & 0xFF) &&
	          !(p->i2c_line_D5 & 0xFF) &&
	          !(p->i2c_line_D6 & 0xFF) &&
	          !(p->i2c_line_D7 & 0xFF) &&
	          !(p->i2c_line_BLR & 0xFF) &&
	          !(p->i2c_line_BLG & 0xFF) &&
	          !(p->i2c_line_BLB & 0xFF);
	data[2] = 0xFF && // Port B
	          !((p->i2c_line_RS >> 8) & 0xFF) &&
	          !((p->i2c_line_RW >> 8) & 0xFF) &&
	          !((p->i2c_line_EN >> 8) & 0xFF) &&
	          !((p->i2c_line_D4 >> 8) & 0xFF) &&
	          !((p->i2c_line_D5 >> 8) & 0xFF) &&
	          !((p->i2c_line_D6 >> 8) & 0xFF) &&
	          !((p->i2c_line_D7 >> 8) & 0xFF) &&
	          !((p->i2c_line_BLR >> 8) & 0xFF) &&
	          !((p->i2c_line_BLG >> 8) & 0xFF) &&
	          !((p->i2c_line_BLB >> 8) & 0xFF);

	if (i2c_write(p->i2c, data, 3) < 0) {
		report(RPT_ERR, "HD44780: I2C-ADAFRUIT: i2c set pull-ups failed: %s", strerror(errno));
	}

	hd44780_functions->senddata = i2c_HD44780_senddata;
	hd44780_functions->backlight = i2c_HD44780_backlight;
	hd44780_functions->close = i2c_HD44780_close;

	// powerup the lcd now
	/* We'll now send 0x03 a couple of times,
	 * which is in fact (FUNCSET | IF_8BIT) >> 4
	 */
	i2c_out(p, p->i2c_line_D4 | p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);

	i2c_out(p, p->i2c_line_EN | p->i2c_line_D4 | p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_D4 | p->i2c_line_D5);
	hd44780_functions->uPause(p, 15000);

	i2c_out(p, p->i2c_line_EN | p->i2c_line_D4 | p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_D4 | p->i2c_line_D5);
	hd44780_functions->uPause(p, 5000);

	i2c_out(p, p->i2c_line_EN | p->i2c_line_D4 | p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_D4 | p->i2c_line_D5);
	hd44780_functions->uPause(p, 100);

	i2c_out(p, p->i2c_line_EN | p->i2c_line_D4 | p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_D4 | p->i2c_line_D5);
	hd44780_functions->uPause(p, 100);

	// now in 8-bit mode...  set 4-bit mode
	/*
	OLD   (FUNCSET | IF_4BIT) >> 4 0x02
	ALT   (FUNCSET | IF_4BIT)      0x20
	*/
	i2c_out(p, p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);

	i2c_out(p, p->i2c_line_EN | p->i2c_line_D5);
	if (p->delayBus)
		hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_D5);
	hd44780_functions->uPause(p, 100);

	// Set up two-line, small character (5x8) mode
	//hd44780_functions->senddata(p, 0, RS_INSTR, FUNCSET | IF_4BIT | TWOLINE | SMALLCHAR);
	//hd44780_functions->uPause(p, 40);

	common_init(p, IF_4BIT);

	return 0;
}

void
i2c_HD44780_close(PrivateData *p) {
	if (p->i2c >= 0)
		i2c_close(p->i2c);
}


/**
 * Send data or commands to the display.
 * \param p          Pointer to driver's private data structure.
 * \param displayID  ID of the display (or 0 for all) to send data to.
 * \param flags      Defines whether to end a command or data.
 * \param ch         The value to send.
 */
void
i2c_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch)
{
	unsigned char portControl = 0;
	unsigned char h=0;
	unsigned char l=0;
	if( ch & 0x80 ) h |= p->i2c_line_D7;
	if( ch & 0x40 ) h |= p->i2c_line_D6;
	if( ch & 0x20 ) h |= p->i2c_line_D5;
	if( ch & 0x10 ) h |= p->i2c_line_D4;
	if( ch & 0x08 ) l |= p->i2c_line_D7;
	if( ch & 0x04 ) l |= p->i2c_line_D6;
	if( ch & 0x02 ) l |= p->i2c_line_D5;
	if( ch & 0x01 ) l |= p->i2c_line_D4;
	if (flags == RS_INSTR)
		portControl = 0;
	else //if (flags == RS_DATA)
		portControl = p->i2c_line_RS;

	portControl |= p->backlight_bit;

	i2c_out(p, portControl | h);
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_EN | portControl | h);
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	i2c_out(p, portControl | h);

	i2c_out(p, portControl | l);
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	i2c_out(p, p->i2c_line_EN | portControl | l);
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	i2c_out(p, portControl | l);
}


/**
 * Turn display backlight on or off.
 * \param p      Pointer to driver's private data structure.
 * \param state  New backlight status.
 */
void i2c_HD44780_backlight(PrivateData *p, unsigned char state)
{
	// TODO: Handle inversion ? Or at i2c_out ?
	p->backlight_bit = state;

	// Should we i2c_out here or will it be automatically
	// be written the next time?

	/*
	if ( p->i2c_backlight_invert_R == 0 )
		p->backlight_bit = ((!have_backlight_pin(p)||state) ? 0 : p->i2c_line_BL);
	else // Inverted backlight - npn transistor
		p->backlight_bit = ((have_backlight_pin(p) && state) ? p->i2c_line_BL : 0);
	i2c_out(p, p->backlight_bit);
	*/
}
