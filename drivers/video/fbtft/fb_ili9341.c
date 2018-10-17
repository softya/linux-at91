/*
 * FB driver for the ILI9341 LCD display controller
 *
 * This display uses 9-bit SPI: Data/Command bit + 8 data bits
 * For platforms that doesn't support 9-bit, the driver is capable
 * of emulating this using 8-bit transfer.
 * This is done by transfering eight 9-bit words in 9 bytes.
 *
 * Copyright (C) 2013 Christian Vogelgsang
 * Based on adafruit22fb.c by Noralf Tronnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME		"fb_ili9341"
#define WIDTH		240
#define HEIGHT		320
#define TXBUFLEN	(4 * PAGE_SIZE)
#define DEFAULT_GAMMA	"1F 1A 18 0A 0F 06 45 87 32 0A 07 02 07 05 00\n" \
			"00 25 27 05 10 09 3A 78 4D 05 18 0D 38 3A 1F"

static void ili9341_set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par,
		"%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

	/* Column address set */
	write_reg(par, 0x2A,
		(xs >> 8) & 0xFF, xs & 0xFF, (xe >> 8) & 0xFF, xe & 0xFF);

	/* Row adress set */
	write_reg(par, 0x2B,
		(ys >> 8) & 0xFF, ys & 0xFF, (ye >> 8) & 0xFF, ye & 0xFF);

	/* Memory write */
	write_reg(par, 0x2C);
}

#define MEM_Y   (7) /* MY row address order */
#define MEM_X   (6) /* MX column address order */
#define MEM_V   (5) /* MV row / column exchange */
#define MEM_L   (4) /* ML vertical refresh order */
#define MEM_H   (2) /* MH horizontal refresh order */
#define MEM_BGR (3) /* RGB-BGR Order */
static int set_var(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	switch (par->info->var.rotate) {
	case 0:
		write_reg(par, 0x36, (1 << MEM_X) | (par->bgr << MEM_BGR));
		break;
	case 270:
		write_reg(par, 0x36,
			(1<<MEM_V) | (1 << MEM_L) | (par->bgr << MEM_BGR));
		break;
	case 180:
		write_reg(par, 0x36, (1 << MEM_Y) | (par->bgr << MEM_BGR));
		break;
	case 90:
		write_reg(par, 0x36, (1 << MEM_Y) | (1 << MEM_X) |
				     (1 << MEM_V) | (par->bgr << MEM_BGR));
		break;
	}

	return 0;
}

/********************************************************
*
* Function:
* 	ili9341_controller_init
*
* Purpose:
*   Initializes the display controller
*********************************************************/
static int ili9341_controller_init(struct fbtft_par *par)
{

    fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);


	//Software Reset
	write_reg(par, 0x01);
	//Supposed to wait at least 120ms, wait 250 to be safe
	mdelay(250);
	write_reg(par, 0x28); // display off

	write_reg(par,0xEF
			,0x03
			,0x80
			,0x02);
#if 0
	write_reg(par, 0xCF, 0x00, 0xC1, 0x30);
	write_reg(par, 0xED, 0x64, 0x03, 0x12, 0x81);
	write_reg(par, 0xE8, 0x85, 0x00, 0x78);
	write_reg(par, 0xCB, 0x39, 0X2C, 0x00, 0x34, 0x02);

	write_reg(par, 0xF6, 0x01, 0x00, 0x04); // RIM

	write_reg(par, 0xF7, 0x20);
	write_reg(par, 0xEA, 0x00, 0x00);
	/* ------------power control-------------------------------- */
	write_reg(par, 0xC0, 0x23);
	write_reg(par, 0xC1, 0x10);
	/* ------------VCOM --------- */
	write_reg(par, 0xC5, 0x3e, 0x28);
	write_reg(par, 0xC7, 0x86);
#else
	/*Configure ILI9341*/
	write_reg(par, 0xCA
	              ,0xC3
	              ,0x08
	              ,0x50);

	write_reg(par, 0xCF
	              ,0x00
	              ,0xC1
	              ,0x30);

	write_reg(par, 0xED
	              ,0x64
	              ,0x03
	              ,0x12
	              ,0x81);

	write_reg(par, 0xE8
	              ,0x85
	              ,0x00
	              ,0x78);

	write_reg(par, 0xCB
	              ,0x39
	              ,0x2C
	              ,0x00
	              ,0x34
	              ,0x02);

	write_reg(par, 0xF7
	              ,0x20);

	write_reg(par, 0xEA
	              ,0x00
	              ,0x00);

	write_reg(par, 0xB1
	              ,0x00
	              ,0x1B);

	write_reg(par, 0xB6
	              ,0x0A
	              ,0xA2);

	write_reg(par, 0xC0
	              ,0x10);

	write_reg(par, 0xC1
	              ,0x10);

	write_reg(par, 0xC5
	              ,0x45
	              ,0x15);

	write_reg(par, 0xC7
	              ,0x90);

#endif

	if(par->pdata->display.bpp == 18)
	    write_reg(par, 0x3A, 0x66); /* 18bit pixel */
	else
	    write_reg(par, 0x3A, 0x55); /* 16bit pixel */

	/* ------------frame rate----------------------------------- */
	write_reg(par, 0xB0, 0xC0); // RCM

	/* Frame Rate Control */
	/* Division ratio = fosc, Frame Rate = 79Hz */
	write_reg(par, 0xB1, 0x00, 0x18);
//	write_reg(par, 0xB6, 0x08, 0x82, 0x27);

	/* ------------Gamma---------------------------------------- */
//	write_reg(par, 0xF2, 0x00); /* Gamma Function Disable */

	write_reg(par, 0x36
	              ,0xc8);

	write_reg(par, 0xF2
	              ,0x00);

	write_reg(par, 0xB0
	              ,0xC0);

	write_reg(par, 0xB6
	              ,0x0A
	              ,0xA7
	              ,0x27
	              ,0x04);

	/* colomn address set */
	write_reg(par, 0x2A
	              ,0x00
	              ,0x00
	              ,0x00
	              ,0xEF);

	/* Page Address Set */
	write_reg(par, 0x2B
	              ,0x00
	              ,0x00
	              ,0x01
	              ,0x3F);

	write_reg(par, 0xF6
	              ,0x01
	              ,0x00
	              ,0x06);

	write_reg(par, 0x2C);
	mdelay(200);

	write_reg(par, 0x26
	              ,0x01);

	write_reg(par, 0xE0
	              ,0x0F
	              ,0x29
	              ,0x24
	              ,0x0C
	              ,0x0E
	              ,0x09
	              ,0x4E
	              ,0x78
	              ,0x3C
	              ,0x09
	              ,0x13
	              ,0x05
	              ,0x17
	              ,0x11
	              ,0x00);

	write_reg(par, 0xE1
	              ,0x00
	              ,0x16
	              ,0x1B
	              ,0x04
	              ,0x11
	              ,0x07
	              ,0x31
	              ,0x33
	              ,0x42
	              ,0x05
	              ,0x0C
	              ,0x0A
	              ,0x28
	              ,0x2F
	              ,0x0F);

	write_reg(par, 0x11);
	mdelay(200);
	write_reg(par, 0x29);

	/* GRAM start writing */
	write_reg(par, 0x2C);

	return 0;
}

static int init_display2(struct fbtft_par *par)
{
       fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

       par->fbtftops.reset(par);

       /* startup sequence for MI0283QT-9A */
       write_reg(par, 0x01); /* software reset */
       mdelay(250);
       write_reg(par, 0x28); /* display off */
       /* --------------------------------------------------------- */
       write_reg(par, 0xCF, 0x00, 0x83, 0x30);
       write_reg(par, 0xED, 0x64, 0x03, 0x12, 0x81);
       write_reg(par, 0xE8, 0x85, 0x01, 0x79);
       write_reg(par, 0xCB, 0x39, 0X2C, 0x00, 0x34, 0x02);
       write_reg(par, 0xF7, 0x20);
       write_reg(par, 0xEA, 0x00, 0x00);
       /* ------------power control-------------------------------- */
       write_reg(par, 0xC0, 0x26);
       write_reg(par, 0xC1, 0x11);
       /* ------------VCOM --------- */
       write_reg(par, 0xC5, 0x35, 0x3E);
       write_reg(par, 0xC7, 0xBE);
       /* ------------memory access control------------------------ */
       write_reg(par, 0x3A, 0x55); /* 16bit pixel */
       /* ------------frame rate----------------------------------- */
       write_reg(par, 0xB1, 0x00, 0x1B);
       /* ------------Gamma---------------------------------------- */
       /* write_reg(par, 0xF2, 0x08); */ /* Gamma Function Disable */
       write_reg(par, 0x26, 0x01);
       /* ------------display-------------------------------------- */
       write_reg(par, 0xB7, 0x07); /* entry mode set */
       write_reg(par, 0xB6, 0x0A, 0x82, 0x27, 0x00);
       write_reg(par, 0x11); /* sleep out */
       mdelay(250);
       write_reg(par, 0x29); /* display on */
       mdelay(20);

       return 0;
}

static int init_display(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	par->fbtftops.reset(par);

	/* startup sequence for MI0283QT-9A */
	write_reg(par, 0x01); /* software reset */
	mdelay(250);
	write_reg(par, 0x28); /* display off */
	/* --------------------------------------------------------- */
	write_reg(par,0xEF
			,0x03
			,0x80
			,0x02);

	write_reg(par, 0xCF, 0x00, 0xC1, 0x30);
	write_reg(par, 0xED, 0x64, 0x03, 0x12, 0x81);
	write_reg(par, 0xE8, 0x85, 0x00, 0x78);
	write_reg(par, 0xCB, 0x39, 0X2C, 0x00, 0x34, 0x02);

//	write_reg(par, 0xF6, 0x01, 0x00, 0x04); // RIM

	write_reg(par, 0xF7, 0x20);
	write_reg(par, 0xEA, 0x00, 0x00);
	/* ------------power control-------------------------------- */
	write_reg(par, 0xC0, 0x23);
	write_reg(par, 0xC1, 0x10);
	/* ------------VCOM --------- */
	write_reg(par, 0xC5, 0x3e, 0x28);
	write_reg(par, 0xC7, 0x86);
	/* ------------memory access control------------------------ */

	if(par->pdata->display.bpp == 18)
	    write_reg(par, 0x3A, 0x66); /* 18bit pixel */
	else
	    write_reg(par, 0x3A, 0x55); /* 16bit pixel */

	/* ------------frame rate----------------------------------- */
	write_reg(par, 0xB0, 0xC0); // RCM

	/* Frame Rate Control */
	/* Division ratio = fosc, Frame Rate = 79Hz */
	write_reg(par, 0xB1, 0x00, 0x18);
	write_reg(par, 0xB6, 0x08, 0x82, 0x27);

	/* ------------Gamma---------------------------------------- */
	write_reg(par, 0xF2, 0x00); /* Gamma Function Disable */
	write_reg(par, 0x26, 0x01);

	/* ------------display-------------------------------------- */
//	write_reg(par, 0xB7, 0x07); /* entry mode set */
	write_reg(par, 0x11); /* sleep out */
	mdelay(120);
	write_reg(par, 0x29); /* display on */
//	mdelay(20);

//	set_var(par);
//	/* Positive Gamma Correction */
//	write_reg(par, 0xE0
//			, 0x0F
//			, 0x31
//			, 0x2B
//			, 0x0C
//			, 0x0E
//			, 0x08
//			, 0x4E
//			, 0xF1
//			, 0x37
//			, 0x07
//			, 0x10
//			, 0x03
//			, 0x0E
//			, 0x09
//			, 0x00);
//
//	/* Negative Gamma Correction */
//	write_reg(par, 0xE1
//			, 0x00
//			, 0x0E
//			, 0x14
//			, 0x03
//			, 0x11
//			, 0x07
//			, 0x31
//			, 0xC1
//			, 0x48
//			, 0x08
//			, 0x0F
//			, 0x0C
//			, 0x31
//			, 0x36
//			, 0x0F);

//	set_addr_win(par, 0x0000, 0x0000, 0x00ef, 0x013f);

	//	write_reg(par, 0x13); /* Normal mode ON */

	return 0;
}


/*
  Gamma string format:
    Positive: Par1 Par2 [...] Par15
    Negative: Par1 Par2 [...] Par15
*/
#define CURVE(num, idx)  curves[num*par->gamma.num_values + idx]
static int set_gamma(struct fbtft_par *par, unsigned long *curves)
{
	int i;

	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	for (i = 0; i < par->gamma.num_curves; i++)
		write_reg(par, 0xE0 + i,
			CURVE(i, 0), CURVE(i, 1), CURVE(i, 2),
			CURVE(i, 3), CURVE(i, 4), CURVE(i, 5),
			CURVE(i, 6), CURVE(i, 7), CURVE(i, 8),
			CURVE(i, 9), CURVE(i, 10), CURVE(i, 11),
			CURVE(i, 12), CURVE(i, 13), CURVE(i, 14));

	return 0;
}
#undef CURVE


static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.txbuflen = TXBUFLEN,
	.gamma_num = 2,
	.gamma_len = 15,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = ili9341_controller_init,
		.set_addr_win = ili9341_set_addr_win,
		.set_var = set_var,
		.set_gamma = set_gamma,
	},
};
FBTFT_REGISTER_DRIVER(DRVNAME, "ilitek,ili9341", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:ili9341");
MODULE_ALIAS("platform:ili9341");

MODULE_DESCRIPTION("FB driver for the ILI9341 LCD display controller");
MODULE_AUTHOR("Christian Vogelgsang");
MODULE_LICENSE("GPL");
