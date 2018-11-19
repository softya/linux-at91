/*
 * FB driver for the ST7789V LCD Controller
 *
 * Copyright (C) 2015 Dennis Menschel
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
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <video/mipi_display.h>

#include "fbtft.h"
#include "fb_st7789v.h"

#define DRVNAME "fb_st7789v"

#define DEFAULT_GAMMA \
	"70 2C 2E 15 10 09 48 33 53 0B 19 18 20 25\n" \
	"70 2C 2E 15 10 09 48 33 53 0B 19 18 20 25"

/**
 * enum st7789v_command - ST7789V display controller commands
 *
 * @PORCTRL: porch setting
 * @GCTRL: gate control
 * @VCOMS: VCOM setting
 * @VDVVRHEN: VDV and VRH command enable
 * @VRHS: VRH set
 * @VDVS: VDV set
 * @VCMOFSET: VCOM offset set
 * @PWCTRL1: power control 1
 * @PVGAMCTRL: positive voltage gamma control
 * @NVGAMCTRL: negative voltage gamma control
 *
 * The command names are the same as those found in the datasheet to ease
 * looking up their semantics and usage.
 *
 * Note that the ST7789V display controller offers quite a few more commands
 * which have been omitted from this list as they are not used at the moment.
 * Furthermore, commands that are compliant with the MIPI DCS have been left
 * out as well to avoid duplicate entries.
 */
enum st7789v_command {
	PORCTRL = 0xB2,
	GCTRL = 0xB7,
	VCOMS = 0xBB,
	VDVVRHEN = 0xC2,
	VRHS = 0xC3,
	VDVS = 0xC4,
	VCMOFSET = 0xC5,
	PWCTRL1 = 0xD0,
	PVGAMCTRL = 0xE0,
	NVGAMCTRL = 0xE1,
};

#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */


/**
 * set_var() - apply LCD properties like rotation and BGR mode
 *
 * @par: FBTFT parameter object
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_var(struct fbtft_par *par)
{
	u8 madctl_par = 0;

    printk(">>>>>>>>>>>> %s \n", __func__);

    if (par->bgr)
		madctl_par |= MADCTL_BGR;
	switch (par->info->var.rotate) {
	case 0:
		break;
	case 90:
		madctl_par |= (MADCTL_MV | MADCTL_MY);
		break;
	case 180:
		madctl_par |= (MADCTL_MX | MADCTL_MY);
		break;
	case 270:
		madctl_par |= (MADCTL_MV | MADCTL_MX);
		break;
	default:
		return -EINVAL;
	}
	write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
	return 0;
}

/**
 * init_display() - initialize the display controller
 *
 * @par: FBTFT parameter object
 *
 * Most of the commands in this init function set their parameters to the
 * same default values which are already in place after the display has been
 * powered up. (The main exception to this rule is the pixel format which
 * would default to 18 instead of 16 bit per pixel.)
 * Nonetheless, this sequence can be used as a template for concrete
 * displays which usually need some adjustments.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int init_display(struct fbtft_par *par)
{

	printk("%s() entered\n", __func__);

	/* turn off sleep mode */
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);

	/* ------------memory access control------------------------ */
	if(par->pdata->display.bpp == 18){
		/* 18bit pixel RGB666 */
		write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_18BIT);
	} else {
		 /* 16bit pixel */
		/* set pixel format to RGB-565 */
		write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);
	}

	write_reg(par, PORCTRL, 0x08, 0x08, 0x00, 0x22, 0x22);

	write_reg(par, ST7789_RGBCTRL, 0x41, 0x02, 0x14);
	/*
	 * VGH = 13.26V
	 * VGL = -10.43V
	 */
	write_reg(par, GCTRL, 0x35);

	/*
	 * VDV and VRH register values come from command write
	 * (instead of NVM)
	 */
	write_reg(par, VDVVRHEN, 0x01, 0xFF);

	/*
	 * VAP =  4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 * VAN = -4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 */
	write_reg(par, VRHS, 0x0B);

	/* VDV = 0V */
	write_reg(par, VDVS, 0x20);

	/* VCOM = 0.9V */
	write_reg(par, VCOMS, 0x20);

	/* VCOM offset = 0V */
	write_reg(par, VCMOFSET, 0x20);

	/*
	 * AVDD = 6.8V
	 * AVCL = -4.8V
	 * VDS = 2.3V
	 */
	write_reg(par, PWCTRL1, 0xA4, 0xA1);

	set_var(par);

	write_reg(par, ST7789_CASET,    // Column address set
					0x00,
					0x00,
					0x00,
					0xE5);    // 239

	write_reg(par, ST7789_RASET,    // Row address set
					0x00,
					0x00,
					0x01,
					0x3F);    // 319

	write_reg(par, ST7789_RAMWR);

	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);
	write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	mdelay(20);

	return 0;
}

static int st7789_init_display(struct fbtft_par *par)
{
  write_reg(par, ST7789_SLPOUT);   // Sleep out
  mdelay(120);

  write_reg(par, ST7789_NORON);    // Normal display mode on

  //------------------------------display and color format setting--------------------------------//
  write_reg(par, ST7789_MADCTL,
  					0x00,
#ifdef CGRAM_OFFSET
  					0x48); // BGR colour order for 240 x 240 TFT
#else
  					0x40); // RGB colour order for 240 x 320 TFT (Issue #232)
#endif


  // JLX240 display datasheet
  write_reg(par, 0xB6,
  					0x0A,
  					0x82);

  write_reg(par, ST7789_COLMOD,
  					0x55);

  //--------------------------------ST7789V Frame rate setting----------------------------------//
  write_reg(par, ST7789_PORCTRL,
  					0x0c,
  					0x0c,
  					0x00,
  					0x33,
  					0x33);

  write_reg(par, ST7789_GCTRL,     // Voltages: VGH / VGL
  					0x35);

  //---------------------------------ST7789V Power setting--------------------------------------//
  write_reg(par, ST7789_VCOMS,
  					0x28);		// JLX240 display datasheet

  write_reg(par, ST7789_LCMCTRL,
  					0x0C);

  write_reg(par, ST7789_VDVVRHEN,
		  	0x01,
			0xFF);

  write_reg(par, ST7789_VRHS,     // voltage VRHS
			0x10);

  write_reg(par, ST7789_VDVSET,
			0x20);

  write_reg(par, ST7789_FRCTR2,
				0x0f);

  write_reg(par, ST7789_PWCTRL1,
			0xa4,
			0xa1);

  //--------------------------------ST7789V gamma setting---------------------------------------//
  write_reg(par, ST7789_PVGAMCTRL,
			0xd0,
			0x00,
			0x02,
			0x07,
			0x0a,
			0x28,
			0x32,
			0x44,
			0x42,
			0x06,
			0x0e,
			0x12,
			0x14,
			0x17);

  write_reg(par, ST7789_NVGAMCTRL,
            0xd0,
            0x00,
            0x02,
            0x07,
            0x0a,
            0x28,
            0x31,
            0x54,
            0x47,
            0x0e,
            0x1c,
            0x17,
            0x1b,
            0x1e);

  write_reg(par, ST7789_INVOFF);

  set_var(par);

  write_reg(par, ST7789_CASET,    // Column address set
  					0x00,
  					0x00,
  					0x00,
  					0xE5);    // 239

  write_reg(par, ST7789_RASET,    // Row address set
  					0x00,
  					0x00,
  					0x01,
  					0x3F);    // 319

  write_reg(par, ST7789_RAMWR);

//  mdelay(200);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);
	write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	mdelay(20);

//	write_reg(par, ST7789_RAMWR);

	return 0;
}

static int st7789_init_display2(struct fbtft_par *par)
{
    printk(">>>>>>>>>>>> %s \n", __func__);

    write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);

  /* We need to wait 120ms after a sleep out command */
  msleep(120);

  write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
				0x00);

	/* ------------memory access control------------------------ */
	if(par->pdata->display.bpp == 18){
		/* 18bit pixel RGB666 */
		write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT,
		  (MIPI_DCS_PIXEL_FMT_18BIT << 4) |
				   (MIPI_DCS_PIXEL_FMT_18BIT));
	} else {
		 /* 16bit pixel */
		/* set pixel format to RGB-565 */
		write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);
	}


  write_reg(par, ST7789V_PORCTRL_CMD,
	0x0c,
	0x0c,
	0x00,
	ST7789V_PORCTRL_IDLE_BP(3) | ST7789V_PORCTRL_IDLE_FP(3),
	ST7789V_PORCTRL_PARTIAL_BP(3) | ST7789V_PORCTRL_PARTIAL_FP(3));

  write_reg(par, ST7789V_GCTRL_CMD,
	ST7789V_GCTRL_VGLS(5) | ST7789V_GCTRL_VGHS(3));

  write_reg(par, ST7789V_VCOMS_CMD,
	0x2b);

  write_reg(par, ST7789V_LCMCTRL_CMD,
	ST7789V_LCMCTRL_XMH | ST7789V_LCMCTRL_XMX | ST7789V_LCMCTRL_XBGR);

  write_reg(par, ST7789V_VDVVRHEN_CMD,
	ST7789V_VDVVRHEN_CMDEN);

  write_reg(par, ST7789V_VRHS_CMD,
	0xf);

  write_reg(par, ST7789V_VDVS_CMD,
	0x20);

  write_reg(par, ST7789V_FRCTRL2_CMD,
	0xf);

  write_reg(par, ST7789V_PWCTRL1_CMD,
	ST7789V_PWCTRL1_MAGIC,
	ST7789V_PWCTRL1_AVDD(2) | ST7789V_PWCTRL1_AVCL(2) | ST7789V_PWCTRL1_VDS(1));

  write_reg(par, ST7789V_PVGAMCTRL_CMD,
	ST7789V_PVGAMCTRL_VP63(0xd),
	ST7789V_PVGAMCTRL_VP1(0xca),
	ST7789V_PVGAMCTRL_VP2(0xe),
	ST7789V_PVGAMCTRL_VP4(8),
	ST7789V_PVGAMCTRL_VP6(9),
	ST7789V_PVGAMCTRL_VP13(7),
	ST7789V_PVGAMCTRL_VP20(0x2d),
	ST7789V_PVGAMCTRL_VP27(0xb) | ST7789V_PVGAMCTRL_VP36(3),
	ST7789V_PVGAMCTRL_VP43(0x3d),
	ST7789V_PVGAMCTRL_JP1(3) | ST7789V_PVGAMCTRL_VP50(4),
	ST7789V_PVGAMCTRL_VP57(0xa),
	ST7789V_PVGAMCTRL_VP59(0xa),
	ST7789V_PVGAMCTRL_VP61(0x1b),
	ST7789V_PVGAMCTRL_VP62(0x28));

  write_reg(par, ST7789V_NVGAMCTRL_CMD,
	ST7789V_NVGAMCTRL_VN63(0xd),
	ST7789V_NVGAMCTRL_VN1(0xca),
	ST7789V_NVGAMCTRL_VN2(0xf),
	ST7789V_NVGAMCTRL_VN4(8),
	ST7789V_NVGAMCTRL_VN6(8),
	ST7789V_NVGAMCTRL_VN13(7),
	ST7789V_NVGAMCTRL_VN20(0x2e),
	ST7789V_NVGAMCTRL_VN27(0xc) |
			   ST7789V_NVGAMCTRL_VN36(5),
	ST7789V_NVGAMCTRL_VN43(0x40),
	ST7789V_NVGAMCTRL_JN1(3) |
			   ST7789V_NVGAMCTRL_VN50(4),
	ST7789V_NVGAMCTRL_VN57(9),
	ST7789V_NVGAMCTRL_VN59(0xb),
	ST7789V_NVGAMCTRL_VN61(0x1b),
	ST7789V_NVGAMCTRL_VN62(0x28));

//	  write_reg(par, MIPI_DCS_ENTER_INVERT_MODE);

  write_reg(par, ST7789V_RAMCTRL_CMD,
	ST7789V_RAMCTRL_DM_RGB |
			   ST7789V_RAMCTRL_RM_RGB,
	ST7789V_RAMCTRL_EPF(3) |
			  ST7789V_RAMCTRL_MAGIC);

  write_reg(par, ST7789V_RGBCTRL_CMD,
		  ST7789V_RGBCTRL_WO | ST7789V_RGBCTRL_RCM(2) |
			   ST7789V_RGBCTRL_VSYNC_HIGH | ST7789V_RGBCTRL_HSYNC_HIGH |
			   ST7789V_RGBCTRL_PCLK_HIGH,
			   ST7789V_RGBCTRL_VBP(8),
			   ST7789V_RGBCTRL_HBP(20));

  write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
  mdelay(120);
  write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
  mdelay(20);

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
#if 0
static int st7789_controller_init(struct fbtft_par *par)
{
	if(par->lcd_iface == LCD_INTERFACE_PARALLEL_RGB){
		printk("%s() Initialize LCD conroller config for LCD_INTERFACE_PARALLEL_RGB...\n", __func__);
		ili9341_controller_init_parallel_rgb(par);
	} else {
		printk("%s() Initialize LCD conroller config for LCD_INTERFACE_SPI_RGB...\n", __func__);
		ili9341_controller_init_spi_rgb(par);
	}

	return 0;
}
#endif

/**
 * set_gamma() - set gamma curves
 *
 * @par: FBTFT parameter object
 * @curves: gamma curves
 *
 * Before the gamma curves are applied, they are preprocessed with a bitmask
 * to ensure syntactically correct input for the display controller.
 * This implies that the curves input parameter might be changed by this
 * function and that illegal gamma values are auto-corrected and not
 * reported as errors.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_gamma(struct fbtft_par *par, unsigned long *curves)
{
	int i;
	int j;
	int c; /* curve index offset */

    printk(">>>>>>>>>>>> %s \n", __func__);
#if 0
    /*
	 * Bitmasks for gamma curve command parameters.
	 * The masks are the same for both positive and negative voltage
	 * gamma curves.
	 */
	const u8 gamma_par_mask[] = {
		0xFF, /* V63[3:0], V0[3:0]*/
		0x3F, /* V1[5:0] */
		0x3F, /* V2[5:0] */
		0x1F, /* V4[4:0] */
		0x1F, /* V6[4:0] */
		0x3F, /* J0[1:0], V13[3:0] */
		0x7F, /* V20[6:0] */
		0x77, /* V36[2:0], V27[2:0] */
		0x7F, /* V43[6:0] */
		0x3F, /* J1[1:0], V50[3:0] */
		0x1F, /* V57[4:0] */
		0x1F, /* V59[4:0] */
		0x3F, /* V61[5:0] */
		0x3F, /* V62[5:0] */
	};

	for (i = 0; i < par->gamma.num_curves; i++) {
		c = i * par->gamma.num_values;
		for (j = 0; j < par->gamma.num_values; j++)
			curves[c + j] &= gamma_par_mask[j];
		write_reg(
			par, PVGAMCTRL + i,
			curves[c + 0], curves[c + 1], curves[c + 2],
			curves[c + 3], curves[c + 4], curves[c + 5],
			curves[c + 6], curves[c + 7], curves[c + 8],
			curves[c + 9], curves[c + 10], curves[c + 11],
			curves[c + 12], curves[c + 13]);
	}
#endif
	return 0;
}

/**
 * blank() - blank the display
 *
 * @par: FBTFT parameter object
 * @on: whether to enable or disable blanking the display
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int blank(struct fbtft_par *par, bool on)
{
    printk(">>>>>>>>>>>> %s \n", __func__);
	if (on)
		write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
	else
		write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 240,
	.height = 320,
	.gamma_num = 2,
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
//		.init_display = init_display,
//		.init_display = st7789_init_display,
		.init_display = st7789_init_display2,
		.set_var = set_var,
		.set_gamma = set_gamma,
		.blank = blank,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7789v1", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7789v");
MODULE_ALIAS("platform:st7789v");

MODULE_DESCRIPTION("FB driver for the ST7789V LCD Controller");
MODULE_AUTHOR("Dennis Menschel");
MODULE_LICENSE("GPL");
