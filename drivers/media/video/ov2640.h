/*
 * drivers/media/video/ov2640.h
 *
 * Register definitions for the OmniVision OV2640 CameraChip.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV2640_H
#define OV2640_H

#define OV2640_I2C_ADDR		0x30

/* define register offsets for the OV2640 sensor chip */
#define OV2640_GAIN		0x00
#define OV2640_COM1		0x03
#define OV2640_REG04		0x04
#define OV2640_REG08		0x08
#define OV2640_COM2		0x09
#define OV2640_PIDH		0x0A
#define OV2640_PIDL		0x0B
#define OV2640_COM3		0x0C
#define OV2640_AEC		0x10
#define OV2640_CLKRC		0x11
#define OV2640_COM7		0x12
#define OV2640_COM8		0x13
#define OV2640_COM9		0x14
#define OV2640_COM10		0x15
#define OV2640_HREFST		0x17
#define OV2640_HREFEND		0x18
#define OV2640_VSTRT		0x19
#define OV2640_VEND		0x1A
#define OV2640_MIDH		0x1C
#define OV2640_MIDL		0x1D
#define OV2640_AEW		0x24
#define OV2640_AEB		0x25
#define OV2640_W		0x26
#define OV2640_REG2A		0x2A
#define OV2640_FRARL		0x2B
#define OV2640_ADDVSL		0x2D
#define OV2640_ADDVSH		0x2E
#define OV2640_YAVG		0x2F
#define OV2640_REG32		0x32
#define OV2640_ARCOM2		0x32
#define OV2640_REG45		0x45
#define OV2640_FLL		0x46
#define OV2640_FLH		0x47
#define OV2640_COM19		0x48
#define OV2640_ZOOMS		0x49
#define OV2640_COM22		0x4B
#define OV2640_COM25		0x4E
#define OV2640_BD50		0x4F
#define OV2640_BD60		0x50
#define OV2640_REG5D		0x5D
#define OV2640_REG5E		0x5E
#define OV2640_REG5F		0x5F
#define OV2640_REG60		0x60
#define OV2640_HISTO_LOW	0x61
#define OV2640_HISTO_HIGH	0x62

#define OV2640_NUM_REGS		(OV2640_HISTO_HIGH + 1)

#define OV2640_PID_MAGIC	0x26	/* high byte of product ID number */
#define OV2640_MIDH_MAGIC	0x7F	/* high byte of mfg ID */
#define OV2640_MIDL_MAGIC	0xA2	/* low byte of mfg ID */

#define OV2640_REG_TERM 0xFF	/* terminating list entry for reg */
#define OV2640_VAL_TERM 0xFF	/* terminating list entry for val */

/*
 * The nominal xclk input frequency of the OV2640 is 24MHz, maximum
 * frequency is 48MHz, and minimum frequency is 10MHz.
 */
#define OV2640_XCLK_MIN 10000000
#define OV2640_XCLK_MAX 48000000
#define OV2640_XCLK_NOM 24000000

/* Reset value in register COM7*/
#define OV2640_RESET	0x80
#define OV2640_COLORBAR	0x02
/* define a structure for ov2640 register initialization values */
struct ov2640_reg {
	unsigned char reg;
	unsigned char val;
};

enum image_size { QQCIF, QQVGA, QCIF, QVGA, CIF, VGA, SVGA, SXGA };
enum pixel_format { YUV, RGB565, RGB555 };

#define NUM_IMAGE_SIZES 8
#define NUM_PIXEL_FORMATS 3

struct capture_size {
	unsigned long width;
	unsigned long height;
};

struct ov2640_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*power_set)(int power);
	/* Default registers written after power-on or reset. */
	const struct ov2640_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
};

/*
 * Array of image sizes supported by OV2640.  These must be ordered from
 * smallest image size to largest.
 */
static const struct capture_size ov2640_sizes[] = {
	{   88,  72 },	/* QQCIF */
	{  160, 120 },	/* QQVGA */
	{  176, 144 },	/* QCIF */
	{  320, 240 },	/* QVGA */
	{  352, 288 },	/* CIF */
	{  640, 480 },	/* VGA */
	{  800, 600 },	/* SVGA */
	{ 1280, 960 },	/* SXGA */
};

#endif /* ifndef OV2640_H */
