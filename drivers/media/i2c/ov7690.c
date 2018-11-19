/*
 * A V4L2 driver for OmniVision OV7690 cameras.
 *
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
//#include <linux/gpio.h>
//#include <linux/gpio/consumer.h>
#include <media/v4l2-device.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
//#include <media/v4l2-image-sizes.h>
#include <media/ov7690.h>

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov7690 sensors");
MODULE_LICENSE("GPL");

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");


static inline struct ov7690_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7690_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov7690_info, hdl)->sd;
}

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char mask;
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings provide VGA YUYV(or UYVY depending on OMAP3 ISP bridge configuration).
 */


static struct regval_list ov7690_default_regs[] = {
	{REG_REG12, REG12_RESET },
	{REG_PWC0, 0x0c} , // Power is at 1.8 v
/*
** Apparently soft reset reg_0x12.bit7 = 1 enables
** both pll and rata/control outputs
** It creates deadly consequences for omap isp.
** During init soft reset must be issued
** We try to follow reset with disabling outputs as closely in time
** as possible
*/
//	{REG_REG0C,  REG0C_UV_SWAP |
//	 (0 &  (REG0C_DATAOUT_ENABLE | REG0C_CTLOUT_ENABLE)) |	 0},
	{0x48, 0x42 }, // Omnivision magic
	{REG_ANA1, 0x43 }, // More magic
	{0x4c, 0x73},
	{REG_REG81, 0xFF}, // Enable SDE functions
	{REG_AECGM, 0x44},
	{REG_REG16, 0x03},
	{0x39, 0x80},
	{REG_REG1E, 0xb1},
	/* Format */
	// YUV
	{REG_REG12, 0x00},
	{REG_REG82, 0x03}, // YUV422
	{REG_REGD0, 0x48}, // Boundary offset
	{REG_REG80, 0x7f},
	{REG_REG3E, 0x30}, // Double pclk for YUV format

	{REG_REG22, 0x00},

	/* Resolution */
	{REG_HSTART, 0x69},
	{REG_HSIZE, 0xa4},
	{REG_VSTART, 0x0c},
	{REG_VSIZE, 0xf6},

	{REG_REGC8, (VGA_WIDTH>>8)&3},
	{REG_REGC9, VGA_WIDTH & 0xff}, //;ISP input hsize (640)
	{REG_REGCA, (VGA_HEIGHT >>8)&1},
	{REG_REGCB, VGA_HEIGHT&0xff }, //;ISP input vsize (480)

	{REG_REGCC, (VGA_WIDTH>>8)&3},
	{REG_REGCD, VGA_WIDTH & 0xff}, //;ISP output hsize (640)
	{REG_REGCE, (VGA_HEIGHT >>8)&1},
	{REG_REGCF, VGA_HEIGHT&0xff }, //;ISP output vsize (480)

	/* Lens Correction */
	{REG_LCC0, 0x90},
	{REG_LCC1, 0x00},
	{REG_LCC2, 0x00},
	{REG_LCC3, 0x10},
	{REG_LCC4, 0x30},
	{REG_LCC5, 0x29},
	{REG_LCC6, 0x26},

	/* Color Matrix */
	{REG_REGBB, 0x80},
	{REG_REGBC, 0x62},
	{REG_REGBD, 0x1e},
	{REG_REGBE, 0x26},
	{REG_REGBF, 0x7b},
	{REG_REGC0, 0xac},
	{REG_REGC1, 0x1e},

	/* Edge + Denoise */
	{REG_REGB7, 0x05},
	{REG_REGB8, 0x09},
	{REG_REGB9, 0x00},
	{REG_REGBA, 0x18},

	/* UVAdjust */
	{REG_UVCTR0, 0x4A},
	{REG_UVCTR1, 0x9F},
	{REG_UVCTR2, 0x48},
	{REG_UVCTR3, 0x32},

	/* AEC/AGC target */
	{REG_WPT, 0x78},
	{REG_BPT, 0x68},
	{REG_VPT, 0xb3},

	/* Gamma */
	{REG_GAM(1), 0x0b},
	{REG_GAM(2), 0x15},
	{REG_GAM(3), 0x2a},
	{REG_GAM(4), 0x51},
	{REG_GAM(5), 0x63},
	{REG_GAM(6), 0x74},
	{REG_GAM(7), 0x83},
	{REG_GAM(8), 0x91},
	{REG_GAM(9), 0x9e},
	{REG_GAM(10), 0xaa},
	{REG_GAM(11), 0xbe},
	{REG_GAM(12), 0xce},
	{REG_GAM(13), 0xe5},
	{REG_GAM(14), 0xf3},
	{REG_GAM(15), 0xfb},
	{REG_SLOPE, 0x06},


	/* AWB */
	/* Simple */
//;42 8e 92 ; simple AWB
//;42 96 ff
//;42 97 00 ;unlimit AWB range.

	/* Advanced */
	{REG_AWB(0), 0x5d},
	{REG_AWB(1), 0x11},
	{REG_AWB(2), 0x12},
	{REG_AWB(3), 0x11},
	{REG_AWB(4), 0x50},
	{REG_AWB(5), 0x22},
	{REG_AWB(6), 0xd1},
	{REG_AWB(7), 0xa7},
	{REG_AWB(8), 0x23},
	{REG_AWB(9), 0x3b},
	{REG_AWB(10), 0xff},
	{REG_AWB(11), 0x00},
	{REG_AWB(12), 0x4a},
	{REG_AWB(13), 0x46},
	{REG_AWB(14), 0x3d},
	{REG_AWB(15), 0x3a},
	{REG_AWB(16), 0xf0},
	{REG_AWB(17), 0xf0},
	{REG_AWB(18), 0xf0},
	{REG_AWB(19), 0xff},
	{REG_AWB(20), 0x56},
	{REG_AWB(21), 0x55},
	{REG_AWB(22), 0x13},

	/* General Control */
	{REG_BD50ST, 0x9a},
	{REG_BD60ST, 0x80},
	{REG_AECGM, 0x23},

	{REG_REG14, 0x28},
	{REG_REG13, 0xf7},
/*
*	rate = clock(divider+1)
*	0 - 30 fps
*	1 - 15 fps
* 	2 - 10 fps
*	3 - 7.5 fps
*/
	{REG_CLKRC, 0x00},

	{REG_0E, 0x01,0x3},	// set drive strength to x2

	REGVAL_LIST_END	/* END MARKER */
};

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see ov7690_s_fmt for why.
 */


static struct regval_list ov7690_fmt_yuv422[] = {
	{REG_REG12, 0x00, 0x3f},
	{REG_REG82, 0x03, 0x03},
	{REG_REG3E, 0x10, 0x10},
	REGVAL_LIST_END
};

static struct regval_list ov7690_fmt_yvu422[] = {
	{REG_REG0C, 0x00, REG0C_UV_SWAP },
	{REG_REG12, 0x00, 0x3f},
	{REG_REG82, 0x03, 0x03},
	{REG_REG3E, 0x10, 0x10},
	REGVAL_LIST_END
};

static const struct regval_list ov7690_fmt_rgb565[] = {
	{REG_REG12, 0x06, 0x3f},
	{REG_REG82, 0x03, 0x03},
	{REG_REG3E, 0x10, 0x10},
	REGVAL_LIST_END
};

/*
 * Frame interval register settings
 * values come from OmniVision.
 * Frame rates are more then just pll, AEC related values change as well
 */
static const struct regval_list ov7690_15fps[] = {
	{REG_BD50ST, 0x4c},
	{REG_BD60ST, 0x3f},
	{REG_AECGM, 0x57 },
	{REG_REG20, 0x0},
	{REG_CLKRC, 0x1, CLK_SCALE},
	REGVAL_LIST_END
};

static const struct regval_list ov7690_30fps[] = {
	{REG_BD50ST, 0x9a},
	{REG_BD60ST, 0x80},
	{REG_AECGM, 0x23 },
	{REG_REG20, 0x0},
	{REG_CLKRC, 0x0, CLK_SCALE},
	REGVAL_LIST_END
};

static const struct ov7690_interval ov7690_intervals[] = {
	{ov7690_30fps,{33300,1000000}}, /* 30fps */
	{ov7690_15fps,{15000,1000000}}, /* 15fps */
};


/*
 * Low-level register I/O.
 *
 * Note that there are two versions of these.  On the XO 1, the
 * i2c controller only does SMBUS, so that's what we use.  The
 * ov7690 is not really an SMBUS device, though, so the communication
 * is not always entirely reliable.
 */
static int ov7690_read_smbus(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

//	printk("%s() <<<<<<<<<<<<<<<<<< \n", __func__);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		*value = (unsigned char)ret;
		ret = 0;
	}
	return ret;
}


static int ov7690_write_smbus(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if ( reg == REG_REG12 && (value & REG12_RESET)) msleep(5);

	return ret;
}

/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int ov7690_read_i2c(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data = reg;
	struct i2c_msg msg;
	int ret;

	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		printk(KERN_ERR "Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		*value = data;
		ret = 0;
	}
	return ret;
}


static int ov7690_write_i2c(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[2] = { reg, value };
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;
	if ( reg == REG_REG12 && (value & REG12_RESET)) msleep(5);

	return ret;
}

static int ov7690_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct ov7690_info *info = to_state(sd);
	int ret;
	if (info->use_smbus)
		ret =  ov7690_read_smbus(sd, reg, value);
	else
		ret =  ov7690_read_i2c(sd, reg, value);

//	printk("\t%s() [ %.2X, %.2X ],\n", __func__, reg, *value);

	return ret;
}

static int ov7690_write(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct ov7690_info *info = to_state(sd);

	printk("\t%s() [ %.2X, %.2X ],\n", __func__, reg, value);
	if (info->use_smbus)
		return ov7690_write_smbus(sd, reg, value);
	else
		return ov7690_write_i2c(sd, reg, value);
}

static int ov7690_write_mask(struct v4l2_subdev *sd, unsigned char reg, unsigned char value, unsigned char mask)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	unsigned char curr_value;

	printk("\t%s() [ %.2X, %.2X, %.2X ],\n", __func__, reg, value, mask);

	ret = ov7690_read(sd, reg, &curr_value);
	if (ret < 0) return ret;
	curr_value = (unsigned char) curr_value & (~mask);
	curr_value |= value & mask;

	return ov7690_write(sd, reg, (unsigned char) curr_value);
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7690_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	printk("%s() <<<<<<<<<<<<<<<<<< \n\n", __func__);

	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret;
		if (vals->mask)
			ret=ov7690_write_mask(sd, vals->reg_num, vals->value, vals->mask);
		else
			ret = ov7690_write(sd, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static int ov7690_reset(struct v4l2_subdev *sd, u32 val)
{
	ov7690_write(sd, REG_REG12, REG12_RESET);
	msleep(5);

	return 0;
}


static int ov7690_init(struct v4l2_subdev *sd, u32 val)
{
	unsigned char value = -1, test_reg = 0x82;
	int res;

	msleep(10);
	ov7690_read(sd, test_reg, &value);
	printk("%s BEFORE test register %.2X = %.2X\n", __func__, test_reg, value);

	res = ov7690_write_array(sd, ov7690_default_regs /*ov7690_default_samsung*/);

	msleep(10);
	ov7690_read(sd, test_reg, &value);
	printk("%s AFTER  test register %.2X = %.2X\n", __func__, test_reg, value);

	return res;
}

static int ov7690_detect(struct v4l2_subdev *sd)
{
	unsigned char v;
	int ret;

	ret = ov7690_init(sd, 0);
	if (ret < 0)
		return ret;
	ret = ov7690_read(sd, REG_MIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x7f) /* OV manuf. id. */
		return -ENODEV;
	ret = ov7690_read(sd, REG_MIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0xa2)
		return -ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7690_read(sd, REG_PID, &v);
	if (ret < 0)
		return ret;
	if (v != 0x76)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	ret = ov7690_read(sd, REG_VER, &v);
	if (ret < 0)
		return ret;
	if (v != 0x91)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix numbers come from OmniVision.
 */
static struct ov7690_format_struct {
	enum v4l2_mbus_pixelcode mbus_code;
	enum v4l2_colorspace colorspace;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
} ov7690_formats[] = {
	{
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov7690_fmt_yuv422,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
	},
	{
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs		= ov7690_fmt_yvu422,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
	},
	{
		.mbus_code	= V4L2_MBUS_FMT_RGB565_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs		= ov7690_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
	},
//	{
//		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
//		.colorspace	= V4L2_COLORSPACE_SRGB,
//		.regs 		= ov7690_fmt_raw,
//		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
//	},
};
#define N_OV7690_FMTS ARRAY_SIZE(ov7690_formats)


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

/*
 * QCIF mode is done (by OV) in a very strange way - it actually looks like
 * VGA with weird scaling options - they do *not* use the canned QCIF mode
 * which is allegedly provided by the sensor.  So here's the weird register
 * settings.
 */

static struct regval_list ov7690_res_qcif[] = {
	/* 176 x 144 */
	{0x16,0x40},
	{0x17,0x83},
	{0x18,0x96},
	{0x19,0x06},
	{0x1a,0xf6},
	{0x22,0x10},
	{0xc8,0x02},
	{0xc9,0x4b},
	{0xca,0x00},
	{0xcb,0xf0},
	{0xcc,0x00},
	{0xcd,0xb0},
	{0xce,0x00},
	{0xcf,0x90},

	{0xFF, 0xFF}
};
static struct regval_list ov7690_res_cif[] = {
	/* 352 x 288 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x01},
	{0xcd, 0x60},
	{0xce, 0x01},
	{0xcf, 0x20},

	{0xFF, 0xFF}
};

static struct regval_list ov7690_res_qvga[] = {
	/* 320 x 240 */
	{0x16,0x03},
	{0x17,0x69},
	{0x18,0xa4},
	{0x19,0x06},
	{0x1a,0xf6},
	{0x22,0x10},
	{0xc8,0x02},
	{0xc9,0x80},
	{0xca,0x00},
	{0xcb,0xf0},
	{0xcc,0x01},
	{0xcd,0x40},
	{0xce,0x00},
	{0xcf,0xf0},

	{0xFF, 0xFF}
};

static struct regval_list ov7690_res_vga[] = {
	/* 640 x 480 */
//	{0x16,0x03},
//	{0x17,0x69},
//	{0x18,0xa4},
//	{0x19,0x0c},
//	{0x1a,0xf6},
//	{0x22,0x00},
//	{0xc8,0x02},
//	{0xc9,0x80},
//	{0xca,0x01},
//	{0xcb,0xe0},
//	{0xcc,0x02},
//	{0xcd,0x80},
//	{0xce,0x01},
//	{0xcf,0xe0},

	{0xFF, 0xFF}
};

/*

static struct regval_list ov7690_qcif_regs[] = {
	{ REG_COM3, COM3_SCALEEN|COM3_DCWEN },
	{ REG_COM3, COM3_DCWEN },
	{ REG_COM14, COM14_DCWEN | 0x01},
	{ 0x73, 0xf1 },
	{ 0xa2, 0x52 },
	{ 0x7b, 0x1c },
	{ 0x7c, 0x28 },
	{ 0x7d, 0x3c },
	{ 0x7f, 0x69 },
	{ REG_COM9, 0x38 },
	{ 0xa1, 0x0b },
	{ 0x74, 0x19 },
	{ 0x9a, 0x80 },
	{ 0x43, 0x14 },
	{ REG_COM13, 0xc0 },
	{ 0xff, 0xff },
};
*/

static struct ov7690_win_size ov7690_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
//		.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,	/* These values from */
		.hstop		=  14,	/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs		= ov7690_res_vga,
	},
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
//		.com7_bit	= COM7_FMT_CIF,
		.hstart		= 170,	/* Empirically determined */
		.hstop		=  90,
		.vstart		=  14,
		.vstop		= 494,
		.regs		= ov7690_res_cif,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
//		.com7_bit	= COM7_FMT_QVGA,
		.hstart		= 168,	/* Empirically determined */
		.hstop		=  24,
		.vstart		=  12,
		.vstop		= 492,
		.regs		= ov7690_res_qvga,
	},
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
//		.com7_bit	= COM7_FMT_VGA, /* see comment above */
		.hstart		= 456,	/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs		= ov7690_res_qcif,
	}
};


static void ov7690_get_framerate_legacy(struct v4l2_subdev *sd,
				 struct v4l2_fract *tpf)
{
	struct ov7690_info *info = to_state(sd);

	tpf->numerator = 1;
	tpf->denominator = info->clock_speed;
	if ((info->clkrc & CLK_EXT) == 0 && (info->clkrc & CLK_SCALE) > 1)
		tpf->denominator /= (info->clkrc & CLK_SCALE);
}
static int ov7690_set_framerate_legacy(struct v4l2_subdev *sd,
					struct v4l2_fract *tpf)
{
	return 0;
}

static int ov7690_set_framerate_legacy2(struct v4l2_subdev *sd,
					struct v4l2_fract *tpf)
{
	struct ov7690_info *info = to_state(sd);
	int div;

	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator * info->clock_speed) / tpf->denominator;
	if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	info->clkrc = (info->clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = info->clock_speed / div;
	return ov7690_write(sd, REG_CLKRC, info->clkrc);
}

/*
 * Store a set of start/stop values into the camera.
 */
static int ov7690_set_hw(struct v4l2_subdev *sd, int hstart, int hstop,
		int vstart, int vstop)
{
	int ret;
//	unsigned char v;
/*
 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
 * a mystery "edge offset" value in the top two bits of href.
 */
	ret =  ov7690_write(sd, REG_HSTART, (hstart >> 3) & 0xff);
	ret += ov7690_write(sd, REG_HSIZE, (hstop >> 3) & 0xff);
//	ret += ov7690_read(sd, REG_HREF, &v);
//	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
//	msleep(10);
//	ret += ov7690_write(sd, REG_HREF, v);
/*
 * Vertical: similar arrangement, but only 10 bits.
 */
	ret += ov7690_write(sd, REG_VSTART, (vstart >> 2) & 0xff);
	ret += ov7690_write(sd, REG_HSIZE, (vstop >> 2) & 0xff);
//	ret += ov7690_read(sd, REG_VREF, &v);
//	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
//	msleep(10);
//	ret += ov7690_write(sd, REG_VREF, v);
	return ret;
}

#if 0
static int ov7690_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_OV7690_FMTS)
		return -EINVAL;

	code->code = ov7690_formats[code->index].mbus_code;
	return 0;
}
#endif
static int ov7690_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt,
		struct ov7690_format_struct **ret_fmt,
		struct ov7690_win_size **ret_wsize)
{
	int index, i;
	struct ov7690_win_size *wsize;
	struct ov7690_info *info = to_state(sd);
	unsigned int n_win_sizes = info->devtype->n_win_sizes;
	unsigned int win_sizes_limit = n_win_sizes;

	for (index = 0; index < N_OV7690_FMTS; index++)
		if (ov7690_formats[index].mbus_code == fmt->code){
			printk("%s() found mbus code %.4X\n", __func__, fmt->code);
			break;
		}
	if (index >= N_OV7690_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = ov7690_formats[0].mbus_code;
	}
	if (ret_fmt != NULL)
		*ret_fmt = ov7690_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;

	/*
	 * Don't consider values that don't match min_height and min_width
	 * constraints.
	 */
	if (info->min_width || info->min_height)
		for (i = 0; i < n_win_sizes; i++) {
			wsize = info->devtype->win_sizes + i;

			if (wsize->width < info->min_width ||
				wsize->height < info->min_height) {
				printk("%s() found good resolution (h=%d, w=%d) for mbus code %.4X\n",
						__func__,
						info->min_width,
						info->min_height,
						fmt->code);
				win_sizes_limit = i;
				break;
			}
		}
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = info->devtype->win_sizes;
	     wsize < info->devtype->win_sizes + win_sizes_limit; wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;
	if (wsize >= info->devtype->win_sizes + win_sizes_limit)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;
	fmt->height = wsize->height;
	fmt->colorspace = ov7690_formats[index].colorspace;
	return 0;
}
#if 0
static int ov7690_set_fmt2(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret = 0;
	u8 val;

	ov7690_read(sd, 0x49, &val);
	val = 0x0c;
//	ov7690_write(sd, 0x49, val);
	/*read DOVDD IO voltage setting*/
	ov7690_read(sd, 0x49, &val);

	return 0;
}
/*
 * Set a format.
 */
static int ov7690_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ov7690_format_struct *ovfmt;
	struct ov7690_win_size *wsize;
	struct ov7690_info *info = to_state(sd);
//	unsigned char com7;
	int ret;

	printk("%s code %x\n", __func__, format->format.code);

//	if (format->pad)
//		return -EINVAL;

//	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
//		ret = ov7690_try_fmt_internal(sd, &format->format, NULL, NULL);
//		if (ret)
//			return ret;
//		cfg->try_fmt = format->format;
//		return 0;
//	}

	ret = ov7690_try_fmt_internal(sd, &format->format, &ovfmt, &wsize);

	if (ret)
		return ret;
	/*
	 * COM7 is a pain in the ass, it doesn't like to be read then
	 * quickly written afterward.  But we have everything we need
	 * to set it absolutely here, as long as the format-specific
	 * register sets list it first.
//	 */
//	com7 = ovfmt->regs[0].value;
//	com7 |= wsize->com7_bit;
//	ov7690_write(sd, REG_COM7, com7);
	/*
	 * Now write the rest of the array.  Also store start/stops
	 */
//	ov7690_write_array(sd, ovfmt->regs + 1);
//	ov7690_set_hw(sd, wsize->hstart, wsize->hstop, wsize->vstart,
//			wsize->vstop);
	ret = 0;
	if (wsize->regs)
		ret = ov7690_write_array(sd, wsize->regs);
	info->fmt = ovfmt;
	printk("%s code %x, bla bla bla i2c...\n", __func__, format->format.code);

	/*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 * (Update) Now that we retain clkrc state, we should be able
	 * to write it unconditionally, and that will make the frame
	 * rate persistent too.
	 */
//	if (ret == 0)
//		ret = ov7690_write(sd, REG_CLKRC, info->clkrc);
	return 0;
}
#endif
/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov7690_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct ov7690_info *info = to_state(sd);

	printk("%s 1\n", __func__);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	info->devtype->get_framerate(sd, &cp->timeperframe);

	return 0;
}

static int ov7690_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct ov7690_info *info = to_state(sd);

	printk("%s 1\n", __func__);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (cp->extendedmode != 0)
		return -EINVAL;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	return info->devtype->set_framerate(sd, tpf);
}


/*
 * Frame intervals.  Since frame rates are controlled with the clock
 * divider, we can only do 30/n for integer n values.  So no continuous
 * or stepwise options.  Here we just pick a handful of logical values.
 */

static int ov7690_frame_rates[] = { 30, 15, 10, 5, 1 };
#if 0
static int ov7690_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov7690_info *info = to_state(sd);
	unsigned int n_win_sizes = info->devtype->n_win_sizes;
	int i;

	printk("%s fie->index %d \n", __func__, fie->index);
	if (fie->pad)
		return -EINVAL;
	if (fie->index >= ARRAY_SIZE(ov7690_frame_rates))
		return -EINVAL;

	printk("%s 2\n", __func__);
	/*
	 * Check if the width/height is valid.
	 *
	 * If a minimum width/height was requested, filter out the capture
	 * windows that fall outside that.
	 */
	for (i = 0; i < n_win_sizes; i++) {
		struct ov7690_win_size *win = &info->devtype->win_sizes[i];

		if (info->min_width && win->width < info->min_width)
			continue;
		if (info->min_height && win->height < info->min_height)
			continue;
		if (fie->width == win->width && fie->height == win->height)
			break;
	}
	if (i == n_win_sizes)
		return -EINVAL;
	fie->interval.numerator = 1;
	fie->interval.denominator = ov7690_frame_rates[fie->index];
	return 0;
}
#endif
/*
 * Frame size enumeration
 */
static int ov7690_enum_frame_size(struct v4l2_subdev *sd,
//				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov7690_info *info = to_state(sd);
	int i;
	int num_valid = -1;
	__u32 index = fse->index;
	unsigned int n_win_sizes = info->devtype->n_win_sizes;

	printk("%s 1\n", __func__);
	if (fse->pad)
		return -EINVAL;

	printk("%s 2\n", __func__);
	/*
	 * If a minimum width/height was requested, filter out the capture
	 * windows that fall outside that.
	 */
	for (i = 0; i < n_win_sizes; i++) {
		struct ov7690_win_size *win = &info->devtype->win_sizes[i];
		if (info->min_width && win->width < info->min_width)
			continue;
		if (info->min_height && win->height < info->min_height)
			continue;
		if (index == ++num_valid) {
			fse->min_width = fse->max_width = win->width;
			fse->min_height = fse->max_height = win->height;
			printk("%s width %d, height %d\n", __func__, win->width, win->height);
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * Code for dealing with controls.
 */

static int ov7690_store_cmatrix(struct v4l2_subdev *sd,
		int matrix[CMATRIX_LEN])
{
	int i, ret;
	unsigned char signbits = 0;

	printk("%s \n", __func__);
	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
//	ret = ov7690_read(sd, REG_CMATRIX_SIGN, &signbits);
	signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += ov7690_write(sd, CMATRIX_BASE + i, raw);
	}
//	ret += ov7690_write(sd, REG_CMATRIX_SIGN, signbits);
	return ret;
}


/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#define SIN_STEP 5
static const int ov7690_sin_table[] = {
	   0,	 87,   173,   258,   342,   422,
	 499,	573,   642,   707,   766,   819,
	 866,	906,   939,   965,   984,   996,
	1000
};

static int ov7690_sine(int theta)
{
	int chs = 1;
	int sine;

	printk("%s \n", __func__);

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = ov7690_sin_table[theta/SIN_STEP];
	else {
		theta -= 90;
		sine = 1000 - ov7690_sin_table[theta/SIN_STEP];
	}
	return sine*chs;
}

static int ov7690_cosine(int theta)
{
	theta = 90 - theta;

	printk("%s \n", __func__);

	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return ov7690_sine(theta);
}




static void ov7690_calc_cmatrix(struct ov7690_info *info,
		int matrix[CMATRIX_LEN], int sat, int hue)
{
	int i;
	/*
	 * Apply the current saturation setting first.
	 */
	printk("%s \n", __func__);

	for (i = 0; i < CMATRIX_LEN; i++)
		matrix[i] = (info->fmt->cmatrix[i] * sat) >> 7;
	/*
	 * Then, if need be, rotate the hue value.
	 */
	if (hue != 0) {
		int sinth, costh, tmpmatrix[CMATRIX_LEN];

		memcpy(tmpmatrix, matrix, CMATRIX_LEN*sizeof(int));
		sinth = ov7690_sine(hue);
		costh = ov7690_cosine(hue);

		matrix[0] = (matrix[3]*sinth + matrix[0]*costh)/1000;
		matrix[1] = (matrix[4]*sinth + matrix[1]*costh)/1000;
		matrix[2] = (matrix[5]*sinth + matrix[2]*costh)/1000;
		matrix[3] = (matrix[3]*costh - matrix[0]*sinth)/1000;
		matrix[4] = (matrix[4]*costh - matrix[1]*sinth)/1000;
		matrix[5] = (matrix[5]*costh - matrix[2]*sinth)/1000;
	}
}



static int ov7690_s_sat_hue(struct v4l2_subdev *sd, int sat, int hue)
{
	struct ov7690_info *info = to_state(sd);
	int matrix[CMATRIX_LEN];
	int ret;

	printk("%s \n", __func__);

	ov7690_calc_cmatrix(info, matrix, sat, hue);
//	ret = ov7690_store_cmatrix(sd, matrix);
//	return ret;
	return 0;
}


/*
 * Some weird registers seem to store values in a sign/magnitude format!
 */

static unsigned char ov7690_abs_to_sm(unsigned char v)
{
	if (v > 127)
		return v & 0x7f;
	return (128 - v) | 0x80;
}

static int ov7690_s_brightness(struct v4l2_subdev *sd, int value)
{
	unsigned char com8 = 0, v;
	int ret;

	printk("%s \n", __func__);

//	ov7690_read(sd, REG_COM8, &com8);
//	com8 &= ~COM8_AEC;
//	ov7690_write(sd, REG_COM8, com8);
	v = ov7690_abs_to_sm(value);
//	ret = ov7690_write(sd, REG_BRIGHT, v);
//	return ret;

	return 0;
}

static int ov7690_s_contrast(struct v4l2_subdev *sd, int value)
{
//	return ov7690_write(sd, REG_CONTRAS, (unsigned char) value);
	return 0;
}

static int ov7690_s_hflip(struct v4l2_subdev *sd, int value)
{
	unsigned char v = 0;
	int ret;

	printk("%s \n", __func__);

//	ret = ov7690_read(sd, REG_MVFP, &v);
//	if (value)
//		v |= MVFP_MIRROR;
//	else
//		v &= ~MVFP_MIRROR;
//	msleep(10);  /* FIXME */
//	ret += ov7690_write(sd, REG_MVFP, v);
//	return ret;
	return 0;
}

static int ov7690_s_vflip(struct v4l2_subdev *sd, int value)
{
	unsigned char v = 0;
	int ret;

	printk("%s \n", __func__);
//
//	ret = ov7690_read(sd, REG_MVFP, &v);
//	if (value)
//		v |= MVFP_FLIP;
//	else
//		v &= ~MVFP_FLIP;
//	msleep(10);  /* FIXME */
//	ret += ov7690_write(sd, REG_MVFP, v);
//	return ret;
	return 0;
}

/*
 * GAIN is split between REG_GAIN and REG_VREF[7:6].  If one believes
 * the data sheet, the VREF parts should be the most significant, but
 * experience shows otherwise.  There seems to be little value in
 * messing with the VREF bits, so we leave them alone.
 */
static int ov7690_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char gain;

	printk("%s \n", __func__);

	ret = ov7690_read(sd, REG_GAIN, &gain);
	*value = gain;
	return ret;
}

static int ov7690_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret;
	unsigned char com8;

	printk("%s \n", __func__);
	ret = ov7690_write(sd, REG_GAIN, value & 0xff);
	/* Have to turn off AGC as well */
//	if (ret == 0) {
//		ret = ov7690_read(sd, REG_COM8, &com8);
//		ret = ov7690_write(sd, REG_COM8, com8 & ~COM8_AGC);
//	}
//	return ret;
	return 0;
}

/*
 * Tweak autogain.
 */
static int ov7690_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret;
	unsigned char com8;

	printk("%s \n", __func__);
//	ret = ov7690_read(sd, REG_COM8, &com8);
//	if (ret == 0) {
//		if (value)
//			com8 |= COM8_AGC;
//		else
//			com8 &= ~COM8_AGC;
//		ret = ov7690_write(sd, REG_COM8, com8);
//	}
//	return ret;
	return 0;
}

static int ov7690_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	unsigned char com1, com8, aech, aechh;

	printk("%s \n", __func__);
//	ret = ov7690_read(sd, REG_COM1, &com1) +
//		ov7690_read(sd, REG_COM8, &com8) +
//		ov7690_read(sd, REG_AECHH, &aechh);
//	if (ret)
//		return ret;
//
//	com1 = (com1 & 0xfc) | (value & 0x03);
//	aech = (value >> 2) & 0xff;
//	aechh = (aechh & 0xc0) | ((value >> 10) & 0x3f);
//	ret = ov7690_write(sd, REG_COM1, com1) +
//		ov7690_write(sd, REG_AECH, aech) +
//		ov7690_write(sd, REG_AECHH, aechh);
//	/* Have to turn off AEC as well */
//	if (ret == 0)
//		ret = ov7690_write(sd, REG_COM8, com8 & ~COM8_AEC);
//	return ret;
	return 0;
}

/*
 * Tweak autoexposure.
 */
static int ov7690_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	int ret;
	unsigned char com8;

	printk("%s \n", __func__);
//	ret = ov7690_read(sd, REG_COM8, &com8);
//	if (ret == 0) {
//		if (value == V4L2_EXPOSURE_AUTO)
//			com8 |= COM8_AEC;
//		else
//			com8 &= ~COM8_AEC;
//		ret = ov7690_write(sd, REG_COM8, com8);
//	}
//	return ret;
	return 0;
}


static int ov7690_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov7690_info *info = to_state(sd);

	printk("%s ctrl->id %x\n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ov7690_g_gain(sd, &info->gain->val);
	}
	return -EINVAL;
}

static int ov7690_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov7690_info *info = to_state(sd);

	printk("%s ctrl->id %x\n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov7690_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ov7690_s_contrast(sd, ctrl->val);
	case V4L2_CID_SATURATION:
		return ov7690_s_sat_hue(sd,
				info->saturation->val, info->hue->val);
	case V4L2_CID_VFLIP:
		return ov7690_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ov7690_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ov7690_s_gain turns off auto gain */
			return ov7690_s_gain(sd, info->gain->val);
		}
		return ov7690_s_autogain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE_AUTO:
		/* Only set manual exposure if auto exposure is not explicitly
		   turned on. */
		if (ctrl->val == V4L2_EXPOSURE_MANUAL) {
			/* ov7690_s_exp turns off auto exposure */
			return ov7690_s_exp(sd, info->exposure->val);
		}
		return ov7690_s_autoexp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ov7690_ctrl_ops = {
	.s_ctrl = ov7690_s_ctrl,
	.g_volatile_ctrl = ov7690_g_volatile_ctrl,
};

static int ov7690_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV7690, 0);
}
#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov7690_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ov7690_read(sd, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov7690_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ov7690_write(sd, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

static int ov7690_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov7690_info *info = to_state(sd);

	printk("%s \n", __func__);

//	if (info->pwdn_gpio)
//		gpiod_direction_output(info->pwdn_gpio, !on);
//	if (on && info->resetb_gpio) {
//		gpiod_set_value(info->resetb_gpio, 1);
//		usleep_range(500, 1000);
//		gpiod_set_value(info->resetb_gpio, 0);
//		usleep_range(3000, 5000);
//	}

	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov7690_core_ops = {
	.g_chip_ident = ov7690_g_chip_ident,
	.reset = ov7690_reset,
	.init = ov7690_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov7690_g_register,
	.s_register = ov7690_s_register,
#endif
	.s_power = ov7690_s_power,
};

static const struct v4l2_subdev_video_ops ov7690_video_ops = {
	.s_parm = ov7690_s_parm,
	.g_parm = ov7690_g_parm,
};
/*
static const struct v4l2_subdev_pad_ops ov7690_pad_ops = {
	.enum_frame_interval = ov7690_enum_frame_interval,
	.enum_frame_size = ov7690_enum_frame_size,
	.enum_mbus_code = ov7690_enum_mbus_code,
	.set_fmt = ov7690_set_fmt,
};
*/
static const struct v4l2_subdev_ops ov7690_ops = {
	.core = &ov7690_core_ops,
	.video = &ov7690_video_ops,
//	.pad = &ov7690_pad_ops,
};

/* ----------------------------------------------------------------------- */

static const struct ov7690_devtype ov7690_devdata[] = {
	[MODEL_OV7690] = {
		.win_sizes = ov7690_win_sizes,
		.n_win_sizes = ARRAY_SIZE(ov7690_win_sizes),
		.set_framerate = ov7690_set_framerate_legacy,
		.get_framerate = ov7690_get_framerate_legacy,
	},
};
/*
static int ov7690_init_gpio(struct i2c_client *client, struct ov7690_info *info)
{
	info->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "powerdown",
			GPIOD_OUT_LOW);
	if (IS_ERR(info->pwdn_gpio)) {
		dev_info(&client->dev, "can't get %s GPIO\n", "powerdown");
		return PTR_ERR(info->pwdn_gpio);
	}

	info->resetb_gpio = devm_gpiod_get_optional(&client->dev, "reset",
			GPIOD_OUT_LOW);
	if (IS_ERR(info->resetb_gpio)) {
		dev_info(&client->dev, "can't get %s GPIO\n", "reset");
		return PTR_ERR(info->resetb_gpio);
	}

	return 0;
}
*/
static int ov7690_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct ov7690_info *info;
	int ret;


	printk("%s() >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> entered \n", __func__);
	info = kzalloc(sizeof(struct ov7690_info), GFP_KERNEL);
//	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	printk("%s() >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 111111111111111 \n", __func__);
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &ov7690_ops);

	printk("%s() >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 2222222222222 \n", __func__);

	info->clock_speed = 30; /* default: a guess */
	if (client->dev.platform_data) {
		struct ov7690_config *config = client->dev.platform_data;

		/*
		 * Must apply configuration before initializing device, because it
		 * selects I/O method.
		 */
		info->min_width = config->min_width;
		info->min_height = config->min_height;
		info->use_smbus = config->use_smbus;

		if (config->clock_speed)
			info->clock_speed = config->clock_speed;

		/*
		 * It should be allowed for ov7690 too when it is migrated to
		 * the new frame rate formula.
		 */
		if (config->pll_bypass && id->driver_data != MODEL_OV7690)
			info->pll_bypass = true;

		if (config->pclk_hb_disable)
			info->pclk_hb_disable = true;
	}
/*
	info->clk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(info->clk))
		return -EPROBE_DEFER;
	clk_prepare_enable(info->clk);

	info->clock_speed = clk_get_rate(info->clk) / 1000000;
	if (info->clock_speed < 10 || info->clock_speed > 48) {
		ret = -EINVAL;
		goto clk_disable;
	}
*/
	printk("%s() >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 333333333333333 \n", __func__);

//	ret = ov7690_init_gpio(client, info);
//	if (ret)
//		goto clk_disable;

//	ov7690_s_power(sd, 1);

	/* Make sure it's an ov7690 */
	ret = ov7690_detect(sd);
	printk("%s() >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 4444444 \n", __func__);
	if (ret) {
		v4l_dbg(1, debug, client,
			"chip found @ 0x%x (%s) is not an ov7690 chip.\n",
			client->addr << 1, client->adapter->name);
		printk("%s() chip found @ 0x%x (%s) is not an ov7690 chip.\n",
			 __func__, client->addr << 1, client->adapter->name);

		kfree(info);
		return ret;
	}
	printk("%s() >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 55555 \n", __func__);
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	info->devtype = &ov7690_devdata[id->driver_data];
	info->fmt = &ov7690_formats[0];
	info->clkrc = 0;

	/* Set default frame rate to 30 fps */
	tpf.numerator = 1;
	tpf.denominator = 30;
	info->devtype->set_framerate(sd, &tpf);

//	if (info->pclk_hb_disable)
//		ov7690_write(sd, REG_COM10, COM10_PCLK_HB);

	v4l2_ctrl_handler_init(&info->hdl, 10);
	v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->saturation = v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_SATURATION, 0, 256, 1, 128);
	info->hue = v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_HUE, -180, 180, 5, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->auto_gain = v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_EXPOSURE, 0, 65535, 1, 500);
	info->auto_exposure = v4l2_ctrl_new_std_menu(&info->hdl, &ov7690_ctrl_ops,
			V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0,
			V4L2_EXPOSURE_AUTO);
	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		kfree(info);
		return err;
	}
	/*
	 * We have checked empirically that hw allows to read back the gain
	 * value chosen by auto gain but that's not the case for auto exposure.
	 */
	v4l2_ctrl_auto_cluster(2, &info->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &info->auto_exposure,
			       V4L2_EXPOSURE_MANUAL, false);
	v4l2_ctrl_cluster(2, &info->saturation);
/*
#if defined(CONFIG_MEDIA_CONTROLLER)
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if (ret < 0)
		goto hdl_free;
#endif
*/
	v4l2_ctrl_handler_setup(&info->hdl);
/*
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto failed_sd;
*/
	return 0;
/*
failed_sd:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&info->sd.entity);
#endif
hdl_free:
	v4l2_ctrl_handler_free(&info->hdl);
power_off:
	ov7690_s_power(sd, 0);

clk_disable:
	clk_disable_unprepare(info->clk);
	return ret;
*/
}


static int ov7690_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7690_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
//	clk_disable_unprepare(info->clk);
//#if defined(CONFIG_MEDIA_CONTROLLER)
//	media_entity_cleanup(&info->sd.entity);
//#endif
//	ov7690_s_power(sd, 0);

	kfree(info);

	return 0;
}

static const struct i2c_device_id ov7690_id[] = {
	{ "ov7690", MODEL_OV7690 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7690_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov7690_of_match[] = {
	{ .compatible = "ovti,ov7690", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov7690_of_match);
#endif

static struct i2c_driver ov7690_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ov7690",
		.of_match_table = of_match_ptr(ov7690_of_match),
	},
	.probe		= ov7690_probe,
	.remove		= ov7690_remove,
	.id_table	= ov7690_id,
};

module_i2c_driver(ov7690_driver);
