/*
 * A V4L2 driver for OmniVision OV7690 cameras.
 *
 * Copyright 2010 One Laptop Per Child
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#ifndef __OV7690_H
#define __OV7690_H

struct ov7690_config {
	int min_width;			/* Filter out smaller sizes */
	int min_height;			/* Filter out smaller sizes */
	int clock_speed;		/* External clock speed (MHz) */
	bool use_smbus;			/* Use smbus I/O instead of I2C */
	bool pll_bypass;		/* Choose whether to bypass the PLL */
	bool pclk_hb_disable;		/* Disable toggling pixclk during horizontal blanking */
};

/*
 * Basic window sizes.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/* Sensor pixel array is 492x656 */
#define OV7690_PIXEL_ARRAY_WIDTH 656
#define OV7690_PIXEL_ARRAY_HEIGHT 492

#define OV7690_WINDOW_MIN_WIDTH 40
#define OV7690_WINDOW_MIN_HEIGHT 30
#define OV7690_WINDOW_MAX_WIDTH (VGA_WIDTH)
#define OV7690_WINDOW_MAX_HEIGHT (VGA_HEIGHT)
#define OV7690_VSTART_DEF 0xc
#define OV7690_HSTART_DEF 0x69
/*
 * The 7690 sits on i2c with ID 0x42
 */
#define OV7690_I2C_ADDR 0x42

/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BGAIN	0x01	/* blue gain */
#define REG_RGAIN       0x02	/* red gain */
#define REG_GGAIN       0x03
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_REG0C          0x0c
#define   REG0C_VFLIP       0x80
#define   REG0C_MIRROR      0x40
#define   REG0C_RB_SWAP     0x20  /* BR SWAP in RGB format */
#define   REG0C_UV_SWAP     0x10  /* YuYv swap in YUV mode */
#define   REG0C_DATAOUT_ENABLE 0x04  /* Data pins enable*/
#define   REG0C_CTLOUT_ENABLE 0x2 /* VSYNC, HREF, PCLK enable*/
#define   REG0C_COLOR_BAR 0x1 	/* Overlay color Bar */
#define REG_0D          0x0d
#define REG_0E          0x0e
#define REG_AECH	0xf	/* Automatic exposure control MSB */
#define REG_AECL	0x10	/* ... LSB */
#define REG_CLKRC	0x11
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_REG12	0x12
#define   REG12_RESET	  0x80	  /* Register reset */
#define REG_REG13	0x13
#define REG_REG14	0x14
#define REG_REG15	0x15
#define REG_REG16	0x16
#define REG_HSTART	0x17
#define REG_HSIZE	0x18
#define REG_VSTART	0x19
#define REG_VSIZE	0x1a
#define REG_SHFT	0x1b
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_REG1E	0x1e
#define REG_REG1F	0x1f
#define REG_REG20	0x20
#define REG_AECGM	0x21
#define REG_REG22	0x22
#define REG_WPT		0x24
#define REG_BPT		0x25
#define REG_VPT		0x26
#define REG_REG27	0x27
#define REG_REG28	0x28
#define REG_PLL		0x29
#define REG_EXCHL	0x2a
#define REG_EXCHH	0x2b
#define REG_DM_LN	0x2c
#define REG_ADVFL	0x2d
#define REG_ADVFH	0x2e
#define REG_StrobeADC	0x38
//#define REG_REG39	0x39
#define REG_REG3E	0x3e
#define REG_REG3F	0x3f
#define REG_ANA1	0x41
#define REG_PWC0	0x49
#define REG_BD50ST	0x50
#define REG_BD60ST	0x51
#define REG_UVCTR0	0x5a
#define REG_UVCTR1	0x5b
#define REG_UVCTR2	0x5c
#define REG_UVCTR3	0x5d
#define REG_REG62	0x62
#define REG_BLC8	0x68
#define REG_BLCOUT	0x6b
#define REG_6F		0x6f
#define REG_REG80	0x80	/* Enables */
#define REG_REG81	0x81	/* SDE: Special digital effects */
#define REG_REG82	0x82
#define REG_LCC0	0x85	/* Lens control and correction*/
#define REG_LCC1	0x86
#define REG_LCC2	0x87
#define REG_LCC3	0x88
#define REG_LCC4	0x89
#define REG_LCC5	0x8a
#define REG_LCC6	0x8b
#define REG_AWB_BASE	0x8c 	/* AWB Control @0x8c:0xa2*/
#define REG_AWB(x)	((x)+REG_AWB_BASE)
#define REG_GAM_BASE	0xa2 	/* Gamma correction constants 1:15 */
#define REG_GAM(x)	((x)+REG_GAM_BASE)
#define REG_SLOPE	0xb2
#define REG_REGB4       0xb4
#define REG_REGB5       0xb5
#define REG_REGB6	0xb6
#define REG_REGB7       0xb7
#define REG_REGB8       0xb8
#define REG_REGB9       0xb9
#define REG_REGBA       0xba
#define REG_REGBB       0xbb
#define REG_REGBC       0xbc
#define REG_REGBD       0xbd
#define REG_REGBE       0xbe
#define REG_REGBF       0xbf
#define REG_REGC0	0xc0
#define REG_REGC1	0xc1
#define   CMATRIX_BASE  (REG_REGBB)
#define   CMATRIX_LEN	(1+REG_REGC1-CMATRIX_BASE)
#define REG_REGC2	0xc2
#define REG_REGC3	0xc3
#define REG_REGC4	0xc4
#define REG_REGC5	0xc5
#define REG_REGC6	0xc6
#define REG_REGC7	0xc7
#define REG_REGC8	0xc8
#define REG_REGC9	0xc9
#define REG_REGCA	0xca
#define REG_REGCB	0xcb
#define REG_REGCC	0xcc
#define REG_REGCD	0xcd
#define REG_REGCE	0xce
#define REG_REGCF	0xcf
#define REG_REGD0	0xd0
#define REG_REGD1	0xd1
#define REG_SDECTRL	0xd2	// Side control for sde
#define   SDECTRL_HUE_EN 0x1	// 0x1 - hue
#define   SDECTRL_SAT_EN 0x2	// 0x2 - saturation
#define   SDECTRL_CONT_EN 0x4	// 0x4 - contrast & brightness
#define   SDECTRL_FIX_UV 0x18   // fix U and V
#define REG_REGD3	0xd3
#define REG_REGD4	0xd4
#define REG_REGD5 	0xd5
#define REG_REGD6 	0xd6	// Hue cos
#define REG_REGD7 	0xd7	// Hue sin
#define REG_REGD8 	0xd8
#define REG_REGD9 	0xd9
#define REG_REGDA 	0xda	//Ureg
#define REG_REGDB 	0xdb	//Vreg
#define REG_REGDC 	0xdc	// Signs for SDE controls
				// 0x33 hue:
				//   00xx01 - 0 <= theta < pi/2
				//   00xx10 - -pi/2 <= theta < 0
				//   11xx01 - pi/2 <= theta < pi
				//   11xx10 - -pi <= theta < -pi/2
				// 0x4 Contrast
				// 0x8 Brightness
#define REG_REGDD 	0xdd
#define REG_REGDE 	0xde
#define REG_REGDF 	0xdf
#define REG_REGE0 	0xe0
#define REG_REGE1 	0xe1
/* Non-existent register number to terminate value lists */
#define REG_DUMMY       0xff

#define REGVAL_LIST_END { REG_DUMMY, REG_DUMMY,}

/* Structure describing frame interval */
struct ov7690_interval
{
	const struct regval_list *regs;
	struct v4l2_fract interval;
};

enum ov7690_model {
	MODEL_OV7690 = 0,
};

struct ov7690_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
};

struct ov7690_devtype {
	/* formats supported for each model */
	struct ov7690_win_size *win_sizes;
	unsigned int n_win_sizes;
	/* callbacks for frame rate control */
	int (*set_framerate)(struct v4l2_subdev *, struct v4l2_fract *);
	void (*get_framerate)(struct v4l2_subdev *, struct v4l2_fract *);
};

/*
 * Information we maintain about a known sensor.
 */
struct ov7690_format_struct;  /* coming later */
struct ov7690_info {
	struct v4l2_subdev sd;
//	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	struct {
		/* gain cluster */
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct {
		/* exposure cluster */
		struct v4l2_ctrl *auto_exposure;
		struct v4l2_ctrl *exposure;
	};
	struct {
		/* saturation/hue cluster */
		struct v4l2_ctrl *saturation;
		struct v4l2_ctrl *hue;
	};
	struct ov7690_format_struct *fmt;  /* Current format */
	struct clk *clk;
//	struct gpio_desc *resetb_gpio;
//	struct gpio_desc *pwdn_gpio;
	int min_width;			/* Filter out smaller sizes */
	int min_height;			/* Filter out smaller sizes */
	int clock_speed;		/* External clock speed (MHz) */
	u8 clkrc;			/* Clock divider value */
	bool use_smbus;			/* Use smbus I/O instead of I2C */
	bool pll_bypass;
	bool pclk_hb_disable;
	const struct ov7690_devtype *devtype; /* Device specifics */
};

#endif
