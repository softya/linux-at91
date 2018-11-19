// Change the width and height if required (defined in portrait mode)
// or use the constructor to over-ride defaults
#ifndef TFT_WIDTH
  #define TFT_WIDTH  240
#endif
#ifndef TFT_HEIGHT
  #define TFT_HEIGHT 320
#endif

#if (TFT_HEIGHT == 240) && (TFT_WIDTH == 240)
  #define CGRAM_OFFSET
#endif

// Delay between some initialisation commands
#define TFT_INIT_DELAY 0x80 // Not used unless commandlist invoked


// Generic commands used by TFT_eSPI.cpp
#define TFT_NOP     0x00
#define TFT_SWRST   0x01

#define TFT_SLPIN   0x10
#define TFT_SLPOUT  0x11
#define TFT_NORON   0x13

#define TFT_INVOFF  0x20
#define TFT_INVON   0x21
#define TFT_DISPOFF 0x28
#define TFT_DISPON  0x29
#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C
#define TFT_RAMRD   0x2E
#define TFT_MADCTL  0x36
#define TFT_COLMOD  0x3A

// Flags for TFT_MADCTL
#define TFT_MAD_MY  0x80
#define TFT_MAD_MX  0x40
#define TFT_MAD_MV  0x20
#define TFT_MAD_ML  0x10
#define TFT_MAD_RGB 0x00
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH  0x04
#define TFT_MAD_SS  0x02
#define TFT_MAD_GS  0x01

#define TFT_IDXRD   0x00 // ILI9341 only, indexed control register read

// ST7789 specific commands used in init
#define ST7789_NOP			0x00
#define ST7789_SWRESET		0x01
#define ST7789_RDDID		0x04
#define ST7789_RDDST		0x09

#define ST7789_RDDPM		0x0A      // Read display power mode
#define ST7789_RDD_MADCTL	0x0B      // Read display MADCTL
#define ST7789_RDD_COLMOD	0x0C      // Read display pixel format
#define ST7789_RDDIM		0x0D      // Read display image mode
#define ST7789_RDDSM		0x0E      // Read display signal mode
#define ST7789_RDDSR		0x0F      // Read display self-diagnostic result (ST7789V)

#define ST7789_SLPIN		0x10
#define ST7789_SLPOUT		0x11
#define ST7789_PTLON		0x12
#define ST7789_NORON		0x13

#define ST7789_INVOFF		0x20
#define ST7789_INVON		0x21
#define ST7789_GAMSET		0x26      // Gamma set
#define ST7789_DISPOFF		0x28
#define ST7789_DISPON		0x29
#define ST7789_CASET		0x2A
#define ST7789_RASET		0x2B
#define ST7789_RAMWR		0x2C
#define ST7789_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define ST7789_RAMRD		0x2E

#define ST7789_PTLAR		0x30
#define ST7789_VSCRDEF		0x33      // Vertical scrolling definition (ST7789V)
#define ST7789_TEOFF		0x34      // Tearing effect line off
#define ST7789_TEON			0x35      // Tearing effect line on
#define ST7789_MADCTL		0x36      // Memory data access control
#define ST7789_IDMOFF		0x38      // Idle mode off
#define ST7789_IDMON		0x39      // Idle mode on
#define ST7789_RAMWRC		0x3C      // Memory write continue (ST7789V)
#define ST7789_RAMRDC		0x3E      // Memory read continue (ST7789V)
#define ST7789_COLMOD		0x3A

#define ST7789_RAMCTRL		0xB0      // RAM control
#define ST7789_RGBCTRL		0xB1      // RGB control
#define ST7789_PORCTRL		0xB2      // Porch control
#define ST7789_FRCTRL1		0xB3      // Frame rate control
#define ST7789_PARCTRL		0xB5      // Partial mode control
#define ST7789_GCTRL		0xB7      // Gate control
#define ST7789_GTADJ		0xB8      // Gate on timing adjustment
#define ST7789_DGMEN		0xBA      // Digital gamma enable
#define ST7789_VCOMS		0xBB      // VCOMS setting
#define ST7789_LCMCTRL		0xC0      // LCM control
#define ST7789_IDSET		0xC1      // ID setting
#define ST7789_VDVVRHEN		0xC2      // VDV and VRH command enable
#define ST7789_VRHS			0xC3      // VRH set
#define ST7789_VDVSET		0xC4      // VDV setting
#define ST7789_VCMOFSET		0xC5      // VCOMS offset set
#define ST7789_FRCTR2		0xC6      // FR Control 2
#define ST7789_CABCCTRL		0xC7      // CABC control
#define ST7789_REGSEL1		0xC8      // Register value section 1
#define ST7789_REGSEL2		0xCA      // Register value section 2
#define ST7789_PWMFRSEL		0xCC      // PWM frequency selection
#define ST7789_PWCTRL1		0xD0      // Power control 1
#define ST7789_VAPVANEN		0xD2      // Enable VAP/VAN signal output
#define ST7789_CMD2EN		0xDF      // Command 2 enable
#define ST7789_PVGAMCTRL	0xE0      // Positive voltage gamma control
#define ST7789_NVGAMCTRL	0xE1      // Negative voltage gamma control
#define ST7789_DGMLUTR		0xE2      // Digital gamma look-up table for red
#define ST7789_DGMLUTB		0xE3      // Digital gamma look-up table for blue
#define ST7789_GATECTRL		0xE4      // Gate control
#define ST7789_SPI2EN		0xE7      // SPI2 enable
#define ST7789_PWCTRL2		0xE8      // Power control 2
#define ST7789_EQCTRL		0xE9      // Equalize time control
#define ST7789_PROMCTRL		0xEC      // Program control
#define ST7789_PROMEN		0xFA      // Program mode enable
#define ST7789_NVMSET		0xFC      // NVM setting
#define ST7789_PROMACT		0xFE      // Program action



/////////////////////////
#define ST7789V_COLMOD_RGB_FMT_18BITS		(6 << 4)
#define ST7789V_COLMOD_CTRL_FMT_18BITS		(6 << 0)

#define ST7789V_RAMCTRL_CMD		0xb0
#define ST7789V_RAMCTRL_RM_RGB			BIT(4)
#define ST7789V_RAMCTRL_DM_RGB			BIT(0)
#define ST7789V_RAMCTRL_MAGIC			(3 << 6)
#define ST7789V_RAMCTRL_EPF(n)			(((n) & 3) << 4)

#define ST7789V_RGBCTRL_CMD		0xb1
#define ST7789V_RGBCTRL_WO			BIT(7)
#define ST7789V_RGBCTRL_RCM(n)			(((n) & 3) << 5)
#define ST7789V_RGBCTRL_VSYNC_HIGH		BIT(3)
#define ST7789V_RGBCTRL_HSYNC_HIGH		BIT(2)
#define ST7789V_RGBCTRL_PCLK_HIGH		BIT(1)
#define ST7789V_RGBCTRL_VBP(n)			((n) & 0x7f)
#define ST7789V_RGBCTRL_HBP(n)			((n) & 0x1f)

#define ST7789V_PORCTRL_CMD		0xb2
#define ST7789V_PORCTRL_IDLE_BP(n)		(((n) & 0xf) << 4)
#define ST7789V_PORCTRL_IDLE_FP(n)		((n) & 0xf)
#define ST7789V_PORCTRL_PARTIAL_BP(n)		(((n) & 0xf) << 4)
#define ST7789V_PORCTRL_PARTIAL_FP(n)		((n) & 0xf)

#define ST7789V_GCTRL_CMD		0xb7
#define ST7789V_GCTRL_VGHS(n)			(((n) & 7) << 4)
#define ST7789V_GCTRL_VGLS(n)			((n) & 7)

#define ST7789V_VCOMS_CMD		0xbb

#define ST7789V_LCMCTRL_CMD		0xc0
#define ST7789V_LCMCTRL_XBGR			BIT(5)
#define ST7789V_LCMCTRL_XMX			BIT(3)
#define ST7789V_LCMCTRL_XMH			BIT(2)

#define ST7789V_VDVVRHEN_CMD		0xc2
#define ST7789V_VDVVRHEN_CMDEN			BIT(0)

#define ST7789V_VRHS_CMD		0xc3

#define ST7789V_VDVS_CMD		0xc4

#define ST7789V_FRCTRL2_CMD		0xc6

#define ST7789V_PWCTRL1_CMD		0xd0
#define ST7789V_PWCTRL1_MAGIC			0xa4
#define ST7789V_PWCTRL1_AVDD(n)			(((n) & 3) << 6)
#define ST7789V_PWCTRL1_AVCL(n)			(((n) & 3) << 4)
#define ST7789V_PWCTRL1_VDS(n)			((n) & 3)

#define ST7789V_PVGAMCTRL_CMD		0xe0
#define ST7789V_PVGAMCTRL_JP0(n)		(((n) & 3) << 4)
#define ST7789V_PVGAMCTRL_JP1(n)		(((n) & 3) << 4)
#define ST7789V_PVGAMCTRL_VP0(n)		((n) & 0xf)
#define ST7789V_PVGAMCTRL_VP1(n)		((n) & 0x3f)
#define ST7789V_PVGAMCTRL_VP2(n)		((n) & 0x3f)
#define ST7789V_PVGAMCTRL_VP4(n)		((n) & 0x1f)
#define ST7789V_PVGAMCTRL_VP6(n)		((n) & 0x1f)
#define ST7789V_PVGAMCTRL_VP13(n)		((n) & 0xf)
#define ST7789V_PVGAMCTRL_VP20(n)		((n) & 0x7f)
#define ST7789V_PVGAMCTRL_VP27(n)		((n) & 7)
#define ST7789V_PVGAMCTRL_VP36(n)		(((n) & 7) << 4)
#define ST7789V_PVGAMCTRL_VP43(n)		((n) & 0x7f)
#define ST7789V_PVGAMCTRL_VP50(n)		((n) & 0xf)
#define ST7789V_PVGAMCTRL_VP57(n)		((n) & 0x1f)
#define ST7789V_PVGAMCTRL_VP59(n)		((n) & 0x1f)
#define ST7789V_PVGAMCTRL_VP61(n)		((n) & 0x3f)
#define ST7789V_PVGAMCTRL_VP62(n)		((n) & 0x3f)
#define ST7789V_PVGAMCTRL_VP63(n)		(((n) & 0xf) << 4)

#define ST7789V_NVGAMCTRL_CMD		0xe1
#define ST7789V_NVGAMCTRL_JN0(n)		(((n) & 3) << 4)
#define ST7789V_NVGAMCTRL_JN1(n)		(((n) & 3) << 4)
#define ST7789V_NVGAMCTRL_VN0(n)		((n) & 0xf)
#define ST7789V_NVGAMCTRL_VN1(n)		((n) & 0x3f)
#define ST7789V_NVGAMCTRL_VN2(n)		((n) & 0x3f)
#define ST7789V_NVGAMCTRL_VN4(n)		((n) & 0x1f)
#define ST7789V_NVGAMCTRL_VN6(n)		((n) & 0x1f)
#define ST7789V_NVGAMCTRL_VN13(n)		((n) & 0xf)
#define ST7789V_NVGAMCTRL_VN20(n)		((n) & 0x7f)
#define ST7789V_NVGAMCTRL_VN27(n)		((n) & 7)
#define ST7789V_NVGAMCTRL_VN36(n)		(((n) & 7) << 4)
#define ST7789V_NVGAMCTRL_VN43(n)		((n) & 0x7f)
#define ST7789V_NVGAMCTRL_VN50(n)		((n) & 0xf)
#define ST7789V_NVGAMCTRL_VN57(n)		((n) & 0x1f)
#define ST7789V_NVGAMCTRL_VN59(n)		((n) & 0x1f)
#define ST7789V_NVGAMCTRL_VN61(n)		((n) & 0x3f)
#define ST7789V_NVGAMCTRL_VN62(n)		((n) & 0x3f)
#define ST7789V_NVGAMCTRL_VN63(n)		(((n) & 0xf) << 4)
/////////////////////////
