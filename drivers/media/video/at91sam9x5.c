/*
 * Copyright (C) 2011 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

/*
 * XXX:
 * - handle setting of global alpha
 * - handle more formats
 * - complete this list :-)
 */

#define DEBUG

#include <linux/err.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME "at91sam9x5-video"

#define REG_HEOCHER		0x00
#define REG_HEOCHER_CHEN		0x00000001
#define REG_HEOCHER_UPDATEEN		0x00000002
#define REG_HEOCHER_A2QEN		0x00000004

#define REG_HEOCHDR		0x04
#define REG_HEOCHDR_CHDIS		0x00000001
#define REG_HEOCHDR_CHRST		0x00000100

#define REG_HEOCHSR		0x08
#define REG_HEOCHSR_CHSR		0x00000001
#define REG_HEOCHSR_UPDATESR		0x00000002
#define REG_HEOCHSR_A2QSR		0x00000004

#define REG_HEOIER		0x0c
#define REG_HEOIDR		0x10
#define REG_HEOIMR		0x14
#define REG_HEOISR		0x18
#define REG_HEOIxR_DMA			0x00000004
#define REG_HEOIxR_DSCR			0x00000008
#define REG_HEOIxR_ADD			0x00000010
#define REG_HEOIxR_DONE			0x00000020
#define REG_HEOIxR_OVR			0x00000040
#define REG_HEOIxR_UDMA			0x00000400
#define REG_HEOIxR_UDSCR		0x00000800
#define REG_HEOIxR_UADD			0x00001000
#define REG_HEOIxR_UDONE		0x00002000
#define REG_HEOIxR_UOVR			0x00004000
#define REG_HEOIxR_VDMA			0x00040000
#define REG_HEOIxR_VDSCR		0x00080000
#define REG_HEOIxR_VADD			0x00100000
#define REG_HEOIxR_VDONE		0x00200000
#define REG_HEOIxR_VOVR			0x00400000

#define REG_HEOADDR		0x20
#define REG_HEOUADDR		0x30
#define REG_HEOVADDR		0x30

#define REG_HEOCFG0		0x4c
#define REG_HEOCFG0_DLBO		0x00000100
#define REG_HEOCFG0_BLEN		0x00000030
#define REG_HEOCFG0_BLEN_INCR1			0x00000000
#define REG_HEOCFG0_BLEN_INCR4			0x00000010
#define REG_HEOCFG0_BLEN_INCR8			0x00000020
#define REG_HEOCFG0_BLEN_INCR16			0x00000030
#define REG_HEOCFG0_BLENUV		0x000000c0
#define REG_HEOCFG0_BLENUV_INCR1		0x00000000
#define REG_HEOCFG0_BLENUV_INCR4		0x00000040
#define REG_HEOCFG0_BLENUV_INCR8		0x00000080
#define REG_HEOCFG0_BLENUV_INCR16		0x000000c0

#define REG_HEOCFG1		0x50
#define REG_HEOCFG1_CLUTEN		0x00000001
#define REG_HEOCFG1_YUVEN		0x00000002
#define REG_HEOCFG1_YUVMODE_12YCBCRP	0x00008000

#define REG_HEOCFG2		0x54
#define REG_HEOCFG2_XPOS		0x000007ff
#define REG_HEOCFG2_YPOS		0x07ff0000

#define REG_HEOCFG3		0x58
#define REG_HEOCFG3_XSIZE		0x000007ff
#define REG_HEOCFG3_YSIZE		0x07ff0000

#define REG_HEOCFG4		0x5c
#define REG_HEOCFG4_XMEMSIZE		0x000007ff
#define REG_HEOCFG4_YMEMSIZE		0x07ff0000

#define REG_HEOCFG5		0x60
#define REG_HEOCFG5_XSTRIDE		0xffffffff

#define REG_HEOCFG6		0x64
#define REG_HEOCFG6_PSTRIDE		0xffffffff

#define REG_HEOCFG7		0x68
#define REG_HEOCFG7_UVXSTRIDE		0xffffffff

#define REG_HEOCFG8		0x6c
#define REG_HEOCFG8_UVPSTRIDE		0xffffffff

#define REG_HEOCFG9		0x70
#define REG_HEOCFG10		0x74
#define REG_HEOCFG11		0x78

#define REG_HEOCFG12		0x7c
#define REG_HEOCFG12_CRKEY		0x00000001
#define REG_HEOCFG12_INV		0x00000002
#define REG_HEOCFG12_ITER2BL		0x00000004
#define REG_HEOCFG12_ITER		0x00000008
#define REG_HEOCFG12_REVALPHA		0x00000010
#define REG_HEOCFG12_GAEN		0x00000020
#define REG_HEOCFG12_LAEN		0x00000040
#define REG_HEOCFG12_OVR		0x00000080
#define REG_HEOCFG12_DMA		0x00000100
#define REG_HEOCFG12_REP		0x00000200
#define REG_HEOCFG12_DSTKEY		0x00000400
#define REG_HEOCFG12_VIDPRI		0x00001000
#define REG_HEOCFG12_GA			0x00ff0000

#define REG_HEOCFG13		0x80
#define REG_HEOCFG13_XFACTOR		0x00001fff
#define REG_HEOCFG13_YFACTOR		0x1fff0000
#define REG_HEOCFG13_SCALEN		0x80000000

#define valtomask(val, mask)	(((val) << __ffs((mask))) & (mask))
#define valfrommask(val, mask)	(((val) & (mask)) >> __ffs((mask)))

static inline void _yuv420_align(unsigned *width, unsigned *height)
{
#ifdef ASSUME_ALIGNMENT
	*width = ALIGN(*width, 2);
	*height = ALIGN(*height, 2);
#endif
}
static inline unsigned yuv420_offset_cb(unsigned width, unsigned height)
{
	_yuv420_align(&width, &height);
	return width * height;
}

static inline unsigned yuv420_offset_cr(unsigned width, unsigned height)
{
	_yuv420_align(&width, &height);
	return width * height + ALIGN(width, 2) * ALIGN(height, 2) / 4;
}

static inline unsigned yuv420_size(unsigned width, unsigned height)
{
	_yuv420_align(&width, &height);
	return width * height + ALIGN(width, 2) * ALIGN(height, 2) / 2;
}

struct at91sam9x5_video_pdata {
	u16 base_width;
	u16 base_height;
};

static const struct at91sam9x5_video_csc_config {
	u32 cscr;
	u32 cscg;
	u32 cscb;
} at91sam9x5_video_csc_config[] = {
	/*
	 * Taken from http://en.wikipedia.org/wiki/Yuv#Conversion_to.2Ffrom_RGB
	 * (BT.601)
	 *
	 * | R |          |  128    0  146 |   | Y       |
	 * | G | = 2^-7 * |  128  -51  -74 | x | U - 128 |
	 * | B |          |  128  260    0 |   | V - 128 |
	 */
	{ 0x09200080, 0x7b6f3480, 0x40041080, },
	/*
	 * Derived from
	 * http://en.wikipedia.org/wiki/Yuv#Conversion_to.2Ffrom_RGB
	 * (BT.709)
	 * | R |          |  128    0  146 |   | Y       |
	 * | G | = 2^-7 * |  128  -27  -49 | x | U - 128 |
	 * | B |          |  128  272    0 |   | V - 128 |
	 */
	{ 0x9200080, 0x7d0f9480, 0x40044080, },

	/*
	 * Suggested by Nicolas Ferre <nicolas.ferre@atmel.com>
	 *
	 * | R |          |  145    0  201 |   | Y -  16 |
	 * | G | = 2^-7 * |  144  -44  -91 | x | U - 128 |
	 * | B |          |  144  258    0 |   | V - 128 |
	 */
	{ 0x4c900091, 0x7a5f5090, 0x40040890, },
	/*
	 * BT.601?
	 *
	 * | R |          |  149    0  204 |   | Y -  16 |
	 * | G | = 2^-7 * |  149  -50 -104 | x | U - 128 |
	 * | B |          |  149  258    0 |   | V - 128 |
	 */
	{ 0x4cc00095, 0x798f3895, 0x40040895, },
	/*
	 * BT.709?
	 *
	 * | R |          |  149    0  230 |   | Y -  16 |
	 * | G | = 2^-7 * |  149  -27  -68 | x | U - 128 |
	 * | B |          |  149  271    0 |   | V - 128 |
	 */
	{ 0x4e600095, 0x7bcf9495, 0x40043c95, },
};

struct at91sam9x5_video_priv {
	struct platform_device *pdev;
	struct notifier_block fb_notifier;

	/* protects the registers and values for registers in this struct */
	spinlock_t lock;

	struct video_device *video_dev;
	struct fb_info *fbinfo;

	void __iomem *regbase;

	struct vb2_buffer *cur, *next;

	enum {
		at91sam9x5_video_IDLE,
		at91sam9x5_video_RUNNING,
		at91sam9x5_video_INVISIBLE,
	} state;

	/* video size */
	unsigned short xmem_size, ymem_size;

	/*
	 * overlay window position and size;
	 * if xsize == 0 then no window is shown
	 */
	int xpos, ypos;
	unsigned xsize, ysize;

	/*
	 * count of cols (rows) that are cut of if the overlay window's
	 * x(y)-position is negative. 0 otherwise.
	 */
	unsigned int xmem_skip, ymem_skip;

	/* values for HEOCFG14 to HEOCFG16 */
	const struct at91sam9x5_video_csc_config *csc_config;

	struct vb2_queue queue;
	void *alloc_ctx;

	unsigned int irq;
};

static u32 at91sam9x5_video_read32(struct at91sam9x5_video_priv *priv,
		size_t offset)
{
	void __iomem *base = priv->regbase;
	if (offset >= 0x280)
		offset -= 0x280;
	return __raw_readl(base + offset);
}

static void at91sam9x5_video_write32(struct at91sam9x5_video_priv *priv,
		size_t offset, u32 val)
{
	void __iomem *base = priv->regbase;
	if (offset >= 0x280)
		offset -= 0x280;
	__raw_writel(val, base + offset);
}

static void at91sam9x5_video_ovlconfig_valid(struct at91sam9x5_video_priv *priv)
{
	unsigned hwxpos, hwypos, hwxsize, hwysize;
	unsigned hwxmem_size, hwymem_size;
	int hwxstride;

	/* XXX: check for oboes */
	if (priv->xpos < 0)
		hwxpos = 0;
	else
		hwxpos = priv->xpos;

	if (priv->ypos < 0)
		hwypos = 0;
	else
		hwypos = priv->ypos;

	if (priv->xpos + priv->xsize > priv->fbinfo->var.xres)
		hwxsize = priv->fbinfo->var.xres - hwxpos;
	else
		hwxsize = priv->xpos + priv->xsize - hwxpos;

	if (priv->ypos + priv->ysize > priv->fbinfo->var.yres)
		hwysize = priv->fbinfo->var.yres - hwypos;
	else
		hwysize = priv->ypos + priv->ysize - hwypos;

#if 0
	dev_info(&priv->video_dev->dev, "ovlconfig: (%u, %u)+(%u, %u)\n",
			hwxpos, hwypos, hwxsize, hwysize);
#endif

	at91sam9x5_video_write32(priv, REG_HEOCFG2,
			valtomask(hwxpos, REG_HEOCFG2_XPOS) |
			valtomask(hwypos, REG_HEOCFG2_YPOS));

	at91sam9x5_video_write32(priv, REG_HEOCFG3,
			valtomask(hwxsize - 1, REG_HEOCFG3_XSIZE) |
			valtomask(hwysize - 1, REG_HEOCFG3_YSIZE));

	/* XXX: rounding? overflow won't happen, no? */
	hwxmem_size = DIV_ROUND_CLOSEST(priv->xmem_size * hwxsize, priv->xsize);
	hwymem_size = DIV_ROUND_CLOSEST(priv->ymem_size * hwysize, priv->ysize);
	at91sam9x5_video_write32(priv, REG_HEOCFG4,
			valtomask(hwxmem_size - 1, REG_HEOCFG4_XMEMSIZE) |
			valtomask(hwymem_size - 1, REG_HEOCFG4_YMEMSIZE));

	priv->xmem_skip = DIV_ROUND_CLOSEST((hwxpos - priv->xpos) *
			priv->xmem_size, priv->xsize);
	priv->ymem_skip = DIV_ROUND_CLOSEST((hwypos - priv->ypos) *
			priv->ymem_size, priv->ysize);

	at91sam9x5_video_write32(priv, REG_HEOCFG13,
			REG_HEOCFG13_SCALEN |
			valtomask(1024 * hwxmem_size / hwxsize,
				REG_HEOCFG13_XFACTOR) |
			valtomask(1024 * hwymem_size / hwysize,
				REG_HEOCFG13_YFACTOR));

	hwxstride = priv->xmem_size - hwxmem_size;
	at91sam9x5_video_write32(priv, REG_HEOCFG5,
			valtomask(hwxstride, REG_HEOCFG5_XSTRIDE));

	at91sam9x5_video_write32(priv, REG_HEOCFG6,
			valtomask(0, REG_HEOCFG6_PSTRIDE));

	/* XXX rounding correct? depends on format */
	at91sam9x5_video_write32(priv, REG_HEOCFG7,
			valtomask(hwxstride / 2, REG_HEOCFG7_UVXSTRIDE));

	at91sam9x5_video_write32(priv, REG_HEOCFG8,
			valtomask(0, REG_HEOCFG8_UVPSTRIDE));

	if (priv->state == at91sam9x5_video_RUNNING)
		at91sam9x5_video_write32(priv,
				REG_HEOCHER, REG_HEOCHER_UPDATEEN);
}

static void at91sam9x5_video_ovlconfig(struct at91sam9x5_video_priv *priv)
{
	if (priv->xsize == 0) {
		if (priv->state == at91sam9x5_video_RUNNING)
			at91sam9x5_video_write32(priv, REG_HEOCHDR,
					REG_HEOCHDR_CHDIS);
		priv->state = at91sam9x5_video_INVISIBLE;
	} else {
		at91sam9x5_video_ovlconfig_valid(priv);
		priv->state = at91sam9x5_video_IDLE;
	}
}

static int at91sam9x5_video_bufinuse(struct at91sam9x5_video_priv *priv,
		struct vb2_buffer *vb)
{
	/* XXX: depends on format */
	dma_addr_t curpos;

	curpos = at91sam9x5_video_read32(priv, REG_HEOADDR);
	if (curpos - vb2_dma_contig_plane_paddr(vb, 0) <
			yuv420_size(priv->xmem_size, priv->ymem_size))
		return 1;

	curpos = at91sam9x5_video_read32(priv, REG_HEOUADDR);
	if (curpos - vb2_dma_contig_plane_paddr(vb, 0) <
			yuv420_size(priv->xmem_size, priv->ymem_size))
		return 1;

	curpos = at91sam9x5_video_read32(priv, REG_HEOVADDR);
	if (curpos - vb2_dma_contig_plane_paddr(vb, 0) <
			yuv420_size(priv->xmem_size, priv->ymem_size))
		return 1;

	return 0;
}

static int at91sam9x5_video_handleirq(struct at91sam9x5_video_priv *priv)
{
	/* XXX */

	return IRQ_NONE;
}

static irqreturn_t at91sam9x5_video_irq(int irq, void *data)
{
	struct at91sam9x5_video_priv *priv = data;

	printk("%s\n", __func__);
	return at91sam9x5_video_handleirq(priv);
}

static int at91sam9x5_video_vidioc_querycap(struct file *filp,
		void *fh, struct v4l2_capability *cap)
{
	strcpy(cap->driver, DRIVER_NAME);
	cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING |
		V4L2_CAP_VIDEO_OVERLAY;
	/* XXX */
	cap->version = 0;
	cap->card[0] = '\0';
	cap->bus_info[0] = '\0';
	return 0;
}

static int at91sam9x5_video_vidioc_g_fmt_vid_out(struct file *filp,
		void *fh, struct v4l2_format *f)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);

	pr_debug("%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* XXX */
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;

	f->fmt.pix.bytesperline = priv->xmem_size;
	f->fmt.pix.width = priv->xmem_size;
	f->fmt.pix.height = priv->ymem_size;
	f->fmt.pix.sizeimage = yuv420_size(priv->xmem_size, priv->ymem_size);

	return 0;
}

static int at91sam9x5_video_vidioc_s_fmt_vid_out(struct file *filp,
		void *fh, struct v4l2_format *f)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	unsigned long flags;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

#if 0
	dev_info(&vdev->dev, "s_fmt_vid_out: width=%u, height=%u\n",
			pix->width, pix->height);
#endif

	/* XXX */
	pix->pixelformat = V4L2_PIX_FMT_YUV420;
	pix->bytesperline = pix->width;
	pix->sizeimage = ALIGN(pix->width, 2) * ALIGN(pix->height, 2) * 3 / 2;

	spin_lock_irqsave(&priv->lock, flags);

	at91sam9x5_video_write32(priv, REG_HEOCFG1,
			REG_HEOCFG1_YUVMODE_12YCBCRP |
			REG_HEOCFG1_YUVEN);
	at91sam9x5_video_write32(priv, REG_HEOCFG12,
			REG_HEOCFG12_GAEN |
			REG_HEOCFG12_OVR |
			REG_HEOCFG12_DMA |
			REG_HEOCFG12_REP |
			REG_HEOCFG12_GA); // 0xff03a0);

	/* XXX: what about playing back a yuv video with odd size? */
	priv->xmem_size = pix->width;
	priv->ymem_size = pix->height;

	at91sam9x5_video_ovlconfig(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
static int at91sam9x5_video_vidioc_g_fmt_vid_overlay(struct file *filp,
		void *fh, struct v4l2_format *f)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);
	struct v4l2_window *win = &f->fmt.win;
	struct v4l2_rect *rect = &win->w;
	unsigned long flags;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->xpos) {
		rect->left = priv->xpos;
		rect->top = priv->ypos;
		rect->width = priv->xsize;
		rect->height = priv->ysize;
	} else {
		/*
		 * XXX: is this allowed? if not another flag for validity in
		 * priv is needed
		 */
		rect->left = 0;
		rect->top = 0;
		rect->width = 0;
		rect->height = 0;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

#if 0
	dev_info(&vdev->dev, "g_fmt_vid_overlay: left=%d, top=%d, width=%d, height=%d\n",
			rect->left, rect->top, rect->width, rect->height);
#endif

	return 0;
}

static int at91sam9x5_video_vidioc_s_fmt_vid_overlay(struct file *filp,
		void *fh, struct v4l2_format *f)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);
	struct v4l2_window *win = &f->fmt.win;
	struct v4l2_rect *rect = &win->w;
	unsigned long flags;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	/* XXX: handle field?, chromakey, clips, bitmap and global_alpha */
#if 0
	dev_info(&vdev->dev, "s_fmt_vid_overlay: left=%d, top=%d, width=%d, height=%d\n",
			rect->left, rect->top, rect->width, rect->height);
#endif

	/* handle is win->field? win->chromakey? win->clips? win->bitmap? */

	/* XXX: actually this should not be necessary to return -EINVAL here,
	 * but instead should just hide the window.
	 */
	if (0 && (rect->width <= 0 || rect->height <= 0))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	if (rect->height > 0 &&
			rect->width > 0 &&
			rect->left >= -rect->width &&
			rect->top >= -rect->height &&
			rect->left < (int)priv->fbinfo->var.xres &&
			rect->top < (int)priv->fbinfo->var.yres) {
		priv->xpos = rect->left;
		priv->ypos = rect->top;
		priv->xsize = rect->width;
		priv->ysize = rect->height;
	} else
		priv->xsize = 0;

	at91sam9x5_video_ovlconfig(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int at91sam9x5_video_vidioc_enum_fmt_vid_out(struct file *filp,
		void *fh, struct v4l2_fmtdesc *f)
{
	pr_debug("%s\n", __func__);
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_YUV420;
	return 0;
}

static int at91sam9x5_video_vb_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned long sizes[], void *alloc_ctxs[])
{
	struct at91sam9x5_video_priv *priv =
		container_of(q, struct at91sam9x5_video_priv, queue);

	pr_debug("%s: num_planes=%u num_buffers=%u\n", __func__,
			*num_planes, *num_buffers);

	/* XXX */
	*num_planes = 1;
	/* mx27-vout uses 32 here, this makes the at91 fail when allocating the
	 * buffers. queue setup is called with 12 ... */
	//*num_buffers = 9;
	/* The last 9 (aligned) words are used for the 3 dma descriptors (3 * 3
	 * * 32 bits). The additional 32 bits are for alignment.
	 * XXX: Is that allowed? And if so, is it done right?
	 */
	sizes[0] = yuv420_size(priv->xmem_size, priv->ymem_size) +
		32 + 9 * 32;
	alloc_ctxs[0] = priv->alloc_ctx;

	return 0;
}

static void at91sam9x5_video_show_buf(struct vb2_buffer *vb,
		struct at91sam9x5_video_priv *priv)
{
	/* XXX: plane_no = 0? */
	dma_addr_t buffer = vb2_dma_contig_plane_paddr(vb, 0);
	void *vaddr = vb2_plane_vaddr(vb, 0);
	size_t offset_dmadesc =
		ALIGN(yuv420_size(priv->xmem_size, priv->ymem_size), 32);
	u32 *dmadesc = vaddr + offset_dmadesc;

	struct vb2_buffer *vbdone;

	/* y */
	dmadesc[0] = buffer +
#ifdef ASSUME_ALIGNMENT
		ALIGN(priv->xmem_size, 2)
#else
		priv->xmem_size
#endif
		* priv->ymem_skip + priv->xmem_skip;
	dmadesc[1] = 1; /* XXX: use symbolic values */
	dmadesc[2] = buffer + offset_dmadesc;

	/* u */
	dmadesc[3] = buffer +
		yuv420_offset_cb(priv->xmem_size, priv->ymem_size) +
		ALIGN(priv->xmem_size, 2) * (priv->ymem_skip / 2) + priv->xmem_skip / 2;
	dmadesc[4] = 1;
	dmadesc[5] = buffer + offset_dmadesc + 3 * 4;

	/* v */
	dmadesc[6] = buffer +
		yuv420_offset_cr(priv->xmem_size, priv->ymem_size) +
		ALIGN(priv->xmem_size, 2) * (priv->ymem_skip / 2) + priv->xmem_skip / 2;
	dmadesc[7] = 1;
	dmadesc[8] = buffer + offset_dmadesc + 6 * 4;

	if (priv->state == at91sam9x5_video_IDLE) {
		at91sam9x5_video_write32(priv, 0x020, dmadesc[0]);
		at91sam9x5_video_write32(priv, 0x024, dmadesc[1]);
		at91sam9x5_video_write32(priv, 0x028, dmadesc[2]);
		at91sam9x5_video_write32(priv, 0x030, dmadesc[3]);
		at91sam9x5_video_write32(priv, 0x034, dmadesc[4]);
		at91sam9x5_video_write32(priv, 0x038, dmadesc[5]);
		at91sam9x5_video_write32(priv, 0x040, dmadesc[6]);
		at91sam9x5_video_write32(priv, 0x044, dmadesc[7]);
		at91sam9x5_video_write32(priv, 0x048, dmadesc[8]);
		at91sam9x5_video_write32(priv, 0x000, 3);
		priv->state = at91sam9x5_video_RUNNING;
	} else {
		at91sam9x5_video_write32(priv, 0x01c, dmadesc[2]);
		at91sam9x5_video_write32(priv, 0x02c, dmadesc[5]);
		at91sam9x5_video_write32(priv, 0x03c, dmadesc[8]);
		at91sam9x5_video_write32(priv, 0x000, 4);
	}

	/* XXX: read HEOISR and determine which vb to mark as done. Do I need to
	 * keep 2 buffers for cur as there are up to three descriptors per
	 * buffer?
	 */
#if 1
	/* This doesn't work yet */
	vbdone = priv->cur;
	priv->cur = vb;
	if (vbdone)
		vb2_buffer_done(vbdone, VB2_BUF_STATE_DONE);
#else
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
#endif
}

static void at91sam9x5_video_vb_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *q = vb->vb2_queue;
	struct at91sam9x5_video_priv *priv =
		container_of(q, struct at91sam9x5_video_priv, queue);
	unsigned long flags;

	pr_debug("%s: state=%d\n", __func__, priv->state);

	spin_lock_irqsave(&priv->lock, flags);
	switch (priv->state) {
	case at91sam9x5_video_RUNNING:
	case at91sam9x5_video_IDLE:
		at91sam9x5_video_show_buf(vb, priv);
		break;
	case at91sam9x5_video_INVISIBLE:
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		break;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
}

const struct vb2_ops at91sam9x5_video_vb_ops = {
	.queue_setup = at91sam9x5_video_vb_queue_setup,
	.buf_queue = at91sam9x5_video_vb_buf_queue,
};

static int at91sam9x5_video_vidioc_reqbufs(struct file *filp,
		void *fh, struct v4l2_requestbuffers *b)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);
	struct vb2_queue *q = &priv->queue;
	int ret;

	dev_dbg(&vdev->dev, "%s\n", __func__);

	q->ops = &at91sam9x5_video_vb_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->type = b->type;
	q->io_modes = VB2_MMAP | VB2_USERPTR; // | VB2_WRITE;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	return vb2_reqbufs(q, b);
}

static int at91sam9x5_video_vidioc_querybuf(struct file *filp,
		void *fh, struct v4l2_buffer *b)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);

	pr_debug("%s: b->index = %lu, queue->num_buffers = %u\n", __func__,
			(unsigned long)b->index, priv->queue.num_buffers);

	return vb2_querybuf(&priv->queue, b);
}

static int at91sam9x5_video_vidioc_qbuf(struct file *filp,
		void *fh, struct v4l2_buffer *b)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);

	//pr_info("%s: buffer=%p\n", __func__, b);

	return vb2_qbuf(&priv->queue, b);
}

static int at91sam9x5_video_vidioc_dqbuf(struct file *filp,
		void *fh, struct v4l2_buffer *b)
{
	struct video_device *vdev = filp->private_data;
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);
#if 0

	unsigned bufsize;

	pr_info("%s: buffer=%p\n", __func__, b);
	spin_lock_irqsave(&priv->lock);

	/* XXX: what if the size changed recently? */
	bufsize = yuv420_offset_cb(priv->xmem_size, priv->ymem_size);

	if (priv->state == at91sam9x5_video_RUNNING) {
		dma_addr_t curpos = at91sam9x5_video_read32(priv, REG_HEOADDR);

		printk("hui: %lx\n", (unsigned long)curpos);
		if (priv->cur &&
			curpos - vb2_dma_contig_plane_paddr(priv->cur, 0) > bufsize) {

			vb2_buffer_done(priv->cur, VB2_BUF_STATE_DONE);
			priv->cur = NULL;
		}
#else
	unsigned long flags;

	//pr_debug("%s: buffer=%p\n", __func__, b);
	spin_lock_irqsave(&priv->lock, flags);

	/* XXX: handle pending irqs */
	at91sam9x5_video_handleirq(priv);

	if (priv->cur && priv->next) {
	//	at91sam9x5_video_write32(priv, , );
#endif
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return vb2_dqbuf(&priv->queue, b, filp->f_flags & O_NONBLOCK);
}

static int at91sam9x5_video_vidioc_streamon(struct file *filp,
		void *fh, enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(filp);
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);

	pr_debug("%s\n", __func__);
	/* XXX: config */

	return vb2_streamon(&priv->queue, type);
}

static int at91sam9x5_video_vidioc_streamoff(struct file *filp,
		void *fh, enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(filp);
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);
	unsigned long flags;

	pr_debug("%s\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	at91sam9x5_video_write32(priv, REG_HEOCHDR, REG_HEOCHDR_CHDIS);

	/* XXX: wait for buffers to be unused or reset channel */
	if (priv->cur)
		vb2_buffer_done(priv->cur, VB2_BUF_STATE_DONE);

	if (priv->next)
		vb2_buffer_done(priv->next, VB2_BUF_STATE_DONE);

	priv->state = at91sam9x5_video_IDLE;

	spin_unlock_irqrestore(&priv->lock, flags);
	/* XXX: config? */
	return vb2_streamoff(&priv->queue, type);
}

static const struct v4l2_ioctl_ops at91sam9x5_video_ioctl_ops = {
	.vidioc_querycap = at91sam9x5_video_vidioc_querycap,
	.vidioc_g_fmt_vid_out = at91sam9x5_video_vidioc_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out = at91sam9x5_video_vidioc_s_fmt_vid_out,
	.vidioc_g_fmt_vid_overlay = at91sam9x5_video_vidioc_g_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay = at91sam9x5_video_vidioc_s_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_out = at91sam9x5_video_vidioc_enum_fmt_vid_out,
	.vidioc_reqbufs = at91sam9x5_video_vidioc_reqbufs,
	.vidioc_querybuf = at91sam9x5_video_vidioc_querybuf,
	.vidioc_qbuf = at91sam9x5_video_vidioc_qbuf,
	.vidioc_dqbuf = at91sam9x5_video_vidioc_dqbuf,
	.vidioc_streamon = at91sam9x5_video_vidioc_streamon,
	.vidioc_streamoff = at91sam9x5_video_vidioc_streamoff,
};

static int at91sam9x5_video_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);

	pr_debug("%s\n", __func__);

	/* XXX: allow to open only once? */
	filp->private_data = vdev;

	//vdev->debug |= V4L2_DEBUG_IOCTL_ARG;

	return 0;
}

static int at91sam9x5_video_release(struct file *filp)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int at91sam9x5_video_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(filp);
	struct at91sam9x5_video_priv *priv = video_get_drvdata(vdev);

	pr_debug("%s\n", __func__);

	//return -EIO; // -> results in segfault of gst-launch
	return vb2_mmap(&priv->queue, vma);
}

static struct v4l2_file_operations at91sam9x5_video_fops = {
	.owner = THIS_MODULE,
	.open = at91sam9x5_video_open,
	.release = at91sam9x5_video_release,
	.ioctl = video_ioctl2,
	.mmap = at91sam9x5_video_mmap,
};

static int at91sam9x5_video_register(struct fb_info *fbinfo,
		struct at91sam9x5_video_priv *priv)
{
	int ret = -ENOMEM;
	struct platform_device *pdev = priv->pdev;
	struct resource *res;
	/* XXX: interpret colorspace member of v4l2_pix_format */
	const struct at91sam9x5_video_csc_config *cscc =
		&at91sam9x5_video_csc_config[2];
	const struct at91sam9x5_video_pdata *pdata =
		dev_get_platdata(&pdev->dev);

	dev_dbg(&pdev->dev, "%s: fbinfo=%p\n", __func__, fbinfo);

	/* XXX: this doesn't belong here, does it? */
	priv->pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if (!pdata) {
		dev_err(&pdev->dev, "failed to get platform data\n");
		goto err_get_pdata;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get register base\n");
		goto err_get_regbase;
	}

	priv->regbase = ioremap(res->start, resource_size(res));
	if (!priv->regbase) {
		dev_err(&pdev->dev, "failed to remap register base\n");
		goto err_remap;
	}

	/* XXX: hm, video_device_alloc is just a kzalloc, so embedding struct
	 * video_device into struct at91sam9x5_video_priv would work, too.
	 * Is that allowed?
	 */
	priv->video_dev = video_device_alloc();
	if (!priv->video_dev) {
		dev_err(&pdev->dev, "failed to alloc video device for %p\n",
				fbinfo);
		goto err_video_device_alloc;
	}

	priv->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(priv->alloc_ctx)) {
		ret = PTR_ERR(priv->alloc_ctx);
		dev_err(&pdev->dev, "failed to init alloc_ctx (%d)\n", ret);
		goto err_init_ctx;
	}

	priv->video_dev->fops = &at91sam9x5_video_fops;
	priv->video_dev->ioctl_ops = &at91sam9x5_video_ioctl_ops;
	priv->video_dev->release = video_device_release;

	video_set_drvdata(priv->video_dev, priv);
	priv->fbinfo = fbinfo;
	priv->state = at91sam9x5_video_IDLE;

	/* reset channel and clear status */
	at91sam9x5_video_write32(priv, REG_HEOCHDR, REG_HEOCHDR_CHRST);
	(void)at91sam9x5_video_read32(priv, REG_HEOISR);

	/* Color Space Conversion */
	/* XXX: should go to s_fmt_vid_out */
	at91sam9x5_video_write32(priv, 0x304, cscc->cscr);
	at91sam9x5_video_write32(priv, 0x308, cscc->cscg);
	at91sam9x5_video_write32(priv, 0x30c, cscc->cscb);

	/* set maximal bursting */
	at91sam9x5_video_write32(priv, REG_HEOCFG0,
			REG_HEOCFG0_BLEN_INCR16 |
			REG_HEOCFG0_BLENUV_INCR16);

	/* Initialize 2DSC to full screen */
	priv->xsize = pdata->base_width;
	priv->ysize = pdata->base_height;

	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "failed to get irq from resources (%d)\n",
				ret);
		if (!ret)
			ret = -ENXIO;
		goto err_get_irq;
	}
	priv->irq = ret;

	ret = request_irq(priv->irq, at91sam9x5_video_irq, IRQF_SHARED,
			DRIVER_NAME, priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq (%d)\n", ret);
		goto err_request_irq;
	}

	ret = video_register_device(priv->video_dev,
			/* XXX: ??? */ VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register video device (%d)\n",
				ret);

		free_irq(priv->irq, priv);
 err_request_irq:
 err_get_irq:

		vb2_dma_contig_cleanup_ctx(priv->alloc_ctx);
 err_init_ctx:

		video_device_release(priv->video_dev);
 err_video_device_alloc:

		iounmap(priv->regbase);
	}
 err_remap:
 err_get_regbase:
 err_get_pdata:

	return ret;
}

static int at91sam9x5_video_fb_event_notify(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct at91sam9x5_video_priv *priv = container_of(self,
			struct at91sam9x5_video_priv, fb_notifier);
	struct fb_event *event = data;
	struct fb_info *fbinfo = event->info;

	switch (action) {
	case FB_EVENT_FB_REGISTERED:
		at91sam9x5_video_register(fbinfo, priv);
		break;

	case FB_EVENT_FB_UNREGISTERED:
		/* XXX */
		break;
	}
	return 0;
}

static int __devinit at91sam9x5_video_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	int i;
	struct at91sam9x5_video_priv *priv = kzalloc(sizeof(*priv), GFP_KERNEL);

	if (!priv) {
		dev_err(&pdev->dev, "failed to allocate driver private data\n");
		goto err_alloc_priv;
	}

	priv->pdev = pdev;
	priv->fb_notifier.notifier_call = at91sam9x5_video_fb_event_notify;

	spin_lock_init(&priv->lock);

	platform_set_drvdata(pdev, priv);

	ret = fb_register_client(&priv->fb_notifier);
	if (ret) {
		dev_err(&pdev->dev, "failed to register fb client (%d)\n", ret);
		kfree(priv);

 err_alloc_priv:
		return ret;
	}

	/* XXX: race: if a new fb is registerd here then
	 * at91sam9x5_video_register is called twice. This should be solved
	 * somewhere in drivers/fb. */

	for (i = 0; i < ARRAY_SIZE(registered_fb); ++i)
		if (registered_fb[i])
			at91sam9x5_video_register(registered_fb[i], priv);
	return 0;
}

int __devexit at91sam9x5_video_remove(struct platform_device *pdev)
{
	/* XXX */
	return 0;
}

static struct platform_driver at91sam9x5_video_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = at91sam9x5_video_probe,
	.remove = at91sam9x5_video_remove,
};

static struct platform_device *at91sam9x5_video_device;
static int __init at91sam9x5_video_init(void)
{
	/* XXX: register the device in arch/arm/mach-at91 */
	int ret;
	const struct resource res[] = {
		{
			.start = 0xf8038280,
			.end = 0xf803833f,
			.flags = IORESOURCE_MEM,
		}, {
			.start = 25,
			.end = 25,
			.flags = IORESOURCE_IRQ,
		},
	};
	const struct at91sam9x5_video_pdata pdata = {
		.base_width = 800,
		.base_height = 480,
	};

	ret = platform_driver_register(&at91sam9x5_video_driver);
	if (ret) {
		pr_err("failed to register driver (%d)", ret);
		goto err_driver_register;
	}

	at91sam9x5_video_device = platform_device_register_resndata(NULL,
			DRIVER_NAME, -1,
			res, ARRAY_SIZE(res), &pdata, sizeof(pdata));
	if (IS_ERR(at91sam9x5_video_device)) {
		ret = PTR_ERR(at91sam9x5_video_device);
		pr_err("failed to register device (%d)", ret);
		platform_driver_unregister(&at91sam9x5_video_driver);
	}

 err_driver_register:
	return ret;
}
module_init(at91sam9x5_video_init);

static void __exit at91sam9x5_video_exit(void)
{
	platform_device_unregister(at91sam9x5_video_device);
	platform_driver_unregister(&at91sam9x5_video_driver);
}
module_exit(at91sam9x5_video_exit);

MODULE_AUTHOR("Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>");
MODULE_LICENSE("GPL v2");
