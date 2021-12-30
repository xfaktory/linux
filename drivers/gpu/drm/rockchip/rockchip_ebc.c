// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Samuel Holland <samuel@sholland.org>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/iio/consumer.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_epd_lut.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_simple_kms_helper.h>

#define EBC_DSP_START			0x0000
#define EBC_DSP_START_DSP_OUT_LOW		BIT(31)
#define EBC_DSP_START_DSP_SDCE_WIDTH(x)		((x) << 16)
#define EBC_DSP_START_DSP_EINK_MODE		BIT(13)
#define EBC_DSP_START_SW_BURST_CTRL		BIT(12)
#define EBC_DSP_START_DSP_FRM_TOTAL(x)		((x) << 2)
#define EBC_DSP_START_DSP_RST			BIT(1)
#define EBC_DSP_START_DSP_FRM_START		BIT(0)
#define EBC_EPD_CTRL			0x0004
#define EBC_EPD_CTRL_EINK_MODE_SWAP		BIT(31)
#define EBC_EPD_CTRL_DSP_GD_END(x)		((x) << 16)
#define EBC_EPD_CTRL_DSP_GD_ST(x)		((x) << 8)
#define EBC_EPD_CTRL_DSP_THREE_WIN_MODE		BIT(7)
#define EBC_EPD_CTRL_DSP_SDDW_MODE		BIT(6)
#define EBC_EPD_CTRL_EPD_AUO			BIT(5)
#define EBC_EPD_CTRL_EPD_PWR(x)			((x) << 2)
#define EBC_EPD_CTRL_EPD_GDRL			BIT(1)
#define EBC_EPD_CTRL_EPD_SDSHR			BIT(0)
#define EBC_DSP_CTRL			0x0008
#define EBC_DSP_CTRL_DSP_SWAP_MODE(x)		((x) << 30)
#define EBC_DSP_CTRL_DSP_DIFF_MODE		BIT(29)
#define EBC_DSP_CTRL_DSP_LUT_MODE		BIT(28)
#define EBC_DSP_CTRL_DSP_VCOM_MODE		BIT(27)
#define EBC_DSP_CTRL_DSP_GDOE_POL		BIT(26)
#define EBC_DSP_CTRL_DSP_GDSP_POL		BIT(25)
#define EBC_DSP_CTRL_DSP_GDCLK_POL		BIT(24)
#define EBC_DSP_CTRL_DSP_SDCE_POL		BIT(23)
#define EBC_DSP_CTRL_DSP_SDOE_POL		BIT(22)
#define EBC_DSP_CTRL_DSP_SDLE_POL		BIT(21)
#define EBC_DSP_CTRL_DSP_SDCLK_POL		BIT(20)
#define EBC_DSP_CTRL_DSP_SDCLK_DIV(x)		((x) << 16)
#define EBC_DSP_CTRL_DSP_BACKGROUND(x)		((x) << 0)
#define EBC_DSP_HTIMING0		0x000c
#define EBC_DSP_HTIMING0_DSP_HTOTAL(x)		((x) << 16)
#define EBC_DSP_HTIMING0_DSP_HS_END(x)		((x) << 0)
#define EBC_DSP_HTIMING1		0x0010
#define EBC_DSP_HTIMING1_DSP_HACT_END(x)	((x) << 16)
#define EBC_DSP_HTIMING1_DSP_HACT_ST(x)		((x) << 0)
#define EBC_DSP_VTIMING0		0x0014
#define EBC_DSP_VTIMING0_DSP_VTOTAL(x)		((x) << 16)
#define EBC_DSP_VTIMING0_DSP_VS_END(x)		((x) << 0)
#define EBC_DSP_VTIMING1		0x0018
#define EBC_DSP_VTIMING1_DSP_VACT_END(x)	((x) << 16)
#define EBC_DSP_VTIMING1_DSP_VACT_ST(x)		((x) << 0)
#define EBC_DSP_ACT_INFO		0x001c
#define EBC_DSP_ACT_INFO_DSP_HEIGHT(x)		((x) << 16)
#define EBC_DSP_ACT_INFO_DSP_WIDTH(x)		((x) << 0)
#define EBC_WIN_CTRL			0x0020
#define EBC_WIN_CTRL_WIN2_FIFO_THRESHOLD(x)	((x) << 19)
#define EBC_WIN_CTRL_WIN_EN			BIT(18)
#define EBC_WIN_CTRL_AHB_INCR_NUM_REG(x)	((x) << 13)
#define EBC_WIN_CTRL_AHB_BURST_REG(x)		((x) << 10)
#define EBC_WIN_CTRL_WIN_FIFO_THRESHOLD(x)	((x) << 2)
#define EBC_WIN_CTRL_WIN_FMT(x)			((x) << 0)
#define EBC_WIN_MST0			0x0024
#define EBC_WIN_MST1			0x0028
#define EBC_WIN_VIR			0x002c
#define EBC_WIN_VIR_WIN_VIR_HEIGHT(x)		((x) << 16)
#define EBC_WIN_VIR_WIN_VIR_WIDTH(x)		((x) << 0)
#define EBC_WIN_ACT			0x0030
#define EBC_WIN_ACT_WIN_ACT_HEIGHT(x)		((x) << 16)
#define EBC_WIN_ACT_WIN_ACT_WIDTH(x)		((x) << 0)
#define EBC_WIN_DSP			0x0034
#define EBC_WIN_DSP_WIN_DSP_HEIGHT(x)		((x) << 16)
#define EBC_WIN_DSP_WIN_DSP_WIDTH(x)		((x) << 0)
#define EBC_WIN_DSP_ST			0x0038
#define EBC_WIN_DSP_ST_WIN_DSP_YST(x)		((x) << 16)
#define EBC_WIN_DSP_ST_WIN_DSP_XST(x)		((x) << 0)
#define EBC_INT_STATUS			0x003c
#define EBC_INT_STATUS_DSP_FRM_INT_NUM(x)	((x) << 12)
#define EBC_INT_STATUS_LINE_FLAG_INT_CLR	BIT(11)
#define EBC_INT_STATUS_DSP_FRM_INT_CLR		BIT(10)
#define EBC_INT_STATUS_DSP_END_INT_CLR		BIT(9)
#define EBC_INT_STATUS_FRM_END_INT_CLR		BIT(8)
#define EBC_INT_STATUS_LINE_FLAG_INT_MSK	BIT(7)
#define EBC_INT_STATUS_DSP_FRM_INT_MSK		BIT(6)
#define EBC_INT_STATUS_DSP_END_INT_MSK		BIT(5)
#define EBC_INT_STATUS_FRM_END_INT_MSK		BIT(4)
#define EBC_INT_STATUS_LINE_FLAG_INT_ST		BIT(3)
#define EBC_INT_STATUS_DSP_FRM_INT_ST		BIT(2)
#define EBC_INT_STATUS_DSP_END_INT_ST		BIT(1)
#define EBC_INT_STATUS_FRM_END_INT_ST		BIT(0)
#define EBC_VCOM0			0x0040
#define EBC_VCOM1			0x0044
#define EBC_VCOM2			0x0048
#define EBC_VCOM3			0x004c
#define EBC_CONFIG_DONE			0x0050
#define EBC_CONFIG_DONE_REG_CONFIG_DONE		BIT(0)
#define EBC_VNUM			0x0054
#define EBC_VNUM_DSP_VCNT(x)			((x) << 16)
#define EBC_VNUM_LINE_FLAG_NUM(x)		((x) << 0)
#define EBC_WIN_MST2			0x0058
#define EBC_LUT_DATA			0x1000
#define EBC_LUT_DATA_SIZE			0x4000

#define EBC_LUT_REGS			(EBC_LUT_DATA_SIZE / 4)
#define EBC_MAX_PHASES			256
#define EBC_NUM_SUPPLIES		3

enum rockchip_ebc_state {
	EBC_STATE_RESET,
	EBC_STATE_ON,
	EBC_STATE_CLEAR,
	EBC_STATE_OFF,
};

struct rockchip_ebc {
	struct clk			*dclk;
	struct clk			*hclk;
	struct completion		update_complete;
	struct drm_device		drm;
	struct drm_display_mode		mode;
	struct drm_epd_lut		lut;
	struct drm_simple_display_pipe	pipe;
	struct iio_channel		*temperature_channel;
	struct regmap			*regmap;
	struct regulator_bulk_data	supplies[EBC_NUM_SUPPLIES];
	u8				*count_buffer;
	u8				*prev_buffer;
	u8				*next_buffer;
	enum rockchip_ebc_state		state;
	u32				dsp_start;
};

static inline struct rockchip_ebc *
pipe_to_ebc(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct rockchip_ebc, pipe);
}

static inline unsigned int ebc_read(struct rockchip_ebc *ebc, unsigned int reg)
{
	unsigned int value;

	regmap_read(ebc->regmap, reg, &value);

	return value;
}

static inline void ebc_write(struct rockchip_ebc *ebc, unsigned int reg,
			      unsigned int value)
{
	regmap_write(ebc->regmap, reg, value);
}

static void rockchip_ebc_refresh(struct rockchip_ebc *ebc,
				 enum drm_epd_lut_waveform waveform)
{
	unsigned long res, timeout = msecs_to_jiffies(2000);
	bool three_win_mode = waveform == DRM_EPD_WF_GC16;
	struct drm_display_mode mode = ebc->mode;
	u8 last_phase = ebc->lut.num_phases - 1;
	u32 px = mode.hdisplay * mode.vdisplay;
	struct device *dev = ebc->drm.dev;
	struct drm_display_mode sdck;
	u16 hact_start, vact_start;
	u16 pixels_per_sdck;
	bool bus_16bit;
	u32 i, j;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to resume: %d\n", ret);
		return;
	}

	ret = drm_epd_lut_set_waveform(&ebc->lut, waveform);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set waveform to %d: %d\n",
			      waveform, ret);
	} else if (ret) {
		DRM_DEV_INFO(dev, "Writing new LUT\n");

		regmap_bulk_write(ebc->regmap, EBC_LUT_DATA,
				  ebc->lut.lut, EBC_LUT_REGS);
	}

	DRM_DEV_INFO(dev, "Starting update, mode=" DRM_MODE_FMT " wf=%d ph=%d\n",
		     DRM_MODE_ARG(&mode), waveform, ebc->lut.num_phases);

	/*
	 * Hardware needs horizontal timing values in SDCK == source driver
	 * clock cycles, not pixels. Bus width is either 8 (normal) or 16
	 * (DRM_MODE_FLAG_CLKDIV2), and each pixel uses two data bits.
	 */
	bus_16bit = !!(mode.flags & DRM_MODE_FLAG_CLKDIV2);
	pixels_per_sdck = bus_16bit ? 8 : 4;
	sdck.hdisplay = mode.hdisplay / pixels_per_sdck;
	sdck.hsync_start = mode.hsync_start / pixels_per_sdck;
	sdck.hsync_end = mode.hsync_end / pixels_per_sdck;
	sdck.htotal = mode.htotal / pixels_per_sdck;
	sdck.hskew = mode.hskew / pixels_per_sdck;

	/*
	 * Linux timing order is display/fp/sync/bp. Hardware timing
	 * order is sync/bp/display/fp, aka sync/start/display/end.
	 */
	hact_start = sdck.htotal - sdck.hsync_start;
	vact_start = mode.vtotal - mode.vsync_start;

	ebc->dsp_start = EBC_DSP_START_DSP_SDCE_WIDTH(sdck.hdisplay) |
			 EBC_DSP_START_SW_BURST_CTRL;
	ebc_write(ebc, EBC_DSP_START,
		  ebc->dsp_start);
	ebc_write(ebc, EBC_EPD_CTRL,
		  EBC_EPD_CTRL_DSP_GD_END(hact_start + sdck.hdisplay - sdck.hskew) |
		  EBC_EPD_CTRL_DSP_GD_ST(hact_start + sdck.hskew) |
		  EBC_EPD_CTRL_DSP_THREE_WIN_MODE * three_win_mode |
		  EBC_EPD_CTRL_DSP_SDDW_MODE * bus_16bit);
	ebc_write(ebc, EBC_DSP_CTRL,
		  EBC_DSP_CTRL_DSP_SWAP_MODE(bus_16bit ? 2 : 3) |
		  EBC_DSP_CTRL_DSP_DIFF_MODE |
		  EBC_DSP_CTRL_DSP_LUT_MODE |
		  EBC_DSP_CTRL_DSP_SDCLK_DIV(pixels_per_sdck - 1));
	ebc_write(ebc, EBC_DSP_HTIMING0,
		  EBC_DSP_HTIMING0_DSP_HTOTAL(sdck.htotal) |
		  EBC_DSP_HTIMING0_DSP_HS_END(sdck.hsync_end - sdck.hsync_start));
	ebc_write(ebc, EBC_DSP_HTIMING1,
		  EBC_DSP_HTIMING1_DSP_HACT_END(hact_start + sdck.hdisplay) |
		  EBC_DSP_HTIMING1_DSP_HACT_ST(hact_start - 1));
	ebc_write(ebc, EBC_DSP_VTIMING0,
		  EBC_DSP_VTIMING0_DSP_VTOTAL(mode.vtotal) |
		  EBC_DSP_VTIMING0_DSP_VS_END(mode.vsync_end - mode.vsync_start));
	ebc_write(ebc, EBC_DSP_VTIMING1,
		  EBC_DSP_VTIMING1_DSP_VACT_END(vact_start + mode.vdisplay) |
		  EBC_DSP_VTIMING1_DSP_VACT_ST(vact_start));
	ebc_write(ebc, EBC_DSP_ACT_INFO,
		  EBC_DSP_ACT_INFO_DSP_HEIGHT(mode.vdisplay) |
		  EBC_DSP_ACT_INFO_DSP_WIDTH(mode.hdisplay));
	ebc_write(ebc, EBC_WIN_CTRL,
		  EBC_WIN_CTRL_WIN2_FIFO_THRESHOLD(496) | /* FIFO depth - 16 */
		  EBC_WIN_CTRL_WIN_EN |
		  EBC_WIN_CTRL_AHB_BURST_REG(7) | /* INCR16 */
		  EBC_WIN_CTRL_WIN_FIFO_THRESHOLD(240) | /* FIFO depth - 16 */
		  EBC_WIN_CTRL_WIN_FMT(1)); /* Y8 */
	ebc_write(ebc, EBC_WIN_MST0, virt_to_phys(ebc->next_buffer));
	ebc_write(ebc, EBC_WIN_MST1, virt_to_phys(ebc->prev_buffer));
	ebc_write(ebc, EBC_WIN_VIR,
		  EBC_WIN_VIR_WIN_VIR_HEIGHT(mode.vdisplay) |
		  EBC_WIN_VIR_WIN_VIR_WIDTH(mode.hdisplay));
	ebc_write(ebc, EBC_WIN_ACT,
		  EBC_WIN_ACT_WIN_ACT_HEIGHT(mode.vdisplay) |
		  EBC_WIN_ACT_WIN_ACT_WIDTH(mode.hdisplay));
	ebc_write(ebc, EBC_WIN_DSP,
		  EBC_WIN_DSP_WIN_DSP_HEIGHT(mode.vdisplay) |
		  EBC_WIN_DSP_WIN_DSP_WIDTH(mode.hdisplay));
	ebc_write(ebc, EBC_WIN_DSP_ST,
		  EBC_WIN_DSP_ST_WIN_DSP_YST(vact_start) |
		  EBC_WIN_DSP_ST_WIN_DSP_XST(hact_start));
	ebc_write(ebc, EBC_INT_STATUS,
		  EBC_INT_STATUS_DSP_FRM_INT_NUM(ebc->lut.num_phases - 1) |
		  EBC_INT_STATUS_DSP_FRM_INT_CLR |
		  EBC_INT_STATUS_DSP_END_INT_CLR |
		  EBC_INT_STATUS_LINE_FLAG_INT_MSK |
		  EBC_INT_STATUS_FRM_END_INT_MSK);
	ebc_write(ebc, EBC_WIN_MST2, virt_to_phys(ebc->count_buffer));

	ebc_write(ebc, EBC_CONFIG_DONE,
		  EBC_CONFIG_DONE_REG_CONFIG_DONE);

	for (i = 0; i <= last_phase; ++i) {
		for (j = 0; j < mode.htotal * mode.vtotal / 2; j += 2) {
			ebc->count_buffer[j + 0] = i;
			ebc->count_buffer[j + 1] = last_phase;
		}
		dma_sync_single_for_device(dev, virt_to_phys(ebc->count_buffer),
					   mode.htotal * mode.vtotal, DMA_TO_DEVICE);
		dma_sync_single_for_device(dev, virt_to_phys(ebc->prev_buffer),
					   px, DMA_TO_DEVICE);
		dma_sync_single_for_device(dev, virt_to_phys(ebc->next_buffer),
					   px, DMA_TO_DEVICE);
		// pr_info("b count: %*ph\n", 16, ebc->count_buffer);
		// pr_info("b  prev: %*ph\n", 16, ebc->prev_buffer);
		// pr_info("b  next: %*ph\n", 16, ebc->next_buffer);
		reinit_completion(&ebc->update_complete);
		ebc_write(ebc, EBC_DSP_START,
			  ebc->dsp_start |
			  EBC_DSP_START_DSP_FRM_TOTAL(last_phase) * !three_win_mode |
			  EBC_DSP_START_DSP_FRM_START);
		res = wait_for_completion_timeout(&ebc->update_complete, timeout);
		// pr_info("a count: %*ph\n", 16, ebc->count_buffer);
		// pr_info("a  prev: %*ph\n", 16, ebc->prev_buffer);
		// pr_info("a  next: %*ph\n", 16, ebc->next_buffer);
		if (!three_win_mode)
			break;
	}
	memcpy(ebc->prev_buffer, ebc->next_buffer, px);

	DRM_DEV_INFO(dev, "Finished refresh, %d us/phase\n",
		     jiffies_to_usecs(timeout - res) / ebc->lut.num_phases);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}

static irqreturn_t rockchip_ebc_irq(int irq, void *dev_id)
{
	struct rockchip_ebc *ebc = dev_id;
	struct device *dev = ebc->drm.dev;
	unsigned int status = ebc_read(ebc, EBC_INT_STATUS);

	if (status & EBC_INT_STATUS_DSP_FRM_INT_ST) {
		DRM_DEV_INFO(dev, "VSYNC! 0x%08x\n", status);
		status |= EBC_INT_STATUS_DSP_FRM_INT_CLR;
	}
	if (status & EBC_INT_STATUS_DSP_END_INT_ST) {
		status |= EBC_INT_STATUS_DSP_END_INT_CLR;
		complete(&ebc->update_complete);
	}

	ebc_write(ebc, EBC_INT_STATUS, status);

	return IRQ_HANDLED;
}

DEFINE_DRM_GEM_FOPS(rockchip_ebc_fops);

static const struct drm_driver rockchip_ebc_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &rockchip_ebc_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name			= "rockchip_ebc",
	.desc			= "Rockchip E-Book Controller",
	.date			= "20220202",
	.major			= 0,
	.minor			= 2,
};

static const struct drm_mode_config_funcs rockchip_ebc_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create_with_dirty,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void rockchip_ebc_cleanup(struct rockchip_ebc *ebc)
{
	kfree(ebc->count_buffer);
	kfree(ebc->prev_buffer);
	kfree(ebc->next_buffer);
	ebc->count_buffer = ebc->prev_buffer = ebc->next_buffer = NULL;
}

static int rockchip_ebc_pipe_check(struct drm_simple_display_pipe *pipe,
				   struct drm_plane_state *plane_state,
				   struct drm_crtc_state *crtc_state)
{
	u32 px = crtc_state->mode.hdisplay * crtc_state->mode.vdisplay;
	struct drm_plane_state *old_plane_state = pipe->plane.state;
	struct drm_crtc_state *old_crtc_state = pipe->crtc.state;
	struct rockchip_ebc *ebc = pipe_to_ebc(pipe);
	struct device *dev = ebc->drm.dev;
	u8 *count_buffer;
	u8 *prev_buffer;
	u8 *next_buffer;
	long ret;

	DRM_DEV_INFO(dev, "Atomic Check: e:%d->%d a:%d->%d v:%d->%d cc:%d mc:%d\n",
		     old_crtc_state->enable, crtc_state->enable,
		     old_crtc_state->active, crtc_state->active,
		     old_plane_state->visible, plane_state->visible,
		     crtc_state->connectors_changed, crtc_state->mode_changed);

	if (ebc->state == EBC_STATE_ON && !crtc_state->mode_changed)
		return 0;

	DRM_DEV_INFO(dev, "Acquiring resources...\n");

	ret = clk_round_rate(ebc->dclk, crtc_state->mode.clock * 1000);
	if (ret < 0)
		return ret;

	// FIXME this isn't atomic!
	count_buffer = kzalloc(crtc_state->mode.htotal * crtc_state->mode.vtotal, GFP_KERNEL);
	prev_buffer = kmalloc(px, GFP_KERNEL);
	next_buffer = kmalloc(px, GFP_KERNEL);
	if (!count_buffer || !prev_buffer || !next_buffer) {
		DRM_DEV_ERROR(dev, "Failed to allocate buffers\n");

		kfree(count_buffer);
		kfree(prev_buffer);
		kfree(next_buffer);

		return -ENOMEM;
	}

	if (old_crtc_state->enable && crtc_state->enable)
		rockchip_ebc_cleanup(ebc);
	ebc->count_buffer = count_buffer;
	ebc->prev_buffer = prev_buffer;
	ebc->next_buffer = next_buffer;

	ebc->state = EBC_STATE_RESET;

	return 0;
}

static bool rockchip_ebc_blit(struct rockchip_ebc *ebc, const void *src,
			      const struct drm_framebuffer *fb,
			      const struct drm_rect *clip)
{
	unsigned int cnt_pitch = ebc->mode.htotal / 2;
	unsigned int dst_pitch = ebc->mode.hdisplay;
	unsigned int src_pitch = fb->pitches[0];
	struct device *dev = ebc->drm.dev;
	void *cnt = ebc->count_buffer;
	void *dst = ebc->next_buffer;
	u8 damage = false;
	unsigned int x, y;

	cnt += clip->y1 * cnt_pitch;
	dst += clip->y1 * dst_pitch + clip->x1;
	src += clip->y1 * src_pitch + clip->x1 * fb->format->cpp[0];
	for (y = clip->y1; y < clip->y2; y++) {
		const u32 *src32 = src;
		u8 *cnt8 = cnt;
		u8 *dst8 = dst;

		for (x = clip->x1; x < clip->x2; x++) {
			u32 rgb = *src32++;
			u8 gray;

			/* Truncate the RGB values to 5 bits each. */
			rgb &= 0x00f8f8f8U;
			/* Put 2R+5G+B in bits 24-31. */
			rgb *= 0x0020a040U;
			/* Shift the grayscale value down to one byte. */
			gray = rgb >> 24;

			if (gray != *dst8) {
				damage = true;
				cnt8[x/2] = 0;
				*dst8 = gray;
			}
			dst8++;
		}

		cnt += cnt_pitch;
		dst += dst_pitch;
		src += src_pitch;
	}

	DRM_DEV_INFO(dev, "Blit " DRM_RECT_FMT " %s damage\n",
		     DRM_RECT_ARG(clip), damage ? "found" : "no");

	return damage;
}

static void rockchip_ebc_pipe_update(struct drm_simple_display_pipe *pipe,
				     struct drm_plane_state *old_plane_state)
{
	struct drm_plane_state *plane_state = pipe->plane.state;
	struct drm_crtc_state *crtc_state = pipe->crtc.state;
	struct drm_framebuffer *fb = plane_state->fb;
	struct rockchip_ebc *ebc = pipe_to_ebc(pipe);
	struct drm_display_mode *mode = &ebc->mode;
	struct drm_atomic_helper_damage_iter iter;
	struct device *dev = ebc->drm.dev;
	struct dma_buf_map *map;
	struct drm_rect clip;
	bool damage = false;
	u32 px;

	DRM_DEV_INFO(dev, "Plane Update: e:X->%d a:X->%d v:%d->%d state:%d\n",
		     crtc_state->enable, crtc_state->active,
		     old_plane_state->visible, plane_state->visible,
		     ebc->state);

	if (ebc->state == EBC_STATE_ON && !crtc_state->enable)
		ebc->state = EBC_STATE_CLEAR;

	switch(ebc->state) {
	case EBC_STATE_RESET:
		DRM_DEV_INFO(dev, "Plane Update in RESET\n");

		*mode = crtc_state->mode;
		clk_set_rate(ebc->dclk, mode->clock * 1000);

		/* The RESET waveform ends with all white pixels. */
		px = mode->hdisplay * mode->vdisplay;
		memset(ebc->count_buffer, 0, mode->htotal * mode->vtotal);
		memset(ebc->prev_buffer, 0x00, px);
		memset(ebc->next_buffer, 0xff, px);
		rockchip_ebc_refresh(ebc, DRM_EPD_WF_RESET);

		ebc->state = EBC_STATE_ON;
		fallthrough;
	case EBC_STATE_ON:
		map = to_drm_shadow_plane_state(plane_state)->data;
		drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
		drm_atomic_for_each_plane_damage(&iter, &clip)
			damage |= rockchip_ebc_blit(ebc, map->vaddr, fb, &clip);
		if (damage)
			rockchip_ebc_refresh(ebc, DRM_EPD_WF_GC16);

		break;
	case EBC_STATE_CLEAR:
		DRM_DEV_INFO(dev, "Plane Update in CLEAR\n");

		px = mode->hdisplay * mode->vdisplay;
		memset(ebc->count_buffer, 0, px);
		memset(ebc->next_buffer, 0xff, px);
		rockchip_ebc_refresh(ebc, DRM_EPD_WF_DU);
		rockchip_ebc_cleanup(ebc);

		ebc->state = EBC_STATE_OFF;
		break;
	case EBC_STATE_OFF:
		break;
	}
}

static const struct drm_simple_display_pipe_funcs rockchip_ebc_pipe_funcs = {
	.check		= rockchip_ebc_pipe_check,
	.update		= rockchip_ebc_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const u32 rockchip_ebc_formats[] = {
	// DRM_FORMAT_R8,
	DRM_FORMAT_XRGB8888,
};

static const u64 rockchip_ebc_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static const struct regmap_config rockchip_ebc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= EBC_LUT_DATA + EBC_LUT_DATA_SIZE - 4,
};

static const char *const rockchip_ebc_supplies[EBC_NUM_SUPPLIES] = {
	"panel",
	"vcom",
	"vdrive",
};

static int rockchip_ebc_runtime_suspend(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);

	DRM_DEV_INFO(dev, "Suspending...\n");

	/* Ensure frame start is not set, and drive the output pins low. */
	ebc_write(ebc, EBC_DSP_START,
		  ebc->dsp_start | EBC_DSP_START_DSP_OUT_LOW);

	clk_disable_unprepare(ebc->dclk);
	clk_disable_unprepare(ebc->hclk);
	regulator_bulk_disable(EBC_NUM_SUPPLIES, ebc->supplies);

	return 0;
}

static int rockchip_ebc_runtime_resume(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);
	int ret;

	DRM_DEV_INFO(dev, "Resuming...\n");

	ret = regulator_bulk_enable(EBC_NUM_SUPPLIES, ebc->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ebc->hclk);
	if (ret)
		goto err_disable_supplies;

	ret = clk_prepare_enable(ebc->dclk);
	if (ret)
		goto err_disable_hclk;

	/* Load the LUT data. */
	regmap_bulk_write(ebc->regmap, EBC_LUT_DATA,
			  ebc->lut.lut, EBC_LUT_REGS);

	/* Return the pins to normal operation. */
	ebc_write(ebc, EBC_DSP_START,
		  ebc->dsp_start);

	return 0;

err_disable_hclk:
	clk_disable_unprepare(ebc->hclk);
err_disable_supplies:
	regulator_bulk_disable(EBC_NUM_SUPPLIES, ebc->supplies);

	return ret;
}

static int rockchip_ebc_suspend(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&ebc->drm);
}

static int rockchip_ebc_resume(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&ebc->drm);
}

static const struct dev_pm_ops rockchip_ebc_pm_ops = {
	SET_RUNTIME_PM_OPS(rockchip_ebc_runtime_suspend,
			   rockchip_ebc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(rockchip_ebc_suspend, rockchip_ebc_resume)
};

static int rockchip_ebc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct rockchip_ebc *ebc;
	struct drm_device *drm;
	void __iomem *base;
	int i, ret;

	ebc = devm_drm_dev_alloc(dev, &rockchip_ebc_drm_driver,
				 struct rockchip_ebc, drm);
	if (IS_ERR(ebc))
		return PTR_ERR(ebc);

	platform_set_drvdata(pdev, ebc);
	init_completion(&ebc->update_complete);

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, ret, "failed to get bridge\n");

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ebc->regmap = devm_regmap_init_mmio(dev, base, &rockchip_ebc_regmap_config);
	if (IS_ERR(ebc->regmap))
		return PTR_ERR(ebc->regmap);

	ebc->dclk = devm_clk_get(dev, "dclk");
	if (IS_ERR(ebc->dclk))
		return dev_err_probe(dev, PTR_ERR(ebc->dclk),
				     "failed to get dclk\n");

	ebc->hclk = devm_clk_get(dev, "hclk");
	if (IS_ERR(ebc->hclk))
		return dev_err_probe(dev, PTR_ERR(ebc->hclk),
				     "failed to get hclk\n");

	ebc->temperature_channel = devm_iio_channel_get(dev, NULL);
	if (IS_ERR(ebc->temperature_channel))
		return dev_err_probe(dev, PTR_ERR(ebc->temperature_channel),
				     "failed to get temperature I/O channel\n");

	ret = devm_request_irq(dev, platform_get_irq(pdev, 0),
			       rockchip_ebc_irq, 0, dev_name(dev), ebc);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	for (i = 0; i < EBC_NUM_SUPPLIES; ++i)
		ebc->supplies[i].supply = rockchip_ebc_supplies[i];

	ret = devm_regulator_bulk_get(dev, EBC_NUM_SUPPLIES, ebc->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	pm_runtime_set_autosuspend_delay(dev, 10000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	drm = &ebc->drm;
	ret = drmm_epd_lut_init(drm, &ebc->lut, "waveform.bin",
				DRM_EPD_LUT_4BIT_PACKED, EBC_MAX_PHASES);
	if (ret)
		goto err_disable_pm;

	ret = drmm_mode_config_init(drm);
	if (ret)
		goto err_disable_pm;

	drm->mode_config.max_width = DRM_SHADOW_PLANE_MAX_WIDTH;
	drm->mode_config.max_height = DRM_SHADOW_PLANE_MAX_HEIGHT;
	drm->mode_config.funcs = &rockchip_ebc_mode_config_funcs;

	ret = drm_simple_display_pipe_init(drm, &ebc->pipe,
					   &rockchip_ebc_pipe_funcs,
					   rockchip_ebc_formats,
					   ARRAY_SIZE(rockchip_ebc_formats),
					   rockchip_ebc_format_modifiers,
					   NULL);
	if (ret)
		goto err_disable_pm;

	ret = drm_simple_display_pipe_attach_bridge(&ebc->pipe, bridge);
	if (ret)
		goto err_disable_pm;

	drm_plane_enable_fb_damage_clips(&ebc->pipe.plane);
	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_disable_pm;

	drm_fbdev_generic_setup(drm, 0);

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		rockchip_ebc_runtime_suspend(dev);

	return ret;
}

static int rockchip_ebc_remove(struct platform_device *pdev)
{
	struct rockchip_ebc *ebc = platform_get_drvdata(pdev);
	struct drm_device *drm = &ebc->drm;
	struct device *dev = &pdev->dev;

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		rockchip_ebc_runtime_suspend(dev);

	return 0;
}

static void rockchip_ebc_shutdown(struct platform_device *pdev)
{
	struct rockchip_ebc *ebc = platform_get_drvdata(pdev);
	struct drm_device *drm = &ebc->drm;
	struct device *dev = &pdev->dev;

	drm_atomic_helper_shutdown(drm);

	if (!pm_runtime_status_suspended(dev))
		rockchip_ebc_runtime_suspend(dev);
}

static const struct of_device_id rockchip_ebc_of_match[] = {
	{ .compatible = "rockchip,rk3568-ebc-tcon" },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_ebc_of_match);

static struct platform_driver rockchip_ebc_driver = {
	.probe		= rockchip_ebc_probe,
	.remove		= rockchip_ebc_remove,
	.shutdown	= rockchip_ebc_shutdown,
	.driver		= {
		.name		= "rockchip-ebc",
		.of_match_table	= rockchip_ebc_of_match,
		.pm		= &rockchip_ebc_pm_ops,
	},
};
module_platform_driver(rockchip_ebc_driver);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("Rockchip EBC driver");
MODULE_FIRMWARE("waveform.bin");
MODULE_LICENSE("GPL v2");
