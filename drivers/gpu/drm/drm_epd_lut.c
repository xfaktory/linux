// SPDX-License-Identifier: GPL-2.0

#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/vmalloc.h>

#include <drm/drm_device.h>
#include <drm/drm_epd_lut.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>

struct pvi_wf_offset {
	u8			b[3];
};

struct pvi_wf_pointer {
	struct pvi_wf_offset	offset;
	u8			checksum;
};

struct pvi_wf_file_header {
	__le32			checksum;		// 0x00
	__le32			file_size;		// 0x04
	__le32			serial;			// 0x08
	u8			run_type;		// 0x0c
	u8			fpl_platform;		// 0x0d
	__le16			fpl_lot;		// 0x0e
	u8			mode_version;		// 0x10
	u8			wf_version;		// 0x11
	u8			wf_subversion;		// 0x12
	u8			wf_type;		// 0x13
	u8			panel_size;		// 0x14
	u8			amepd_part_number;	// 0x15
	u8			wf_rev;			// 0x16
	u8			frame_rate_bcd;		// 0x17
	u8			frame_rate_hex;		// 0x18
	u8			vcom_offset;		// 0x19
	u8			unknown[2];		// 0x1a
	struct pvi_wf_offset	xwia;			// 0x1c
	u8			cs1;			// 0x1f
	struct pvi_wf_offset	wmta;			// 0x20
	u8			fvsn;			// 0x23
	u8			luts;			// 0x24
	u8			mode_count;		// 0x25
	u8			temp_range_count;	// 0x26
	u8			advanced_wf_flags;	// 0x27
	u8			eb;			// 0x28
	u8			sb;			// 0x29
	u8			reserved[5];		// 0x2a
	u8			cs2;			// 0x2f
	u8			temp_range_table[];	// 0x30
};
static_assert(sizeof(struct pvi_wf_file_header) == 0x30);

struct pvi_wf_mode_info {
	u8			versions[2];
	u8			format;
	u8			modes[DRM_EPD_WF_MAX];
};

static const struct pvi_wf_mode_info pvi_wf_mode_info_table[] = {
	{
		.versions = {
			0x09,
		},
		.format = DRM_EPD_LUT_4BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 1,
			[DRM_EPD_WF_GC16]	= 2,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 3,
			[DRM_EPD_WF_GLD16]	= 3,
			[DRM_EPD_WF_A2]		= 4,
			[DRM_EPD_WF_GCC16]	= 3,
		},
	},
	{
		.versions = {
			0x12,
		},
		.format = DRM_EPD_LUT_4BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 7,
			[DRM_EPD_WF_GC16]	= 3,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 5,
			[DRM_EPD_WF_GLD16]	= 6,
			[DRM_EPD_WF_A2]		= 4,
			[DRM_EPD_WF_GCC16]	= 5,
		},
	},
	{
		.versions = {
			0x16,
		},
		.format = DRM_EPD_LUT_5BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 1,
			[DRM_EPD_WF_GC16]	= 2,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 4,
			[DRM_EPD_WF_GLD16]	= 4,
			[DRM_EPD_WF_A2]		= 6,
			[DRM_EPD_WF_GCC16]	= 5,
		},
	},
	{
		.versions = {
			0x18,
			0x20,
		},
		.format = DRM_EPD_LUT_5BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 1,
			[DRM_EPD_WF_GC16]	= 2,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 4,
			[DRM_EPD_WF_GLD16]	= 5,
			[DRM_EPD_WF_A2]		= 6,
			[DRM_EPD_WF_GCC16]	= 4,
		},
	},
	{
		.versions = {
			0x19,
			0x43,
		},
		.format = DRM_EPD_LUT_5BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 7,
			[DRM_EPD_WF_GC16]	= 2,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 4,
			[DRM_EPD_WF_GLD16]	= 5,
			[DRM_EPD_WF_A2]		= 6,
			[DRM_EPD_WF_GCC16]	= 4,
		},
	},
	{
		.versions = {
			0x23,
		},
		.format = DRM_EPD_LUT_4BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 5,
			[DRM_EPD_WF_GC16]	= 2,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 3,
			[DRM_EPD_WF_GLD16]	= 3,
			[DRM_EPD_WF_A2]		= 4,
			[DRM_EPD_WF_GCC16]	= 3,
		},
	},
	{
		.versions = {
			0x54,
		},
		.format = DRM_EPD_LUT_4BIT_PACKED,
		.modes = {
			[DRM_EPD_WF_RESET]	= 0,
			[DRM_EPD_WF_DU]		= 1,
			[DRM_EPD_WF_DU4]	= 1,
			[DRM_EPD_WF_GC16]	= 2,
			[DRM_EPD_WF_GL16]	= 3,
			[DRM_EPD_WF_GLR16]	= 4,
			[DRM_EPD_WF_GLD16]	= 4,
			[DRM_EPD_WF_A2]		= 5,
			[DRM_EPD_WF_GCC16]	= 4,
		},
	},
};

static int pvi_wf_get_mode_info(struct drm_epd_lut *lut)
{
	const struct pvi_wf_file_header *header = (const void *)lut->fw->data;
	const struct pvi_wf_mode_info *mode_info;
	int i, v;

	for (i = 0; i < ARRAY_SIZE(pvi_wf_mode_info_table); i++) {
		mode_info = &pvi_wf_mode_info_table[i];

		for (v = 0; v < ARRAY_SIZE(mode_info->versions); v++) {
			if (header->mode_version == mode_info->versions[v]) {
				lut->mode_info = mode_info;
				return 0;
			}
		}
	}

	return -ENOTSUPP;
}

static int pvi_wf_validate_header(struct drm_epd_lut *lut)
{
	const struct pvi_wf_file_header *header = (const void *)lut->fw->data;
	int ret;

	if (le32_to_cpu(header->file_size) > lut->fw->size)
		return -EBADF;

	ret = pvi_wf_get_mode_info(lut);
	if (ret)
		return ret;

	DRM_DEV_INFO(lut->dev, "Found PVI waveform, version=0x%02x\n",
		     header->mode_version);

	return 0;
}

static const void *pvi_wf_apply_offset(struct drm_epd_lut *lut,
				       const struct pvi_wf_offset *offset)
{
	u32 bytes = offset->b[0] | offset->b[1] << 8 | offset->b[2] << 16;

	if (bytes >= lut->fw->size)
		return NULL;

	return lut->fw->data + bytes;
}

static const void *pvi_wf_dereference(struct drm_epd_lut *lut,
				      const struct pvi_wf_pointer *ptr)
{
	u8 sum = ptr->offset.b[0] + ptr->offset.b[1] + ptr->offset.b[2];

	if (ptr->checksum != sum)
		return NULL;

	return pvi_wf_apply_offset(lut, &ptr->offset);
}

static int pvi_wf_decode_lut(struct drm_epd_lut *lut, const u8 *lut_data)
{

	unsigned int copies, max_bytes, phase_size_order, state, total_bytes;
	const u8 *in = lut_data;
	u8 *out = lut->lut;
	u8 token;

	phase_size_order = drm_epd_lut_phase_size_order(lut->mode_info->format);
	max_bytes = lut->max_phases << phase_size_order;
	total_bytes = 0;
	state = 1;

	/* Read tokens until reaching the end-of-input marker. */
	while ((token = *in++) != 0xff) {
		/* Special handling for the state switch token. */
		if (token == 0xfc) {
			state = !state;
			token = *in++;
		}

		/*
		 * State 0 is a sequence of data bytes.
		 * State 1 is a sequence of [data byte, extra copies] pairs.
		 */
		copies = 1 + (state ? *in++ : 0);

		total_bytes += copies;
		if (total_bytes > max_bytes) {
			DRM_DEV_ERROR(lut->dev, "LUT is too big (%d+ bytes)!\n",
				      total_bytes);
			lut->num_phases = 0;
			return -E2BIG;
		}

		while (copies--)
			*out++ = token;
	}
	lut->num_phases = total_bytes >> phase_size_order;

	DRM_DEV_INFO(lut->dev, "LUT contains %d phases (%ld => %ld bytes)\n",
		     lut->num_phases, in - lut_data, out - lut->lut);

	return 0;
}

static int pvi_wf_find_lut(struct drm_epd_lut *lut, const u8 **lut_data)
{
	const struct pvi_wf_file_header *header = (const void *)lut->fw->data;
	const struct pvi_wf_pointer *mode_table, *temp_table;

	mode_table = pvi_wf_apply_offset(lut, &header->wmta);
	if (!mode_table)
		return -EFAULT;

	temp_table = pvi_wf_dereference(lut, &mode_table[lut->mode_index]);
	if (!temp_table)
		return -EFAULT;

	*lut_data = pvi_wf_dereference(lut, &temp_table[lut->temp_index]);
	if (!*lut_data)
		return -EFAULT;

	DRM_DEV_INFO(lut->dev, "LUT for mi=%d ti=%d starts at 0x%lx\n",
		     lut->mode_index, lut->temp_index,
		     *lut_data - lut->fw->data);

	return 0;
}

static int pvi_wf_get_lut(struct drm_epd_lut *lut)
{
	const u8 *compressed_lut;
	int ret;

	ret = pvi_wf_find_lut(lut, &compressed_lut);
	if (ret)
		return ret;

	ret = pvi_wf_decode_lut(lut, compressed_lut);
	if (ret)
		return ret;

	return 0;
}

static int pvi_wf_get_mode_index(struct drm_epd_lut *lut,
				 enum drm_epd_lut_waveform waveform)
{
	return lut->mode_info->modes[waveform];
}

static int pvi_wf_get_temp_index(struct drm_epd_lut *lut,
				 int temperature)
{
	const struct pvi_wf_file_header *header = (const void *)lut->fw->data;
	int i;

	for (i = 0; i < header->temp_range_count; i++)
		if (temperature < header->temp_range_table[i])
			return i - 1;

	return header->temp_range_count - 1;
}

int drm_epd_lut_phase_size_order(enum drm_epd_lut_format format)
{
	switch (format) {
	/* 4 bits/pixel => 1 pixel/byte */
	case DRM_EPD_LUT_4BIT:
		return 4 + 4;
	/* 4 bits/pixel => 4 pixels/byte */
	case DRM_EPD_LUT_4BIT_PACKED:
		return 4 + 4 - 2;
	/* 5 bits/pixel => 1 pixel/byte */
	case DRM_EPD_LUT_5BIT:
		return 5 + 5;
	/* 5 bits/pixel => 4 pixels/byte */
	case DRM_EPD_LUT_5BIT_PACKED:
		return 5 + 5 - 2;
	}

	return INT_MAX;
}
EXPORT_SYMBOL_GPL(drm_epd_lut_phase_size_order);

static int drm_epd_lut_convert(struct drm_epd_lut *lut)
{
	enum drm_epd_lut_format from = lut->mode_info->format;
	enum drm_epd_lut_format to = lut->format;
	u8 *buf = lut->lut;
	size_t x, y;

	if (from == to)
		return 0;

	/* Currently only 5-bit waveform files are supported. */
	if (from != DRM_EPD_LUT_5BIT_PACKED)
		return -ENOTSUPP;

	switch (to) {
	case DRM_EPD_LUT_4BIT:
		for (y = 0; y < 16 * lut->num_phases; ++y) {
			for (x = 8; x--;) {
				u8 byte = buf[16 * y + x];

				buf[16 * y + 2 * x + 0] = (byte >> 0) & 0x03;
				buf[16 * y + 2 * x + 1] = (byte >> 4) & 0x03;
			}
		}
		break;
	case DRM_EPD_LUT_4BIT_PACKED:
		for (y = 0; y < 16 * lut->num_phases; ++y) {
			for (x = 4; x--;) {
				u8 lo_byte = buf[16 * y + 2 * x + 0] & 0x33;
				u8 hi_byte = buf[16 * y + 2 * x + 1] & 0x33;

				/* Copy bits 4:5 => bits 2:3. */
				lo_byte |= lo_byte >> 2;
				hi_byte |= hi_byte >> 2;

				buf[4 * y + x] = (lo_byte & 0xf) |
						 (hi_byte << 4);
			}
		}
		break;
	case DRM_EPD_LUT_5BIT:
		for (x = 32 * 8 * lut->num_phases; x--;) {
			u8 byte = buf[x];

			buf[4 * x + 0] = (byte >> 0) & 0x03;
			buf[4 * x + 1] = (byte >> 2) & 0x03;
			buf[4 * x + 2] = (byte >> 4) & 0x03;
			buf[4 * x + 3] = (byte >> 6) & 0x03;
		}
		break;
	case DRM_EPD_LUT_5BIT_PACKED:
		/* Nothing to do. */
		break;
	}

	return 0;
}

static int drm_epd_lut_update(struct drm_epd_lut *lut)
{
	int ret;

	ret = pvi_wf_get_lut(lut);
	if (ret)
		return ret;

	ret = drm_epd_lut_convert(lut);
	if (ret)
		return ret;

	return 1;
}

int drm_epd_lut_set_temperature(struct drm_epd_lut *lut,
				int temperature)
{
	int temp_index;

	temp_index = pvi_wf_get_temp_index(lut, temperature);
	if (temp_index < 0)
		return -ENOENT;

	if (temp_index == lut->temp_index)
		return 0;

	DRM_DEV_INFO(lut->dev, "LUT update due to temperature change (%d)\n",
		     temperature);

	lut->temp_index = temp_index;

	return drm_epd_lut_update(lut);
}
EXPORT_SYMBOL_GPL(drm_epd_lut_set_temperature);

int drm_epd_lut_set_waveform(struct drm_epd_lut *lut,
			     enum drm_epd_lut_waveform waveform)
{
	int mode_index;

	mode_index = pvi_wf_get_mode_index(lut, waveform);
	if (mode_index < 0)
		return -ENOENT;

	if (mode_index == lut->mode_index)
		return 0;

	DRM_DEV_INFO(lut->dev, "LUT update due to waveform change (%d)\n",
		     waveform);

	lut->mode_index = mode_index;

	return drm_epd_lut_update(lut);
}
EXPORT_SYMBOL_GPL(drm_epd_lut_set_waveform);

static void drm_epd_lut_free(struct drm_device *dev, void *res)
{
	struct drm_epd_lut *lut = res;

	release_firmware(lut->fw);
	vfree(lut->lut);
}

/*
 * drmm_epd_lut_init - Initialize a managed EPD LUT from a waveform firmware.
 * @dev: The DRM device owning this LUT
 * @lut: The LUT to initialize
 * @file_name: The file name of the waveform firmware
 * @format: The LUT buffer format needed by the driver
 * @max_phases: The maximum number of waveform phases supported by the driver
 */
int drmm_epd_lut_init(struct drm_device *dev,
		      struct drm_epd_lut *lut,
		      const char *file_name,
		      enum drm_epd_lut_format format,
		      unsigned int max_phases)
{
	size_t max_lut_size, max_order;
	int ret;

	lut->dev	= dev->dev;
	lut->format	= format;
	lut->max_phases	= max_phases;

	ret = request_firmware(&lut->fw, file_name, dev->dev);
	if (ret)
		return ret;

	ret = pvi_wf_validate_header(lut);
	if (ret)
		goto err_release_firmware;

	max_order = max(drm_epd_lut_phase_size_order(lut->mode_info->format),
			drm_epd_lut_phase_size_order(format));
	max_lut_size = max_phases << max_order;

	lut->lut = vmalloc(max_lut_size);
	if (!lut->lut) {
		ret = -ENOMEM;
		goto err_release_firmware;
	}

	ret = drmm_add_action_or_reset(dev, drm_epd_lut_free, lut);
	if (ret)
		return ret;

	/* Set sane initial values for the temperature and waveform. */
	lut->temp_index = pvi_wf_get_temp_index(lut, 25);
	if (lut->temp_index < 0)
		return -ENOENT;

	lut->mode_index = pvi_wf_get_mode_index(lut, DRM_EPD_WF_RESET);
	if (lut->mode_index < 0)
		return -ENOENT;

	ret = drm_epd_lut_update(lut);
	if (ret < 0)
		return ret;

	return 0;

err_release_firmware:
	release_firmware(lut->fw);

	return ret;
}
EXPORT_SYMBOL_GPL(drmm_epd_lut_init);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("DRM EPD waveform LUT management");
MODULE_LICENSE("GPL");
