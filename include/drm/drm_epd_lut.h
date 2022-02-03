// SPDX-License-Identifier: GPL-2.0

#ifndef __DRM_EPD_LUT_H__
#define __DRM_EPD_LUT_H__

struct device;
struct drm_device;
struct firmware;
struct pvi_wf_mode_info;

/**
 * enum drm_epd_lut_format - EPD LUT buffer format
 * @DRM_EPD_LUT_4BIT: 4-bit grayscale indexes, 1 byte per element
 * @DRM_EPD_LUT_4BIT_PACKED: 4-bit grayscale indexes, 2 bits per element
 * @DRM_EPD_LUT_5BIT: 5-bit grayscale indexes, 1 byte per element
 * @DRM_EPD_LUT_5BIT_PACKED: 5-bit grayscale indexes, 2 bits per element
 */
enum drm_epd_lut_format {
	DRM_EPD_LUT_4BIT,
	DRM_EPD_LUT_4BIT_PACKED,
	DRM_EPD_LUT_5BIT,
	DRM_EPD_LUT_5BIT_PACKED,
};

/**
 * enum drm_epd_lut_waveform - EPD pixel update waveform selection
 */
enum drm_epd_lut_waveform {
	DRM_EPD_WF_RESET,
	DRM_EPD_WF_DU,
	DRM_EPD_WF_DU4,
	DRM_EPD_WF_GC16,
	DRM_EPD_WF_GL16,
	DRM_EPD_WF_GLR16,
	DRM_EPD_WF_GLD16,
	DRM_EPD_WF_A2,
	DRM_EPD_WF_GCC16,
	DRM_EPD_WF_MAX
};

/**
 * struct drm_epd_lut - Context for loading and parsing EPD LUTs
 *
 * @dev: The device owning this LUT
 * @file: A buffer holding the unparsed waveform firmware file
 * @fw: The waveform firmware file object
 * @lut: A buffer in @format holding the current LUT
 * @mode_info: Private data about available waveforms
 * @format: The LUT buffer format needed by the driver
 * @max_phases: The maximum number of waveform phases supported by the driver
 * @num_phases: The number of waveform phases in the current LUT
 * @temp_index: Private identifier for the current temperature
 * @mode_index: Private identifier for the current waveform mode
 */
struct drm_epd_lut {
	struct device			*dev;
	const void			*file;
	const struct firmware		*fw;
	u8				*lut;
	const struct pvi_wf_mode_info	*mode_info;
	enum drm_epd_lut_format		format;
	unsigned int			max_phases;
	unsigned int			num_phases;
	int				temp_index;
	int				mode_index;
};

int drm_epd_lut_phase_size_order(enum drm_epd_lut_format format);

int drm_epd_lut_set_temperature(struct drm_epd_lut *lut,
				int temperature);
int drm_epd_lut_set_waveform(struct drm_epd_lut *lut,
			     enum drm_epd_lut_waveform waveform);

int drmm_epd_lut_init(struct drm_device *dev,
		      struct drm_epd_lut *lut,
		      const char *file_name,
		      enum drm_epd_lut_format format,
		      unsigned int max_phases);

#endif /* __DRM_EPD_LUT_H__ */
