// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SoC pinctrl+GPIO+external IRQ driver
 *
 * Copyright (C) The Asahi Linux Contributors
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: pinctrl-pistachio.c
 * Copyright (C) 2014 Imagination Technologies Ltd.
 * Copyright (C) 2014 Google, Inc.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-utils.h"
#include "core.h"
#include "pinmux.h"

struct bflb_pinctrl_desc {
	const char *const	*functions;
	u8			num_functions;
	u8			num_pins;
};

struct bflb_gpio {
	const struct bflb_pinctrl_desc	*desc;
	void __iomem			*base;
	struct pinctrl_dev		*pctldev;
	struct pinctrl_desc		pinctrl_desc;
};

static const struct pinctrl_ops bflb_gpio_pinctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_pin,
	.dt_free_map		= pinctrl_utils_free_map,
};

static int bflb_gpio_pin_config_get(struct pinctrl_dev *pctldev, unsigned pin,
				    unsigned long *config)
{
	return 0;
}

static int bflb_gpio_pin_config_set(struct pinctrl_dev *pctldev, unsigned pin,
				    unsigned long *configs, unsigned num_configs)
{
	return 0;
}

static const struct pinconf_ops bflb_gpio_pinconf_ops = {
	.is_generic		= true,
	.pin_config_get		= bflb_gpio_pin_config_get,
	.pin_config_set		= bflb_gpio_pin_config_set,
};

/* Pin multiplexer functions */

static int bflb_gpio_pinmux_set(struct pinctrl_dev *pctldev, unsigned func,
                                unsigned group)
{
	struct bflb_gpio *pctl = pinctrl_dev_get_drvdata(pctldev);

	// writel(BIT(22) | func << 8 | BIT(0), pctl->base + 4 * group);

	return 0;
}

static const struct pinmux_ops bflb_gpio_pinmux_ops = {
	.get_functions_count	= pinmux_generic_get_function_count,
	.get_function_name	= pinmux_generic_get_function_name,
	.get_function_groups	= pinmux_generic_get_function_groups,
	.set_mux		= bflb_gpio_pinmux_set,
	.strict			= true,
};

/* Probe & register */

static int bflb_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct bflb_pinctrl_desc *desc;
	struct pinctrl_pin_desc *pins;
	struct bflb_gpio *pctl;
	const char **pin_names;
	unsigned int *pin_nums;
	int ret;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -EINVAL;

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	dev_set_drvdata(dev, pctl);
	pctl->desc = desc;

	pctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	pins = devm_kmalloc_array(dev, desc->num_pins, sizeof(pins[0]),
				  GFP_KERNEL);
	pin_names = devm_kmalloc_array(dev, desc->num_pins, sizeof(pin_names[0]),
				       GFP_KERNEL);
	pin_nums = devm_kmalloc_array(dev, desc->num_pins, sizeof(pin_nums[0]),
				      GFP_KERNEL);
	if (!pins || !pin_names || !pin_nums)
		return -ENOMEM;

	for (int i = 0; i < desc->num_pins; i++) {
		pins[i].number = i;
		pins[i].name = devm_kasprintf(dev, GFP_KERNEL, "GPIO%u", i);
		pins[i].drv_data = pctl;
		pin_names[i] = pins[i].name;
		pin_nums[i] = i;
	}

	pctl->pinctrl_desc.name = dev_name(dev);
	pctl->pinctrl_desc.pins = pins;
	pctl->pinctrl_desc.npins = desc->num_pins;
	pctl->pinctrl_desc.confops = &bflb_gpio_pinconf_ops;
	pctl->pinctrl_desc.pctlops = &bflb_gpio_pinctrl_ops;
	pctl->pinctrl_desc.pmxops = &bflb_gpio_pinmux_ops;

	pctl->pctldev =	devm_pinctrl_register(dev, &pctl->pinctrl_desc, pctl);
	if (IS_ERR(pctl->pctldev))
		return dev_err_probe(dev, PTR_ERR(pctl->pctldev),
				     "Failed to register pinctrl device");

	for (int i = 0; i < desc->num_pins; i++) {
		ret = pinctrl_generic_add_group(pctl->pctldev, pins[i].name,
						pin_nums + i, 1, pctl);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to register group");
	}

	for (int i = 0; i < desc->num_functions; ++i) {
		if (!desc->functions[i])
			continue;
		ret = pinmux_generic_add_function(pctl->pctldev, desc->functions[i],
						  pin_names, desc->num_pins, pctl);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to register function.");
	}

	return 0;
}

static const char *const bl808_pinctrl_functions[] = {
	[0]	= "sdh",
	[1]	= "spi0",
	[2]	= "flash",
	[3]	= "i2s",
	[4]	= "pdm",
	[5]	= "i2c0",
	[6]	= "i2c1",
	[7]	= "uart",
	[8]	= "emac",
	[9]	= "cam",
	[10]	= "analog",
	[11]	= "gpio",
	[12]	= "sdu",
	[16]	= "pwm0",
	[17]	= "pwm1",
	[18]	= "spi1",
	[19]	= "i2c2",
	[20]	= "i2c3",
	[21]	= "mm_uart",
	[22]	= "dbi_b",
	[23]	= "dbi_c",
	[24]	= "dpi",
	[25]	= "jtag_lp",
	[26]	= "jtag_m0",
	[27]	= "jtag_d0",
	[31]	= "clkout",
};

static const struct bflb_pinctrl_desc bl808_pinctrl_desc = {
	.functions	= bl808_pinctrl_functions,
	.num_functions	= ARRAY_SIZE(bl808_pinctrl_functions),
	.num_pins	= 46,
};

static const struct of_device_id bflb_gpio_of_match[] = {
	{ .compatible = "bflb,bl808-gpio",
	  .data = &bl808_pinctrl_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, bflb_gpio_of_match);

static struct platform_driver bflb_gpio_driver = {
	.probe	= bflb_gpio_probe,
	.driver	= {
		.name		= "bflb-gpio",
		.of_match_table	= bflb_gpio_of_match,
	},
};
module_platform_driver(bflb_gpio_driver);
