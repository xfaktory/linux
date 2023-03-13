// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>

#include "sdhci-pltfm.h"

static const struct sdhci_ops sdhci_bflb_ops = {
	.set_clock		= sdhci_set_clock,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.get_timeout_clock	= sdhci_pltfm_clk_get_max_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.reset			= sdhci_reset,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_bflb_pdata = {
	.ops	= &sdhci_bflb_ops,
	.quirks	= SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
};

static int sdhci_bflb_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_bflb_pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	pltfm_host->clk = devm_clk_get(&pdev->dev, "mod");

	if (!IS_ERR(pltfm_host->clk))
		clk_prepare_enable(pltfm_host->clk);

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	clk_disable_unprepare(pltfm_host->clk);
	sdhci_pltfm_free(pdev);
	return ret;
}

static const struct of_device_id sdhci_bflb_of_match_table[] = {
	{ .compatible = "bflb,bl808-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_bflb_of_match_table);

static struct platform_driver sdhci_bflb_driver = {
	.driver		= {
		.name	= "sdhci-bflb",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm	= &sdhci_pltfm_pmops,
		.of_match_table = sdhci_bflb_of_match_table,
	},
	.probe		= sdhci_bflb_probe,
	.remove		= sdhci_pltfm_unregister,
};

module_platform_driver(sdhci_bflb_driver);

MODULE_DESCRIPTION("SDHCI driver for Bflb");
MODULE_LICENSE("GPL v2");
