/*
 * ASoC driver for EMAC Inc. TI SoC based SoMs.
 *
 * Copyright (C) 2015 EMAC Inc.
 * Author:      Michael Welling <mwelling@ieee.org>
 *
 * Based on davinci-evm.c
 *
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

struct snd_soc_card_drvdata_emac {
	struct clk *mclk;
	unsigned sysclk;
};

static int som_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_emac *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

static void som_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_emac *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

static int som_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	int ret = 0;
	unsigned sysclk = ((struct snd_soc_card_drvdata_emac *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set the CPU system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops som_ops = {
	.startup = som_startup,
	.shutdown = som_shutdown,
	.hw_params = som_hw_params,
};

static const struct snd_soc_dapm_widget cs4271_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line Out connected to AOUTA, AOUTB */
	{"Line Out", NULL, "AOUTA+"},
	{"Line Out", NULL, "AOUTA-"},
	{"Line Out", NULL, "AOUTB+"},
	{"Line Out", NULL, "AOUTB+"},

	/* Line In connected to AINA, AINB */
	{"AINA", NULL, "Line In"},
	{"AINB", NULL, "Line In"},
};

static int som_cs4271_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct device_node *np = card->dev->of_node;
	int ret;

	/* Add emac-som specific widgets */
	snd_soc_dapm_new_controls(&card->dapm, cs4271_dapm_widgets,
				  ARRAY_SIZE(cs4271_dapm_widgets));

	if (np) {
		ret = snd_soc_of_parse_audio_routing(card, "ti,audio-routing");
		if (ret)
			return ret;
	} else {
		/* Set up emac-som specific audio path audio_map */
		snd_soc_dapm_add_routes(&card->dapm, audio_map,
					ARRAY_SIZE(audio_map));
	}

	return 0;
}

#if defined(CONFIG_OF)

/*
 * The struct is used as place holder. It will be completely
 * filled with data from dt node.
 */
static struct snd_soc_dai_link som_dai_cs4271 = {
	.name		= "CS4271",
	.stream_name	= "CS4271",
	.codec_dai_name	= "cs4271-hifi",
	.ops            = &som_ops,
	.init           = som_cs4271_init,
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM |
	   SND_SOC_DAIFMT_NB_NF,
};

static const struct of_device_id emac_som_dt_ids[] = {
	{
		.compatible = "emac,som3354-cs4271-snd",
		.data = (void *) &som_dai_cs4271,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, emac_som_dt_ids);

static struct snd_soc_card som_soc_card = {
	.owner = THIS_MODULE,
	.num_links = 1,
};

static int emac_som_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(emac_som_dt_ids), &pdev->dev);
	struct snd_soc_dai_link *dai = (struct snd_soc_dai_link *) match->data;
	struct snd_soc_card_drvdata_emac *drvdata = NULL;
	struct clk *mclk;
	int ret = 0;

	som_soc_card.dai_link = dai;

	dai->codec_of_node = of_parse_phandle(np, "ti,audio-codec", 0);
	if (!dai->codec_of_node)
		return -EINVAL;

	dai->cpu_of_node = of_parse_phandle(np, "ti,mcasp-controller", 0);
	if (!dai->cpu_of_node)
		return -EINVAL;

	dai->platform_of_node = dai->cpu_of_node;

	som_soc_card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&som_soc_card, "ti,model");
	if (ret)
		return ret;

	mclk = devm_clk_get(&pdev->dev, "mclk");

	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "mclk not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->mclk = mclk;

	ret = of_property_read_u32(np, "ti,codec-clock-rate", &drvdata->sysclk);


	if (ret < 0) {
		if (!drvdata->mclk) {
			dev_err(&pdev->dev,
				"No clock or clock rate defined.\n");
			return -EINVAL;
		}
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
	} else if (drvdata->mclk) {
		unsigned int requestd_rate = drvdata->sysclk;
dev_warn(&pdev->dev, "mclk %u\n",requestd_rate);
		clk_set_rate(drvdata->mclk, drvdata->sysclk);
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
		if (drvdata->sysclk != requestd_rate)
			dev_warn(&pdev->dev,
				 "Could not get requested rate %u using %u.\n",
				 requestd_rate, drvdata->sysclk);
	}

	snd_soc_card_set_drvdata(&som_soc_card, drvdata);
	ret = devm_snd_soc_register_card(&pdev->dev, &som_soc_card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
}

static struct platform_driver emac_som_driver = {
	.probe		= emac_som_probe,
	.driver		= {
		.name	= "emac_som",
		.pm	= &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(emac_som_dt_ids),
	},
};
#endif

static int __init som_init(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt())
		return platform_driver_register(&emac_som_driver);
#endif

	return -EINVAL;
}

static void __exit som_exit(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&emac_som_driver);
		return;
	}
#endif
}

module_init(som_init);
module_exit(som_exit);

MODULE_AUTHOR("Michael Welling");
MODULE_DESCRIPTION("EMAC TI SoM ASoC driver");
MODULE_LICENSE("GPL");
