// SPDX-License-Identifier: GPL-2.0
// Pablito hardware control
// Copyright (C) 2024 Filipe Coelho <falktx@darkglass.com>

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/soc.h>

struct birds_ctrl_priv {
	struct gpio_desc *gpiod_phantom_enable;
	struct gpio_desc *gpiod_audio_switcher_enable;
	struct gpio_desc *gpiod_codec_ldo_enable;
	struct gpio_desc *gpiod_powersave;

	bool phantom_enable;
	bool audio_switcher_enable;
	bool codec_ldo_enable;
	bool powersave;
};

static int birds_ctrl_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

#define JOIN(a, b) a ## b

#define CTRL_BOOL(VAR) \
	static int JOIN(birds_ctrl_get_, VAR)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) { \
		struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol); \
		struct birds_ctrl_priv *priv = snd_soc_component_get_drvdata(c); \
		ucontrol->value.integer.value[0] = priv->VAR;  \
		return 0; \
	} \
	static int JOIN(birds_ctrl_put_, VAR)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) { \
		struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol); \
		struct birds_ctrl_priv *priv = snd_soc_component_get_drvdata(c); \
		int changed = 0; \
		if (priv->VAR != ucontrol->value.integer.value[0]) { \
			priv->VAR = ucontrol->value.integer.value[0]; \
			gpiod_set_value(priv->JOIN(gpiod_, VAR), priv->VAR); \
			changed = 1; \
		} \
		return changed; \
	}

CTRL_BOOL(phantom_enable)
CTRL_BOOL(audio_switcher_enable)
CTRL_BOOL(codec_ldo_enable)
CTRL_BOOL(powersave)

static const struct snd_kcontrol_new birds_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Phantom",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = birds_ctrl_switch_info,
		.get = JOIN(birds_ctrl_get_, phantom_enable),
		.put = JOIN(birds_ctrl_put_, phantom_enable)
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Switcher",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = birds_ctrl_switch_info,
		.get = JOIN(birds_ctrl_get_, audio_switcher_enable),
		.put = JOIN(birds_ctrl_put_, audio_switcher_enable)
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "LDO",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = birds_ctrl_switch_info,
		.get = JOIN(birds_ctrl_get_, codec_ldo_enable),
		.put = JOIN(birds_ctrl_put_, codec_ldo_enable)
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Powersave",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = birds_ctrl_switch_info,
		.get = JOIN(birds_ctrl_get_, powersave),
		.put = JOIN(birds_ctrl_put_, powersave)
	},
};

static const struct snd_soc_component_driver birds_ctrl_component_driver = {
	.controls = birds_snd_controls,
	.num_controls = ARRAY_SIZE(birds_snd_controls),
};

static int birds_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct birds_ctrl_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	// load all gpios
	priv->gpiod_phantom_enable = devm_gpiod_get_optional(dev, "phantom-enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_phantom_enable))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_phantom_enable), "Failed to get 'phantom-enable' gpio");

	priv->gpiod_audio_switcher_enable = devm_gpiod_get_optional(dev, "audio-switcher-enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_audio_switcher_enable))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_audio_switcher_enable), "Failed to get 'audio-switcher-enable' gpio");

	priv->gpiod_codec_ldo_enable = devm_gpiod_get_optional(dev, "codec-ldo-enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_codec_ldo_enable))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_codec_ldo_enable), "Failed to get 'codec-ldo-enable' gpio");

	priv->gpiod_powersave = devm_gpiod_get_optional(dev, "powersave", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_powersave))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_powersave), "Failed to get 'powersave' gpio");

	// force initial known state
	priv->phantom_enable = false;
	priv->audio_switcher_enable = true;
	priv->codec_ldo_enable = true;
	priv->powersave = false;

	gpiod_set_value(priv->gpiod_phantom_enable, 0);
	gpiod_set_value(priv->gpiod_audio_switcher_enable, 1);
	gpiod_set_value(priv->gpiod_codec_ldo_enable, 1);
	gpiod_set_value(priv->gpiod_powersave, 0);

	return devm_snd_soc_register_component(dev, &birds_ctrl_component_driver, NULL, 0);
}

static const struct of_device_id birds_ctrl_of_match[] = {
	{ .compatible = "birds,audio-ctrl" },
	{},
};
MODULE_DEVICE_TABLE(of, birds_ctrl_of_match);

static struct platform_driver birds_ctrl_driver = {
	.driver = {
		.name	= "birds-ctrl",
		.of_match_table = of_match_ptr(birds_ctrl_of_match),
	},
	.probe = birds_ctrl_probe,
};
module_platform_driver(birds_ctrl_driver);

MODULE_AUTHOR("Filipe Coelho <falktx@darkglass.com>");
MODULE_DESCRIPTION("Birds hardware control Driver");
MODULE_LICENSE("GPL v2");
