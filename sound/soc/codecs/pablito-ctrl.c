// SPDX-License-Identifier: GPL-2.0
// Pablito hardware control
// Copyright (C) 2024 Filipe Coelho <falktx@darkglass.com>

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/soc.h>

struct pablito_ctrl_priv {
	struct gpio_desc *gpiod_dac_mute;
	struct gpio_desc *gpiod_hp1;
	struct gpio_desc *gpiod_hp2;
	struct gpio_desc *gpiod_xlr_gl;

	int hp_gain;
	bool dac_mute;
	bool xlr_gl;
};

static int pablito_ctrl_headphone_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 2;
	return 0;
}

static int pablito_ctrl_headphone_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct pablito_ctrl_priv *priv = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] = priv->hp_gain;
	return 0;
}

static int pablito_ctrl_headphone_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct pablito_ctrl_priv *priv = snd_soc_component_get_drvdata(c);
	int changed = 0;

	if (priv->hp_gain != ucontrol->value.integer.value[0]) {
		priv->hp_gain = ucontrol->value.integer.value[0];

		/* 0: 1-high 2-high -> -10dB
		 * 1: 1-low 2-high -> 0dB
		 * 2: 1-low 2-low -> +10dB
		 */
		switch (priv->hp_gain) {
		case 0:
			gpiod_set_value(priv->gpiod_hp1, 1);
			gpiod_set_value(priv->gpiod_hp2, 1);
			break;
		case 1:
			gpiod_set_value(priv->gpiod_hp1, 0);
			gpiod_set_value(priv->gpiod_hp2, 1);
			break;
		case 2:
			gpiod_set_value(priv->gpiod_hp1, 0);
			gpiod_set_value(priv->gpiod_hp2, 0);
			break;
		}

		changed = 1;
	}

	return changed;
}

static int pablito_ctrl_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int pablito_ctrl_dac_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct pablito_ctrl_priv *priv = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] = priv->dac_mute;
	return 0;
}

static int pablito_ctrl_dac_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct pablito_ctrl_priv *priv = snd_soc_component_get_drvdata(c);
	int changed = 0;

	if (priv->dac_mute != ucontrol->value.integer.value[0]) {
		priv->dac_mute = ucontrol->value.integer.value[0];
		gpiod_set_value(priv->gpiod_dac_mute, priv->dac_mute);
		changed = 1;
	}
	return changed;
}

static int pablito_ctrl_xlr_gl_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct pablito_ctrl_priv *priv = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] = priv->xlr_gl;
	return 0;
}

static int pablito_ctrl_xlr_gl_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct pablito_ctrl_priv *priv = snd_soc_component_get_drvdata(c);
	int changed = 0;

	if (priv->xlr_gl != ucontrol->value.integer.value[0]) {
		priv->xlr_gl = ucontrol->value.integer.value[0];
		gpiod_set_value(priv->gpiod_xlr_gl, priv->xlr_gl);
		changed = 1;
	}
	return changed;
}

static const struct snd_kcontrol_new pablito_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DAC Mute",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pablito_ctrl_switch_info,
		.get = pablito_ctrl_dac_mute_get,
		.put = pablito_ctrl_dac_mute_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Headphone Gain",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pablito_ctrl_headphone_info,
		.get = pablito_ctrl_headphone_get,
		.put = pablito_ctrl_headphone_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "XLR Ground Lift",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pablito_ctrl_switch_info,
		.get = pablito_ctrl_xlr_gl_get,
		.put = pablito_ctrl_xlr_gl_put
	},
};

static const struct snd_soc_component_driver pablito_ctrl_component_driver = {
	.controls = pablito_snd_controls,
	.num_controls = ARRAY_SIZE(pablito_snd_controls),
};

static int pablito_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pablito_ctrl_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	// load all gpios
	priv->gpiod_dac_mute = devm_gpiod_get_optional(dev, "dac-mute", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_dac_mute))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_dac_mute), "Failed to get 'dac-mute' gpio");

	priv->gpiod_hp1 = devm_gpiod_get_optional(dev, "hp1", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_hp1))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_hp1), "Failed to get 'hp1' gpio");

	priv->gpiod_hp2 = devm_gpiod_get_optional(dev, "hp2", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_hp2))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_hp2), "Failed to get 'hp2' gpio");

	priv->gpiod_xlr_gl = devm_gpiod_get_optional(dev, "xlr-gl", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_xlr_gl))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_xlr_gl), "Failed to get 'xlr-gl' gpio");

	// force initial known state
	priv->hp_gain = 0;
	priv->dac_mute = false;
	priv->xlr_gl = false;

	gpiod_set_value(priv->gpiod_dac_mute, 0);
	gpiod_set_value(priv->gpiod_hp1, 1);
	gpiod_set_value(priv->gpiod_hp2, 1);
	gpiod_set_value(priv->gpiod_xlr_gl, 0);

	return devm_snd_soc_register_component(dev, &pablito_ctrl_component_driver, NULL, 0);
}

static const struct of_device_id pablito_ctrl_of_match[] = {
	{ .compatible = "pablito,audio-ctrl" },
	{},
};
MODULE_DEVICE_TABLE(of, pablito_ctrl_of_match);

static struct platform_driver pablito_ctrl_driver = {
	.driver = {
		.name	= "pablito-ctrl",
		.of_match_table = of_match_ptr(pablito_ctrl_of_match),
	},
	.probe = pablito_ctrl_probe,
};
module_platform_driver(pablito_ctrl_driver);

MODULE_AUTHOR("Filipe Coelho <falktx@darkglass.com>");
MODULE_DESCRIPTION("Pablito hardware control Driver");
MODULE_LICENSE("GPL v2");
