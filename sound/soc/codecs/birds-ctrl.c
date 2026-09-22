// SPDX-License-Identifier: GPL-2.0
// Pablito hardware control
// Copyright (C) 2024 Filipe Coelho <falktx@darkglass.com>

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/soc.h>

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <sound/tlv.h>

/*
 * The whole device is one 16 bit shift register, clocked in MSB first and
 * latched on the rising edge of CS:
 *
 *   15:13  zero
 *   12:8   GAIN[4:0]
 *    7:6   zero
 *      5   GPOMODE   1 = GPO updates wait for a zero crossing
 *      4   GAINMODE  1 = gain updates wait for a zero crossing
 *    3:0   GPO[3:0]
 */
#define THAT5173_REG			0
#define THAT5173_GAIN_MASK		GENMASK(12, 8)
#define THAT5173_GAIN_SHIFT		8
#define THAT5173_GPOMODE_SHIFT		5
#define THAT5173_GAINMODE_SHIFT		4
#define THAT5173_GPOMODE		BIT(THAT5173_GPOMODE_SHIFT)
#define THAT5173_GAINMODE		BIT(THAT5173_GAINMODE_SHIFT)
#define THAT5173_GPO_MASK		GENMASK(3, 0)

/* Codes above 20 all clamp to 60 dB in hardware, so 20 is the useful maximum. */
#define THAT5173_GAIN_MAX		20
#define THAT5173_NGPO			4

#define THAT5173_MAX_SPEED_HZ		10000000

/* tCSH: minimum CS high time between transmissions. */
#define THAT5173_TCSH_NS		25000

/* Default RT/CT zero crossing timeout recommended by the datasheet. */
#define THAT5173_ZC_TIMEOUT_MS		22

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

struct birds_spi_priv {
	struct spi_device *spi;
	struct regmap *regmap;
	struct gpio_chip gpio;
	struct gpio_desc *reset;
	struct gpio_desc *busy;
	struct completion busy_done;
	unsigned int zc_timeout_ms;
	ktime_t last_cs_rise;
	/*
	 * Shadow of the last value written.  regmap holds its own cache, but
	 * reg_write() runs with the regmap lock held and so cannot read it
	 * back.
	 */
	u16 shadow;

	/* DMA safe, and serialised by the regmap lock. */
	__be16 buf ____cacheline_aligned;
};

static int birds_ctrl_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static bool birds_spi_zc_enabled(u16 val)
{
	return val & (THAT5173_GAINMODE | THAT5173_GPOMODE);
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

static const struct snd_kcontrol_new birds_ctrl_snd_controls[] = {
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
	.controls = birds_ctrl_snd_controls,
	.num_controls = ARRAY_SIZE(birds_ctrl_snd_controls),
};

/* 0 dB to 60 dB in 3 dB steps. */
static const DECLARE_TLV_DB_SCALE(birds_spi_gain_tlv, 0, 300, 0);

static const struct snd_kcontrol_new birds_spi_snd_controls[] = {
	SOC_SINGLE_TLV("Mic Gain Volume", THAT5173_REG, THAT5173_GAIN_SHIFT, THAT5173_GAIN_MAX, 0, birds_spi_gain_tlv),
	SOC_SINGLE("Gain Zero Cross Switch", THAT5173_REG, THAT5173_GAINMODE_SHIFT, 1, 0),
	SOC_SINGLE("GPO Zero Cross Switch", THAT5173_REG, THAT5173_GPOMODE_SHIFT, 1, 0),
};

static const struct snd_soc_component_driver birds_spi_component_driver = {
	.controls = birds_spi_snd_controls,
	.num_controls = ARRAY_SIZE(birds_spi_snd_controls),
};

/*
 * A command issued while a zero crossing update is still pending replaces that
 * update instead of queueing behind it, and restarts the timeout.  A burst of
 * gain changes would therefore drop everything but the last one, so hold off
 * until the previous update has retired.
 */
static void that5173_wait_idle(struct birds_spi_priv *p)
{
	s64 elapsed_ms;

	if (!birds_spi_zc_enabled(p->shadow))
		return;

	if (!p->busy) {
		/* No BSY line wired: pace by the worst case timeout. */
		elapsed_ms = ktime_ms_delta(ktime_get(), p->last_cs_rise);
		if (elapsed_ms < p->zc_timeout_ms)
			msleep((unsigned int)(p->zc_timeout_ms - elapsed_ms));
		return;
	}

	if (!gpiod_get_value_cansleep(p->busy))
		return;

	reinit_completion(&p->busy_done);

	/* BSY may have fallen while the completion was being reinitialised. */
	if (!gpiod_get_value_cansleep(p->busy))
		return;

	if (!wait_for_completion_timeout(&p->busy_done,
					 msecs_to_jiffies(p->zc_timeout_ms + 5)))
		dev_dbg(&p->spi->dev, "timed out waiting for BSY to clear\n");
}

static irqreturn_t that5173_busy_irq(int irq, void *data)
{
	struct birds_spi_priv *p = data;

	complete(&p->busy_done);

	return IRQ_HANDLED;
}

static int that5173_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct birds_spi_priv *p = context;
	s64 since_ns;
	int ret;

	that5173_wait_idle(p);

	/* Honour tCSH: the controller drops CS the instant a message ends. */
	since_ns = ktime_to_ns(ktime_sub(ktime_get(), p->last_cs_rise));
	if (since_ns < THAT5173_TCSH_NS) {
		unsigned long us = DIV_ROUND_UP(THAT5173_TCSH_NS - since_ns,
						NSEC_PER_USEC);

		usleep_range(us, us + 10);
	}

	p->buf = cpu_to_be16(val);

	ret = spi_write(p->spi, &p->buf, sizeof(p->buf));
	if (ret) {
		dev_err(&p->spi->dev, "SPI write of 0x%04x failed: %d\n", val,
			ret);
		return ret;
	}

	/* The value takes effect on the rising edge of CS, i.e. now. */
	p->last_cs_rise = ktime_get();
	p->shadow = val;

	return 0;
}

static bool that5173_accessible_reg(struct device *dev, unsigned int reg)
{
	return reg == THAT5173_REG;
}

static const struct reg_default that5173_reg_defaults[] = {
	{ THAT5173_REG, 0x0000 },	/* 0 dB, GPOs off, zero crossing off */
};

/*
 * A regmap with no bus: reads are served from the cache because the device
 * cannot be read back (DOUT is a daisy chain pass through of DIN, not a
 * register readback), and every write goes through that5173_reg_write().
 */
static const struct regmap_config that5173_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = THAT5173_REG,
	.readable_reg = that5173_accessible_reg,
	.writeable_reg = that5173_accessible_reg,
	.reg_write = that5173_reg_write,
	.cache_type = REGCACHE_FLAT,
	.reg_defaults = that5173_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(that5173_reg_defaults),
};

static int that5173_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct birds_spi_priv *p = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(p->regmap, THAT5173_REG, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset));
}

static void that5173_gpio_set(struct gpio_chip *gc, unsigned int offset,
			      int value)
{
	struct birds_spi_priv *p = gpiochip_get_data(gc);
	int ret;

	ret = regmap_update_bits(p->regmap, THAT5173_REG, BIT(offset),
				 value ? BIT(offset) : 0);
	if (ret)
		dev_err(&p->spi->dev, "failed to set GPO%u: %d\n", offset, ret);
}

static int that5173_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int offset, int value)
{
	that5173_gpio_set(gc, offset, value);

	return 0;
}

static int that5173_gpio_get_direction(struct gpio_chip *gc,
				       unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static const char * const that5173_gpio_names[THAT5173_NGPO] = {
	"GPO0", "GPO1", "GPO2", "GPO3",
};

static const struct gpio_chip that5173_gpio_chip = {
	.owner			= THIS_MODULE,
	.base			= -1,
	.ngpio			= THAT5173_NGPO,
	.names			= that5173_gpio_names,
	.can_sleep		= true,
	.get			= that5173_gpio_get,
	.set			= that5173_gpio_set,
	.get_direction		= that5173_gpio_get_direction,
	.direction_output	= that5173_gpio_direction_output,
};

static void that5173_hw_reset(struct birds_spi_priv *p)
{
	if (!p->reset)
		return;

	gpiod_set_value_cansleep(p->reset, 1);
	udelay(1);
	gpiod_set_value_cansleep(p->reset, 0);
	udelay(1);		/* tSRS, 100 ns */

	p->last_cs_rise = ktime_get();
}

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

static int birds_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct birds_spi_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	spi_set_drvdata(spi, priv);

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	priv->spi = spi;
	init_completion(&priv->busy_done);

	if (spi->max_speed_hz > THAT5173_MAX_SPEED_HZ) {
		dev_warn(dev, "clamping SPI clock from %u Hz to %u Hz\n",
			 spi->max_speed_hz, THAT5173_MAX_SPEED_HZ);
		spi->max_speed_hz = THAT5173_MAX_SPEED_HZ;
	}

	/* DIN is sampled on the rising edge of SCLK: mode 0, MSB first. */
	spi->mode = (spi->mode & ~SPI_MODE_X_MASK) | SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up SPI\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	priv->zc_timeout_ms = THAT5173_ZC_TIMEOUT_MS;
	device_property_read_u32(dev, "that,zc-timeout-ms", &priv->zc_timeout_ms);

	priv->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "failed to get reset GPIO\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	priv->busy = devm_gpiod_get_optional(dev, "busy", GPIOD_IN);
	if (IS_ERR(priv->busy))
		return dev_err_probe(dev, PTR_ERR(priv->busy),
				     "failed to get busy GPIO\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	if (priv->busy) {
		int irq = gpiod_to_irq(priv->busy);

		if (irq < 0)
			return dev_err_probe(dev, irq,
					     "busy GPIO cannot be used as an interrupt\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

		ret = devm_request_threaded_irq(dev, irq, NULL,
						that5173_busy_irq,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						dev_name(dev), priv);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request BSY irq\n");
	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	}

	that5173_hw_reset(priv);

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	priv->regmap = devm_regmap_init(dev, NULL, priv, &that5173_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	/* Put known state on the wire so the cache provably matches hardware. */
	ret = regmap_write(priv->regmap, THAT5173_REG, 0x0000);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise device\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	priv->gpio = that5173_gpio_chip;
	priv->gpio.label = dev_name(dev);
	priv->gpio.parent = dev;

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	ret = devm_gpiochip_add_data(dev, &priv->gpio, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpiochip\n");

	dev_warn(dev, "%s %d\n", __FUNCTION__, __LINE__);

	return devm_snd_soc_register_component(dev, &birds_spi_component_driver, NULL, 0);
}

static const struct of_device_id birds_ctrl_of_match[] = {
	{ .compatible = "birds,audio-ctrl" },
	{},
};
MODULE_DEVICE_TABLE(of, birds_ctrl_of_match);

static struct platform_driver birds_ctrl_driver = {
	.driver = {
		.name = "birds-ctrl",
		.of_match_table = of_match_ptr(birds_ctrl_of_match),
	},
	.probe = birds_ctrl_probe,
};
module_platform_driver(birds_ctrl_driver);

static struct spi_driver birds_spi_driver = {
	.driver = {
		.name = "birds-spi",
		.of_match_table = of_match_ptr(birds_ctrl_of_match),
	},
	.probe = birds_spi_probe,
};
module_spi_driver(birds_spi_driver);

MODULE_AUTHOR("Filipe Coelho <falktx@darkglass.com>");
MODULE_DESCRIPTION("Birds hardware control Driver");
MODULE_LICENSE("GPL v2");
