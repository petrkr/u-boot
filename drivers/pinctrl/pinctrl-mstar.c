// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar Infinity6E PADTOP pin controller and GPIO.
 *
 * Every pad has its own 16-bit PADTOP register holding its GPIO state and
 * pin configuration; the function muxes live in shared PADTOP, CHIPTOP and
 * PM sleep registers. Pin numbers are the ones in
 * dt-bindings/gpio/sstar-infinity6e-gpio.h, the register layout follows the
 * vendor pinmux driver.
 */
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <dm/pinctrl.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <dt-bindings/gpio/sstar-infinity6e-gpio.h>

/* per-pad register */
#define PAD_IN			BIT(0)
#define PAD_OUT			BIT(1)
#define PAD_OEN			BIT(2)
#define PAD_PULL		BIT(4)
#define PAD_DRV			BIT(7)

/* PADTOP function muxes */
#define PAD_MODE_SD		0x67
#define PAD_MODE_SD0		BIT(8)
#define PAD_MODE_SD0_CDZ	BIT(9)
#define PAD_MODE_SPI		0x68
#define PAD_MODE_SPI0		GENMASK(2, 0)
#define PAD_MODE_SPI0_SD0	3	/* SPI0 mode 3 uses the SD0 pads */

/* CHIPTOP */
#define CHIP_ALLPAD		0x140
#define CHIP_ALLPAD_IN		BIT(15)

/* PM sleep pad muxes */
#define PM_LED_MODE		0xa0
#define PM_LED_MODE_MASK	GENMASK(5, 4)
#define PM_LED_MODE_ETH		1
#define PM_SPI_IS_GPIO		0xd4
/* SPI pads as SPI, SPI_CZ2 as GPIO; value from the vendor SPI NAND driver */
#define PM_SPI_IS_GPIO_FSP	0x08

struct mstar_pin {
	const char *name;
	u8 reg;
};

#define PIN(_name, _reg) \
	[SSTAR_INFINITY6E_GPIO_##_name] = { .name = #_name, .reg = _reg }

static const struct mstar_pin infinity6e_pins[SSTAR_INFINITY6E_GPIO_COUNT] = {
	PIN(GPIO0, 0x00),
	PIN(GPIO1, 0x01),
	PIN(GPIO2, 0x02),
	PIN(GPIO3, 0x03),
	PIN(GPIO4, 0x04),
	PIN(GPIO5, 0x05),
	PIN(GPIO6, 0x06),
	PIN(GPIO7, 0x07),
	PIN(GPIO8, 0x08),
	PIN(GPIO9, 0x09),
	PIN(GPIO10, 0x0a),
	PIN(GPIO11, 0x0b),
	PIN(GPIO12, 0x0c),
	PIN(GPIO13, 0x0d),
	PIN(GPIO14, 0x0e),
	PIN(GPIO15, 0x0f),
	PIN(SD0_GPIO0, 0x10),
	PIN(SD0_CDZ, 0x11),
	PIN(SD0_D1, 0x12),
	PIN(SD0_D0, 0x13),
	PIN(SD0_CLK, 0x14),
	PIN(SD0_CMD, 0x15),
	PIN(SD0_D3, 0x16),
	PIN(SD0_D2, 0x17),
	PIN(I2S0_MCLK, 0x18),
	PIN(I2S0_BCK, 0x19),
	PIN(I2S0_WCK, 0x1a),
	PIN(I2S0_DI, 0x1b),
	PIN(I2S0_DO, 0x1c),
	PIN(I2C0_SCL, 0x1d),
	PIN(I2C0_SDA, 0x1e),
	PIN(ETH_LED0, 0x20),
	PIN(ETH_LED1, 0x21),
	PIN(FUART_RX, 0x22),
	PIN(FUART_TX, 0x23),
	PIN(FUART_CTS, 0x24),
	PIN(FUART_RTS, 0x25),
	PIN(SD1_CDZ, 0x26),
	PIN(SD1_D1, 0x27),
	PIN(SD1_D0, 0x28),
	PIN(SD1_CLK, 0x29),
	PIN(SD1_CMD, 0x2a),
	PIN(SD1_D3, 0x2b),
	PIN(SD1_D2, 0x2c),
	PIN(SD1_GPIO0, 0x2d),
	PIN(SD1_GPIO1, 0x2e),
	PIN(SPI_CZ, 0x30),
	PIN(SPI_CK, 0x31),
	PIN(SPI_DI, 0x32),
	PIN(SPI_DO, 0x33),
	PIN(SPI_WPZ, 0x34),
	PIN(SPI_HLD, 0x35),
	PIN(SR0_IO00, 0x36),
	PIN(SR0_IO01, 0x37),
	PIN(SR0_IO02, 0x38),
	PIN(SR0_IO03, 0x39),
	PIN(SR0_IO04, 0x3a),
	PIN(SR0_IO05, 0x3b),
	PIN(SR0_IO06, 0x3c),
	PIN(SR0_IO07, 0x3d),
	PIN(SR0_IO08, 0x3e),
	PIN(SR0_IO09, 0x3f),
	PIN(SR0_IO10, 0x40),
	PIN(SR0_IO11, 0x41),
	PIN(SR0_IO12, 0x42),
	PIN(SR0_IO13, 0x43),
	PIN(SR0_IO14, 0x44),
	PIN(SR0_IO15, 0x45),
	PIN(SR0_IO16, 0x46),
	PIN(SR0_IO17, 0x47),
	PIN(SR0_IO18, 0x48),
	PIN(SR0_IO19, 0x49),
	PIN(SR1_IO00, 0x4a),
	PIN(SR1_IO01, 0x4b),
	PIN(SR1_IO02, 0x4c),
	PIN(SR1_IO03, 0x4d),
	PIN(SR1_IO04, 0x4e),
	PIN(SR1_IO05, 0x4f),
	PIN(SR1_IO06, 0x50),
	PIN(SR1_IO07, 0x51),
	PIN(SR1_IO08, 0x52),
	PIN(SR1_IO09, 0x53),
	PIN(SR1_IO10, 0x54),
	PIN(SR1_IO11, 0x55),
	PIN(SR1_IO12, 0x56),
	PIN(SR1_IO13, 0x57),
	PIN(SR1_IO14, 0x58),
	PIN(SR1_IO15, 0x59),
	PIN(SR1_IO16, 0x5a),
	PIN(SR1_IO17, 0x5b),
	PIN(SR1_IO18, 0x5c),
	PIN(SR1_IO19, 0x5d),
};

static const unsigned int sd0_pins[] = {
	SSTAR_INFINITY6E_GPIO_SD0_D1, SSTAR_INFINITY6E_GPIO_SD0_D0,
	SSTAR_INFINITY6E_GPIO_SD0_CLK, SSTAR_INFINITY6E_GPIO_SD0_CMD,
	SSTAR_INFINITY6E_GPIO_SD0_D3, SSTAR_INFINITY6E_GPIO_SD0_D2,
};

static const unsigned int sd0_cdz_pins[] = {
	SSTAR_INFINITY6E_GPIO_SD0_CDZ,
};

enum mstar_group {
	GROUP_SD0,
	GROUP_SD0_CDZ,
	GROUP_PM_SPI,
	GROUP_PM_LED,
};

struct mstar_group_desc {
	const char *name;
	const unsigned int *pins;
	unsigned int npins;
};

/* The PM sleep groups have no PADTOP pins */
static const struct mstar_group_desc mstar_groups[] = {
	[GROUP_SD0]	= { "sd0", sd0_pins, ARRAY_SIZE(sd0_pins) },
	[GROUP_SD0_CDZ]	= { "sd0_cdz", sd0_cdz_pins, ARRAY_SIZE(sd0_cdz_pins) },
	[GROUP_PM_SPI]	= { "pm_spi" },
	[GROUP_PM_LED]	= { "pm_led" },
};

enum mstar_function {
	FUNC_GPIO,
	FUNC_SD0,
	FUNC_SD0_CDZ,
	FUNC_PM_SPI,
	FUNC_ETH_LED,
};

static const char * const mstar_functions[] = {
	[FUNC_GPIO]	= "gpio",
	[FUNC_SD0]	= "sd0",
	[FUNC_SD0_CDZ]	= "sd0_cdz",
	[FUNC_PM_SPI]	= "pm_spi",
	[FUNC_ETH_LED]	= "eth_led",
};

struct mstar_pinctrl_priv {
	void __iomem *padtop;
	struct regmap *chiptop;
	struct regmap *pmsleep;
};

static void __iomem *mstar_pad(void __iomem *padtop, unsigned int pin)
{
	return padtop + infinity6e_pins[pin].reg * 4;
}

static int mstar_get_pins_count(struct udevice *dev)
{
	return ARRAY_SIZE(infinity6e_pins);
}

static const char *mstar_get_pin_name(struct udevice *dev,
				      unsigned int selector)
{
	return infinity6e_pins[selector].name;
}

static int mstar_get_groups_count(struct udevice *dev)
{
	return ARRAY_SIZE(mstar_groups);
}

static const char *mstar_get_group_name(struct udevice *dev,
					unsigned int selector)
{
	return mstar_groups[selector].name;
}

static int mstar_get_functions_count(struct udevice *dev)
{
	return ARRAY_SIZE(mstar_functions);
}

static const char *mstar_get_function_name(struct udevice *dev,
					   unsigned int selector)
{
	return mstar_functions[selector];
}

static int mstar_mux_sd0(struct mstar_pinctrl_priv *priv, bool on)
{
	void __iomem *mode = priv->padtop + PAD_MODE_SD * 4;
	void __iomem *spi = priv->padtop + PAD_MODE_SPI * 4;
	unsigned int i;

	if (!on) {
		clrbits_le16(mode, PAD_MODE_SD0);
		return 0;
	}
	if (FIELD_GET(PAD_MODE_SPI0, readw(spi)) == PAD_MODE_SPI0_SD0)
		clrbits_le16(spi, PAD_MODE_SPI0);
	setbits_le16(mode, PAD_MODE_SD0);
	for (i = 0; i < ARRAY_SIZE(sd0_pins); i++)
		clrbits_le16(mstar_pad(priv->padtop, sd0_pins[i]), PAD_DRV);
	return 0;
}

static int mstar_pinmux_group_set(struct udevice *dev,
				  unsigned int group_selector,
				  unsigned int func_selector)
{
	struct mstar_pinctrl_priv *priv = dev_get_priv(dev);
	void __iomem *mode = priv->padtop + PAD_MODE_SD * 4;

	switch (group_selector) {
	case GROUP_SD0:
		if (func_selector != FUNC_SD0 && func_selector != FUNC_GPIO)
			break;
		return mstar_mux_sd0(priv, func_selector == FUNC_SD0);
	case GROUP_SD0_CDZ:
		if (func_selector == FUNC_SD0_CDZ) {
			setbits_le16(mode, PAD_MODE_SD0_CDZ);
			return 0;
		}
		if (func_selector == FUNC_GPIO) {
			clrbits_le16(mode, PAD_MODE_SD0_CDZ);
			return 0;
		}
		break;
	case GROUP_PM_SPI:
		if (func_selector == FUNC_PM_SPI)
			return regmap_write(priv->pmsleep, PM_SPI_IS_GPIO,
					    PM_SPI_IS_GPIO_FSP);
		break;
	case GROUP_PM_LED:
		if (func_selector == FUNC_ETH_LED)
			return regmap_update_bits(priv->pmsleep, PM_LED_MODE,
						  PM_LED_MODE_MASK,
						  FIELD_PREP(PM_LED_MODE_MASK,
							     PM_LED_MODE_ETH));
		break;
	}

	dev_err(dev, "group %s: function %s not supported\n",
		mstar_groups[group_selector].name,
		mstar_functions[func_selector]);
	return -EINVAL;
}

static const struct pinconf_param mstar_pinconf_params[] = {
	{ "bias-disable", PIN_CONFIG_BIAS_DISABLE, 0 },
	{ "bias-pull-up", PIN_CONFIG_BIAS_PULL_UP, 1 },
	{ "input-enable", PIN_CONFIG_INPUT_ENABLE, 1 },
	{ "output-low", PIN_CONFIG_OUTPUT, 0 },
	{ "output-high", PIN_CONFIG_OUTPUT, 1 },
};

static int mstar_pinconf_set(struct udevice *dev, unsigned int pin,
			     unsigned int param, unsigned int arg)
{
	struct mstar_pinctrl_priv *priv = dev_get_priv(dev);
	void __iomem *reg = mstar_pad(priv->padtop, pin);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		clrbits_le16(reg, PAD_PULL);
		return 0;
	case PIN_CONFIG_BIAS_PULL_UP:
		setbits_le16(reg, PAD_PULL);
		return 0;
	case PIN_CONFIG_INPUT_ENABLE:
		setbits_le16(reg, PAD_OEN);
		return 0;
	case PIN_CONFIG_OUTPUT:
		clrsetbits_le16(reg, PAD_OUT, arg ? PAD_OUT : 0);
		clrbits_le16(reg, PAD_OEN);
		return 0;
	}

	return -EOPNOTSUPP;
}

static int mstar_pinconf_group_set(struct udevice *dev,
				   unsigned int group_selector,
				   unsigned int param, unsigned int arg)
{
	const struct mstar_group_desc *group = &mstar_groups[group_selector];
	unsigned int i;
	int ret;

	for (i = 0; i < group->npins; i++) {
		ret = mstar_pinconf_set(dev, group->pins[i], param, arg);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinctrl_ops mstar_pinctrl_ops = {
	.get_pins_count		= mstar_get_pins_count,
	.get_pin_name		= mstar_get_pin_name,
	.get_groups_count	= mstar_get_groups_count,
	.get_group_name		= mstar_get_group_name,
	.get_functions_count	= mstar_get_functions_count,
	.get_function_name	= mstar_get_function_name,
	.pinmux_group_set	= mstar_pinmux_group_set,
	.pinconf_num_params	= ARRAY_SIZE(mstar_pinconf_params),
	.pinconf_params		= mstar_pinconf_params,
	.pinconf_set		= mstar_pinconf_set,
	.pinconf_group_set	= mstar_pinconf_group_set,
	.set_state		= pinctrl_generic_set_state,
};

static int mstar_pinctrl_get_regmap(struct udevice *dev, const char *prop,
				    struct regmap **map)
{
	struct udevice *syscon;
	int ret;

	ret = uclass_get_device_by_phandle(UCLASS_SYSCON, dev, prop, &syscon);
	if (ret)
		return ret;
	*map = syscon_get_regmap(syscon);
	return *map ? 0 : -ENODEV;
}

static int mstar_pinctrl_probe(struct udevice *dev)
{
	struct mstar_pinctrl_priv *priv = dev_get_priv(dev);
	int ret;

	priv->padtop = dev_read_addr_ptr(dev);
	if (!priv->padtop)
		return -EINVAL;
	ret = mstar_pinctrl_get_regmap(dev, "sstar,chiptop", &priv->chiptop);
	if (ret)
		return ret;
	ret = mstar_pinctrl_get_regmap(dev, "sstar,pmsleep", &priv->pmsleep);
	if (ret)
		return ret;

	/* release all pads from the reset-time "all pads input" mode */
	return regmap_update_bits(priv->chiptop, CHIP_ALLPAD, CHIP_ALLPAD_IN, 0);
}

static int mstar_pinctrl_bind(struct udevice *dev)
{
	return device_bind_driver_to_node(dev, "mstar_gpio", "gpio",
					  dev_ofnode(dev), NULL);
}

static const struct udevice_id mstar_pinctrl_ids[] = {
	{ .compatible = "sstar,infinity6e-pinctrl" },
	{ }
};

U_BOOT_DRIVER(mstar_pinctrl) = {
	.name		= "mstar_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= mstar_pinctrl_ids,
	.bind		= mstar_pinctrl_bind,
	.probe		= mstar_pinctrl_probe,
	.ops		= &mstar_pinctrl_ops,
	.priv_auto	= sizeof(struct mstar_pinctrl_priv),
};

struct mstar_gpio_priv {
	void __iomem *padtop;
};

static void __iomem *mstar_gpio_reg(struct udevice *dev, unsigned int offset)
{
	struct mstar_gpio_priv *priv = dev_get_priv(dev);

	return mstar_pad(priv->padtop, offset);
}

static int mstar_gpio_direction_input(struct udevice *dev, unsigned int offset)
{
	setbits_le16(mstar_gpio_reg(dev, offset), PAD_OEN);
	return 0;
}

static int mstar_gpio_direction_output(struct udevice *dev,
				       unsigned int offset, int value)
{
	void __iomem *reg = mstar_gpio_reg(dev, offset);

	clrsetbits_le16(reg, PAD_OUT, value ? PAD_OUT : 0);
	clrbits_le16(reg, PAD_OEN);
	return 0;
}

static int mstar_gpio_get_value(struct udevice *dev, unsigned int offset)
{
	return !!(readw(mstar_gpio_reg(dev, offset)) & PAD_IN);
}

static int mstar_gpio_set_value(struct udevice *dev, unsigned int offset,
				int value)
{
	clrsetbits_le16(mstar_gpio_reg(dev, offset), PAD_OUT,
			value ? PAD_OUT : 0);
	return 0;
}

static int mstar_gpio_get_function(struct udevice *dev, unsigned int offset)
{
	if (readw(mstar_gpio_reg(dev, offset)) & PAD_OEN)
		return GPIOF_INPUT;
	return GPIOF_OUTPUT;
}

static int mstar_gpio_probe(struct udevice *dev)
{
	struct mstar_gpio_priv *priv = dev_get_priv(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	priv->padtop = dev_read_addr_ptr(dev);
	if (!priv->padtop)
		return -EINVAL;
	uc_priv->bank_name = "gpio";
	uc_priv->gpio_count = ARRAY_SIZE(infinity6e_pins);
	return 0;
}

static const struct dm_gpio_ops mstar_gpio_ops = {
	.direction_input	= mstar_gpio_direction_input,
	.direction_output	= mstar_gpio_direction_output,
	.get_value		= mstar_gpio_get_value,
	.set_value		= mstar_gpio_set_value,
	.get_function		= mstar_gpio_get_function,
};

U_BOOT_DRIVER(mstar_gpio) = {
	.name		= "mstar_gpio",
	.id		= UCLASS_GPIO,
	.probe		= mstar_gpio_probe,
	.ops		= &mstar_gpio_ops,
	.priv_auto	= sizeof(struct mstar_gpio_priv),
};
