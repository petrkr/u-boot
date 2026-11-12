// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar SSC338Q pin controller and GPIO
 *
 * U-Boot port of drivers/pinctrl/pinctrl-ssc338q.c of the Linux kernel, using
 * the same device tree binding (sstar,ssc338q-pinctrl) and the pin numbers of
 * dt-bindings/gpio/sstar,ssc338q-gpio.h.
 *
 * Every pad has its own 16-bit PADTOP register holding its GPIO state and
 * configuration. The function muxes live in shared PADTOP, CHIPTOP and PM
 * sleep registers. The PM pads have the same kind of register in the PM GPIO
 * block. The pads of the SAR ADC are switched between their analog input and
 * GPIO modes in the SAR block, which also holds their GPIO state.
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
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
#include <dt-bindings/gpio/sstar,ssc338q-gpio.h>

/* Pad register, 16 bits on the RIU with a 4-byte stride */
#define PAD_IN			BIT(0)
#define PAD_OUT			BIT(1)
#define PAD_OEN			BIT(2)
#define PAD_PM_GPIO_MODE	BIT(3)
#define PAD_PULL		BIT(4)
#define PAD_DRV			BIT(7)

/* SAR block, one bit per pad in each byte */
#define SAR_AISEL		0x44
#define  SAR_AISEL_ANALOG(n)	BIT(n)
#define  SAR_AISEL_OEN(n)	BIT(8 + (n))
#define SAR_GPIO		0x48
#define  SAR_GPIO_OUT(n)	BIT(n)
#define  SAR_GPIO_IN(n)		BIT(8 + (n))

/* Pad register bank */
#define PAD_BANK_PADTOP		0
#define PAD_BANK_PM		1
#define PAD_BANK_SAR		2

/* PADTOP function muxes */
#define PAD_MODE_SD		0x67
#define  PAD_MODE_SD0		BIT(8)
#define  PAD_MODE_SD0_CDZ	BIT(9)
#define PAD_MODE_I2C		0x6f
#define  PAD_MODE_I2C0		GENMASK(2, 0)
#define  PAD_MODE_I2C0_PADS	1	/* the I2C0_SCL and I2C0_SDA pads */
#define  PAD_MODE_I2C1		GENMASK(5, 4)
#define  PAD_MODE_I2C1_SR0	1	/* the SR0_IO18 and SR0_IO19 pads */
#define PAD_MODE_UART		0x6d
#define  PAD_MODE_UART1		GENMASK(6, 4)
#define PAD_MODE_FUART		0x6e
#define  PAD_MODE_FUART_MASK	GENMASK(6, 4)
#define  PAD_MODE_FUART_RX_TX	5	/* FUART_RX and FUART_TX, no flow control */
#define PAD_MODE_SPI		0x68
#define  PAD_MODE_SPI0		GENMASK(2, 0)
#define  PAD_MODE_SPI0_SD0	3	/* SPI0 mode 3 uses the SD0 pads */

/* CHIPTOP */
#define CHIP_ALLPAD		0x140
#define  CHIP_ALLPAD_IN		BIT(15)

/* PM sleep */
#define PM_LED_MODE		0xa0
#define  PM_LED_MODE_MASK	GENMASK(5, 4)
#define  PM_LED_MODE_ETH	1
#define PM_SPI_IS_GPIO		0xd4
/* SPI pads used by the flash controller, SPI_CZ2 as GPIO */
#define  PM_SPI_IS_GPIO_FSP	0x08

struct ssc338q_pin {
	const char *name;
	u8 bank;
	/* PADTOP/PM register index or SAR bit */
	u8 index;
};

#define PIN(_name, _bank, _index) \
	[SSC338Q_GPIO_##_name] = { \
		.name = #_name, .bank = PAD_BANK_##_bank, .index = _index \
	}

static const struct ssc338q_pin ssc338q_pins[SSC338Q_GPIO_COUNT] = {
	PIN(GPIO0, PADTOP, 0x00),
	PIN(GPIO1, PADTOP, 0x01),
	PIN(GPIO2, PADTOP, 0x02),
	PIN(GPIO3, PADTOP, 0x03),
	PIN(GPIO4, PADTOP, 0x04),
	PIN(GPIO5, PADTOP, 0x05),
	PIN(GPIO6, PADTOP, 0x06),
	PIN(GPIO7, PADTOP, 0x07),
	PIN(GPIO8, PADTOP, 0x08),
	PIN(GPIO9, PADTOP, 0x09),
	PIN(GPIO10, PADTOP, 0x0a),
	PIN(GPIO11, PADTOP, 0x0b),
	PIN(GPIO12, PADTOP, 0x0c),
	PIN(GPIO13, PADTOP, 0x0d),
	PIN(GPIO14, PADTOP, 0x0e),
	PIN(GPIO15, PADTOP, 0x0f),
	PIN(SD0_GPIO0, PADTOP, 0x10),
	PIN(SD0_CDZ, PADTOP, 0x11),
	PIN(SD0_D1, PADTOP, 0x12),
	PIN(SD0_D0, PADTOP, 0x13),
	PIN(SD0_CLK, PADTOP, 0x14),
	PIN(SD0_CMD, PADTOP, 0x15),
	PIN(SD0_D3, PADTOP, 0x16),
	PIN(SD0_D2, PADTOP, 0x17),
	PIN(I2S0_MCLK, PADTOP, 0x18),
	PIN(I2S0_BCK, PADTOP, 0x19),
	PIN(I2S0_WCK, PADTOP, 0x1a),
	PIN(I2S0_DI, PADTOP, 0x1b),
	PIN(I2S0_DO, PADTOP, 0x1c),
	PIN(I2C0_SCL, PADTOP, 0x1d),
	PIN(I2C0_SDA, PADTOP, 0x1e),
	PIN(ETH_LED0, PADTOP, 0x20),
	PIN(ETH_LED1, PADTOP, 0x21),
	PIN(FUART_RX, PADTOP, 0x22),
	PIN(FUART_TX, PADTOP, 0x23),
	PIN(FUART_CTS, PADTOP, 0x24),
	PIN(FUART_RTS, PADTOP, 0x25),
	PIN(SD1_CDZ, PADTOP, 0x26),
	PIN(SD1_D1, PADTOP, 0x27),
	PIN(SD1_D0, PADTOP, 0x28),
	PIN(SD1_CLK, PADTOP, 0x29),
	PIN(SD1_CMD, PADTOP, 0x2a),
	PIN(SD1_D3, PADTOP, 0x2b),
	PIN(SD1_D2, PADTOP, 0x2c),
	PIN(SD1_GPIO0, PADTOP, 0x2d),
	PIN(SD1_GPIO1, PADTOP, 0x2e),
	PIN(SPI_CZ, PADTOP, 0x30),
	PIN(SPI_CK, PADTOP, 0x31),
	PIN(SPI_DI, PADTOP, 0x32),
	PIN(SPI_DO, PADTOP, 0x33),
	PIN(SPI_WPZ, PADTOP, 0x34),
	PIN(SPI_HLD, PADTOP, 0x35),
	PIN(SR0_IO00, PADTOP, 0x36),
	PIN(SR0_IO01, PADTOP, 0x37),
	PIN(SR0_IO02, PADTOP, 0x38),
	PIN(SR0_IO03, PADTOP, 0x39),
	PIN(SR0_IO04, PADTOP, 0x3a),
	PIN(SR0_IO05, PADTOP, 0x3b),
	PIN(SR0_IO06, PADTOP, 0x3c),
	PIN(SR0_IO07, PADTOP, 0x3d),
	PIN(SR0_IO08, PADTOP, 0x3e),
	PIN(SR0_IO09, PADTOP, 0x3f),
	PIN(SR0_IO10, PADTOP, 0x40),
	PIN(SR0_IO11, PADTOP, 0x41),
	PIN(SR0_IO12, PADTOP, 0x42),
	PIN(SR0_IO13, PADTOP, 0x43),
	PIN(SR0_IO14, PADTOP, 0x44),
	PIN(SR0_IO15, PADTOP, 0x45),
	PIN(SR0_IO16, PADTOP, 0x46),
	PIN(SR0_IO17, PADTOP, 0x47),
	PIN(SR0_IO18, PADTOP, 0x48),
	PIN(SR0_IO19, PADTOP, 0x49),
	PIN(SR1_IO00, PADTOP, 0x4a),
	PIN(SR1_IO01, PADTOP, 0x4b),
	PIN(SR1_IO02, PADTOP, 0x4c),
	PIN(SR1_IO03, PADTOP, 0x4d),
	PIN(SR1_IO04, PADTOP, 0x4e),
	PIN(SR1_IO05, PADTOP, 0x4f),
	PIN(SR1_IO06, PADTOP, 0x50),
	PIN(SR1_IO07, PADTOP, 0x51),
	PIN(SR1_IO08, PADTOP, 0x52),
	PIN(SR1_IO09, PADTOP, 0x53),
	PIN(SR1_IO10, PADTOP, 0x54),
	PIN(SR1_IO11, PADTOP, 0x55),
	PIN(SR1_IO12, PADTOP, 0x56),
	PIN(SR1_IO13, PADTOP, 0x57),
	PIN(SR1_IO14, PADTOP, 0x58),
	PIN(SR1_IO15, PADTOP, 0x59),
	PIN(SR1_IO16, PADTOP, 0x5a),
	PIN(SR1_IO17, PADTOP, 0x5b),
	PIN(SR1_IO18, PADTOP, 0x5c),
	PIN(SR1_IO19, PADTOP, 0x5d),
	PIN(PM_GPIO0, PM, 0x00),
	PIN(PM_GPIO1, PM, 0x01),
	PIN(PM_GPIO2, PM, 0x02),
	PIN(PM_GPIO3, PM, 0x03),
	PIN(PM_GPIO4, PM, 0x04),
	PIN(PM_GPIO5, PM, 0x05),
	PIN(PM_GPIO6, PM, 0x06),
	PIN(PM_GPIO7, PM, 0x07),
	PIN(PM_GPIO8, PM, 0x08),
	PIN(PM_GPIO9, PM, 0x09),
	PIN(PM_GPIO10, PM, 0x10),
	PIN(SAR_GPIO0, SAR, 0),
	PIN(SAR_GPIO1, SAR, 1),
	PIN(SAR_GPIO2, SAR, 2),
	PIN(SAR_GPIO3, SAR, 3),
};

static const unsigned int ssc338q_sd0_pins[] = {
	SSC338Q_GPIO_SD0_D1, SSC338Q_GPIO_SD0_D0, SSC338Q_GPIO_SD0_CLK,
	SSC338Q_GPIO_SD0_CMD, SSC338Q_GPIO_SD0_D3, SSC338Q_GPIO_SD0_D2,
};

static const unsigned int ssc338q_sd0_cdz_pins[] = {
	SSC338Q_GPIO_SD0_CDZ,
};

static const unsigned int ssc338q_i2c0_pins[] = {
	SSC338Q_GPIO_I2C0_SCL, SSC338Q_GPIO_I2C0_SDA,
};

static const unsigned int ssc338q_i2c1_pins[] = {
	SSC338Q_GPIO_SR0_IO18, SSC338Q_GPIO_SR0_IO19,
};

/* UART1 pad pairs; the vendor looks the mode up by the first pad as TX */
static const unsigned int ssc338q_uart1_gpio0_pins[] = {
	SSC338Q_GPIO_GPIO0, SSC338Q_GPIO_GPIO1,
};

static const unsigned int ssc338q_uart1_i2s0_pins[] = {
	SSC338Q_GPIO_I2S0_DI, SSC338Q_GPIO_I2S0_DO,
};

static const unsigned int ssc338q_uart1_eth_led_pins[] = {
	SSC338Q_GPIO_ETH_LED0, SSC338Q_GPIO_ETH_LED1,
};

static const unsigned int ssc338q_uart1_gpio4_pins[] = {
	SSC338Q_GPIO_GPIO4, SSC338Q_GPIO_GPIO5,
};

static const unsigned int ssc338q_uart1_fuart_pins[] = {
	SSC338Q_GPIO_FUART_CTS, SSC338Q_GPIO_FUART_RTS,
};

static const unsigned int ssc338q_uart1_gpio8_pins[] = {
	SSC338Q_GPIO_GPIO8, SSC338Q_GPIO_GPIO9,
};

static const unsigned int ssc338q_fuart_pins[] = {
	SSC338Q_GPIO_FUART_RX, SSC338Q_GPIO_FUART_TX,
};

static const unsigned int ssc338q_sar0_pins[] = { SSC338Q_GPIO_SAR_GPIO0 };
static const unsigned int ssc338q_sar1_pins[] = { SSC338Q_GPIO_SAR_GPIO1 };
static const unsigned int ssc338q_sar2_pins[] = { SSC338Q_GPIO_SAR_GPIO2 };
static const unsigned int ssc338q_sar3_pins[] = { SSC338Q_GPIO_SAR_GPIO3 };

enum ssc338q_group {
	GROUP_SD0,
	GROUP_SD0_CDZ,
	GROUP_PM_SPI,
	GROUP_PM_LED,
	GROUP_I2C0,
	GROUP_SAR0,
	GROUP_SAR1,
	GROUP_SAR2,
	GROUP_SAR3,
	GROUP_I2C1,
	GROUP_UART1_GPIO0,
	GROUP_UART1_I2S0,
	GROUP_UART1_ETH_LED,
	GROUP_UART1_GPIO4,
	GROUP_UART1_FUART,
	GROUP_UART1_GPIO8,
	GROUP_FUART,
};

struct ssc338q_group_desc {
	const char *name;
	const unsigned int *pins;
	unsigned int npins;
};

#define GROUP(_id, _name, _pins) \
	[_id] = { _name, _pins, ARRAY_SIZE(_pins) }

/* The PM sleep groups have no PADTOP pads */
static const struct ssc338q_group_desc ssc338q_groups[] = {
	GROUP(GROUP_SD0, "sd0", ssc338q_sd0_pins),
	GROUP(GROUP_SD0_CDZ, "sd0_cdz", ssc338q_sd0_cdz_pins),
	[GROUP_PM_SPI] = { "pm_spi" },
	[GROUP_PM_LED] = { "pm_led" },
	GROUP(GROUP_I2C0, "i2c0", ssc338q_i2c0_pins),
	GROUP(GROUP_SAR0, "sar0", ssc338q_sar0_pins),
	GROUP(GROUP_SAR1, "sar1", ssc338q_sar1_pins),
	GROUP(GROUP_SAR2, "sar2", ssc338q_sar2_pins),
	GROUP(GROUP_SAR3, "sar3", ssc338q_sar3_pins),
	GROUP(GROUP_I2C1, "i2c1", ssc338q_i2c1_pins),
	GROUP(GROUP_UART1_GPIO0, "uart1_gpio0", ssc338q_uart1_gpio0_pins),
	GROUP(GROUP_UART1_I2S0, "uart1_i2s0", ssc338q_uart1_i2s0_pins),
	GROUP(GROUP_UART1_ETH_LED, "uart1_eth_led", ssc338q_uart1_eth_led_pins),
	GROUP(GROUP_UART1_GPIO4, "uart1_gpio4", ssc338q_uart1_gpio4_pins),
	GROUP(GROUP_UART1_FUART, "uart1_fuart", ssc338q_uart1_fuart_pins),
	GROUP(GROUP_UART1_GPIO8, "uart1_gpio8", ssc338q_uart1_gpio8_pins),
	GROUP(GROUP_FUART, "fuart", ssc338q_fuart_pins),
};

enum ssc338q_function {
	FUNC_GPIO,
	FUNC_SD0,
	FUNC_SD0_CDZ,
	FUNC_PM_SPI,
	FUNC_ETH_LED,
	FUNC_I2C0,
	FUNC_ADC,
	FUNC_I2C1,
	FUNC_UART1,
	FUNC_FUART,
};

static const char * const ssc338q_functions[] = {
	[FUNC_GPIO]	= "gpio",
	[FUNC_SD0]	= "sd0",
	[FUNC_SD0_CDZ]	= "sd0_cdz",
	[FUNC_PM_SPI]	= "pm_spi",
	[FUNC_ETH_LED]	= "eth_led",
	[FUNC_I2C0]	= "i2c0",
	[FUNC_ADC]	= "adc",
	[FUNC_I2C1]	= "i2c1",
	[FUNC_UART1]	= "uart1",
	[FUNC_FUART]	= "fuart",
};

struct ssc338q_pinctrl_priv {
	void __iomem *padtop;
	void __iomem *pm_gpio;
	struct regmap *chiptop;
	struct regmap *pmsleep;
	struct regmap *sar;
};

/* The pad register of a PADTOP or PM pad */
static void __iomem *ssc338q_pad(struct ssc338q_pinctrl_priv *priv,
				 unsigned int pin)
{
	void __iomem *base = ssc338q_pins[pin].bank == PAD_BANK_PM ?
			     priv->pm_gpio : priv->padtop;

	return base + ssc338q_pins[pin].index * 4;
}

static int ssc338q_get_pins_count(struct udevice *dev)
{
	return ARRAY_SIZE(ssc338q_pins);
}

static const char *ssc338q_get_pin_name(struct udevice *dev,
					unsigned int selector)
{
	return ssc338q_pins[selector].name;
}

static int ssc338q_get_groups_count(struct udevice *dev)
{
	return ARRAY_SIZE(ssc338q_groups);
}

static const char *ssc338q_get_group_name(struct udevice *dev,
					  unsigned int selector)
{
	return ssc338q_groups[selector].name;
}

static int ssc338q_get_functions_count(struct udevice *dev)
{
	return ARRAY_SIZE(ssc338q_functions);
}

static const char *ssc338q_get_function_name(struct udevice *dev,
					     unsigned int selector)
{
	return ssc338q_functions[selector];
}

static int ssc338q_mux_sd0(struct ssc338q_pinctrl_priv *priv, bool enable)
{
	void __iomem *mode = priv->padtop + PAD_MODE_SD * 4;
	void __iomem *spi = priv->padtop + PAD_MODE_SPI * 4;
	unsigned int i;

	if (!enable) {
		clrbits_le16(mode, PAD_MODE_SD0);
		return 0;
	}

	/* SPI0 mode 3 claims the SD0 pads */
	if (FIELD_GET(PAD_MODE_SPI0, readw(spi)) == PAD_MODE_SPI0_SD0)
		clrbits_le16(spi, PAD_MODE_SPI0);

	setbits_le16(mode, PAD_MODE_SD0);

	for (i = 0; i < ARRAY_SIZE(ssc338q_sd0_pins); i++)
		clrbits_le16(ssc338q_pad(priv, ssc338q_sd0_pins[i]), PAD_DRV);

	return 0;
}

static int ssc338q_pinmux_group_set(struct udevice *dev,
				    unsigned int group,
				    unsigned int func)
{
	struct ssc338q_pinctrl_priv *priv = dev_get_priv(dev);
	void __iomem *mode = priv->padtop + PAD_MODE_SD * 4;
	void __iomem *i2c = priv->padtop + PAD_MODE_I2C * 4;

	switch (group) {
	case GROUP_SD0:
		if (func == FUNC_SD0 || func == FUNC_GPIO)
			return ssc338q_mux_sd0(priv, func == FUNC_SD0);
		break;
	case GROUP_SD0_CDZ:
		if (func == FUNC_SD0_CDZ) {
			setbits_le16(mode, PAD_MODE_SD0_CDZ);
			return 0;
		}
		if (func == FUNC_GPIO) {
			clrbits_le16(mode, PAD_MODE_SD0_CDZ);
			return 0;
		}
		break;
	case GROUP_I2C0:
		if (func == FUNC_I2C0 || func == FUNC_GPIO) {
			clrsetbits_le16(i2c, PAD_MODE_I2C0,
					func == FUNC_I2C0 ?
					FIELD_PREP(PAD_MODE_I2C0,
						   PAD_MODE_I2C0_PADS) : 0);
			return 0;
		}
		break;
	case GROUP_I2C1:
		if (func == FUNC_I2C1 || func == FUNC_GPIO) {
			clrsetbits_le16(i2c, PAD_MODE_I2C1,
					func == FUNC_I2C1 ?
					FIELD_PREP(PAD_MODE_I2C1,
						   PAD_MODE_I2C1_SR0) : 0);
			return 0;
		}
		break;
	case GROUP_UART1_GPIO0 ... GROUP_UART1_GPIO8:
		if (func == FUNC_UART1 || func == FUNC_GPIO) {
			unsigned int n = group - GROUP_UART1_GPIO0 + 1;

			/* The ETH_LED pads are also claimed by the PM LED mux */
			if (func == FUNC_UART1 && group == GROUP_UART1_ETH_LED)
				regmap_update_bits(priv->pmsleep, PM_LED_MODE,
						   PM_LED_MODE_MASK, 0);

			/* Only the UART1 field, the UART0 mode shares the register */
			clrsetbits_le16(priv->padtop + PAD_MODE_UART * 4,
					PAD_MODE_UART1,
					func == FUNC_UART1 ?
					FIELD_PREP(PAD_MODE_UART1, n) : 0);
			return 0;
		}
		break;
	case GROUP_FUART:
		if (func == FUNC_FUART || func == FUNC_GPIO) {
			clrsetbits_le16(priv->padtop + PAD_MODE_FUART * 4,
					PAD_MODE_FUART_MASK,
					func == FUNC_FUART ?
					FIELD_PREP(PAD_MODE_FUART_MASK,
						   PAD_MODE_FUART_RX_TX) : 0);
			return 0;
		}
		break;
	case GROUP_SAR0 ... GROUP_SAR3:
		if (func == FUNC_ADC || func == FUNC_GPIO) {
			unsigned int n = group - GROUP_SAR0;

			return regmap_update_bits(priv->sar, SAR_AISEL,
						  SAR_AISEL_ANALOG(n),
						  func == FUNC_ADC ?
						  SAR_AISEL_ANALOG(n) : 0);
		}
		break;
	case GROUP_PM_SPI:
		if (func == FUNC_PM_SPI)
			return regmap_write(priv->pmsleep, PM_SPI_IS_GPIO,
					    PM_SPI_IS_GPIO_FSP);
		break;
	case GROUP_PM_LED:
		if (func == FUNC_ETH_LED)
			return regmap_update_bits(priv->pmsleep, PM_LED_MODE,
						  PM_LED_MODE_MASK,
						  FIELD_PREP(PM_LED_MODE_MASK,
							     PM_LED_MODE_ETH));
		break;
	}

	dev_err(dev, "group %s does not support function %s\n",
		ssc338q_groups[group].name, ssc338q_functions[func]);

	return -EINVAL;
}

static const struct pinconf_param ssc338q_pinconf_params[] = {
	{ "bias-disable", PIN_CONFIG_BIAS_DISABLE, 0 },
	{ "bias-pull-up", PIN_CONFIG_BIAS_PULL_UP, 1 },
	{ "input-enable", PIN_CONFIG_INPUT_ENABLE, 1 },
	{ "output-low", PIN_CONFIG_OUTPUT, 0 },
	{ "output-high", PIN_CONFIG_OUTPUT, 1 },
};

static int ssc338q_pinconf_set(struct udevice *dev, unsigned int pin,
			       unsigned int param, unsigned int arg)
{
	struct ssc338q_pinctrl_priv *priv = dev_get_priv(dev);
	void __iomem *reg;

	if (ssc338q_pins[pin].bank == PAD_BANK_SAR)
		return -EOPNOTSUPP;

	reg = ssc338q_pad(priv, pin);

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
		clrsetbits_le16(reg, PAD_OUT | PAD_OEN, arg ? PAD_OUT : 0);
		return 0;
	}

	return -EOPNOTSUPP;
}

static int ssc338q_pinconf_group_set(struct udevice *dev, unsigned int group,
				     unsigned int param, unsigned int arg)
{
	const struct ssc338q_group_desc *grp = &ssc338q_groups[group];
	unsigned int i;
	int ret;

	for (i = 0; i < grp->npins; i++) {
		ret = ssc338q_pinconf_set(dev, grp->pins[i], param, arg);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinctrl_ops ssc338q_pinctrl_ops = {
	.get_pins_count		= ssc338q_get_pins_count,
	.get_pin_name		= ssc338q_get_pin_name,
	.get_groups_count	= ssc338q_get_groups_count,
	.get_group_name		= ssc338q_get_group_name,
	.get_functions_count	= ssc338q_get_functions_count,
	.get_function_name	= ssc338q_get_function_name,
	.pinmux_group_set	= ssc338q_pinmux_group_set,
	.pinconf_num_params	= ARRAY_SIZE(ssc338q_pinconf_params),
	.pinconf_params		= ssc338q_pinconf_params,
	.pinconf_set		= ssc338q_pinconf_set,
	.pinconf_group_set	= ssc338q_pinconf_group_set,
	.set_state		= pinctrl_generic_set_state,
};

static int ssc338q_pinctrl_get_regmap(struct udevice *dev, const char *prop,
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

static int ssc338q_pinctrl_probe(struct udevice *dev)
{
	struct ssc338q_pinctrl_priv *priv = dev_get_priv(dev);
	int ret;

	priv->padtop = dev_read_addr_index_ptr(dev, 0);
	priv->pm_gpio = dev_read_addr_index_ptr(dev, 1);
	if (!priv->padtop || !priv->pm_gpio)
		return -EINVAL;

	ret = ssc338q_pinctrl_get_regmap(dev, "sstar,chiptop", &priv->chiptop);
	if (ret)
		return ret;
	ret = ssc338q_pinctrl_get_regmap(dev, "sstar,pmsleep", &priv->pmsleep);
	if (ret)
		return ret;
	ret = ssc338q_pinctrl_get_regmap(dev, "sstar,sar", &priv->sar);
	if (ret)
		return ret;

	/* Release the pads from the reset-time "all pads input" mode */
	return regmap_update_bits(priv->chiptop, CHIP_ALLPAD, CHIP_ALLPAD_IN, 0);
}

static int ssc338q_pinctrl_bind(struct udevice *dev)
{
	return device_bind_driver_to_node(dev, "ssc338q_gpio", "gpio",
					  dev_ofnode(dev), NULL);
}

static const struct udevice_id ssc338q_pinctrl_ids[] = {
	{ .compatible = "sstar,ssc338q-pinctrl" },
	{ }
};

U_BOOT_DRIVER(ssc338q_pinctrl) = {
	.name		= "ssc338q_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= ssc338q_pinctrl_ids,
	.bind		= ssc338q_pinctrl_bind,
	.probe		= ssc338q_pinctrl_probe,
	.ops		= &ssc338q_pinctrl_ops,
	.priv_auto	= sizeof(struct ssc338q_pinctrl_priv),
};

/*
 * The GPIO driver is bound to the pin controller node and shares the
 * pin controller's state through its parent.
 */
static struct ssc338q_pinctrl_priv *ssc338q_gpio_priv(struct udevice *dev)
{
	return dev_get_priv(dev->parent);
}

static int ssc338q_gpio_request(struct udevice *dev, unsigned int offset,
				const char *label)
{
	struct ssc338q_pinctrl_priv *priv = ssc338q_gpio_priv(dev);
	unsigned int n = ssc338q_pins[offset].index;

	switch (ssc338q_pins[offset].bank) {
	case PAD_BANK_PM:
		setbits_le16(ssc338q_pad(priv, offset), PAD_PM_GPIO_MODE);
		break;
	case PAD_BANK_SAR:
		return regmap_update_bits(priv->sar, SAR_AISEL,
					  SAR_AISEL_ANALOG(n), 0);
	}

	return 0;
}

static int ssc338q_gpio_direction_input(struct udevice *dev,
					unsigned int offset)
{
	struct ssc338q_pinctrl_priv *priv = ssc338q_gpio_priv(dev);
	unsigned int n = ssc338q_pins[offset].index;

	if (ssc338q_pins[offset].bank == PAD_BANK_SAR)
		return regmap_update_bits(priv->sar, SAR_AISEL,
					  SAR_AISEL_OEN(n), SAR_AISEL_OEN(n));

	setbits_le16(ssc338q_pad(priv, offset), PAD_OEN);

	return 0;
}

static int ssc338q_gpio_direction_output(struct udevice *dev,
					 unsigned int offset, int value)
{
	struct ssc338q_pinctrl_priv *priv = ssc338q_gpio_priv(dev);
	unsigned int n = ssc338q_pins[offset].index;
	int ret;

	if (ssc338q_pins[offset].bank == PAD_BANK_SAR) {
		ret = regmap_update_bits(priv->sar, SAR_GPIO, SAR_GPIO_OUT(n),
					 value ? SAR_GPIO_OUT(n) : 0);
		if (ret)
			return ret;

		return regmap_update_bits(priv->sar, SAR_AISEL,
					  SAR_AISEL_OEN(n), 0);
	}

	clrsetbits_le16(ssc338q_pad(priv, offset), PAD_OUT | PAD_OEN,
			value ? PAD_OUT : 0);

	return 0;
}

static int ssc338q_gpio_get_value(struct udevice *dev, unsigned int offset)
{
	struct ssc338q_pinctrl_priv *priv = ssc338q_gpio_priv(dev);
	unsigned int n = ssc338q_pins[offset].index;
	unsigned int val;
	int ret;

	if (ssc338q_pins[offset].bank == PAD_BANK_SAR) {
		ret = regmap_read(priv->sar, SAR_GPIO, &val);
		if (ret)
			return ret;

		return !!(val & SAR_GPIO_IN(n));
	}

	return !!(readw(ssc338q_pad(priv, offset)) & PAD_IN);
}

static int ssc338q_gpio_set_value(struct udevice *dev, unsigned int offset,
				  int value)
{
	struct ssc338q_pinctrl_priv *priv = ssc338q_gpio_priv(dev);
	unsigned int n = ssc338q_pins[offset].index;

	if (ssc338q_pins[offset].bank == PAD_BANK_SAR)
		return regmap_update_bits(priv->sar, SAR_GPIO, SAR_GPIO_OUT(n),
					  value ? SAR_GPIO_OUT(n) : 0);

	clrsetbits_le16(ssc338q_pad(priv, offset), PAD_OUT,
			value ? PAD_OUT : 0);

	return 0;
}

static int ssc338q_gpio_get_function(struct udevice *dev, unsigned int offset)
{
	struct ssc338q_pinctrl_priv *priv = ssc338q_gpio_priv(dev);
	unsigned int n = ssc338q_pins[offset].index;
	unsigned int val;
	bool input;
	int ret;

	if (ssc338q_pins[offset].bank == PAD_BANK_SAR) {
		ret = regmap_read(priv->sar, SAR_AISEL, &val);
		if (ret)
			return ret;
		input = val & SAR_AISEL_OEN(n);
	} else {
		input = readw(ssc338q_pad(priv, offset)) & PAD_OEN;
	}

	return input ? GPIOF_INPUT : GPIOF_OUTPUT;
}

static int ssc338q_gpio_probe(struct udevice *dev)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	uc_priv->bank_name = "gpio";
	uc_priv->gpio_count = ARRAY_SIZE(ssc338q_pins);

	return 0;
}

static const struct dm_gpio_ops ssc338q_gpio_ops = {
	.request		= ssc338q_gpio_request,
	.direction_input	= ssc338q_gpio_direction_input,
	.direction_output	= ssc338q_gpio_direction_output,
	.get_value		= ssc338q_gpio_get_value,
	.set_value		= ssc338q_gpio_set_value,
	.get_function		= ssc338q_gpio_get_function,
};

U_BOOT_DRIVER(ssc338q_gpio) = {
	.name	= "ssc338q_gpio",
	.id	= UCLASS_GPIO,
	.probe	= ssc338q_gpio_probe,
	.ops	= &ssc338q_gpio_ops,
};
