// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar SSC338Q (Infinity6E) clock generator and MStar MSC313 MPLL
 *
 * U-Boot port of drivers/clk/mstar/clk-ssc338q-clkgen.c and clk-msc313-mpll.c
 * of the Linux kernel, using the same device tree bindings: the clkgen takes
 * its parents from the "clocks"/"clock-names" inputs and the clock IDs are
 * the ones of dt-bindings/clock/sstar,ssc338q-clkgen.h.
 *
 * Each clkgen clock is controlled by a 16-bit register on the RIU (4-byte
 * stride), some registers hold two clocks, one per byte. A bit of the clock's
 * field (bit 0 unless stated otherwise) gates the clock when set and the bits
 * starting at bit 2 select the parent.
 *
 * The fixed dividers in front of some mux inputs, which the kernel registers
 * as separate fixed factor clocks, are folded into the parent table here.
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <div64.h>
#include <dm/device_compat.h>
#include <asm/io.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>

#include <dt-bindings/clock/sstar,ssc338q-clkgen.h>

#define SSC338Q_CLKGEN_MUX_SHIFT	2

/* A mux input: a clkgen input clock, optionally divided by a fixed factor */
struct ssc338q_parent {
	const char *input;
	unsigned int div;
};

#define IN(_name)		{ .input = _name, .div = 1 }
#define DIV(_name, _div)	{ .input = _name, .div = _div }

struct ssc338q_clkgen_clk {
	unsigned int id;
	const char *name;
	unsigned int offset;
	/* Position of the clock's field in the register */
	unsigned int shift;
	/* Position of the gate bit relative to the field */
	unsigned int gate_bit;
	unsigned int mux_width;
	/* Register values of the parents, when not all mux inputs are used */
	const u32 *mux_table;
	const struct ssc338q_parent *parents;
	unsigned int num_parents;
};

/* Bit set once at probe */
struct ssc338q_clkgen_setup {
	unsigned int offset;
	unsigned int bit;
};

struct ssc338q_clkgen_data {
	const struct ssc338q_clkgen_setup *setup;
	unsigned int num_setup;
	const struct ssc338q_clkgen_clk *clks;
	unsigned int num_clks;
};

static const struct ssc338q_parent ssc338q_uart_parents[] = {
	IN("mpll-div5"),
	IN("mpll-div6"),
	IN("xtal-div2"),
};

static const struct ssc338q_parent ssc338q_emac_ahb_parents[] = {
	IN("mpll-div6"),
	IN("mpll-div7"),
	IN("mpll-div10"),
};

static const struct ssc338q_parent ssc338q_aesdma_parents[] = {
	IN("mpll-div10"),
	IN("mpll-div5"),
};

/* All the inputs of the SD clock mux are provided */
static const struct ssc338q_parent ssc338q_sd_parents[] = {
	DIV("utmi-192m", 4),
	DIV("mpll-div10", 2),
	DIV("utmi-160m", 4),
	DIV("mpll-div3", 8),
	DIV("utmi-160m", 5),
	DIV("utmi-160m", 8),
	IN("xtal-div2"),
	DIV("xtal-div2", 40),
};

/*
 * The other input of the SPI flash clock mux comes from the encoder PLL,
 * which has no driver.
 */
static const struct ssc338q_parent ssc338q_spi_flash_parents[] = {
	IN("xtal-div2"),
	DIV("mpll-div4", 8),
	DIV("mpll-div3", 8),
	DIV("mpll-div4", 4),
	DIV("mpll-div3", 4),
	IN("mpll-div10"),
	DIV("mpll-div4", 2),
};

static const u32 ssc338q_spi_flash_mux_table[] = { 0, 1, 2, 4, 5, 6, 7 };

/* The other inputs of the PWM clock mux are dividers of the crystal */
static const struct ssc338q_parent ssc338q_pwm_parents[] = {
	IN("xtal-div2"),
	IN("mpll-div10"),
};

static const u32 ssc338q_pwm_mux_table[] = { 0, 5 };

/* The I2C timing counters are set up for the 12 MHz input */
static const struct ssc338q_parent ssc338q_miic_parents[] = {
	IN("xtal-div2"),
};

static const u32 ssc338q_miic_mux_table[] = { 2 };

/*
 * The other inputs of the sensor clock mux come from the LPLL and the ARM
 * PLL, which have no driver.
 */
static const struct ssc338q_parent ssc338q_sr_mclk_parents[] = {
	DIV("mpll-div4", 8),
	DIV("mpll-div3", 4),
	DIV("mpll-div7", 2),
	DIV("mpll-div4", 4),
	DIV("utmi-192m", 4),
	DIV("mpll-div10", 2),
	DIV("mpll-div3", 8),
	IN("xtal"),
	DIV("mpll-div10", 4),
	IN("xtal-div2"),
	DIV("mpll-div10", 16),
};

/* The last two inputs of the MIPI CSI clock muxes are not connected */
static const struct ssc338q_parent ssc338q_csi_parents[] = {
	IN("mpll-div4"),
	IN("utmi-240m"),
	IN("mpll-div5"),
	IN("mpll-div6"),
	IN("mpll-div10"),
	DIV("utmi-160m", 4),
};

/* The other input comes from the LPLL, which has no driver */
static const struct ssc338q_parent ssc338q_mipi_tx_csi_parents[] = {
	IN("utmi-160m"),
	IN("mpll-div6"),
	DIV("mpll-div4", 2),
	IN("mpll-div4"),
	IN("utmi-240m"),
	IN("mpll-div3"),
	IN("upll-320m"),
};

static const u32 ssc338q_mipi_tx_csi_mux_table[] = { 1, 2, 3, 4, 5, 6, 7 };

/* The other input comes from the encoder PLL, which has no driver */
static const struct ssc338q_parent ssc338q_vhe_parents[] = {
	IN("utmi-480m"),
	IN("mpll-div2"),
	IN("upll-384m"),
	IN("upll-320m"),
	IN("mpll-div3"),
	IN("utmi-192m"),
	IN("mpll-div7"),
};

static const u32 ssc338q_vhe_mux_table[] = { 1, 2, 3, 4, 5, 6, 7 };

static const struct ssc338q_parent ssc338q_mfe_parents[] = {
	IN("mpll-div4"),
	IN("upll-384m"),
	IN("upll-320m"),
};

static const u32 ssc338q_mfe_mux_table[] = { 0, 1, 2 };

/* The last two inputs of the JPEG encoder clock mux are not connected */
static const struct ssc338q_parent ssc338q_jpe_parents[] = {
	IN("utmi-480m"),
	IN("mpll-div2"),
	IN("upll-384m"),
	IN("upll-320m"),
	IN("mpll-div3"),
	IN("mpll-div4"),
};

/* All the inputs of the ISP clock mux are provided */
static const struct ssc338q_parent ssc338q_isp_parents[] = {
	IN("upll-320m"),
	IN("mpll-div3"),
	IN("mpll-div4"),
	IN("utmi-192m"),
	IN("mpll-div5"),
	IN("mpll-div7"),
	DIV("mpll-div3", 4),
	IN("upll-384m"),
};

/* All the inputs of the FCLK1 clock mux are provided */
static const struct ssc338q_parent ssc338q_fclk1_parents[] = {
	IN("mpll-345m"),
	IN("mpll-div2"),
	IN("upll-384m"),
	IN("upll-320m"),
	IN("mpll-div3"),
	IN("utmi-240m"),
	IN("mpll-div5"),
	IN("mpll-div7"),
};

/*
 * The other input of the ODCLK clock mux comes from the LPLL, which has no
 * driver.
 */
static const struct ssc338q_parent ssc338q_odclk_parents[] = {
	IN("mpll-div10"),
	DIV("mpll-div10", 2),
	DIV("mpll-div10", 4),
};

/* The last input of the IVE clock mux is not connected */
static const struct ssc338q_parent ssc338q_ive_parents[] = {
	IN("mpll-div4"),
	IN("mpll-div5"),
	IN("mpll-div7"),
	IN("mpll-div10"),
	IN("mpll-div3"),
	IN("upll-320m"),
	IN("upll-384m"),
};

/* All the inputs of the MCU clock mux are provided */
static const struct ssc338q_parent ssc338q_mcu_parents[] = {
	IN("mpll-div4"),
	IN("mpll-div2"),
	IN("upll-384m"),
	DIV("mpll-div4", 2),
};

/* The last input of the LDC clock mux is not connected */
static const struct ssc338q_parent ssc338q_ldc_parents[] = {
	IN("utmi-480m"),
	IN("mpll-div2"),
	IN("upll-384m"),
	IN("upll-320m"),
	IN("mpll-div3"),
	IN("mpll-div4"),
	IN("mpll-div10"),
};

/* All the inputs of the DIP clock mux are provided */
static const struct ssc338q_parent ssc338q_dip_parents[] = {
	IN("utmi-480m"),
	IN("mpll-div2"),
	IN("upll-384m"),
	IN("upll-320m"),
	IN("mpll-div3"),
	IN("utmi-240m"),
	IN("mpll-div5"),
	IN("mpll-div7"),
};

/* The other inputs of the audio clock mux are not provided */
static const struct ssc338q_parent ssc338q_bach_parents[] = {
	IN("upll-384m"),
};

static const u32 ssc338q_bach_mux_table[] = { 0 };

/*
 * Bit 5 of the SD and SPI flash clock registers switches between the 12 MHz
 * boot clock and the clock selected by the mux, use the mux.
 */
static const struct ssc338q_clkgen_setup ssc338q_clkgen_setup[] = {
	{ .offset = 0x0e0, .bit = 5 },
	{ .offset = 0x10c, .bit = 5 },
};

static const struct ssc338q_clkgen_clk ssc338q_clkgen_clks[] = {
	{
		.id = SSC338Q_CLKGEN_UART0,
		.name = "uart0",
		.offset = 0xc4,
		.mux_width = 2,
		.parents = ssc338q_uart_parents,
		.num_parents = ARRAY_SIZE(ssc338q_uart_parents),
	},
	{
		.id = SSC338Q_CLKGEN_UART1,
		.name = "uart1",
		.offset = 0xc4,
		.shift = 8,
		.mux_width = 2,
		.parents = ssc338q_uart_parents,
		.num_parents = ARRAY_SIZE(ssc338q_uart_parents),
	},
	{
		/*
		 * The synthesizer input is not provided. The register also
		 * holds the synthesizer clock and its enable and reset.
		 */
		.id = SSC338Q_CLKGEN_FUART,
		.name = "fuart",
		.offset = 0x0d0,
		.mux_width = 2,
		.parents = ssc338q_uart_parents,
		.num_parents = ARRAY_SIZE(ssc338q_uart_parents),
	},
	{
		.id = SSC338Q_CLKGEN_EMAC_AHB,
		.name = "emac_ahb",
		.offset = 0x108,
		.mux_width = 2,
		.parents = ssc338q_emac_ahb_parents,
		.num_parents = ARRAY_SIZE(ssc338q_emac_ahb_parents),
	},
	{
		.id = SSC338Q_CLKGEN_AESDMA,
		.name = "aesdma",
		.offset = 0x184,
		.mux_width = 2,
		.parents = ssc338q_aesdma_parents,
		.num_parents = ARRAY_SIZE(ssc338q_aesdma_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SD,
		.name = "sd",
		.offset = 0x10c,
		.mux_width = 3,
		.parents = ssc338q_sd_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sd_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SPI_FLASH,
		.name = "spi_flash",
		.offset = 0x0e0,
		.mux_width = 3,
		.mux_table = ssc338q_spi_flash_mux_table,
		.parents = ssc338q_spi_flash_parents,
		.num_parents = ARRAY_SIZE(ssc338q_spi_flash_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MIIC0,
		.name = "miic0",
		.offset = 0x0dc,
		.mux_width = 2,
		.mux_table = ssc338q_miic_mux_table,
		.parents = ssc338q_miic_parents,
		.num_parents = ARRAY_SIZE(ssc338q_miic_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MIIC1,
		.name = "miic1",
		.offset = 0x0dc,
		.shift = 8,
		.mux_width = 2,
		.mux_table = ssc338q_miic_mux_table,
		.parents = ssc338q_miic_parents,
		.num_parents = ARRAY_SIZE(ssc338q_miic_parents),
	},
	{
		.id = SSC338Q_CLKGEN_PWM,
		.name = "pwm",
		.offset = 0x0e0,
		.shift = 8,
		.mux_width = 3,
		.mux_table = ssc338q_pwm_mux_table,
		.parents = ssc338q_pwm_parents,
		.num_parents = ARRAY_SIZE(ssc338q_pwm_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SR00_MCLK,
		.name = "sr00_mclk",
		.offset = 0x188,
		.shift = 8,
		.mux_width = 4,
		.parents = ssc338q_sr_mclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sr_mclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SR01_MCLK,
		.name = "sr01_mclk",
		.offset = 0x194,
		.mux_width = 4,
		.parents = ssc338q_sr_mclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sr_mclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SR1_MCLK,
		.name = "sr1_mclk",
		.offset = 0x194,
		.shift = 8,
		.mux_width = 4,
		.parents = ssc338q_sr_mclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sr_mclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_LPTX_M00,
		.name = "csi_lptx_m00",
		.offset = 0x160,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_MAC_M00,
		.name = "csi_mac_m00",
		.offset = 0x160,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_NS_M00,
		.name = "ns_m00",
		.offset = 0x164,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_LPTX_M01,
		.name = "csi_lptx_m01",
		.offset = 0x164,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_MAC_M01,
		.name = "csi_mac_m01",
		.offset = 0x168,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_NS_M01,
		.name = "ns_m01",
		.offset = 0x168,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_LPTX_M1,
		.name = "csi_lptx_m1",
		.offset = 0x16c,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_MAC_M1,
		.name = "csi_mac_m1",
		.offset = 0x16c,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_NS_M1,
		.name = "ns_m1",
		.offset = 0x170,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MIPI1_TX_CSI,
		.name = "mipi1_tx_csi",
		.offset = 0x170,
		.shift = 8,
		.mux_width = 3,
		.mux_table = ssc338q_mipi_tx_csi_mux_table,
		.parents = ssc338q_mipi_tx_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_mipi_tx_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_VHE,
		.name = "vhe",
		.offset = 0x1a0,
		.gate_bit = 7,
		.mux_width = 3,
		.mux_table = ssc338q_vhe_mux_table,
		.parents = ssc338q_vhe_parents,
		.num_parents = ARRAY_SIZE(ssc338q_vhe_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MFE,
		.name = "mfe",
		.offset = 0x1a4,
		.mux_width = 2,
		.mux_table = ssc338q_mfe_mux_table,
		.parents = ssc338q_mfe_parents,
		.num_parents = ARRAY_SIZE(ssc338q_mfe_parents),
	},
	{
		.id = SSC338Q_CLKGEN_JPE,
		.name = "jpe",
		.offset = 0x1a8,
		.mux_width = 3,
		.parents = ssc338q_jpe_parents,
		.num_parents = ARRAY_SIZE(ssc338q_jpe_parents),
	},
	{
		.id = SSC338Q_CLKGEN_ISP,
		.name = "isp",
		.offset = 0x184,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_isp_parents,
		.num_parents = ARRAY_SIZE(ssc338q_isp_parents),
	},
	{
		.id = SSC338Q_CLKGEN_FCLK1,
		.name = "fclk1",
		.offset = 0x190,
		.mux_width = 3,
		.parents = ssc338q_fclk1_parents,
		.num_parents = ARRAY_SIZE(ssc338q_fclk1_parents),
	},
	{
		.id = SSC338Q_CLKGEN_ODCLK,
		.name = "odclk",
		.offset = 0x198,
		.mux_width = 2,
		.parents = ssc338q_odclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_odclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_IVE,
		.name = "ive",
		.offset = 0x1a8,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_ive_parents,
		.num_parents = ARRAY_SIZE(ssc338q_ive_parents),
	},
	{
		/* It also clocks blocks without a driver */
		.id = SSC338Q_CLKGEN_MCU,
		.name = "mcu",
		.offset = 0x004,
		.mux_width = 2,
		.parents = ssc338q_mcu_parents,
		.num_parents = ARRAY_SIZE(ssc338q_mcu_parents),
	},
	{
		.id = SSC338Q_CLKGEN_LDC,
		.name = "ldc",
		.offset = 0x14c,
		.mux_width = 3,
		.parents = ssc338q_ldc_parents,
		.num_parents = ARRAY_SIZE(ssc338q_ldc_parents),
	},
	{
		.id = SSC338Q_CLKGEN_DIP,
		.name = "dip",
		.offset = 0x148,
		.mux_width = 3,
		.parents = ssc338q_dip_parents,
		.num_parents = ARRAY_SIZE(ssc338q_dip_parents),
	},
	{
		.id = SSC338Q_CLKGEN_BACH,
		.name = "bach",
		.offset = 0x1b4,
		.gate_bit = 7,
		.mux_width = 2,
		.mux_table = ssc338q_bach_mux_table,
		.parents = ssc338q_bach_parents,
		.num_parents = ARRAY_SIZE(ssc338q_bach_parents),
	},
	/*
	 * The BDMA clock comes from the MIU clock, which has no driver. Its
	 * parent mux is left at its reset value, which selects the MIU clock.
	 */
	{
		.id = SSC338Q_CLKGEN_BDMA,
		.name = "bdma",
		.offset = 0x180,
	},
};

static const struct ssc338q_clkgen_data ssc338q_clkgen_data = {
	.setup = ssc338q_clkgen_setup,
	.num_setup = ARRAY_SIZE(ssc338q_clkgen_setup),
	.clks = ssc338q_clkgen_clks,
	.num_clks = ARRAY_SIZE(ssc338q_clkgen_clks),
};

/*
 * The EMAC RX and TX clocks come from the internal PHY or the RMII pads.
 * Their parent mux is left at its reset value, which selects the internal
 * PHY, so they are gates without a parent.
 */
static const struct ssc338q_clkgen_clk ssc338q_scgp_clks[] = {
	{
		.id = SSC338Q_SCGP_EMAC_RX,
		.name = "emac_rx",
		.offset = 0x88,
	},
	{
		.id = SSC338Q_SCGP_EMAC_TX,
		.name = "emac_tx",
		.offset = 0x8c,
	},
};

/*
 * Bit 7 of this register switches the SD clock between the 12 MHz boot
 * clock and the clock generator output, use the clock generator.
 */
static const struct ssc338q_clkgen_setup ssc338q_scgp_setup[] = {
	{ .offset = 0x94, .bit = 7 },
};

static const struct ssc338q_clkgen_data ssc338q_scgp_data = {
	.setup = ssc338q_scgp_setup,
	.num_setup = ARRAY_SIZE(ssc338q_scgp_setup),
	.clks = ssc338q_scgp_clks,
	.num_clks = ARRAY_SIZE(ssc338q_scgp_clks),
};

struct ssc338q_clkgen_priv {
	void __iomem *base;
	const struct ssc338q_clkgen_data *data;
};

static const struct ssc338q_clkgen_clk *
ssc338q_clkgen_find(struct ssc338q_clkgen_priv *priv, unsigned long id)
{
	unsigned int i;

	for (i = 0; i < priv->data->num_clks; i++)
		if (priv->data->clks[i].id == id)
			return &priv->data->clks[i];

	return NULL;
}

static u32 ssc338q_clkgen_mux_value(const struct ssc338q_clkgen_clk *c,
				    unsigned int index)
{
	return c->mux_table ? c->mux_table[index] : index;
}

/* Index of the parent selected by the mux, or a negative errno */
static int ssc338q_clkgen_get_index(struct ssc338q_clkgen_priv *priv,
				    const struct ssc338q_clkgen_clk *c)
{
	u32 mask = BIT(c->mux_width) - 1;
	u32 val = (readw(priv->base + c->offset) >>
		   (c->shift + SSC338Q_CLKGEN_MUX_SHIFT)) & mask;
	unsigned int i;

	for (i = 0; i < c->num_parents; i++)
		if (ssc338q_clkgen_mux_value(c, i) == val)
			return i;

	return -EINVAL;
}

static void ssc338q_clkgen_set_index(struct ssc338q_clkgen_priv *priv,
				     const struct ssc338q_clkgen_clk *c,
				     unsigned int index)
{
	unsigned int shift = c->shift + SSC338Q_CLKGEN_MUX_SHIFT;
	u32 mask = (BIT(c->mux_width) - 1) << shift;

	clrsetbits_le16(priv->base + c->offset, mask,
			ssc338q_clkgen_mux_value(c, index) << shift);
}

static int ssc338q_clkgen_get_input(struct udevice *dev,
				    const struct ssc338q_parent *p,
				    struct clk *clk)
{
	return clk_get_by_name(dev, p->input, clk);
}

static ulong ssc338q_clkgen_parent_rate(struct udevice *dev,
					const struct ssc338q_parent *p)
{
	struct clk clk;
	ulong rate;

	if (ssc338q_clkgen_get_input(dev, p, &clk))
		return 0;

	rate = clk_get_rate(&clk);
	if (IS_ERR_VALUE(rate))
		return 0;

	return rate / p->div;
}

static int ssc338q_clkgen_enable(struct clk *clk)
{
	struct ssc338q_clkgen_priv *priv = dev_get_priv(clk->dev);
	const struct ssc338q_clkgen_clk *c = ssc338q_clkgen_find(priv, clk->id);

	if (!c)
		return -EINVAL;

	/* The bit gates the clock when set, leave the mux bits alone */
	clrbits_le16(priv->base + c->offset, BIT(c->shift + c->gate_bit));

	return 0;
}

static int ssc338q_clkgen_disable(struct clk *clk)
{
	struct ssc338q_clkgen_priv *priv = dev_get_priv(clk->dev);
	const struct ssc338q_clkgen_clk *c = ssc338q_clkgen_find(priv, clk->id);

	if (!c)
		return -EINVAL;

	setbits_le16(priv->base + c->offset, BIT(c->shift + c->gate_bit));

	return 0;
}

static ulong ssc338q_clkgen_get_rate(struct clk *clk)
{
	struct ssc338q_clkgen_priv *priv = dev_get_priv(clk->dev);
	const struct ssc338q_clkgen_clk *c = ssc338q_clkgen_find(priv, clk->id);
	int index;

	if (!c)
		return -EINVAL;
	if (!c->num_parents)
		return 0;

	index = ssc338q_clkgen_get_index(priv, c);
	if (index < 0)
		return 0;

	return ssc338q_clkgen_parent_rate(clk->dev, &c->parents[index]);
}

/* Pick the fastest parent that does not exceed the rate, else the slowest */
static ulong ssc338q_clkgen_set_rate(struct clk *clk, ulong rate)
{
	struct ssc338q_clkgen_priv *priv = dev_get_priv(clk->dev);
	const struct ssc338q_clkgen_clk *c = ssc338q_clkgen_find(priv, clk->id);
	ulong best = 0, slowest = 0, r;
	int best_i = -1, slowest_i = -1;
	unsigned int i;

	if (!c)
		return -EINVAL;
	if (!c->num_parents)
		return -ENOSYS;

	for (i = 0; i < c->num_parents; i++) {
		r = ssc338q_clkgen_parent_rate(clk->dev, &c->parents[i]);
		if (!r)
			continue;
		if (r <= rate && (best_i < 0 || r > best)) {
			best = r;
			best_i = i;
		}
		if (slowest_i < 0 || r < slowest) {
			slowest = r;
			slowest_i = i;
		}
	}

	if (best_i < 0) {
		best_i = slowest_i;
		best = slowest;
	}
	if (best_i < 0)
		return -EINVAL;

	ssc338q_clkgen_set_index(priv, c, best_i);

	return best;
}

static int ssc338q_clkgen_set_parent(struct clk *clk, struct clk *parent)
{
	struct ssc338q_clkgen_priv *priv = dev_get_priv(clk->dev);
	const struct ssc338q_clkgen_clk *c = ssc338q_clkgen_find(priv, clk->id);
	struct clk in;
	unsigned int i;

	if (!c)
		return -EINVAL;

	for (i = 0; i < c->num_parents; i++) {
		if (c->parents[i].div != 1)
			continue;
		if (ssc338q_clkgen_get_input(clk->dev, &c->parents[i], &in))
			continue;
		if (in.dev == parent->dev && in.id == parent->id) {
			ssc338q_clkgen_set_index(priv, c, i);
			return 0;
		}
	}

	return -EINVAL;
}

static const struct clk_ops ssc338q_clkgen_ops = {
	.enable		= ssc338q_clkgen_enable,
	.disable	= ssc338q_clkgen_disable,
	.get_rate	= ssc338q_clkgen_get_rate,
	.set_rate	= ssc338q_clkgen_set_rate,
	.set_parent	= ssc338q_clkgen_set_parent,
};

static int ssc338q_clkgen_probe(struct udevice *dev)
{
	struct ssc338q_clkgen_priv *priv = dev_get_priv(dev);
	unsigned int i;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	priv->data = (const struct ssc338q_clkgen_data *)dev_get_driver_data(dev);

	for (i = 0; i < priv->data->num_setup; i++)
		setbits_le16(priv->base + priv->data->setup[i].offset,
			     BIT(priv->data->setup[i].bit));

	return 0;
}

static const struct udevice_id ssc338q_clkgen_ids[] = {
	{ .compatible = "sstar,ssc338q-clkgen",
	  .data = (ulong)&ssc338q_clkgen_data },
	{ .compatible = "sstar,ssc338q-scgp",
	  .data = (ulong)&ssc338q_scgp_data },
	{ }
};

U_BOOT_DRIVER(ssc338q_clkgen) = {
	.name		= "ssc338q_clkgen",
	.id		= UCLASS_CLK,
	.of_match	= ssc338q_clkgen_ids,
	.probe		= ssc338q_clkgen_probe,
	.ops		= &ssc338q_clkgen_ops,
	.priv_auto	= sizeof(struct ssc338q_clkgen_priv),
};

/*
 * MSC313 MPLL: clock 0 is the PLL, clocks 1.. are its fixed dividers
 * (dt-bindings/clock/mstar-msc313-mpll.h).
 */
#define MPLL_CONFIG1			0x8
#define MPLL_CONFIG1_LOOP_DIV_FIRST	GENMASK(9, 8)
#define MPLL_CONFIG1_INPUT_DIV_FIRST	GENMASK(5, 4)
#define MPLL_CONFIG2			0xc
#define MPLL_CONFIG2_OUTPUT_DIV_FIRST	GENMASK(13, 12)
#define MPLL_CONFIG2_LOOP_DIV_SECOND	GENMASK(7, 0)

static const unsigned int msc313_mpll_dividers[] = { 2, 3, 4, 5, 6, 7, 10 };

struct msc313_mpll_priv {
	void __iomem *base;
};

static ulong msc313_mpll_get_rate(struct clk *clk)
{
	struct msc313_mpll_priv *priv = dev_get_priv(clk->dev);
	u32 cfg1 = readw(priv->base + MPLL_CONFIG1);
	u32 cfg2 = readw(priv->base + MPLL_CONFIG2);
	u32 input_div = FIELD_GET(MPLL_CONFIG1_INPUT_DIV_FIRST, cfg1);
	u32 loop_first = FIELD_GET(MPLL_CONFIG1_LOOP_DIV_FIRST, cfg1);
	u32 loop_second = FIELD_GET(MPLL_CONFIG2_LOOP_DIV_SECOND, cfg2);
	u32 output_div = FIELD_GET(MPLL_CONFIG2_OUTPUT_DIV_FIRST, cfg2);
	struct clk parent;
	u64 rate;
	int ret;

	if (clk->id > ARRAY_SIZE(msc313_mpll_dividers))
		return -EINVAL;

	ret = clk_get_by_index(clk->dev, 0, &parent);
	if (ret)
		return ret;

	rate = clk_get_rate(&parent);
	if (IS_ERR_VALUE(rate))
		return rate;

	rate >>= input_div;
	rate *= (1 << loop_first) * max(loop_second, 1U);
	do_div(rate, max(output_div, 1U));

	if (clk->id)
		do_div(rate, msc313_mpll_dividers[clk->id - 1]);

	return rate;
}

static const struct clk_ops msc313_mpll_ops = {
	.get_rate	= msc313_mpll_get_rate,
};

static int msc313_mpll_probe(struct udevice *dev)
{
	struct msc313_mpll_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	return 0;
}

static const struct udevice_id msc313_mpll_ids[] = {
	{ .compatible = "mstar,msc313-mpll" },
	{ }
};

U_BOOT_DRIVER(msc313_mpll) = {
	.name		= "msc313_mpll",
	.id		= UCLASS_CLK,
	.of_match	= msc313_mpll_ids,
	.probe		= msc313_mpll_probe,
	.ops		= &msc313_mpll_ops,
	.priv_auto	= sizeof(struct msc313_mpll_priv),
};
