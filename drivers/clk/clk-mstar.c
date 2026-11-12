// SPDX-License-Identifier: GPL-2.0+
/* SigmaStar Infinity6E clock controller */
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/errno.h>

#include <dt-bindings/clock/sstar-infinity6e.h>

#define MSTAR_CLKGEN_FCIE		0x10c
#define MSTAR_CLKGEN_FCIE_MASK	GENMASK(5, 0)
#define MSTAR_CLKGEN_FCIE_BOOT	BIT(5)
#define MSTAR_CLKGEN_FCIE_SEL	GENMASK(4, 2)
#define MSTAR_SCGP_FCIE		0x94
#define MSTAR_SCGP_FCIE_BOOT	BIT(7)
/* EMAC AHB clock at 123 MHz, and RX/TX clocks sourced from the PHY. */
#define MSTAR_CLKGEN_EMAC_AHB	0x108
#define MSTAR_SCGP_EMAC_RX		0x88
#define MSTAR_SCGP_EMAC_TX		0x8c

static const ulong mstar_fcie_rates[] = {
	48000000, 43200000, 40000000, 36000000,
	32000000, 20000000, 12000000, 300000,
};

struct mstar_clk_priv {
	void __iomem *clkgen;
	void __iomem *scgp;
};

static int mstar_clk_enable(struct clk *clk)
{
	struct mstar_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk_get_id(clk)) {
	case SSTAR_INFINITY6E_CLK_FCIE:
		setbits_le16(priv->scgp + MSTAR_SCGP_FCIE,
			     MSTAR_SCGP_FCIE_BOOT);
		break;
	case SSTAR_INFINITY6E_CLK_EMAC_AHB:
		writeb(0, priv->clkgen + MSTAR_CLKGEN_EMAC_AHB);
		break;
	case SSTAR_INFINITY6E_CLK_EMAC_RX:
		writeb(0, priv->scgp + MSTAR_SCGP_EMAC_RX);
		break;
	case SSTAR_INFINITY6E_CLK_EMAC_TX:
		writeb(0, priv->scgp + MSTAR_SCGP_EMAC_TX);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ulong mstar_clk_get_rate(struct clk *clk)
{
	struct mstar_clk_priv *priv = dev_get_priv(clk->dev);
	unsigned int sel;

	if (clk_get_id(clk) != SSTAR_INFINITY6E_CLK_FCIE)
		return 0;

	sel = (readw(priv->clkgen + MSTAR_CLKGEN_FCIE) &
	       MSTAR_CLKGEN_FCIE_SEL) >> 2;

	return mstar_fcie_rates[sel];
}

static ulong mstar_clk_set_rate(struct clk *clk, ulong rate)
{
	struct mstar_clk_priv *priv = dev_get_priv(clk->dev);
	unsigned int sel;

	if (clk_get_id(clk) != SSTAR_INFINITY6E_CLK_FCIE)
		return 0;

	for (sel = 0; sel < ARRAY_SIZE(mstar_fcie_rates) - 1; sel++)
		if (mstar_fcie_rates[sel] <= rate)
			break;

	clrsetbits_le16(priv->clkgen + MSTAR_CLKGEN_FCIE,
			MSTAR_CLKGEN_FCIE_MASK,
			 MSTAR_CLKGEN_FCIE_BOOT | sel << 2);

	return mstar_fcie_rates[sel];
}

static const struct clk_ops mstar_clk_ops = {
	.enable		= mstar_clk_enable,
	.get_rate	= mstar_clk_get_rate,
	.set_rate	= mstar_clk_set_rate,
};

static int mstar_clk_probe(struct udevice *dev)
{
	struct mstar_clk_priv *priv = dev_get_priv(dev);

	priv->clkgen = dev_read_addr_index_ptr(dev, 0);
	if (!priv->clkgen)
		return -EINVAL;

	priv->scgp = dev_read_addr_index_ptr(dev, 1);
	if (!priv->scgp)
		return -EINVAL;

	return 0;
}

static const struct udevice_id mstar_clk_ids[] = {
	{ .compatible = "sstar,infinity6e-clk" },
	{ }
};

U_BOOT_DRIVER(mstar_clk) = {
	.name		= "mstar_clk",
	.id		= UCLASS_CLK,
	.of_match	= mstar_clk_ids,
	.probe		= mstar_clk_probe,
	.ops		= &mstar_clk_ops,
	.priv_auto	= sizeof(struct mstar_clk_priv),
};
