// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar Infinity6E EMAC and internal Albany PHY.
 *
 * The MAC is AT91RM9200-like but sits behind SigmaStar's 16-bit RIU.  The
 * hardware setup and descriptor format are derived from the vendor driver.
 */
#include <config.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <miiphy.h>
#include <net.h>
#include <phy.h>
#include <time.h>
#include <asm/io.h>
#include <asm/arch/mstar.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/if_ether.h>

#define EMAC_CTL		0x00
#define EMAC_CFG		0x04
#define EMAC_SR			0x08
#define EMAC_TAR		0x0c
#define EMAC_TCR		0x10
#define EMAC_TSR		0x14
#define EMAC_RBQP		0x18
#define EMAC_RBCFG		0x1c
#define EMAC_MAN		0x34
#define EMAC_RBRP		0x38
#define EMAC_RBWP		0x3c
#define EMAC_SA1L		0x98
#define EMAC_SA1H		0x9c
#define EMAC_SA2L		0xa0
#define EMAC_SA4H		0xb4
#define EMAC_J100		0x100
#define EMAC_J104		0x104
#define EMAC_J134		0x134
#define EMAC_J138		0x138

#define EMAC_CTL_RE		BIT(2)
#define EMAC_CTL_TE		BIT(3)
#define EMAC_CTL_MPE		BIT(4)
#define EMAC_CTL_CSR		BIT(5)
#define EMAC_CFG_SPD		BIT(0)
#define EMAC_CFG_FD		BIT(1)
#define EMAC_CFG_CLK		GENMASK(11, 10)
#define EMAC_CFG_CLK_HCLK_32	2
#define EMAC_SR_IDLE		BIT(2)
#define EMAC_TSR_IDLE		BIT(3)
#define EMAC_TSR_BNQ		BIT(4)
#define EMAC_MAN_SOF		GENMASK(31, 30)
#define EMAC_MAN_RW		GENMASK(29, 28)
#define EMAC_MAN_RW_W		1
#define EMAC_MAN_RW_R		2
#define EMAC_MAN_PHYA		GENMASK(27, 23)
#define EMAC_MAN_REGA		GENMASK(22, 18)
#define EMAC_MAN_CODE		GENMASK(17, 16)
#define EMAC_MAN_DATA		GENMASK(15, 0)
#define EMAC_J100_ALBANY	0x0000f011
#define EMAC_J104_SW_DESC	BIT(0)
#define EMAC_DESC_DONE		BIT(0)
#define EMAC_DESC_WRAP		BIT(1)
#define EMAC_DESC_LEN		GENMASK(10, 0)
#define EMAC_RBCFG_SEL		1

/* Registers and fields below are named after the vendor driver, meaning unknown */
#define EMAC_J100_BIT19		BIT(19)
#define EMAC_J134_F20		GENMASK(23, 20)
#define EMAC_J134_F24		GENMASK(25, 24)
#define EMAC_J134_BIT18		BIT(18)
#define EMAC_J134_BIT28		BIT(28)
#define EMAC_J134_BIT29		BIT(29)
#define EMAC_J138_BIT0		BIT(0)

#define EMAC_EMAC1_R00		0x00
#define EMAC_EMAC1_R00_BIT4	BIT(4)
#define EMAC_EMAC1_R39		0x39
#define EMAC_EMAC1_R39_BIT0	BIT(0)
#define EMAC_EMAC1_R3A		0x3a
#define EMAC_EMAC1_R3B		0x3b
#define EMAC_ALBANY2_R80	0x80
#define EMAC_ALBANY2_R80_KEEP	GENMASK(3, 0)

#define EMAC_EMAC1_OFF		0x200
#define EMAC_ALBANY0_OFF	0x000
#define EMAC_ALBANY1_OFF	0x200
#define EMAC_ALBANY2_OFF	0x400

#define EMAC_RX_DESCS		128
#define EMAC_RX_BUF_SIZE	0x600
#define EMAC_RX_ALIGN		0x4000
#define EMAC_RX_BUFS_SIZE	(EMAC_RX_DESCS * EMAC_RX_BUF_SIZE)
#define EMAC_RX_MEM_SIZE	(EMAC_RX_BUFS_SIZE + \
				 EMAC_RX_DESCS * sizeof(struct mstar_emac_desc))
#define EMAC_MDIO_TIMEOUT_MS	100
#define EMAC_TX_TIMEOUT_MS	100

struct mstar_emac_desc {
	u32 addr;
	u32 size;
};

struct mstar_emac {
	void __iomem *base;
	void __iomem *emac1;
	void __iomem *albany;
	u8 *mem;
	u8 *rx_bufs;
	u8 *tx_bufs;
	struct mstar_emac_desc *rx_ring;
	unsigned int rx_next;
	unsigned int tx_next;
	bool rx_pending;
	struct mii_dev *bus;
	struct phy_device *phydev;
	struct clk ahb_clk;
	struct clk rx_clk;
	struct clk tx_clk;
};

static u32 emac_readl(struct mstar_emac *priv, unsigned int reg)
{
	return (readl(priv->base + reg * 2) & 0xffff) |
	       (readl(priv->base + reg * 2 + 4) & 0xffff) << 16;
}

static void emac_writel(struct mstar_emac *priv, u32 val, unsigned int reg)
{
	writel(val & 0xffff, priv->base + reg * 2);
	writel(val >> 16, priv->base + reg * 2 + 4);
}

static void emac_clrsetbits(struct mstar_emac *priv, unsigned int reg, u32 clr,
			    u32 set)
{
	emac_writel(priv, (emac_readl(priv, reg) & ~clr) | set, reg);
}

static void riu_writeb(u8 val, void __iomem *base, unsigned int reg)
{
	writeb(val, base + (reg << 1) - (reg & 1));
}

static u8 riu_readb(void __iomem *base, unsigned int reg)
{
	return readb(base + (reg << 1) - (reg & 1));
}

static int mstar_emac_mdio_xfer(struct mstar_emac *priv, u32 man)
{
	u32 ctl = emac_readl(priv, EMAC_CTL);
	ulong start = get_timer(0);

	emac_writel(priv, ctl | EMAC_CTL_MPE, EMAC_CTL);
	emac_writel(priv, FIELD_PREP(EMAC_MAN_SOF, 1) |
		    FIELD_PREP(EMAC_MAN_CODE, 2) | man, EMAC_MAN);
	while (!(emac_readl(priv, EMAC_SR) & EMAC_SR_IDLE)) {
		if (get_timer(start) > EMAC_MDIO_TIMEOUT_MS) {
			emac_writel(priv, ctl, EMAC_CTL);
			return -ETIMEDOUT;
		}
	}
	man = FIELD_GET(EMAC_MAN_DATA, emac_readl(priv, EMAC_MAN));
	emac_writel(priv, ctl, EMAC_CTL);
	return man;
}

static int mstar_emac_mdio_read(struct mii_dev *bus, int addr, int devad,
				int reg)
{
	return mstar_emac_mdio_xfer(bus->priv,
					FIELD_PREP(EMAC_MAN_RW, EMAC_MAN_RW_R) |
					FIELD_PREP(EMAC_MAN_PHYA, addr) |
					FIELD_PREP(EMAC_MAN_REGA, reg));
}

static int mstar_emac_mdio_write(struct mii_dev *bus, int addr, int devad,
				 int reg, u16 val)
{
	int ret = mstar_emac_mdio_xfer(bus->priv,
					   FIELD_PREP(EMAC_MAN_RW, EMAC_MAN_RW_W) |
					   FIELD_PREP(EMAC_MAN_PHYA, addr) |
					   FIELD_PREP(EMAC_MAN_REGA, reg) |
					   FIELD_PREP(EMAC_MAN_DATA, val));

	return ret < 0 ? ret : 0;
}

struct albany_reg {
	u16 off;
	u8 reg;
	u8 val;
};

/* Raw values from the vendor init sequence */
static const struct albany_reg albany_init[] = {
	{ EMAC_ALBANY1_OFF, 0xb4, 0x02 },
	{ EMAC_ALBANY1_OFF, 0x4f, 0x02 }, { EMAC_ALBANY1_OFF, 0x51, 0x01 },
	{ EMAC_ALBANY1_OFF, 0x77, 0x18 }, { EMAC_ALBANY0_OFF, 0x72, 0xa0 },
	{ EMAC_ALBANY1_OFF, 0xfc, 0x00 }, { EMAC_ALBANY1_OFF, 0xfd, 0x00 },
	{ EMAC_ALBANY2_OFF, 0xa1, 0x80 }, { EMAC_ALBANY1_OFF, 0xcc, 0x40 },
	{ EMAC_ALBANY1_OFF, 0xbb, 0x04 }, { EMAC_ALBANY2_OFF, 0x3a, 0x00 },
	{ EMAC_ALBANY2_OFF, 0xf1, 0x00 }, { EMAC_ALBANY2_OFF, 0x8a, 0x01 },
	{ EMAC_ALBANY1_OFF, 0x3b, 0x01 }, { EMAC_ALBANY1_OFF, 0xc4, 0x44 },
	{ EMAC_ALBANY2_OFF, 0x80, 0x30 }, { EMAC_ALBANY2_OFF, 0xc5, 0x00 },
	{ EMAC_ALBANY2_OFF, 0x30, 0x43 }, { EMAC_ALBANY2_OFF, 0x39, 0x41 },
	{ EMAC_ALBANY2_OFF, 0xf2, 0xf5 }, { EMAC_ALBANY2_OFF, 0xf3, 0x0d },
	{ EMAC_ALBANY0_OFF, 0x79, 0xd0 }, { EMAC_ALBANY0_OFF, 0x77, 0x5a },
	{ EMAC_ALBANY0_OFF, 0x2d, 0x7c }, { EMAC_ALBANY2_OFF, 0xe8, 0x06 },
	{ EMAC_ALBANY0_OFF, 0x2b, 0x00 }, { EMAC_ALBANY2_OFF, 0xe8, 0x00 },
	{ EMAC_ALBANY0_OFF, 0x2b, 0x00 }, { EMAC_ALBANY2_OFF, 0xe8, 0x06 },
	{ EMAC_ALBANY0_OFF, 0xaa, 0x1c }, { EMAC_ALBANY0_OFF, 0xac, 0x1c },
	{ EMAC_ALBANY0_OFF, 0xad, 0x1c }, { EMAC_ALBANY0_OFF, 0xae, 0x1c },
	{ EMAC_ALBANY0_OFF, 0xaf, 0x1c }, { EMAC_ALBANY2_OFF, 0xe8, 0x00 },
	{ EMAC_ALBANY0_OFF, 0xaa, 0x1c }, { EMAC_ALBANY0_OFF, 0xab, 0x28 },
	{ EMAC_ALBANY1_OFF, 0xf5, 0x02 }, { EMAC_ALBANY1_OFF, 0x0f, 0xc9 },
	{ EMAC_ALBANY1_OFF, 0x89, 0x50 }, { EMAC_ALBANY1_OFF, 0x8b, 0x80 },
	{ EMAC_ALBANY1_OFF, 0x8e, 0x0e }, { EMAC_ALBANY1_OFF, 0x90, 0x04 },
	{ EMAC_ALBANY1_OFF, 0xc7, 0x80 }, { EMAC_ALBANY1_OFF, 0x10, 0x53 },
};

static void mstar_emac_clks_disable(struct mstar_emac *priv)
{
	clk_disable(&priv->tx_clk);
	clk_disable(&priv->rx_clk);
	clk_disable(&priv->ahb_clk);
}

static int mstar_emac_hw_init(struct mstar_emac *priv)
{
	unsigned int i;
	int ret;

	riu_writeb(0x00, priv->emac1, EMAC_EMAC1_R3A);
	riu_writeb(0x01, priv->emac1, EMAC_EMAC1_R3B);
	riu_writeb(riu_readb(priv->emac1, EMAC_EMAC1_R00) | EMAC_EMAC1_R00_BIT4,
		   priv->emac1, EMAC_EMAC1_R00);
	riu_writeb(riu_readb(priv->emac1, EMAC_EMAC1_R39) | EMAC_EMAC1_R39_BIT0,
		   priv->emac1, EMAC_EMAC1_R39);
	ret = clk_enable(&priv->ahb_clk);
	if (ret)
		return ret;
	ret = clk_enable(&priv->rx_clk);
	if (ret)
		goto err_ahb;
	ret = clk_enable(&priv->tx_clk);
	if (ret)
		goto err_rx;
	for (i = 0; i < ARRAY_SIZE(albany_init); i++) {
		u8 val = albany_init[i].val;

		if (albany_init[i].off == EMAC_ALBANY2_OFF &&
		    albany_init[i].reg == EMAC_ALBANY2_R80)
			val |= riu_readb(priv->albany + EMAC_ALBANY2_OFF,
					 EMAC_ALBANY2_R80) & EMAC_ALBANY2_R80_KEEP;
		riu_writeb(val, priv->albany + albany_init[i].off,
			   albany_init[i].reg);
	}
	return 0;

err_rx:
	clk_disable(&priv->rx_clk);
err_ahb:
	clk_disable(&priv->ahb_clk);
	return ret;
}

static int mstar_emac_phy_init(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mstar_emac *priv = dev_get_priv(dev);
	struct mii_dev *bus;
	int ret;

	bus = mdio_alloc();
	if (!bus)
		return -ENOMEM;
	bus->read = mstar_emac_mdio_read;
	bus->write = mstar_emac_mdio_write;
	bus->priv = priv;
	snprintf(bus->name, sizeof(bus->name), "%s", dev->name);
	ret = mdio_register(bus);
	if (ret) {
		mdio_free(bus);
		return ret;
	}
	priv->bus = bus;

	emac_writel(priv, EMAC_J100_ALBANY, EMAC_J100);
	priv->phydev = phy_connect(bus, -1, dev, pdata->phy_interface);
	if (!priv->phydev) {
		dev_err(dev, "no PHY found\n");
		ret = -ENODEV;
		goto err_bus;
	}
	ret = phy_config(priv->phydev);
	if (ret)
		goto err_phy;
	return 0;

err_phy:
	free(priv->phydev);
err_bus:
	mdio_unregister(bus);
	mdio_free(bus);
	return ret;
}

static void mstar_emac_set_mac(struct mstar_emac *priv, const u8 *mac)
{
	emac_writel(priv, mac[3] << 24 | mac[2] << 16 | mac[1] << 8 | mac[0],
		    EMAC_SA1L);
	emac_writel(priv, mac[5] << 8 | mac[4], EMAC_SA1H);
}

static void mstar_emac_recycle(struct mstar_emac *priv)
{
	struct mstar_emac_desc *desc = &priv->rx_ring[priv->rx_next];
	unsigned int len = FIELD_GET(EMAC_DESC_LEN, readl(&desc->size));

	emac_writel(priv, emac_readl(priv, EMAC_RBRP) + ALIGN(len, 4), EMAC_RBRP);
	writel(readl(&desc->addr) & ~EMAC_DESC_DONE, &desc->addr);
	mstar_miu_flush();
	priv->rx_next = (priv->rx_next + 1) % EMAC_RX_DESCS;
	priv->rx_pending = false;
}

static int mstar_emac_start(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mstar_emac *priv = dev_get_priv(dev);
	u32 cfg = 0, rx_base = MSTAR_MIU_ADDR(priv->rx_bufs);
	unsigned int i, reg;
	int ret;

	emac_writel(priv, 0, EMAC_CTL);
	for (i = 0; i < EMAC_RX_DESCS; i++) {
		priv->rx_ring[i].addr = rx_base + i * EMAC_RX_BUF_SIZE;
		priv->rx_ring[i].size = 0;
	}
	priv->rx_ring[EMAC_RX_DESCS - 1].addr |= EMAC_DESC_WRAP;
	priv->rx_next = 0;
	priv->rx_pending = false;
	mstar_miu_flush();
	emac_writel(priv, rx_base | EMAC_RBCFG_SEL, EMAC_RBCFG);
	emac_writel(priv, 0, EMAC_RBRP);
	emac_writel(priv, 0, EMAC_RBWP);
	emac_writel(priv, MSTAR_MIU_ADDR(priv->rx_ring), EMAC_RBQP);
	mstar_emac_set_mac(priv, pdata->enetaddr);
	for (reg = EMAC_SA2L; reg <= EMAC_SA4H; reg += 4)
		emac_writel(priv, 0, reg);
	emac_writel(priv, EMAC_J104_SW_DESC, EMAC_J104);
	ret = phy_startup(priv->phydev);
	if (ret) {
		dev_err(dev, "PHY startup failed: %d\n", ret);
		return ret;
	}
	if (!priv->phydev->link) {
		dev_err(dev, "no link\n");
		return -ENETDOWN;
	}
	if (priv->phydev->speed == SPEED_100)
		cfg |= EMAC_CFG_SPD;
	if (priv->phydev->duplex == DUPLEX_FULL)
		cfg |= EMAC_CFG_FD;
	emac_writel(priv, FIELD_PREP(EMAC_CFG_CLK, EMAC_CFG_CLK_HCLK_32) | cfg,
		    EMAC_CFG);
	emac_writel(priv, EMAC_CTL_MPE, EMAC_CTL);
	emac_writel(priv, EMAC_J100_ALBANY, EMAC_J100);
	emac_clrsetbits(priv, EMAC_J100, 0, EMAC_J100_BIT19);
	emac_clrsetbits(priv, EMAC_J134,
			EMAC_J134_F20 | EMAC_J134_BIT29 | EMAC_J134_F24 |
			EMAC_J134_BIT18 | EMAC_J134_BIT28,
			FIELD_PREP(EMAC_J134_F20, 4) | EMAC_J134_BIT29 |
			FIELD_PREP(EMAC_J134_F24, 2) | EMAC_J134_BIT18 |
			EMAC_J134_BIT28);
	emac_clrsetbits(priv, EMAC_J138, 0, EMAC_J138_BIT0);
	emac_writel(priv, emac_readl(priv, EMAC_CTL) | EMAC_CTL_CSR | EMAC_CTL_RE | EMAC_CTL_TE,
		    EMAC_CTL);
	return 0;
}

static int mstar_emac_send(struct udevice *dev, void *packet, int length)
{
	struct mstar_emac *priv = dev_get_priv(dev);
	ulong start = get_timer(0);
	u8 *buf;

	while (!(emac_readl(priv, EMAC_TSR) & (EMAC_TSR_IDLE | EMAC_TSR_BNQ))) {
		if (get_timer(start) > EMAC_TX_TIMEOUT_MS)
			return -ETIMEDOUT;
	}
	buf = priv->tx_bufs + priv->tx_next * PKTSIZE_ALIGN;
	priv->tx_next ^= 1;
	memcpy(buf, packet, length);
	if (length < ETH_ZLEN) {
		memset(buf + length, 0, ETH_ZLEN - length);
		length = ETH_ZLEN;
	}
	mstar_miu_flush();
	emac_writel(priv, MSTAR_MIU_ADDR(buf), EMAC_TAR);
	emac_writel(priv, length, EMAC_TCR);
	return 0;
}

static int mstar_emac_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct mstar_emac *priv = dev_get_priv(dev);
	struct mstar_emac_desc *desc = &priv->rx_ring[priv->rx_next];
	int len;

	if (priv->rx_pending || !(readl(&desc->addr) & EMAC_DESC_DONE))
		return -EAGAIN;
	len = FIELD_GET(EMAC_DESC_LEN, readl(&desc->size));
	if (len <= ETH_FCS_LEN) {
		mstar_emac_recycle(priv);
		return -EAGAIN;
	}
	priv->rx_pending = true;
	*packetp = priv->rx_bufs + priv->rx_next * EMAC_RX_BUF_SIZE;
	return len - ETH_FCS_LEN;
}

static int mstar_emac_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct mstar_emac *priv = dev_get_priv(dev);

	if (priv->rx_pending)
		mstar_emac_recycle(priv);
	return 0;
}

static void mstar_emac_stop(struct udevice *dev)
{
	struct mstar_emac *priv = dev_get_priv(dev);

	emac_writel(priv, emac_readl(priv, EMAC_CTL) & ~(EMAC_CTL_TE | EMAC_CTL_RE),
		    EMAC_CTL);
	phy_shutdown(priv->phydev);
}

static int mstar_emac_write_hwaddr(struct udevice *dev)
{
	struct mstar_emac *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);

	mstar_emac_set_mac(priv, pdata->enetaddr);
	return 0;
}

static int mstar_emac_probe(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mstar_emac *priv = dev_get_priv(dev);
	size_t tx_off;
	int ret;

	priv->base = dev_read_addr_name_ptr(dev, "emac");
	if (!priv->base)
		return -EINVAL;
	pdata->iobase = (phys_addr_t)priv->base;
	priv->emac1 = priv->base + EMAC_EMAC1_OFF;
	priv->albany = dev_read_addr_name_ptr(dev, "phy");
	if (!priv->albany)
		return -EINVAL;
	pdata->phy_interface = dev_read_phy_mode(dev);
	if (pdata->phy_interface == PHY_INTERFACE_MODE_NA)
		return -EINVAL;
	ret = clk_get_by_name(dev, "ahb", &priv->ahb_clk);
	if (ret)
		return ret;
	ret = clk_get_by_name(dev, "rx", &priv->rx_clk);
	if (ret)
		return ret;
	ret = clk_get_by_name(dev, "tx", &priv->tx_clk);
	if (ret)
		return ret;
	priv->mem = memalign(EMAC_RX_ALIGN, EMAC_RX_MEM_SIZE + 2 * PKTSIZE_ALIGN);
	if (!priv->mem)
		return -ENOMEM;
	memset(priv->mem, 0, EMAC_RX_MEM_SIZE + 2 * PKTSIZE_ALIGN);
	priv->rx_bufs = priv->mem;
	priv->rx_ring = (struct mstar_emac_desc *)(priv->mem + EMAC_RX_BUFS_SIZE);
	tx_off = ALIGN(EMAC_RX_MEM_SIZE, PKTSIZE_ALIGN);
	priv->tx_bufs = priv->mem + tx_off;
	ret = mstar_emac_hw_init(priv);
	if (ret)
		goto err_free;
	ret = mstar_emac_phy_init(dev);
	if (ret)
		goto err_clk;
	return 0;

err_clk:
	mstar_emac_clks_disable(priv);
err_free:
	free(priv->mem);
	return ret;
}

static int mstar_emac_remove(struct udevice *dev)
{
	struct mstar_emac *priv = dev_get_priv(dev);

	mstar_emac_stop(dev);
	free(priv->phydev);
	mdio_unregister(priv->bus);
	mdio_free(priv->bus);
	mstar_emac_clks_disable(priv);
	free(priv->mem);
	return 0;
}

static const struct eth_ops mstar_emac_ops = {
	.start		= mstar_emac_start,
	.send		= mstar_emac_send,
	.recv		= mstar_emac_recv,
	.free_pkt	= mstar_emac_free_pkt,
	.stop		= mstar_emac_stop,
	.write_hwaddr	= mstar_emac_write_hwaddr,
};

static const struct udevice_id mstar_emac_ids[] = {
	{ .compatible = "sstar,infinity6e-emac" },
	{ }
};

U_BOOT_DRIVER(mstar_emac) = {
	.name		= "mstar_emac",
	.id		= UCLASS_ETH,
	.of_match	= mstar_emac_ids,
	.probe		= mstar_emac_probe,
	.remove		= mstar_emac_remove,
	.ops		= &mstar_emac_ops,
	.priv_auto	= sizeof(struct mstar_emac),
	.plat_auto	= sizeof(struct eth_pdata),
	.flags		= DM_FLAG_ALLOC_PRIV_DMA,
};
