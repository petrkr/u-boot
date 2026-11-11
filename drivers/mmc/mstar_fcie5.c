// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar Infinity6E FCIE5 SD host controller.
 *
 * The controller has a command/response FIFO and transfers data directly
 * between the card and DDR through its MIU DMA interface.
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/pinctrl.h>
#include <errno.h>
#include <mmc.h>
#include <asm/gpio.h>
#include <asm/arch/mstar.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <wait_bit.h>

#define SD_REG(priv, reg)	((void __iomem *)(priv)->base + (reg) * 4)

#define SD_MIE_EVENT		0x00
#define SD_MMA_PRI		0x02
#define SD_DMA_ADDR_L		0x03
#define SD_DMA_ADDR_H		0x04
#define SD_DMA_LEN_L		0x05
#define SD_DMA_LEN_H		0x06
#define SD_MIE_FUNC_CTL		0x07
#define SD_JOB_BLK_CNT		0x08
#define SD_BLK_SIZE		0x09
#define SD_CMD_RSP_SIZE		0x0a
#define SD_MODE			0x0b
#define SD_CTL			0x0c
#define SD_STS			0x0d
#define SD_BOOT_MOD		0x0e
#define SD_DDR_MOD		0x0f
#define SD_FIFO(reg)		(0x20 + (reg) / 2)
#define SD_BOOT			0x37
#define SD_FCIE_RST		0x3f

#define EV_DATA_END		BIT(0)
#define EV_CMD_END		BIT(1)
#define EV_ERR_STS		BIT(2)
#define EV_BUSY_END		BIT(4)
#define EV_R2N_RDY		BIT(5)
#define EV_ALL			(EV_DATA_END | EV_CMD_END | EV_ERR_STS | \
				 EV_BUSY_END | EV_R2N_RDY)
#define MMA_PRI_RW		(BIT(0) | BIT(1))
#define FUNC_SDIO_MODE		BIT(2)
#define MODE_CLK_EN		BIT(0)
#define MODE_BUS_4		BIT(1)
#define MODE_DMA_RD_CLK_STOP	BIT(7)
#define CTL_RSPR2_EN		BIT(0)
#define CTL_RSP_EN		BIT(1)
#define CTL_CMD_EN		BIT(2)
#define CTL_DTRX_EN		BIT(3)
#define CTL_JOB_DIR		BIT(4)
#define CTL_JOB_START		BIT(6)
#define STS_RD_CERR		BIT(0)
#define STS_WR_CERR		BIT(1)
#define STS_WR_TOUT		BIT(2)
#define STS_CMD_NORSP		BIT(3)
#define STS_CMDRSP_CERR		BIT(4)
#define STS_RD_TOUT		BIT(5)
#define STS_DAT0		BIT(8)
#define STS_ERRS		0x3f
#define STS_TOUTS		(STS_WR_TOUT | STS_CMD_NORSP | STS_RD_TOUT)
#define CMD_SIZE		(5 << 8)
#define BOOT_MOD_BOOT		BIT(2)
#define BOOT_NAND_SRAM_IMI	(BIT(0) | BIT(1) | BIT(2))
#define RST_SOFT		BIT(0)
#define RST_STS			(BIT(1) | BIT(2) | BIT(3))

#define DDR_PAD_CLK_SEL		BIT(10)
#define DDR_PAD_IN_SEL		BIT(13)
#define DDR_FALL_LATCH		BIT(14)
#define SD_DDR_MOD_TIMING	(DDR_PAD_CLK_SEL | DDR_PAD_IN_SEL | \
				 DDR_FALL_LATCH)

#define WT_RESET_MS		100
#define WT_RSP_MS		10
#define WT_READ_MS		2000
#define WT_WRITE_MS		3000
#define WT_DAT0_MS		1000
#define WT_POWERON_MS		60
#define SD_MAX_BLOCKS		1024

struct mstar_fcie5_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct mstar_fcie5_priv {
	fdt_addr_t base;
	struct clk clk;
	struct gpio_desc cd_gpio;
	u16 bus_width;
	unsigned int nrc_us;
	bool initialized;
};

static void sd_reset(struct mstar_fcie5_priv *priv)
{
	clrbits_le16(SD_REG(priv, SD_CTL), CTL_JOB_START);
	clrbits_le16(SD_REG(priv, SD_FCIE_RST), RST_SOFT);
	wait_for_bit_le16(SD_REG(priv, SD_FCIE_RST), RST_STS, true,
			  WT_RESET_MS, false);
	setbits_le16(SD_REG(priv, SD_FCIE_RST), RST_SOFT);
	wait_for_bit_le16(SD_REG(priv, SD_FCIE_RST), RST_STS, false,
			  WT_RESET_MS, false);
}

static u8 sd_fifo_byte(struct mstar_fcie5_priv *priv, unsigned int reg)
{
	return readw(SD_REG(priv, SD_FIFO(reg))) >> (reg & 1 ? 8 : 0);
}

static u32 sd_rsp_word(struct mstar_fcie5_priv *priv, unsigned int first)
{
	return sd_fifo_byte(priv, first) << 24 |
	       sd_fifo_byte(priv, first + 1) << 16 |
	       sd_fifo_byte(priv, first + 2) << 8 |
	       sd_fifo_byte(priv, first + 3);
}

static void sd_rsp_setting(uint resp_type, u16 *size, u16 *ctl)
{
	if (!(resp_type & MMC_RSP_PRESENT)) {
		*size = 0;
		*ctl = 0;
	} else if (resp_type & MMC_RSP_136) {
		*size = 0x10;
		*ctl = CTL_RSPR2_EN | CTL_RSP_EN;
	} else {
		*size = 5;
		*ctl = CTL_RSP_EN;
	}
}

static int sd_status_err(struct mstar_fcie5_priv *priv, uint resp_type)
{
	u16 status = readw(SD_REG(priv, SD_STS)) & STS_ERRS;

	if (!(resp_type & MMC_RSP_CRC))
		status &= ~STS_CMDRSP_CERR;
	if (status & STS_TOUTS)
		return -ETIMEDOUT;

	return status ? -EIO : 0;
}

static void sd_clear_status(struct mstar_fcie5_priv *priv)
{
	writew(STS_ERRS, SD_REG(priv, SD_STS));
	writew(EV_ALL, SD_REG(priv, SD_MIE_EVENT));
}

static void sd_setup_job(struct mstar_fcie5_priv *priv,
			 struct mmc_cmd *cmd, struct mmc_data *data)
{
	u16 rsp_size, rsp_ctl;
	ulong addr;
	u32 len;

	sd_rsp_setting(cmd->resp_type, &rsp_size, &rsp_ctl);
	writew((cmd->cmdarg >> 24 & 0xff) << 8 | (0x40 | cmd->cmdidx),
	       SD_REG(priv, SD_FIFO(0)));
	writew((cmd->cmdarg >> 8 & 0xff) << 8 | (cmd->cmdarg >> 16 & 0xff),
	       SD_REG(priv, SD_FIFO(2)));
	writew(cmd->cmdarg & 0xff, SD_REG(priv, SD_FIFO(4)));

	if (data) {
		addr = data->flags & MMC_DATA_READ ? (ulong)data->dest :
						     (ulong)data->src;
		len = data->blocks * data->blocksize;
		if (data->flags & MMC_DATA_WRITE)
			mstar_miu_flush();
		writew(data->blocksize, SD_REG(priv, SD_BLK_SIZE));
		writew(data->blocks, SD_REG(priv, SD_JOB_BLK_CNT));
		writew(len & 0xffff, SD_REG(priv, SD_DMA_LEN_L));
		writew(len >> 16, SD_REG(priv, SD_DMA_LEN_H));
		writew(MSTAR_MIU_ADDR(addr) & 0xffff, SD_REG(priv, SD_DMA_ADDR_L));
		writew(MSTAR_MIU_ADDR(addr) >> 16, SD_REG(priv, SD_DMA_ADDR_H));
	}

	writew(CMD_SIZE | rsp_size, SD_REG(priv, SD_CMD_RSP_SIZE));
	writew(FUNC_SDIO_MODE, SD_REG(priv, SD_MIE_FUNC_CTL));
	writew(MODE_CLK_EN | priv->bus_width |
	       (data ? MODE_DMA_RD_CLK_STOP : 0), SD_REG(priv, SD_MODE));
	writew(rsp_ctl | (data && data->flags & MMC_DATA_WRITE ?
		CTL_JOB_DIR : 0), SD_REG(priv, SD_CTL));
	writew(MMA_PRI_RW, SD_REG(priv, SD_MMA_PRI));
	writew(SD_DDR_MOD_TIMING, SD_REG(priv, SD_DDR_MOD));
	clrbits_le16(SD_REG(priv, SD_BOOT_MOD), BOOT_MOD_BOOT);
	clrbits_le16(SD_REG(priv, SD_BOOT), BOOT_NAND_SRAM_IMI);
	udelay(priv->nrc_us);
	sd_clear_status(priv);
}

static int sd_write_data(struct mstar_fcie5_priv *priv)
{
	sd_clear_status(priv);
	writew(CTL_JOB_DIR, SD_REG(priv, SD_CTL));
	setbits_le16(SD_REG(priv, SD_CTL), CTL_DTRX_EN | CTL_JOB_START);
	return wait_for_bit_le16(SD_REG(priv, SD_MIE_EVENT), EV_DATA_END, true,
				 WT_WRITE_MS, false);
}

static int mstar_fcie5_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
				struct mmc_data *data)
{
	struct mstar_fcie5_priv *priv = dev_get_priv(dev);
	bool read = data && data->flags & MMC_DATA_READ;
	int ret;

	if (!priv->initialized)
		return -ENOMEDIUM;

	sd_setup_job(priv, cmd, data);
	if (read) {
		setbits_le16(SD_REG(priv, SD_CTL),
			     CTL_CMD_EN | CTL_DTRX_EN | CTL_JOB_START);
		ret = wait_for_bit_le16(SD_REG(priv, SD_MIE_EVENT),
					EV_CMD_END | EV_DATA_END, true,
					WT_RSP_MS + WT_READ_MS, false);
	} else {
		setbits_le16(SD_REG(priv, SD_CTL), CTL_CMD_EN | CTL_JOB_START);
		ret = wait_for_bit_le16(SD_REG(priv, SD_MIE_EVENT), EV_CMD_END,
					true, WT_RSP_MS, false);
	}
	if (ret) {
		ret = sd_status_err(priv, cmd->resp_type) ?: -ETIMEDOUT;
		sd_reset(priv);
		return ret;
	}

	if ((cmd->resp_type & MMC_RSP_BUSY) &&
	    wait_for_bit_le16(SD_REG(priv, SD_STS), STS_DAT0, true,
			      WT_DAT0_MS, false)) {
		sd_reset(priv);
		return -ETIMEDOUT;
	}

	ret = sd_status_err(priv, cmd->resp_type);
	if (!ret && data && !read && sd_write_data(priv)) {
		sd_reset(priv);
		return -ETIMEDOUT;
	}
	if (!ret)
		ret = sd_status_err(priv, cmd->resp_type);

	cmd->response[0] = sd_rsp_word(priv, 1);
	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[1] = sd_rsp_word(priv, 5);
		cmd->response[2] = sd_rsp_word(priv, 9);
		cmd->response[3] = sd_rsp_word(priv, 13) & ~0xff;
	}

	return ret;
}

static unsigned int sd_nrc_for(u32 hz)
{
	if (hz >= 8000000)
		return 1;
	if (hz >= 4000000)
		return 2;
	if (hz >= 2000000)
		return 4;
	if (hz >= 1000000)
		return 8;
	if (hz >= 400000)
		return 20;

	return 27;
}

static int mstar_fcie5_set_ios(struct udevice *dev)
{
	struct mstar_fcie5_priv *priv = dev_get_priv(dev);
	struct mstar_fcie5_plat *plat = dev_get_plat(dev);
	struct mmc *mmc = &plat->mmc;
	ulong rate;

	if (mmc->clock) {
		rate = clk_set_rate(&priv->clk, mmc->clock);
		if (!rate)
			return -EINVAL;
		priv->nrc_us = sd_nrc_for(rate);
	}
	priv->bus_width = mmc->bus_width == 4 ? MODE_BUS_4 : 0;

	return 0;
}

static int mstar_fcie5_get_cd(struct udevice *dev)
{
	struct mstar_fcie5_priv *priv = dev_get_priv(dev);

	return dm_gpio_get_value(&priv->cd_gpio);
}

static int mstar_fcie5_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct mstar_fcie5_plat *plat = dev_get_plat(dev);
	struct mstar_fcie5_priv *priv = dev_get_priv(dev);
	struct mmc_config *cfg = &plat->cfg;
	int ret;

	priv->base = dev_read_addr(dev);
	if (priv->base == FDT_ADDR_T_NONE)
		return -EINVAL;

	cfg->name = dev->name;
	cfg->voltages = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
			MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33;
	cfg->host_caps = MMC_MODE_4BIT | MMC_MODE_HS | MMC_MODE_HS_52MHz;
	cfg->f_min = 300000;
	cfg->f_max = 48000000;
	cfg->b_max = SD_MAX_BLOCKS;
	if (mmc_of_parse(dev, cfg))
		return -EINVAL;

	ret = gpio_request_by_name(dev, "cd-gpios", 0, &priv->cd_gpio,
				   GPIOD_IS_IN);
	if (ret)
		return ret;

	ret = clk_get_by_name(dev, "fcie", &priv->clk);
	if (ret)
		return ret;

	ret = clk_enable(&priv->clk);
	if (ret)
		return ret;

	if (!clk_set_rate(&priv->clk, cfg->f_max))
		return -EINVAL;

	upriv->mmc = &plat->mmc;
	priv->nrc_us = sd_nrc_for(cfg->f_min);
	ret = pinctrl_select_state(dev, "sleep");
	if (ret)
		return ret;
	writew(FUNC_SDIO_MODE, SD_REG(priv, SD_MIE_FUNC_CTL));
	clrbits_le16(SD_REG(priv, SD_MODE), MODE_CLK_EN);

	if (!mstar_fcie5_get_cd(dev))
		return 0;

	ret = pinctrl_select_state(dev, "default");
	if (ret)
		return ret;
	writew(FUNC_SDIO_MODE, SD_REG(priv, SD_MIE_FUNC_CTL));
	setbits_le16(SD_REG(priv, SD_MODE), MODE_CLK_EN);
	mdelay(WT_POWERON_MS);
	sd_reset(priv);
	priv->initialized = true;

	return 0;
}

static int mstar_fcie5_bind(struct udevice *dev)
{
	struct mstar_fcie5_plat *plat = dev_get_plat(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct dm_mmc_ops mstar_fcie5_ops = {
	.send_cmd = mstar_fcie5_send_cmd,
	.set_ios = mstar_fcie5_set_ios,
	.get_cd = mstar_fcie5_get_cd,
};

static const struct udevice_id mstar_fcie5_ids[] = {
	{ .compatible = "sstar,infinity6e-fcie5" },
	{ }
};

U_BOOT_DRIVER(mstar_fcie5) = {
	.name = "mstar_fcie5",
	.id = UCLASS_MMC,
	.of_match = mstar_fcie5_ids,
	.bind = mstar_fcie5_bind,
	.probe = mstar_fcie5_probe,
	.ops = &mstar_fcie5_ops,
	.plat_auto = sizeof(struct mstar_fcie5_plat),
	.priv_auto = sizeof(struct mstar_fcie5_priv),
};
