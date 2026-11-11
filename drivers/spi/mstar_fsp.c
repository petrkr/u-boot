// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar Infinity6E FSP/QSPI SPI memory controller.
 *
 * FSP executes short flash commands. QSPI and BDMA transfer SPI NAND page
 * cache data between the flash and DDR.
 */
#include <asm/arch/mstar.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <cpu_func.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/pinctrl.h>
#include <regmap.h>
#include <syscon.h>
#include <spi.h>
#include <spi-mem.h>
#include <wait_bit.h>
#include <watchdog.h>
#include <linux/delay.h>
#include <linux/errno.h>

#define FSP_CTRL_BASE		(BIT(0) | BIT(1) | BIT(2))
#define FSP_OUT_EN		BIT(12)
#define FSP_BUF_SIZE		10

#define QSPI_BURST_ON		0x01
#define QSPI_BURST_OFF		0x02
#define QSPI_TIMEOUT_EN	BIT(15)
#define QSPI_TIMEOUT_OFF	0x4000
#define QSPI_CKG_DUMMY		0x17
#define QSPI_MODE_FAST		0x01
#define QSPI_FUNC_CACHE		0x1800

#define BDMA_TRIGGER		BIT(0)
#define BDMA_DONE		BIT(3)
#define BDMA_PATH_MIU_FSP	0x2b40
#define BDMA_PATH_SPI_MIU	0x4035
#define BDMA_CHUNK		2048

#define SPINAND_READ_CACHE	0x03
#define SPINAND_READ_CACHE_FAST	0x0b
#define SPINAND_PROGRAM_LOAD	0x02
#define SPINAND_PROGRAM_LOAD_RAND	0x84

#define FSP_TIMEOUT_MS		1000

struct mstar_fsp_priv {
	void __iomem *fsp;
	void __iomem *bdma;
};

#define FSP_REG(priv, reg)	((priv)->fsp + (reg) * 4)
#define QSPI_REG(priv, reg)	((priv)->fsp + 0x200 + (reg) * 4)
#define BDMA_REG(priv, reg)	((priv)->bdma + (reg) * 4)

static int mstar_fsp_wait(struct mstar_fsp_priv *priv)
{
	int ret;

	ret = wait_for_bit_le16(FSP_REG(priv, 0x6e), BIT(0), true,
				FSP_TIMEOUT_MS, false);
	writew(BIT(0), FSP_REG(priv, 0x6f));
	return ret;
}

static void mstar_fsp_setup(struct mstar_fsp_priv *priv)
{
	writew(FSP_CTRL_BASE, FSP_REG(priv, 0x6c));
	writew(0, FSP_REG(priv, 0x75));
	writew(0, FSP_REG(priv, 0x76));
}

static void mstar_fsp_load(struct mstar_fsp_priv *priv, const u8 *buf,
			   unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i += 2)
		writew(buf[i] | (i + 1 < len ? buf[i + 1] << 8 : 0),
		       FSP_REG(priv, 0x60 + i / 2));
}

static int mstar_fsp_run(struct mstar_fsp_priv *priv, const u8 *tx,
			 unsigned int tx_len, u8 *rx,
		      unsigned int rx_len)
{
	unsigned int i;
	int ret;

	if (tx_len > FSP_BUF_SIZE || rx_len > FSP_BUF_SIZE)
		return -EOPNOTSUPP;

	mstar_fsp_setup(priv);
	mstar_fsp_load(priv, tx, tx_len);
	writew(tx_len, FSP_REG(priv, 0x6a));
	writew(rx_len, FSP_REG(priv, 0x6b));
	writew(BIT(0), FSP_REG(priv, 0x6d));
	ret = mstar_fsp_wait(priv);
	for (i = 0; !ret && i < rx_len; i++)
		rx[i] = readw(FSP_REG(priv, 0x65 + i / 2)) >>
			(i & 1 ? 8 : 0);

	return ret;
}

static void mstar_bdma_start(struct mstar_fsp_priv *priv, u16 path, u32 src,
			     u32 dst, u32 len)
{
	writew(0, BDMA_REG(priv, 0x00));
	writew(path, BDMA_REG(priv, 0x02));
	writew(0, BDMA_REG(priv, 0x03));
	writew(src & 0xffff, BDMA_REG(priv, 0x04));
	writew(src >> 16, BDMA_REG(priv, 0x05));
	writew(dst & 0xffff, BDMA_REG(priv, 0x06));
	writew(dst >> 16, BDMA_REG(priv, 0x07));
	writew(len & 0xffff, BDMA_REG(priv, 0x08));
	writew(len >> 16, BDMA_REG(priv, 0x09));
	writew(BDMA_TRIGGER, BDMA_REG(priv, 0x00));
}

static int mstar_bdma_wait(struct mstar_fsp_priv *priv)
{
	int ret;

	ret = wait_for_bit_le16(BDMA_REG(priv, 0x01), BDMA_DONE, true,
				FSP_TIMEOUT_MS, false);
	writew(BDMA_DONE, BDMA_REG(priv, 0x01));
	return ret;
}

static int mstar_fsp_cache_read(struct mstar_fsp_priv *priv, u32 column,
				void *buf, size_t len)
{
	int ret;

	writew(QSPI_FUNC_CACHE, QSPI_REG(priv, 0x7d));
	writew(QSPI_CKG_DUMMY, QSPI_REG(priv, 0x70));
	writew(QSPI_MODE_FAST, QSPI_REG(priv, 0x72));
	mstar_bdma_start(priv, BDMA_PATH_SPI_MIU, column, MSTAR_MIU_ADDR(buf), len);
	ret = mstar_bdma_wait(priv);
	invalidate_dcache_range((ulong)buf,
				ALIGN((ulong)buf + len, ARCH_DMA_MINALIGN));

	return ret;
}

static int mstar_fsp_program_load(struct mstar_fsp_priv *priv, u8 opcode,
				  u32 column, const void *buf, size_t len)
{
	u8 cmd[] = { opcode, column >> 8, column };
	u16 timeout = readw(QSPI_REG(priv, 0x67));
	const u8 *data = buf;
	size_t off = 0;
	int ret;

	if (timeout & QSPI_TIMEOUT_EN)
		writew(QSPI_TIMEOUT_OFF, QSPI_REG(priv, 0x67));

	writew(0, FSP_REG(priv, 0x78));
	writew(0, FSP_REG(priv, 0x79));
	ret = mstar_fsp_run(priv, cmd, sizeof(cmd), NULL, 0);
	while (!ret && off < len) {
		size_t count = min_t(size_t, len - off, BDMA_CHUNK);
		int bdma_ret;

		flush_dcache_range(ALIGN_DOWN((ulong)data + off,
					      ARCH_DMA_MINALIGN),
				   ALIGN((ulong)data + off + count,
					 ARCH_DMA_MINALIGN));
		writew(count + 1, FSP_REG(priv, 0x78));
		writew(FSP_OUT_EN, FSP_REG(priv, 0x79));
		writew(QSPI_BURST_ON, QSPI_REG(priv, 0x0a));
		writew(0, FSP_REG(priv, 0x6a));
		writew(0, FSP_REG(priv, 0x6b));
		writew(0, FSP_REG(priv, 0x75));
		mstar_bdma_start(priv, BDMA_PATH_MIU_FSP, MSTAR_MIU_ADDR(data + off), 0,
				 count);
		writew(BIT(0), FSP_REG(priv, 0x6d));
		bdma_ret = mstar_bdma_wait(priv);
		ret = mstar_fsp_wait(priv);
		if (!ret)
			ret = bdma_ret;
		off += count;
	}

	writew(0, FSP_REG(priv, 0x78));
	writew(0, FSP_REG(priv, 0x79));
	writew(QSPI_BURST_OFF, QSPI_REG(priv, 0x0a));
	if (timeout & QSPI_TIMEOUT_EN)
		writew(timeout, QSPI_REG(priv, 0x67));

	return ret;
}

static bool mstar_fsp_is_cache_read(const struct spi_mem_op *op)
{
	return op->cmd.opcode == SPINAND_READ_CACHE ||
	       op->cmd.opcode == SPINAND_READ_CACHE_FAST;
}

static bool mstar_fsp_is_program_load(const struct spi_mem_op *op)
{
	return op->cmd.opcode == SPINAND_PROGRAM_LOAD ||
	       op->cmd.opcode == SPINAND_PROGRAM_LOAD_RAND;
}

static bool mstar_fsp_supports_op(struct spi_slave *slave,
				  const struct spi_mem_op *op)
{
	unsigned int header_len;

	if (op->cmd.buswidth != 1 ||
	    (op->addr.nbytes && op->addr.buswidth != 1) ||
	    (op->dummy.nbytes && op->dummy.buswidth != 1) ||
	    (op->data.nbytes && op->data.buswidth != 1))
		return false;

	if (op->addr.nbytes > 3 || op->dummy.nbytes > 2)
		return false;

	if (op->data.dir == SPI_MEM_DATA_IN && mstar_fsp_is_cache_read(op))
		return op->addr.nbytes == 2;
	if (op->data.dir == SPI_MEM_DATA_OUT && mstar_fsp_is_program_load(op))
		return op->addr.nbytes == 2;

	header_len = 1 + op->addr.nbytes + op->dummy.nbytes;
	return header_len + op->data.nbytes <= FSP_BUF_SIZE;
}

static int mstar_fsp_exec_op(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct mstar_fsp_priv *priv = dev_get_priv(slave->dev->parent);
	u8 tx[FSP_BUF_SIZE];
	unsigned int i, tx_len = 0;

	if (!mstar_fsp_supports_op(slave, op))
		return -EOPNOTSUPP;

	if (op->data.dir == SPI_MEM_DATA_IN && mstar_fsp_is_cache_read(op))
		return mstar_fsp_cache_read(priv, op->addr.val, op->data.buf.in,
					 op->data.nbytes);
	if (op->data.dir == SPI_MEM_DATA_OUT && mstar_fsp_is_program_load(op))
		return mstar_fsp_program_load(priv, op->cmd.opcode, op->addr.val,
					   op->data.buf.out, op->data.nbytes);

	tx[tx_len++] = op->cmd.opcode;
	for (i = op->addr.nbytes; i; i--)
		tx[tx_len++] = op->addr.val >> ((i - 1) * 8);
	for (i = 0; i < op->dummy.nbytes; i++)
		tx[tx_len++] = 0;
	if (op->data.dir == SPI_MEM_DATA_OUT) {
		memcpy(tx + tx_len, op->data.buf.out, op->data.nbytes);
		tx_len += op->data.nbytes;
	}

	return mstar_fsp_run(priv, tx, tx_len, op->data.dir == SPI_MEM_DATA_IN ?
			  op->data.buf.in : NULL,
			  op->data.dir == SPI_MEM_DATA_IN ? op->data.nbytes : 0);
}

static int mstar_fsp_probe(struct udevice *dev)
{
	struct mstar_fsp_priv *priv = dev_get_priv(dev);
	struct udevice *syscon;
	struct regmap *chiptop;
	u32 bond;
	int fsp_index, ret;

	ret = uclass_get_device_by_phandle(UCLASS_SYSCON, dev, "sstar,chiptop",
					   &syscon);
	if (ret)
		return ret;
	chiptop = syscon_get_regmap(syscon);
	if (!chiptop)
		return -ENODEV;
	ret = regmap_read(chiptop, 0x120, &bond);
	if (ret)
		return ret;
	fsp_index = (bond & 0x30) ? 1 : 0;
	priv->fsp = dev_read_addr_index_ptr(dev, fsp_index);
	priv->bdma = dev_read_addr_index_ptr(dev, 2);
	if (!priv->fsp || !priv->bdma)
		return -EINVAL;
	ret = pinctrl_select_state(dev, "default");
	if (ret)
		return ret;
	writew(0, QSPI_REG(priv, 0x7a));

	return 0;
}

static int mstar_fsp_set_speed(struct udevice *bus, uint speed)
{
	return 0;
}

static int mstar_fsp_set_mode(struct udevice *bus, uint mode)
{
	return 0;
}

static const struct spi_controller_mem_ops mstar_fsp_mem_ops = {
	.supports_op	= mstar_fsp_supports_op,
	.exec_op	= mstar_fsp_exec_op,
};

static const struct dm_spi_ops mstar_fsp_ops = {
	.mem_ops	= &mstar_fsp_mem_ops,
	.set_speed	= mstar_fsp_set_speed,
	.set_mode	= mstar_fsp_set_mode,
};

static const struct udevice_id mstar_fsp_ids[] = {
	{ .compatible = "sstar,infinity6e-fsp" },
	{ }
};

U_BOOT_DRIVER(mstar_fsp_spi) = {
	.name		= "mstar-fsp-spi",
	.id		= UCLASS_SPI,
	.of_match	= mstar_fsp_ids,
	.ops		= &mstar_fsp_ops,
	.probe		= mstar_fsp_probe,
	.priv_auto	= sizeof(struct mstar_fsp_priv),
};
