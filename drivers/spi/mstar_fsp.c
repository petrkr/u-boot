// SPDX-License-Identifier: GPL-2.0+
/*
 * SigmaStar SSC338Q (Infinity6E) FSP/QSPI SPI memory controller.
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
#include <clk.h>
#include <mapmem.h>
#include <spi.h>
#include <spi-mem.h>
#include <wait_bit.h>
#include <watchdog.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/errno.h>

#define FSP_CTRL_BASE		(BIT(0) | BIT(1) | BIT(2))
#define FSP_BUF_SIZE		10

/*
 * The write buffer can take its data from the BDMA: the bytes from the
 * outside replace the buffer starting at the given index.
 */
#define FSP_OUT_EN		BIT(12)
#define FSP_OUT_INDEX		GENMASK(3, 0)
#define FSP_OUT_MAX		0xfff

#define QSPI_DUMMY_USER		BIT(4)
#define QSPI_DUMMY_CYCLES	GENMASK(3, 0)
#define QSPI_FUNC_ADDR2		BIT(11)
#define QSPI_FUNC_DUMMY		BIT(12)

#define BDMA_TRIGGER		BIT(0)
#define BDMA_BUSY		BIT(1)
#define BDMA_INT		BIT(2)
#define BDMA_DONE		BIT(3)
#define BDMA_STATUS_CLEAR	(BDMA_BUSY | BDMA_INT | BDMA_DONE)
#define BDMA_PATH_MIU_FSP	0x2b40
#define BDMA_PATH_SPI_MIU	0x4035

#define SPINAND_PROGRAM_LOAD	0x02
#define SPINAND_PROGRAM_LOAD_RAND	0x84

#define FSP_TIMEOUT_MS		1000

/* QSPI read modes and the read command each of them sends */
static const struct {
	u8 opcode;
	u8 mode;
	u8 addr_buswidth;
	u8 data_buswidth;
} mstar_fsp_read_modes[] = {
	{ 0x03, 0x00, 1, 1 },
	{ 0x0b, 0x01, 1, 1 },
	{ 0x3b, 0x02, 1, 2 },
	{ 0xbb, 0x03, 2, 2 },
	{ 0x6b, 0x0a, 1, 4 },
	{ 0xeb, 0x0b, 4, 4 },
};

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
	writew(BDMA_STATUS_CLEAR, BDMA_REG(priv, 0x01));
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
	writew(BDMA_STATUS_CLEAR, BDMA_REG(priv, 0x01));
	return ret;
}

/* Mode of the QSPI read path for the op, or a negative errno */
static int mstar_fsp_read_mode(const struct spi_mem_op *op)
{
	unsigned int i;

	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1 ||
	    op->addr.nbytes != 2 || op->data.dir != SPI_MEM_DATA_IN)
		return -EINVAL;

	if (op->dummy.nbytes &&
	    op->dummy.nbytes * 8 / op->dummy.buswidth > QSPI_DUMMY_CYCLES + 1)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mstar_fsp_read_modes); i++)
		if (mstar_fsp_read_modes[i].opcode == op->cmd.opcode &&
		    mstar_fsp_read_modes[i].addr_buswidth == op->addr.buswidth &&
		    mstar_fsp_read_modes[i].data_buswidth == op->data.buswidth)
			return mstar_fsp_read_modes[i].mode;

	return -EINVAL;
}

static int mstar_fsp_cache_read(struct mstar_fsp_priv *priv,
				const struct spi_mem_op *op)
{
	void *buf = op->data.buf.in;
	size_t len = op->data.nbytes;
	unsigned int cycles = 0;
	u16 func = QSPI_FUNC_ADDR2;
	int ret;

	if (op->dummy.nbytes) {
		cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;
		func |= QSPI_FUNC_DUMMY;
	}

	writew(func, QSPI_REG(priv, 0x7d));
	writew(QSPI_DUMMY_USER | FIELD_PREP(QSPI_DUMMY_CYCLES,
					    cycles ? cycles - 1 : 0),
	       QSPI_REG(priv, 0x70));
	writew(mstar_fsp_read_mode(op), QSPI_REG(priv, 0x72));
	mstar_bdma_start(priv, BDMA_PATH_SPI_MIU, op->addr.val,
			 MSTAR_MIU_ADDR(buf), len);
	ret = mstar_bdma_wait(priv);
	invalidate_dcache_range((ulong)buf,
				ALIGN((ulong)buf + len, ARCH_DMA_MINALIGN));

	return ret;
}

static bool mstar_fsp_is_program_load(const struct spi_mem_op *op)
{
	return op->data.dir == SPI_MEM_DATA_OUT && op->addr.nbytes &&
	       (op->cmd.opcode == SPINAND_PROGRAM_LOAD ||
		op->cmd.opcode == SPINAND_PROGRAM_LOAD_RAND);
}

static unsigned int mstar_fsp_header(const struct spi_mem_op *op, u8 *buf)
{
	unsigned int i, len = 0;

	buf[len++] = op->cmd.opcode;
	for (i = op->addr.nbytes; i; i--)
		buf[len++] = op->addr.val >> ((i - 1) * 8);
	for (i = 0; i < op->dummy.nbytes; i++)
		buf[len++] = 0;

	return len;
}

/* Load the page cache in one operation with the data from the BDMA */
static int mstar_fsp_program_load(struct mstar_fsp_priv *priv,
				  const struct spi_mem_op *op)
{
	const void *data = op->data.buf.out;
	unsigned int len = op->data.nbytes;
	u8 hdr[FSP_BUF_SIZE];
	unsigned int hdr_len;
	int ret, bdma_ret;

	hdr_len = mstar_fsp_header(op, hdr);
	flush_dcache_range(ALIGN_DOWN((ulong)data, ARCH_DMA_MINALIGN),
			   ALIGN((ulong)data + len, ARCH_DMA_MINALIGN));

	mstar_fsp_setup(priv);
	mstar_fsp_load(priv, hdr, hdr_len);
	writew(hdr_len, FSP_REG(priv, 0x6a));
	writew(0, FSP_REG(priv, 0x6b));
	/* Size and index as programmed by the vendor driver */
	writew(len + 1, FSP_REG(priv, 0x78));
	writew(FSP_OUT_EN | FIELD_PREP(FSP_OUT_INDEX, hdr_len),
	       FSP_REG(priv, 0x79));
	mstar_bdma_start(priv, BDMA_PATH_MIU_FSP, MSTAR_MIU_ADDR(data), 0, len);
	writew(BIT(0), FSP_REG(priv, 0x6d));
	bdma_ret = mstar_bdma_wait(priv);
	ret = mstar_fsp_wait(priv);

	writew(0, FSP_REG(priv, 0x78));
	writew(0, FSP_REG(priv, 0x79));

	return ret ?: bdma_ret;
}

static bool mstar_fsp_supports_op(struct spi_slave *slave,
				  const struct spi_mem_op *op)
{
	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1)
		return false;

	/* Only the QSPI read path drives more than one data line */
	if (mstar_fsp_read_mode(op) >= 0)
		return true;
	if ((op->addr.nbytes && op->addr.buswidth != 1) ||
	    (op->dummy.nbytes && op->dummy.buswidth != 1) ||
	    (op->data.nbytes && op->data.buswidth != 1))
		return false;

	if (op->addr.nbytes > 3 ||
	    1 + op->addr.nbytes + op->dummy.nbytes > FSP_BUF_SIZE)
		return false;

	if (mstar_fsp_is_program_load(op))
		return true;

	if (op->data.dir == SPI_MEM_DATA_OUT)
		return 1 + op->addr.nbytes + op->dummy.nbytes +
		       op->data.nbytes <= FSP_BUF_SIZE;

	return true;
}

static int mstar_fsp_adjust_op_size(struct spi_slave *slave,
				    struct spi_mem_op *op)
{
	if (op->data.dir == SPI_MEM_DATA_IN && mstar_fsp_read_mode(op) < 0)
		op->data.nbytes = min_t(unsigned int, op->data.nbytes,
					FSP_BUF_SIZE);
	else if (mstar_fsp_is_program_load(op))
		op->data.nbytes = min_t(unsigned int, op->data.nbytes,
					FSP_OUT_MAX);

	return 0;
}

static int mstar_fsp_exec_op(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct mstar_fsp_priv *priv = dev_get_priv(slave->dev->parent);
	u8 tx[FSP_BUF_SIZE];
	unsigned int tx_len;

	if (!mstar_fsp_supports_op(slave, op))
		return -EOPNOTSUPP;

	if (mstar_fsp_read_mode(op) >= 0)
		return mstar_fsp_cache_read(priv, op);
	if (mstar_fsp_is_program_load(op))
		return mstar_fsp_program_load(priv, op);

	tx_len = mstar_fsp_header(op, tx);
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
	struct ofnode_phandle_args dma;
	struct clk clk;
	fdt_addr_t bdma;
	int ret;

	priv->fsp = dev_read_addr_ptr(dev);
	if (!priv->fsp)
		return -EINVAL;

	/* The BDMA registers are in the node the "dmas" property refers to */
	ret = ofnode_parse_phandle_with_args(dev_ofnode(dev), "dmas",
					     "#dma-cells", 0, 0, &dma);
	if (ret)
		return ret;
	bdma = ofnode_get_addr(dma.node);
	if (bdma == FDT_ADDR_T_NONE)
		return -EINVAL;
	priv->bdma = map_sysmem(bdma, 0);

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret)
		return ret;
	ret = clk_enable(&clk);
	if (ret && ret != -ENOSYS)
		return ret;

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
	.adjust_op_size	= mstar_fsp_adjust_op_size,
	.supports_op	= mstar_fsp_supports_op,
	.exec_op	= mstar_fsp_exec_op,
};

static const struct dm_spi_ops mstar_fsp_ops = {
	.mem_ops	= &mstar_fsp_mem_ops,
	.set_speed	= mstar_fsp_set_speed,
	.set_mode	= mstar_fsp_set_mode,
};

static const struct udevice_id mstar_fsp_ids[] = {
	{ .compatible = "sstar,ssc338q-fsp" },
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
