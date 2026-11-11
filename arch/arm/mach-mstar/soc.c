// SPDX-License-Identifier: GPL-2.0+
#include <config.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/mstar.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	gd->ram_size = SZ_1M * (1UL << MSTAR_DDR_LOG2_MB);
	return 0;
}

ulong board_get_usable_ram_top(ulong total_size)
{
	return min_t(ulong, gd->ram_top, MSTAR_DDR_BASE + SZ_64M);
}

void mstar_miu_flush(void)
{
	int spins = 1000000;

	writew(0x10, MSTAR_L3_FLUSH);
	writew(0x11, MSTAR_L3_FLUSH);
	while (!(readw(MSTAR_L3_FLUSH_STATUS) & MSTAR_L3_FLUSH_DONE) && --spins)
		;
}

int print_cpuinfo(void)
{
	printf("CPU:   SigmaStar ssc338q (chip id %02x)\n", readw(MSTAR_CHIP_ID) & 0xff);
	return 0;
}
