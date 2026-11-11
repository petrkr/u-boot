// SPDX-License-Identifier: GPL-2.0+
#include <config.h>
#include <asm/global_data.h>
#include <asm/arch/mstar.h>

DECLARE_GLOBAL_DATA_PTR;

int board_init(void)
{
	gd->bd->bi_boot_params = MSTAR_BOOT_PARAMS;
	return 0;
}
