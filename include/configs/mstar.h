/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __CONFIG_MSTAR_H
#define __CONFIG_MSTAR_H

#include <asm/arch/mstar.h>

#define CFG_SYS_SDRAM_BASE		MSTAR_DDR_BASE
#define CFG_SYS_INIT_RAM_ADDR		(CONFIG_TEXT_BASE - 0x200000)
#define CFG_SYS_INIT_RAM_SIZE		0x100000

/* ARM generic timer runs from xtal / 2; CNTFRQ is not set by the boot ROM/IPL */
#define CFG_SYS_HZ_CLOCK		6000000

#endif
