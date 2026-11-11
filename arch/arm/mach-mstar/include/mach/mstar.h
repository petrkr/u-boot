/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __MACH_MSTAR_H
#define __MACH_MSTAR_H

#define MSTAR_RIU_BASE		0x1f000000
#define MSTAR_BANK(bank)		(MSTAR_RIU_BASE + (bank) * 0x200)
#define MSTAR_REG(bank, reg)	(MSTAR_BANK(bank) + (reg) * 4)

#define MSTAR_DDR_BASE		0x20000000
#define MSTAR_BANK_PMTOP		0x001e
#define MSTAR_BANK_MIU		0x1012
#define MSTAR_BANK_L3BRIDGE	0x1022

#define MSTAR_CHIP_ID		MSTAR_REG(MSTAR_BANK_PMTOP, 0x00)
#define MSTAR_DDR_LOG2_MB		(readw(MSTAR_REG(MSTAR_BANK_MIU, 0x69)) >> 12)

#define MSTAR_BOOT_PARAMS		MSTAR_DDR_BASE
#define MSTAR_L3_FLUSH		MSTAR_REG(MSTAR_BANK_L3BRIDGE, 0x05)
#define MSTAR_L3_FLUSH_STATUS	MSTAR_REG(MSTAR_BANK_L3BRIDGE, 0x10)
#define MSTAR_L3_FLUSH_DONE	BIT(12)
#define MSTAR_MIU_ADDR(p)		((unsigned long)(p) - MSTAR_DDR_BASE)

#ifndef __ASSEMBLY__
void mstar_miu_flush(void);
#endif

#endif
