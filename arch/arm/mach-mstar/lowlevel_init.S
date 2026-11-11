/* SPDX-License-Identifier: GPL-2.0+ */
/* IPL_CUST may transfer control with its MMU and D-cache enabled. */
#include <linux/linkage.h>

ENTRY(save_boot_params)
	mrc	p15, 0, r0, c1, c0, 0
	tst	r0, #(1 << 0)
	beq	1f
	push	{r1-r12, lr}
	bl	v7_flush_dcache_all
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)
	bic	r0, r0, #(1 << 0)
	mcr	p15, 0, r0, c1, c0, 0
	isb
	mov	r0, #0
	mcr	p15, 0, r0, c8, c7, 0
	mcr	p15, 0, r0, c7, c5, 0
	mcr	p15, 0, r0, c7, c5, 6
	dsb
	isb
	pop	{r1-r12, lr}
1:	b	save_boot_params_ret
ENDPROC(save_boot_params)
