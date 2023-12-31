/*
 *  linux/arch/arm/kernel/entry-armv.S
 *
 *  Copyright (C) 1996,1997,1998 Russell King.
 *  ARM700 fix by Matthew Godbolt (linux-user@willothewisp.demon.co.uk)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Low-level vector interface routines
 *
 *  Note:  there is a StrongARM bug in the STMIA rn, {regs}^ instruction that causes
 *  it to save wrong values...  Be aware!
 */
#include <linux/config.h>
#include <linux/init.h>
#include <asm/kgdb.h>

#include <asm/thread_info.h>
#include <asm/glue.h>
#include <asm/ptrace.h>
#include <asm/vfpmacros.h>

#include "entry-header.S"

/*
 * Invalid mode handlers
 */
__pabt_invalid:	sub	sp, sp, #S_FRAME_SIZE		@ Allocate frame size in one go
		stmia	sp, {r0 - lr}			@ Save XXX r0 - lr
		ldr	r4, .LCabt
		mov	r1, #BAD_PREFETCH
		b	1f

__dabt_invalid:	sub	sp, sp, #S_FRAME_SIZE
		stmia	sp, {r0 - lr}			@ Save SVC r0 - lr [lr *should* be intact]
		ldr	r4, .LCabt
		mov	r1, #BAD_DATA
		b	1f

__irq_invalid:	sub	sp, sp, #S_FRAME_SIZE		@ Allocate space on stack for frame
		stmfd	sp, {r0 - lr}			@ Save r0 - lr
		ldr	r4, .LCirq
		mov	r1, #BAD_IRQ
		b	1f

__und_invalid:	sub	sp, sp, #S_FRAME_SIZE
		stmia	sp, {r0 - lr}
		ldr	r4, .LCund
		mov	r1, #BAD_UNDEFINSTR		@ int reason

1:		zero_fp
		ldmia	r4, {r5 - r7}			@ Get XXX pc, cpsr, old_r0
		add	r4, sp, #S_PC
		stmia	r4, {r5 - r7}			@ Save XXX pc, cpsr, old_r0
		mov	r0, sp
		and	r2, r6, #31			@ int mode
		b	bad_mode

/*
 * SVC mode handlers
 */

#if defined(CONFIG_AEABI) && (__LINUX_ARM_ARCH__ >= 5)
#define SPFIX(code...) code
#else
#define SPFIX(code...)
#endif

		.align	5
__dabt_svc:	sub	sp, sp, #S_FRAME_SIZE
	SPFIX(	tst	sp, #4		)
	SPFIX(	bicne	sp, sp, #4	)
		stmia	sp, {r0 - r12}			@ save r0 - r12
		ldr	r2, .LCabt
		add	r0, sp, #S_FRAME_SIZE
	SPFIX(	addne	r0, r0, #4	)
		ldmia	r2, {r2 - r4}			@ get pc, cpsr
		add	r5, sp, #S_SP
		mov	r1, lr
		stmia	r5, {r0 - r4}			@ save sp_SVC, lr_SVC, pc, cpsr, old_ro
		mrs	r9, cpsr			@ Enable interrupts if they were
		tst	r3, #PSR_I_BIT
		biceq	r9, r9, #PSR_I_BIT		@ previously
/*
 * This routine must not corrupt r9
 */
#ifdef MULTI_ABORT
		ldr	r4, .LCprocfns			@ pass r2, r3 to
		mov	lr, pc				@ processor code
		ldr	pc, [r4]			@ call processor specific code
#else
		bl	CPU_ABORT_HANDLER
#endif
		msr	cpsr_c, r9
		mov	r2, sp
		bl	do_DataAbort
		disable_irq r0
		ldr	r0, [sp, #S_PSR]
		msr	spsr_cxsf, r0
		ldmia	sp, {r0 - pc}^			@ load r0 - pc, cpsr

		.align	5
__irq_svc:	sub	sp, sp, #S_FRAME_SIZE
	SPFIX(	tst	sp, #4		)
	SPFIX(	bicne	sp, sp, #4	)
		stmia	sp, {r0 - r12}			@ save r0 - r12
		ldr	r7, .LCirq
		add	r5, sp, #S_FRAME_SIZE
	SPFIX(	addne	r5, r5, #4	)
		ldmia	r7, {r7 - r9}
		add	r4, sp, #S_SP
		mov	r6, lr
		stmia	r4, {r5, r6, r7, r8, r9}	@ save sp_SVC, lr_SVC, pc, cpsr, old_ro
#ifdef CONFIG_PREEMPT
		get_thread_info r8
		ldr	r9, [r8, #TI_PREEMPT]		@ get preempt count
		add	r7, r9, #1			@ increment it
		str	r7, [r8, #TI_PREEMPT]
#endif
#ifdef CONFIG_CRITICAL_IRQSOFF_TIMING
		bl trace_irqs_off
#endif
1:		get_irqnr_and_base r0, r6, r5, lr
		movne	r1, sp
                adrsvc  ne, lr, 1b
		@
		@ routine called with r0 = irq number, r1 = struct pt_regs *
		@
                bne     asm_do_IRQ
		adrsvc	ne, lr, 1b
		bne	asm_do_IRQ

#ifdef CONFIG_PREEMPT
		ldr	r0, [r8, #TI_FLAGS]		@ get flags
		tst	r0, #_TIF_NEED_RESCHED
		blne	svc_preempt
preempt_return:
		ldr	r0, [r8, #TI_PREEMPT]		@ read preempt value
		teq	r0, r7
		str	r9, [r8, #TI_PREEMPT]		@ restore preempt count
		strne	r0, [r0, -r0]			@ bug()
#endif
#ifdef CONFIG_CRITICAL_IRQSOFF_TIMING
		bl trace_irqs_on
#endif
		ldr	r0, [sp, #S_PSR]		@ irqs are already disabled
		msr	spsr_cxsf, r0
		ldmia	sp, {r0 - pc}^			@ load r0 - pc, cpsr

		.ltorg

#ifdef CONFIG_PREEMPT
svc_preempt:	teq	r9, #0				@ was preempt count = 0
		ldreq	r6, .LCirq_stat
		movne	pc, lr				@ no
		ldr	r0, [r6, #4]			@ local_irq_count
		ldr	r1, [r6, #8]			@ local_bh_count
		adds	r0, r0, r1
		movne	pc, lr
		mov	r7, #0				@ preempt_schedule_irq
		str	r7, [r8, #TI_PREEMPT]		@ expects preempt_count == 0
1:		disable_irq r0				@ disable IRQs
		bl	preempt_schedule_irq
		ldr	r0, [r8, #TI_FLAGS]		@ get new tasks TI_FLAGS
		tst	r0, #_TIF_NEED_RESCHED
		mov     r7, #0
		str     r7, [r8, #TI_PREEMPT]		@ set PREEMPT_ACTIVE
		beq	preempt_return			@ go again
		b	1b
#endif
		CFI_END_FRAME(__irq_svc)

		.align	5
__und_svc:	sub	sp, sp, #S_FRAME_SIZE
	SPFIX(	tst	sp, #4		)
	SPFIX(	bicne	sp, sp, #4	)
		stmia	sp, {r0 - r12}			@ save r0 - r12
		ldr	r3, .LCund
		mov	r4, lr
		ldmia	r3, {r5 - r7}
		add	r3, sp, #S_FRAME_SIZE
	SPFIX(	addne	r3, r3, #4	)
		add	r2, sp, #S_SP
		stmia	r2, {r3 - r7}			@ save sp_SVC, lr_SVC, pc, cpsr, old_ro

		ldr	r0, [r5, #-4]			@ r0 = instruction
		adrsvc	al, r9, 1f			@ r9 = normal FP return
		bl	call_fpe			@ lr = undefined instr return

		mov	r0, sp				@ struct pt_regs *regs
		bl	do_undefinstr

1:		disable_irq r0
		ldr	lr, [sp, #S_PSR]		@ Get SVC cpsr
		msr	spsr_cxsf, lr
		ldmia	sp, {r0 - pc}^			@ Restore SVC registers

		.align	5
__pabt_svc:	sub	sp, sp, #S_FRAME_SIZE
	SPFIX(	tst	sp, #4		)
	SPFIX(	bicne	sp, sp, #4	)
		stmia	sp, {r0 - r12}			@ save r0 - r12
		ldr	r2, .LCabt
		add	r0, sp, #S_FRAME_SIZE
	SPFIX(	addne	r0, r0, #4	)
		ldmia	r2, {r2 - r4}			@ get pc, cpsr
		add	r5, sp, #S_SP
		mov	r1, lr
		stmia	r5, {r0 - r4}			@ save sp_SVC, lr_SVC, pc, cpsr, old_ro
		mrs	r9, cpsr			@ Enable interrupts if they were
		tst	r3, #PSR_I_BIT
		biceq	r9, r9, #PSR_I_BIT		@ previously
		msr	cpsr_c, r9
		mov	r0, r2				@ address (pc)
		mov	r1, sp				@ regs
		bl	do_PrefetchAbort		@ call abort handler
		disable_irq r0
		ldr	r0, [sp, #S_PSR]
		msr	spsr_cxsf, r0
		ldmia	sp, {r0 - pc}^			@ load r0 - pc, cpsr

		.align	5
.LCirq:		.word	__temp_irq
.LCund:		.word	__temp_und
.LCabt:		.word	__temp_abt
#ifdef MULTI_ABORT
.LCprocfns:	.word	processor
#endif
.LCfp:		.word	fp_enter
#ifdef CONFIG_PREEMPT
.LCirq_stat:	.word	irq_stat
#endif

/*
 * User mode handlers
 *
 * EABI note: sp_svc is always 64-bit aligned here, so should S_FRAME_SIZE
 */

#if defined(CONFIG_AEABI) && (__LINUX_ARM_ARCH__ >= 5) && (S_FRAME_SIZE & 7)
#error "sizeof(struct pt_regs) must be a multiple of 8"
#endif

		.align	5
__dabt_usr:	sub	sp, sp, #S_FRAME_SIZE		@ Allocate frame size in one go
		stmia	sp, {r0 - r12}			@ save r0 - r12
		ldr	r7, .LCabt
		add	r5, sp, #S_PC
		ldmia	r7, {r2 - r4}			@ Get USR pc, cpsr
#if __LINUX_ARM_ARCH__ < 6
		@ make sure our user space atomic helper is aborted
		cmp	r2, #VIRT_OFFSET
		bichs	r3, r3, #PSR_Z_BIT
#endif
		stmia	r5, {r2 - r4}			@ Save USR pc, cpsr, old_r0
		stmdb	r5, {sp, lr}^
		alignment_trap r7, r7, __temp_abt
		zero_fp

#ifdef MULTI_ABORT
		ldr	r4, .LCprocfns			@ pass r2, r3 to
		mov	lr, pc				@ processor code
		ldr	pc, [r4]			@ call processor specific code
#else
		bl	CPU_ABORT_HANDLER
#endif
		enable_irq r2				@ Enable interrupts
		mov	r2, sp
		adrsvc	al, lr, ret_from_exception
		b	do_DataAbort

		.align	5
__irq_usr:	sub	sp, sp, #S_FRAME_SIZE
		stmia	sp, {r0 - r12}			@ save r0 - r12
		ldr	r4, .LCirq
		add	r8, sp, #S_PC
		ldmia	r4, {r5 - r7}			@ get saved PC, SPSR
#if __LINUX_ARM_ARCH__ < 6
		@ make sure our user space atomic helper is aborted
		cmp	r5, #VIRT_OFFSET
		bichs	r6, r6, #PSR_Z_BIT
#endif
		stmia	r8, {r5 - r7}			@ save pc, psr, old_r0
		stmdb	r8, {sp, lr}^
		alignment_trap r4, r7, __temp_irq
		zero_fp
#ifdef CONFIG_PREEMPT
		get_thread_info r8
		ldr	r9, [r8, #TI_PREEMPT]		@ get preempt count
		add	r7, r9, #1			@ increment it
		str	r7, [r8, #TI_PREEMPT]
#endif
1:		get_irqnr_and_base r0, r6, r5, lr
		movne	r1, sp
		@
		@ routine called with r0 = irq number, r1 = struct pt_regs *
		@

		adrsvc	ne, lr, 1b
  		bne	asm_do_IRQ

#ifdef CONFIG_PREEMPT
		ldr	r0, [r8, #TI_PREEMPT]
		teq	r0, r7
		str	r9, [r8, #TI_PREEMPT]
		strne	r0, [r0, -r0]
		mov	tsk, r8
#else
		get_thread_info tsk
#endif
		mov	why, #0
		b	ret_to_user

		.ltorg

		.align	5
__und_usr:	sub	sp, sp, #S_FRAME_SIZE		@ Allocate frame size in one go
		stmia	sp, {r0 - r12}			@ Save r0 - r12
		ldr	r4, .LCund
		add	r8, sp, #S_PC
		ldmia	r4, {r5 - r7}
		stmia	r8, {r5 - r7}			@ Save USR pc, cpsr, old_r0
		stmdb	r8, {sp, lr}^			@ Save user sp, lr
		alignment_trap r4, r7, __temp_und
		zero_fp
		tst	r6, #PSR_T_BIT			@ Thumb mode?
		bne	fpundefinstr			@ ignore FP
		sub	r4, r5, #4
1:		ldrt	r0, [r4]			@ r0  = instruction
		adrsvc	al, r9, ret_from_exception	@ r9  = normal FP return
		adrsvc	al, lr, fpundefinstr		@ lr  = undefined instr return

/*
 * The out of line fixup for the ldrt above.
 */
		.section .fixup, "ax"
2:		mov	pc, r9
		.previous
		.section __ex_table,"a"
		.long	1b, 2b
		.previous

/*
 * Check whether the instruction is a co-processor instruction.
 * If yes, we need to call the relevant co-processor handler.
 *
 * Note that we don't do a full check here for the co-processor
 * instructions; all instructions with bit 27 set are well
 * defined.  The only instructions that should fault are the
 * co-processor instructions.  However, we have to watch out
 * for the ARM6/ARM7 SWI bug.
 *
 * Emulators may wish to make use of the following registers:
 *  r0  = instruction opcode.
 *  r2  = PC+4
 *  r10 = this threads thread_info structure.
 */
call_fpe:
		tst	r0, #0x08000000			@ only CDP/CPRT/LDC/STC have bit 27
#if defined(CONFIG_CPU_ARM610) || defined(CONFIG_CPU_ARM710)
		and	r8, r0, #0x0f000000		@ mask out op-code bits
		teqne	r8, #0x0f000000			@ SWI (ARM6/7 bug)?
#endif
		moveq	pc, lr
		get_thread_info r10			@ get current thread
		and	r8, r0, #0x00000f00		@ mask out CP number
		mov	r7, #1
		add	r6, r10, #TI_USED_CP
		strb	r7, [r6, r8, lsr #8]		@ set appropriate used_cp[]
#ifdef CONFIG_IWMMXT
		@ Test if we need to give access to iWMMXt coprocessors
		ldr	r5, [r10, #TI_FLAGS]
		rsbs	r7, r8, #(1 << 8)		@ CP 0 or 1 only
		movcss	r7, r5, lsr #(TIF_USING_IWMMXT + 1)
		bcs	iwmmxt_task_enable
#endif
		enable_irq r7
		add	pc, pc, r8, lsr #6
		mov	r0, r0

		mov	pc, lr				@ CP#0
		b	do_fpe				@ CP#1 (FPE)
		b	do_fpe				@ CP#2 (FPE)
		mov	pc, lr				@ CP#3
		mov	pc, lr				@ CP#4
		mov	pc, lr				@ CP#5
		mov	pc, lr				@ CP#6
		mov	pc, lr				@ CP#7
		mov	pc, lr				@ CP#8
		mov	pc, lr				@ CP#9
#ifdef CONFIG_VFP
		b	do_vfp				@ CP#10 (VFP)
		b	do_vfp				@ CP#11 (VFP)
#else
		mov	pc, lr				@ CP#10 (VFP)
		mov	pc, lr				@ CP#11 (VFP)
#endif
		mov	pc, lr				@ CP#12
		mov	pc, lr				@ CP#13
		mov	pc, lr				@ CP#14 (Debug)
		mov	pc, lr				@ CP#15 (Control)

do_fpe:		ldr	r4, .LCfp
		add	r10, r10, #TI_FPSTATE		@ r10 = workspace
		ldr	pc, [r4]			@ Call FP module USR entry point

/*
 * The FP module is called with these registers set:
 *  r0  = instruction
 *  r2  = PC+4
 *  r9  = normal "successful" return address
 *  r10 = FP workspace
 *  lr  = unrecognised FP instruction return address
 */

		.data
ENTRY(fp_enter)
		.word	fpundefinstr
		.text

fpundefinstr:	mov	r0, sp
		adrsvc	al, lr, ret_from_exception
		b	do_undefinstr

		.align	5
__pabt_usr:	sub	sp, sp, #S_FRAME_SIZE		@ Allocate frame size in one go
		stmia	sp, {r0 - r12}			@ Save r0 - r12
		ldr	r4, .LCabt
		add	r8, sp, #S_PC
		ldmia	r4, {r5 - r7}			@ Get USR pc, cpsr
		stmia	r8, {r5 - r7}			@ Save USR pc, cpsr, old_r0
		stmdb	r8, {sp, lr}^			@ Save sp_usr lr_usr
		alignment_trap r4, r7, __temp_abt
		zero_fp
		enable_irq r0				@ Enable interrupts
		mov	r0, r5				@ address (pc)
		mov	r1, sp				@ regs
		bl	do_PrefetchAbort		@ call abort handler
		/* fall through */
/*
 * This is the return code to user mode for abort handlers
 */
ENTRY(ret_from_exception)
		bl	latency_check
		get_thread_info tsk
		mov	why, #0
		b	ret_to_user

/*
 * Register switch for ARMv3 and ARMv4 processors
 * r0 = previous task_struct, r1 = previous thread_info, r2 = next thread_info
 * previous and next are guaranteed not to be the same.
 */
ENTRY(__switch_to)
		add	ip, r1, #TI_CPU_SAVE
		ldr	r3, [r2, #TI_TP_VALUE]
		stmia	ip!, {r4 - sl, fp, sp, lr}	@ Store most regs on stack
		ldr	r6, [r2, #TI_CPU_DOMAIN]!
#if defined(CONFIG_CPU_XSCALE) && !defined(CONFIG_IWMMXT)
		mra	r4, r5, acc0
		stmia   ip, {r4, r5}
#endif
#ifdef CONFIG_HAS_TLS_REG
		mcr	p15, 0, r3, c13, c0, 3		@ set TLS register
#else
		mov	r4, #0xffff0fff
		str	r3, [r4, #-15]			@ TLS val at 0xffff0ff0
#endif
		mcr	p15, 0, r6, c3, c0, 0		@ Set domain register
#ifdef CONFIG_VFP
		@ Always disable VFP so we can lazily save/restore the old
		@ state. This occurs in the context of the previous thread.
		VFPFMRX	r4, FPEXC
		bic	r4, r4, #FPEXC_ENABLE
		VFPFMXR	FPEXC, r4
#endif
#if defined(CONFIG_IWMMXT)
		bl	iwmmxt_task_switch
#elif defined(CONFIG_CPU_XSCALE)
		add	r4, r2, #40			@ cpu_context_save->extra
		ldmib	r4, {r4, r5}
		mar	acc0, r4, r5
#endif
		ldmib	r2, {r4 - sl, fp, sp, pc}	@ Load all regs saved previously

		__INIT

/*
 * User helpers.
 *
 * These are segment of kernel provided user code reachable from user space
 * at a fixed address in kernel memory.  This is used to provide user space
 * with some operations which require kernel help because of unimplemented
 * native feature and/or instructions in many ARM CPUs. The idea is for
 * this code to be executed directly in user mode for best efficiency but
 * which is too intimate with the kernel counter part to be left to user
 * libraries.  In fact this code might even differ from one CPU to another
 * depending on the available  instruction set and restrictions like on
 * SMP systems.  In other words, the kernel reserves the right to change
 * this code as needed without warning. Only the entry points and their
 * results are guaranteed to be stable.
 *
 * Each segment is 32-byte aligned and will be moved to the top of the high
 * vector page.  New segments (if ever needed) must be added in front of
 * existing ones.  This mechanism should be used only for things that are
 * really small and justified, and not be abused freely.
 *
 * User space is expected to implement those things inline when optimizing
 * for a processor that has the necessary native support, but only if such
 * resulting binaries are already to be incompatible with earlier ARM
 * processors due to the use of unsupported instructions other than what
 * is provided here.  In other words don't make binaries unable to run on
 * earlier processors just for the sake of not using these kernel helpers
 * if your compiled code is not going to use the new instructions for other
 * purpose.
 */

	.align	5
__kuser_helper_start:

/*
 * Reference prototype:
 *
 *	void __kernel_memory_barrier(void)
 *
 * Input:
 *
 *	lr = return address
 *
 * Output:
 *
 *	none
 *
 * Clobbered:
 *
 *	the Z flag might be lost
 *
 * Definition and user space usage example:
 *
 *	typedef void (__kernel_dmb_t)(void);
 *	#define __kernel_dmb (*(__kernel_dmb_t *)0xffff0fa0)
 *
 * Apply any needed memory barrier to preserve consistency with data modified
 * manually and __kuser_cmpxchg usage.
 *
 * This could be used as follows:
 *
 * #define __kernel_dmb() \
 *         asm volatile ( "mov r0, #0xffff0fff; mov lr, pc; sub pc, r0, #95" \
 *	        : : : "lr","cc" )
 */

__kuser_memory_barrier:				@ 0xffff0fa0

#if __LINUX_ARM_ARCH__ >= 6 && defined(CONFIG_SMP)
	mcr	p15, 0, r0, c7, c10, 5	@ dmb
#endif
	mov	pc, lr

	.align	5

/*
 * Reference prototype:
 *
 *	int __kernel_cmpxchg(int oldval, int newval, int *ptr)
 *
 * Input:
 *
 *	r0 = oldval
 *	r1 = newval
 *	r2 = ptr
 *	lr = return address
 *
 * Output:
 *
 *	r0 = returned value (zero or non-zero)
 *	C flag = set if r0 == 0, clear if r0 != 0
 *
 * Clobbered:
 *
 *	r3, ip, flags
 *
 * Definition and user space usage example:
 *
 *	typedef int (__kernel_cmpxchg_t)(int oldval, int newval, int *ptr);
 *	#define __kernel_cmpxchg (*(__kernel_cmpxchg_t *)0xffff0fc0)
 *
 * Atomically store newval in *ptr if *ptr is equal to oldval for user space.
 * Return zero if *ptr was changed or non-zero if no exchange happened.
 * The C flag is also set if *ptr was changed to allow for assembly
 * optimization in the calling code.
 *
 * Note: this routine already includes memory barriers as needed.
 *
 * For example, a user space atomic_add implementation could look like this:
 *
 * #define atomic_add(ptr, val) \
 *	({ register unsigned int *__ptr asm("r2") = (ptr); \
 *	   register unsigned int __result asm("r1"); \
 *	   asm volatile ( \
 *	       "1: @ atomic_add\n\t" \
 *	       "ldr	r0, [r2]\n\t" \
 *	       "mov	r3, #0xffff0fff\n\t" \
 *	       "add	lr, pc, #4\n\t" \
 *	       "add	r1, r0, %2\n\t" \
 *	       "add	pc, r3, #(0xffff0fc0 - 0xffff0fff)\n\t" \
 *	       "bcc	1b" \
 *	       : "=&r" (__result) \
 *	       : "r" (__ptr), "rIL" (val) \
 *	       : "r0","r3","ip","lr","cc","memory" ); \
 *	   __result; })
 */

__kuser_cmpxchg:				@ 0xffff0fc0

#if __LINUX_ARM_ARCH__ < 6

	/*
	 * Theory of operation:
	 *
	 * We set the Z flag before loading oldval. If ever an exception
	 * occurs we can not be sure the loaded value will still be the same
	 * when the exception returns, therefore the user exception handler
	 * will clear the Z flag whenever the interrupted user code was
	 * actually from the kernel address space (see __irq_usr and
	 * __dabt_usr).
	 *
	 * The post-increment on the str is used to prevent a race with an
	 * exception happening just after the str instruction which would
	 * clear the Z flag although the exchange was done.
	 */
	teq	ip, ip			@ set Z flag
	ldr	ip, [r2]		@ load current val
	add	r3, r2, #1		@ prepare store ptr
	teqeq	ip, r0			@ compare with oldval if still allowed
	streq	r1, [r3, #-1]!		@ store newval if still allowed
	subs	r0, r2, r3		@ if r2 == r3 the str occured
	mov	pc, lr

#else

#ifdef CONFIG_SMP
	mcr	p15, 0, r0, c7, c10, 5	@ dmb
#endif
	ldrex	r3, [r2]
	subs	r3, r3, r0
	strexeq	r3, r1, [r2]
	rsbs	r0, r3, #0
#ifdef CONFIG_SMP
	mcr	p15, 0, r0, c7, c10, 5	@ dmb
#endif
	mov	pc, lr

#endif

	.align	5

/*
 * Reference prototype:
 *
 *	int __kernel_get_tls(void)
 *
 * Input:
 *
 *	lr = return address
 *
 * Output:
 *
 *	r0 = TLS value
 *
 * Clobbered:
 *
 *	the Z flag might be lost
 *
 * Definition and user space usage example:
 *
 *	typedef int (__kernel_get_tls_t)(void);
 *	#define __kernel_get_tls (*(__kernel_get_tls_t *)0xffff0fe0)
 *
 * Get the TLS value as previously set via the __ARM_NR_set_tls syscall.
 *
 * This could be used as follows:
 *
 * #define __kernel_get_tls() \
 *	({ register unsigned int __val asm("r0"); \
 *         asm( "mov r0, #0xffff0fff; mov lr, pc; sub pc, r0, #31" \
 *	        : "=r" (__val) : : "lr","cc" ); \
 *	   __val; })
 */

__kuser_get_tls:				@ 0xffff0fe0

#ifndef CONFIG_HAS_TLS_REG

	ldr	r0, [pc, #(16 - 8)]		@ TLS stored at 0xffff0ff0
	mov	pc, lr

#else

	mrc	p15, 0, r0, c13, c0, 3		@ read TLS register
	mov	pc, lr

#endif

	.rep	5
	.word	0			@ pad up to __kuser_helper_version
	.endr

/*
 * Reference declaration:
 *
 *	extern unsigned int __kernel_helper_version;
 *
 * Definition and user space usage example:
 *
 *	#define __kernel_helper_version (*(unsigned int *)0xffff0ffc)
 *
 * User space may read this to determine the curent number of helpers
 * available.
 */

__kuser_helper_version:				@ 0xffff0ffc
	.word	((__kuser_helper_end - __kuser_helper_start) >> 5)
__kuser_helper_end:


/*
 * Vector stubs.
 *
 * This code is copied to 0x200 or 0xffff0200 so we can use branches in the
 * vectors, rather than ldr's.
 *
 * Common stub entry macro:
 *   Enter in IRQ mode, spsr = SVC/USR CPSR, lr = SVC/USR PC
 */
	.macro	vector_stub, name, sym, correction=0
	.align	5

vector_\name:
	ldr	r13, .LCs\sym
	.if \correction
	sub	lr, lr, #\correction
	.endif
	str	lr, [r13]			@ save lr_IRQ
	mrs	lr, spsr
	str	lr, [r13, #4]			@ save spsr_IRQ
	@
	@ now branch to the relevant MODE handling routine
	@
	mrs	r13, cpsr
	bic	r13, r13, #MODE_MASK
	orr	r13, r13, #MODE_SVC
	msr	spsr_cxsf, r13			@ switch to SVC_32 mode

	and	lr, lr, #15
	ldr	lr, [pc, lr, lsl #2]
	movs	pc, lr				@ Changes mode and branches
	.endm

__stubs_start:
/*
 * Interrupt dispatcher
 */
	vector_stub	irq, irq, 4

	.long	__irq_usr			@  0  (USR_26 / USR_32)
	.long	__irq_invalid			@  1  (FIQ_26 / FIQ_32)
	.long	__irq_invalid			@  2  (IRQ_26 / IRQ_32)
	.long	__irq_svc			@  3  (SVC_26 / SVC_32)
	.long	__irq_invalid			@  4
	.long	__irq_invalid			@  5
	.long	__irq_invalid			@  6
	.long	__irq_invalid			@  7
	.long	__irq_invalid			@  8
	.long	__irq_invalid			@  9
	.long	__irq_invalid			@  a
	.long	__irq_invalid			@  b
	.long	__irq_invalid			@  c
	.long	__irq_invalid			@  d
	.long	__irq_invalid			@  e
	.long	__irq_invalid			@  f

/*
 * Data abort dispatcher
 * Enter in ABT mode, spsr = USR CPSR, lr = USR PC
 */
	vector_stub	dabt, abt, 8

	.long	__dabt_usr			@  0  (USR_26 / USR_32)
	.long	__dabt_invalid			@  1  (FIQ_26 / FIQ_32)
	.long	__dabt_invalid			@  2  (IRQ_26 / IRQ_32)
	.long	__dabt_svc			@  3  (SVC_26 / SVC_32)
	.long	__dabt_invalid			@  4
	.long	__dabt_invalid			@  5
	.long	__dabt_invalid			@  6
	.long	__dabt_invalid			@  7
	.long	__dabt_invalid			@  8
	.long	__dabt_invalid			@  9
	.long	__dabt_invalid			@  a
	.long	__dabt_invalid			@  b
	.long	__dabt_invalid			@  c
	.long	__dabt_invalid			@  d
	.long	__dabt_invalid			@  e
	.long	__dabt_invalid			@  f

/*
 * Prefetch abort dispatcher
 * Enter in ABT mode, spsr = USR CPSR, lr = USR PC
 */
	vector_stub	pabt, abt, 4

	.long	__pabt_usr			@  0 (USR_26 / USR_32)
	.long	__pabt_invalid			@  1 (FIQ_26 / FIQ_32)
	.long	__pabt_invalid			@  2 (IRQ_26 / IRQ_32)
	.long	__pabt_svc			@  3 (SVC_26 / SVC_32)
	.long	__pabt_invalid			@  4
	.long	__pabt_invalid			@  5
	.long	__pabt_invalid			@  6
	.long	__pabt_invalid			@  7
	.long	__pabt_invalid			@  8
	.long	__pabt_invalid			@  9
	.long	__pabt_invalid			@  a
	.long	__pabt_invalid			@  b
	.long	__pabt_invalid			@  c
	.long	__pabt_invalid			@  d
	.long	__pabt_invalid			@  e
	.long	__pabt_invalid			@  f

/*
 * Undef instr entry dispatcher
 * Enter in UND mode, spsr = SVC/USR CPSR, lr = SVC/USR PC
 */
	vector_stub	und, und

	.long	__und_usr			@  0 (USR_26 / USR_32)
	.long	__und_invalid			@  1 (FIQ_26 / FIQ_32)
	.long	__und_invalid			@  2 (IRQ_26 / IRQ_32)
	.long	__und_svc			@  3 (SVC_26 / SVC_32)
	.long	__und_invalid			@  4
	.long	__und_invalid			@  5
	.long	__und_invalid			@  6
	.long	__und_invalid			@  7
	.long	__und_invalid			@  8
	.long	__und_invalid			@  9
	.long	__und_invalid			@  a
	.long	__und_invalid			@  b
	.long	__und_invalid			@  c
	.long	__und_invalid			@  d
	.long	__und_invalid			@  e
	.long	__und_invalid			@  f

	.align	5

/*=============================================================================
 * Undefined FIQs
 *-----------------------------------------------------------------------------
 * Enter in FIQ mode, spsr = ANY CPSR, lr = ANY PC
 * MUST PRESERVE SVC SPSR, but need to switch to SVC mode to show our msg.
 * Basically to switch modes, we *HAVE* to clobber one register...  brain
 * damage alert!  I don't think that we can execute any code in here in any
 * other mode than FIQ...  Ok you can switch to another mode, but you can't
 * get out of that mode without clobbering one register.
 */
vector_fiq:	disable_fiq
		subs	pc, lr, #4

/*=============================================================================
 * Address exception handler
 *-----------------------------------------------------------------------------
 * These aren't too critical.
 * (they're not supposed to happen, and won't happen in 32-bit data mode).
 */

vector_addrexcptn:
		b	vector_addrexcptn

/*
 * We group all the following data together to optimise
 * for CPUs with separate I & D caches.
 */
		.align	5

.LCvswi:	.word	vector_swi

.LCsirq:	.word	__temp_irq
.LCsund:	.word	__temp_und
.LCsabt:	.word	__temp_abt

__stubs_end:

		.equ	__real_stubs_start, .LCvectors + 0x200

.LCvectors:	swi	SYS_ERROR0
		b	__real_stubs_start + (vector_und - __stubs_start)
		ldr	pc, __real_stubs_start + (.LCvswi - __stubs_start)
		b	__real_stubs_start + (vector_pabt - __stubs_start)
		b	__real_stubs_start + (vector_dabt - __stubs_start)
		b	__real_stubs_start + (vector_addrexcptn - __stubs_start)
		b	__real_stubs_start + (vector_irq - __stubs_start)
		b	__real_stubs_start + (vector_fiq - __stubs_start)

ENTRY(__trap_init)
		stmfd	sp!, {r4 - r6, lr}

		mov	r0, #0xff000000
		orr	r0, r0, #0x00ff0000		@ high vectors position
		adr	r1, .LCvectors			@ set up the vectors
		ldmia	r1, {r1, r2, r3, r4, r5, r6, ip, lr}
		stmia	r0, {r1, r2, r3, r4, r5, r6, ip, lr}

		add	r2, r0, #0x200
		adr	r1, __stubs_start		@ copy stubs to 0x200
		adr	r4, __stubs_end
1:		ldr	r3, [r1], #4
		str	r3, [r2], #4
		cmp	r1, r4
		blo	1b

		add	r2, r0, #0x1000			@ top of high vector page
		adr	r4, __kuser_helper_end		@ user helpers to top of page
		adr	r1, __kuser_helper_start	@ going downwards.
1:		ldr	r3, [r4, #-4]!
		str	r3, [r2, #-4]!
		cmp	r4, r1
		bhi	1b

		LOADREGS(fd, sp!, {r4 - r6, pc})

		.data

/*
 * Do not reorder these, and do not insert extra data between...
 */

__temp_irq:	.word	0				@ saved lr_irq
		.word	0				@ saved spsr_irq
		.word	-1				@ old_r0
__temp_und:	.word	0				@ Saved lr_und
		.word	0				@ Saved spsr_und
		.word	-1				@ old_r0
__temp_abt:	.word	0				@ Saved lr_abt
		.word	0				@ Saved spsr_abt
		.word	-1				@ old_r0

		.globl	cr_alignment
		.globl	cr_no_alignment
cr_alignment:
		.space	4
cr_no_alignment:
		.space	4
