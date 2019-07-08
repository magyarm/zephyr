/*
 * Copyright (c) 2019 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <arch/cpu.h>
#include <arch/arm/cortex_m/cmsis.h>
#include <kernel_structs.h>
#include <offsets_short_arch.h>


#if !defined(__GNUC__)
#error __FILE__ goes only with Cortex-M GCC
#endif

#define PRIORITY 0
#define BASEPRI_MODIFIED_1 0x20
#define BASEPRI_MODIFIED_2 0x40
#define SWAP_RETVAL        0x1234

extern int __swap(unsigned int key);
extern void z_move_thread_to_end_of_prio_q(struct k_thread *thread);

static struct k_thread alt_thread;
static K_THREAD_STACK_DEFINE(alt_thread_stack, 1024);

/* Status variable to indicate that context-switch has occurred. */
bool volatile switch_flag;

struct k_thread *p_ztest_thread;

_callee_saved_t ztest_thread_callee_saved_regs_container;

/* Arbitrary values for the callee-saved registers,
 * enforced in the beginning of the test.
 */
const _callee_saved_t ztest_thread_callee_saved_regs_init = {
	.v1 = 0x12345678, .v2 = 0x23456789, .v3 = 0x3456789a, .v4 = 0x456789ab,
	.v5 = 0x56789abc, .v6 = 0x6789abcd, .v7 = 0x789abcde, .v8 = 0x89abcdef
};

static void load_callee_saved_regs(const _callee_saved_t *regs)
{
	/* Load the callee-saved registers with given values */
	__asm__ volatile (
		"mov r1, r7;\n\t"
		"ldmia %0, {v1-v8};\n\t"
		"mov r7, r1;\n\t"
		:
		: "r" (regs)
		: "memory"
	);
	__DSB();
}

static void verify_callee_saved(const _callee_saved_t *src,
		const _callee_saved_t *dst)
{
	/* Verify callee-saved registers are as expected */
	zassert_true((src->v1 == dst->v1)
			&& (src->v2 == dst->v2)
			&& (src->v3 == dst->v3)
			&& (src->v4 == dst->v4)
			&& (src->v5 == dst->v5)
			&& (src->v6 == dst->v6)
			&& (src->v7 == dst->v7)
			&& (src->v8 == dst->v8),
		" got: 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x\n"
		" expected:  0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x\n",
		src->v1,
		src->v2,
		src->v3,
		src->v4,
		src->v5,
		src->v6,
		src->v7,
		src->v8,
		dst->v1,
		dst->v2,
		dst->v3,
		dst->v4,
		dst->v5,
		dst->v6,
		dst->v7,
		dst->v8
	);
}

static void alt_thread_entry(void)
{
	zassert_true(switch_flag == false,
		"Alternative thread: switch flag not false on thread entry\n");

	/* Set switch flag */
	switch_flag = true;

#if defined(CONFIG_NO_OPTIMIZATIONS)
	zassert_true(p_ztest_thread->arch.basepri == 0,
		"ztest thread basepri not preserved in swap-out\n");
#else
	/* Verify that the main test thread has an initial value of zero
	 * for state variable thread.arch.basepri.
	 */
	zassert_true(p_ztest_thread->arch.basepri == BASEPRI_MODIFIED_1,
		"ztest thread basepri not preserved in swap-out\n");

	/* Verify original swap return value (set by __swap() */
	zassert_true(p_ztest_thread->arch.swap_return_value == -EAGAIN,
		"ztest thread swap-return-value not preserved in swap-out\n");
#endif

	/* Verify that the main test thread (ztest) has stored the callee-saved
	 * registers properly in its corresponding callee-saved container.
	 */
	verify_callee_saved(
		(const _callee_saved_t *)&p_ztest_thread->callee_saved,
		&ztest_thread_callee_saved_regs_container);

	/* Zero the container of the callee-saved registers, to validate,
	 * later, that it is populated properly.
	 */
	memset(&ztest_thread_callee_saved_regs_container,
		0, sizeof(_callee_saved_t));

	/* Modify the arch.basepri flag of the main test thread, to verify,
	 * later, that this is passed properly to the BASEPRI.
	 */
	p_ztest_thread->arch.basepri = BASEPRI_MODIFIED_2;

#if !defined(CONFIG_NO_OPTIMIZATIONS)
	/* Modify the arch.swap_return_value flag of the main test thread,
	 * to verify later, that this value is properly returned by swap.
	 */
	p_ztest_thread->arch.swap_return_value = SWAP_RETVAL;
#endif

	z_move_thread_to_end_of_prio_q(_current);

	/* Modify the callee-saved registers by zero-ing them.
	 * The main test thread will, later, assert that they
	 * are restored to their original values upon context
	 * switch.
	 */
	__asm__ volatile (
		"mov r0, r7;\n\t"
		"ldmia %0, {v1-v8};\n\t"
		"mov r7, r0;\n\t"
		: : "r" (&ztest_thread_callee_saved_regs_container)
		: "memory"
	);

	/* Manually trigger a context-switch, to swap-out
	 * the alternative test thread.
	 */
	__DMB();
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	__DSB();
	__ISB();
}

void test_arm_thread_swap(void)
{
	/* Main test thread (ztest)
	 *
	 * Simulating initial conditions:
	 * - set arbitrary values at the callee-saved registers
	 * - zero the thread's callee-saved data structure
	 * - set thread's priority same as the alternative test thread
	 */

	/* Load the callee-saved registers with initial arbitrary values
	 * (from ztest_thread_callee_saved_regs_init)
	 */
	load_callee_saved_regs(&ztest_thread_callee_saved_regs_init);

	k_thread_priority_set(_current, K_PRIO_COOP(PRIORITY));

	/* Export current thread's callee-saved registers pointer
	 * and arch.basepri variable pointer, into global pointer
	 * variables, so they can be easily accessible by other
	 * (alternative) test thread.
	 */
	p_ztest_thread = _current;

	/* Confirm initial conditions before starting the test. */
	zassert_true(switch_flag == false,
		"Switch flag not initialized properly\n");
	zassert_true(_current->arch.basepri == 0,
		"Thread BASEPRI flag not clear at thread start\n");
	/* Verify, also, that the interrupts are unlocked. */
	zassert_true(__get_BASEPRI() == 0,
		"initial BASEPRI not in zero\n");

#if defined(CONFIG_USERSPACE)
	/* The main test thread is set to run in privilege mode */
	zassert_false((z_arch_is_user_context()),
		"Main test thread does not start in privilege mode\n");

	/* Assert that the mode status variable indicates privilege mode */
	zassert_true((_current->arch.mode & CONTROL_nPRIV_Msk) == 0,
		"Thread nPRIV flag not clear for supervisor thread: 0x%0x\n",
		_current->arch.mode);
#endif /* CONFIG_USERSPACE */

#if defined(CONFIG_FLOAT) && defined(CONFIG_FP_SHARING)
	/* The main test thread is not (yet) actively using the FP registers */
	zassert_true((_current->arch.mode & CONTROL_FPCA_Msk) == 0,
		"Thread FPCA flag not clear at initialization 0x%0x\n",
		_current->arch.mode);
#endif /* CONFIG_FLOAT && CONFIG_FP_SHARING */

	/* Create an alternative (supervisor) testing thread */
	k_thread_create(&alt_thread,
		alt_thread_stack,
		K_THREAD_STACK_SIZEOF(alt_thread_stack),
		(k_thread_entry_t)alt_thread_entry,
		NULL, NULL, NULL,
		K_PRIO_COOP(PRIORITY), 0,
		K_NO_WAIT);

	/* Verify context-switch has not occurred. */
	zassert_true(switch_flag == false,
		"Switch flag incremented when it should not have\n");

	/* Prepare to force a context switch to the alternative thread,
	 * by manually adding the current thread to the end of the queue,
	 * so it will be context switched-out.
	 */
	int key;

	key = irq_lock();
	z_move_thread_to_end_of_prio_q(_current);
	irq_unlock(key);

	/* Clear the thread's callee-saved registers' container.
	 * The container will, later, be populated by the swap
	 * mechanism.
	 */
	memcpy(&_current->callee_saved, 0, sizeof(_callee_saved_t));

	/* Verify context-switch has not occurred yet. */
	zassert_true(switch_flag == false,
		"Switch flag incremented by unexpected context-switch.\n");

	/* Store the callee-saved registers to some global memory
	 * accessible to the alternative testing thread. That
	 * thread is going to verify that the callee-saved regs
	 * are successfully loaded into the thread's callee-saved
	 * registers' container.
	 */
	__asm__ volatile (
		"stmia %0, {r4-r11};\n\t"
		:
		: "r" (&ztest_thread_callee_saved_regs_container)
		: "memory"
	);

	/* Manually trigger a context-switch to swap-out the current thread.
	 * Request a return to a different interrupt lock state.
	 */
	__DMB();

#if defined(CONFIG_NO_OPTIMIZATIONS)
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	__DSB();
	__ISB();

	/* The thread is now swapped-back in. */

#else/* CONFIG_NO_OPTIMIZATIONS */

	/* Fake a different irq_unlock key when performing swap.
	 * This will be verified by the alternative test thread.
	 */
	register int swap_return_val __asm__("r0") =
		__swap(BASEPRI_MODIFIED_1);

#endif /* CONFIG_NO_OPTIMIZATIONS */

	/* Dump callee-saved registers to memory. */
	__asm__ volatile (
		"stmia %0, {r4-r11};\n\t"
		:
		: "r" (&ztest_thread_callee_saved_regs_container)
		: "memory"
	);

	/* After swap-back, verify that the callee-saved registers loaded,
	 * look exactly as what is located in the respective callee-saved
	 * container of the thread.
	 */
	verify_callee_saved(
		&ztest_thread_callee_saved_regs_container,
		&_current->callee_saved);

	/* Verify context-switch did occur. */
	zassert_true(switch_flag == true,
		"Switch flag not incremented as expected %u\n",
		switch_flag);

	/* Verify that the arch.basepri flag is cleared, after
	 * the alternative thread modified it, since the thread
	 * is now switched back in.
	 */
	zassert_true(_current->arch.basepri == 0,
		"arch.basepri value not in accordance with the update\n");

	/* Verify that the BASEPRI register is updated during the last
	 * swap-in of the thread.
	 */
	zassert_true(__get_BASEPRI() == BASEPRI_MODIFIED_2,
		"BASEPRI not in accordance with the update: 0x%0x\n",
		__get_BASEPRI());

#if !defined(CONFIG_NO_OPTIMIZATIONS)
	/* The thread is now swapped-back in. */
	zassert_true(_current->arch.swap_return_value = SWAP_RETVAL,
		"Swap value not set as expected\n");
	zassert_true(_current->arch.swap_return_value = swap_return_val,
			"Swap value not returned as expected\n");
#endif
}
/**
 * @}
 */
