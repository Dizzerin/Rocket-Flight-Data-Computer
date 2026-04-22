/*
 * Scheduler.c
 *
 * Simple cooperative task scheduler using HAL_GetTick().
 * Call Scheduler_RegisterTask() for each task, then call Scheduler_Run()
 * from the main loop with no HAL_Delay() blocking anywhere.
 *
 * Optimization: a nextWakeTick variable tracks the earliest time any task is
 * due to run. Scheduler_Run() returns immediately if the current tick hasn't
 * reached nextWakeTick yet, avoiding redundant per-task comparisons on most calls.
 *
 * ---------------------------------------------------------------------------
 * WRAPAROUND-SAFE TIMESTAMP COMPARISON
 * ---------------------------------------------------------------------------
 * HAL_GetTick() returns a uint32_t millisecond count that wraps back to 0
 * every ~49.7 days. A naive comparison like (A < B) breaks near the wraparound
 * point (e.g. take A=0xFFFFFFFE, B=0x00000002: In this case B is actually later, but looks
 * smaller). Instead we use subtraction, which is always correct in modular
 * unsigned arithmetic as long as the two times are within half the counter
 * range of each other (more on that constraint below).
 *
 * QUESTION: "Is timestamp A earlier than timestamp B?"
 *
 *   Method 1 (used here):   (B - A) < 0x80000000U
 *   Method 2 (equivalent):  (int32_t)(A - B) < 0
 *
 * Method 2 works because a uint32_t whose top bit is set (>= 0x80000000)
 * is exactly what becomes negative when reinterpreted as int32_t, so the
 * two methods are mathematically identical, just written differently.
 *
 * Why subtraction works — illustrated with 4-bit counters (wraps at 16,
 * threshold is 8 instead of 0x80000000):
 *
 *   "Is A earlier than B?"  →  (B - A) mod 16 < 8
 *
 *   No wraparound involved:
 *     A=3,  B=7  (A truly IS  earlier): B-A = 7-3   = 4       = 0100b  < 8  → YES ✓
 *     A=7,  B=3  (A truly NOT earlier): B-A = 3-7   = -4 → 12 = 1100b  ≥ 8  → NO  ✓
 *
 *   Wraparound involved (counter rolled past 15 back to 0):
 *     A=14, B=2  (A truly IS  earlier): B-A = 2-14  = -12 → 4  = 0100b  < 8  → YES ✓
 *     A=2,  B=14 (A truly NOT earlier): B-A = 14-2  = 12       = 1100b  ≥ 8  → NO  ✓
 *
 * The key insight: when A truly came first (even across a wraparound), the
 * forward distance from A to B is small — the result of (B - A) stays well
 * below the threshold. When A came after B, the "forward" distance from A
 * to B spans almost the entire counter range, giving a large result above
 * the threshold.
 *
 * Why non-wraparound cases are fine: if neither timestamp is near the
 * wraparound boundary, the subtraction produces a straightforward positive
 * or negative result. The modular arithmetic only "activates" when the
 * subtraction would underflow, and in that case it produces exactly the
 * right answer, as shown in the A=14, B=2 example above.
 *
 * The half-range constraint: this scheme only works correctly when A and B
 * are within half the counter's total range of each other. With 4-bit
 * counters that means within 8 ticks; with 32-bit millisecond ticks that
 * means within ~24.8 days. If two timestamps are more than half the range
 * apart, the method cannot tell which one came first, the result is
 * ambiguous; once two points are more than half the range apart, you 
 * can't tell which side of the "gap" is the forward direction.
 * For this scheduler that means task periods must each be less
 * than ~24.8 days, which is never a concern in practice.
 * ---------------------------------------------------------------------------
 */

#include "Scheduler.h"
#include "stm32h7xx_hal.h"

static Scheduler_Task_t tasks[SCHEDULER_MAX_TASKS];
static uint8_t  taskCount    = 0;
static uint32_t nextWakeTick = 0;  /* Earliest tick at which any task is next due */

void Scheduler_Init(void)
{
    taskCount    = 0;
    nextWakeTick = 0;
}

void Scheduler_RegisterTask(void (*fn)(void), uint32_t periodMs)
{
    if (taskCount >= SCHEDULER_MAX_TASKS) return;

    uint32_t now = HAL_GetTick();
    tasks[taskCount].fn          = fn;
    tasks[taskCount].periodMs    = periodMs;
    tasks[taskCount].lastRunTick = now;
    taskCount++;

    /* Update nextWakeTick if this task is due sooner than the current earliest.
     * (nextWakeTick - taskNextRun) < 0x80000000U means taskNextRun is earlier —
     * i.e., the forward distance from taskNextRun to nextWakeTick is small. */
    uint32_t taskNextRun = now + periodMs;
    if (taskCount == 1 || (nextWakeTick - taskNextRun) < 0x80000000U) {
        nextWakeTick = taskNextRun;
    }
}

void Scheduler_Run(void)
{
    if (taskCount == 0) return;

    uint32_t now = HAL_GetTick();

    /* Fast-path: if now is still before nextWakeTick, nothing is due yet.
     * (now - nextWakeTick) underflows (wraps to a huge value >= 0x80000000)
     * when now < nextWakeTick, which is exactly the condition we want to detect. */
    if ((now - nextWakeTick) >= 0x80000000U) return;

    uint32_t earliestNext = now + tasks[0].periodMs;  /* Pessimistic initial value */

    for (uint8_t i = 0; i < taskCount; i++) {
        uint32_t taskNextRun = tasks[i].lastRunTick + tasks[i].periodMs;

        if ((now - tasks[i].lastRunTick) >= tasks[i].periodMs) {
            tasks[i].lastRunTick = now;
            tasks[i].fn();
            taskNextRun = now + tasks[i].periodMs;
        }

        /* Track the minimum next-run time across all tasks.
         * (earliestNext - taskNextRun) < 0x80000000U means taskNextRun is earlier. */
        if (i == 0 || (earliestNext - taskNextRun) < 0x80000000U) {
            earliestNext = taskNextRun;
        }
    }

    nextWakeTick = earliestNext;
}
