/*
 * Scheduler.c
 *
 * Simple cooperative task scheduler using HAL_GetTick().
 * Call Scheduler_RegisterTask() for each task, then call Scheduler_Run()
 * from the main loop with no HAL_Delay() blocking anywhere.
 * 
 * Note that HAL_GetTick() is a 32-bit millisecond tick count that wraps around every ~49.7 days, 
 * but the subtraction and comparison logic in Scheduler_Run() should work correctly across 
 * wraparounds as long as no single task has a period longer than 49.7 days.
 */

#include "Scheduler.h"
#include "stm32h7xx_hal.h"

static Scheduler_Task_t tasks[SCHEDULER_MAX_TASKS];
static uint8_t taskCount = 0;

void Scheduler_Init(void)
{
    taskCount = 0;
}

void Scheduler_RegisterTask(void (*fn)(void), uint32_t periodMs)
{
    if (taskCount >= SCHEDULER_MAX_TASKS) return;
    tasks[taskCount].fn          = fn;
    tasks[taskCount].periodMs    = periodMs;
    tasks[taskCount].lastRunTick = HAL_GetTick();
    taskCount++;
}

void Scheduler_Run(void)
{
    uint32_t now = HAL_GetTick();
    // TODO we should optimize this better somehow so it doesn't just loop through all tasks doing calculations every time to determine if the task should run, there has to be a better way
    for (uint8_t i = 0; i < taskCount; i++) {
        if ((now - tasks[i].lastRunTick) >= tasks[i].periodMs) {
            tasks[i].lastRunTick = now;
            tasks[i].fn();
        }
    }
}
