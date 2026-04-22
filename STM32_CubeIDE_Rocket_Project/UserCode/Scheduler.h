/*
 * Scheduler.h
 *
 * Simple cooperative task scheduler using HAL_GetTick().
 * Register tasks with a function pointer and period; call Scheduler_Run()
 * from the main loop. No blocking delays — tasks must be non-blocking.
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stdint.h>

#define SCHEDULER_MAX_TASKS 8   // Maximum number of tasks that can be registered

typedef struct {
    void     (*fn)(void); // Function pointer - points to the task function
    uint32_t  periodMs;
    uint32_t  lastRunTick;
} Scheduler_Task_t;

void Scheduler_Init(void);  // Initialize the scheduler (call once at startup)
void Scheduler_RegisterTask(void (*fn)(void), uint32_t periodMs);   // Register a task function to run every periodMs milliseconds
void Scheduler_Run(void);   // Call from main loop to execute due tasks

#endif /* SCHEDULER_H_ */
