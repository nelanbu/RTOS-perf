
/****************************************************************************
 * Includes
 ****************************************************************************/
#include "nuttx/config.h"
#include "nuttx/arch.h"
#include "sched.h"
#include <stdio.h>

/*********************************************************************
 * Definitions
 *********************************************************************/
/* DWT registers and masks */
#define DWT_BASE                    (0xE0001000UL)
#define DWT_CTRL                    (*((volatile uint32_t*)(DWT_BASE + 0x0000)))   /* Control Register */
#define DWT_CYCCNT                  (*((volatile uint32_t*)(DWT_BASE + 0x0004)))   /* Cycle Count Register */
#define DWT_LAR                     (*((volatile uint32_t*)(DWT_BASE + 0x0FB0)))
#define DWT_CTRL_CYCCNTENA_SHIFT    0
#define DWT_CTRL_CYCCNTENA_MASK     (0x1UL << DWT_CTRL_CYCCNTENA_SHIFT)

#define NVIC_DEMCR                  (*((volatile uint32_t *)0xE000EDFC))
#define NVIC_DEMCR_TRCENA_MASK      (1UL << 24)

/* Helpers */
#define TAKE_MEASUREMENT(x)         ((x) = DWT_CYCCNT)

/* Task settings */
#define PRIORITY_INIT   (101)
#define PRIORITY_TASK   (100)
#define NUM_SAMPLES     (2000)

/*********************************************************************
 * Globals
 *********************************************************************/
/* Samples buffer */
uint32_t samples[NUM_SAMPLES];

/* Task PID holder */
pid_t id_init   = 0;
pid_t id_task1  = 0;
pid_t id_task2  = 0;

/****************************************************************************
 * Code
 ****************************************************************************/
int init(int argc, FAR char *argv[])
{

    /* Lower priority of "init" task to run task1 and task2 */
    struct sched_param param = {.sched_priority = PRIORITY_TASK - 1};
    sched_setparam(id_init, &param);

    /* Print results */
    printf("------------ Task switching time test ------------\n");
    printf("Sample id:\t\tcycles:\n");
    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        printf("%u,\t\t%lu\n", i + 1, samples[i]);
    }

    printf("Done.\n");
    while (1)
    {
        /* do nothing */
    }

    return 0;
}

int task1(int argc, FAR char *argv[])
{
    uint32_t start_cycles;
    uint32_t stop_cycles;

    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        TAKE_MEASUREMENT(start_cycles);
        sched_yield();
        TAKE_MEASUREMENT(stop_cycles);

        samples[i] = (stop_cycles - start_cycles) / 2;
    }

    task_delete(id_task1);

    return 0;
}

int task2(int argc, FAR char *argv[])
{
    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        sched_yield();
    }

    task_delete(id_task2);

    return 0;
}

int main_test(int argc, FAR char *argv[])
{
    // Unlock access to DWT - key according to ARM CoreSight specification
    DWT_LAR     = 0xC5ACCE55;                   // enable access - key according to ARM CoreSight specs
    NVIC_DEMCR  |= NVIC_DEMCR_TRCENA_MASK;
    DWT_CYCCNT  = 0;                            // reset the counter
    DWT_CTRL    |= DWT_CTRL_CYCCNTENA_MASK;     // enable the counter

    id_init   = task_create("init",  PRIORITY_INIT, 1024, init,  NULL);
    id_task1  = task_create("task1", PRIORITY_TASK, 1024, task1, NULL);
    id_task2  = task_create("task2", PRIORITY_TASK, 1024, task2, NULL);

    return 0;
}
