
/****************************************************************************
 * Includes
 ****************************************************************************/
#include "nuttx/config.h"
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
#define PRIORITY_INIT   (10)
#define PRIORITY_TASK   (9)
#define NUM_SAMPLES     (100000)
#define ARG_COUNT       (2)

/*********************************************************************
 * Globals
 *********************************************************************/
/* Task PID holder */
pid_t id_init   = 0;
pid_t id_task1  = 0;
pid_t id_task2  = 0;

/* Semaphores */
sem_t sem = {0};

/* Task state */
bool  shuffle   = false;

/*********************************************************************
 * Prototypes
 *********************************************************************/
int task(int argc, char *argv[]);

/****************************************************************************
 * Code
 ****************************************************************************/
int init(int argc, char *argv[])
{
    sem_init(&sem, 0, 1);
    struct sched_param param    = {.sched_priority = PRIORITY_TASK - 1};
    uint32_t overhead_start     = 0;
    uint32_t overhead_stop      = 0;

    /* Create tasks that will shuffle semaphores. Pass false as an argument
     * to measure overhead first. */
    shuffle  = false;
    id_task1 = task_create("task1", PRIORITY_TASK, 2048, task, NULL);
    id_task2 = task_create("task2", PRIORITY_TASK, 2048, task, NULL);

    /* Lower priority of "init" task to run task1 and task2 and measure the
     * overhead */
    TAKE_MEASUREMENT(overhead_start);
    sched_setparam(id_init, &param);
    TAKE_MEASUREMENT(overhead_stop);

    /* Restore initial priority */
    param.sched_priority = PRIORITY_INIT;
    sched_setparam(id_init, &param);

    /* Prepare context for new tasks */
    param.sched_priority    = PRIORITY_TASK - 1;
    uint32_t shuffle_start  = 0;
    uint32_t shuffle_stop   = 0;

    /* Create new tasks but shuffle semaphores this time. */
    shuffle = true;
    id_task1 = task_create("task1", PRIORITY_TASK, 2048, task, NULL);
    id_task2 = task_create("task2", PRIORITY_TASK, 2048, task, NULL);

    /* Lower priority of "init" task and measure time with shuffling.*/
    TAKE_MEASUREMENT(shuffle_start);
    sched_setparam(id_init, &param);
    TAKE_MEASUREMENT(shuffle_stop);

    /* Print results */
    uint32_t shuffle_avg = ((shuffle_stop - shuffle_start) - 
                            (overhead_stop - overhead_start)) / NUM_SAMPLES;

    printf("------------ Semaphore shuffling test ------------\n");
    printf("Shuffle time total:\t\t %lu\n", shuffle_stop  - shuffle_start);
    printf("Overhead total:\t\t\t %lu\n",   overhead_stop - overhead_start);
    printf("Shuffle time (avg):\t\t %lu\n", shuffle_avg);
    printf("Done.\n");

    while (1)
    {
        /* do nothing */
    }

    return 0;
}

int task(int argc, char *argv[])
{
    /* Check if shuffling should be performed. Shuffle only if argument was
    provided */
    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        if (shuffle)
        {
            sem_wait(&sem);
        }
        sched_yield();

        if (shuffle)
        {
            sem_post(&sem);
        }
        sched_yield();
    }

    /* Deletes itself */
    task_delete(0);

    return 0;
}

/****************************************************************************
 * main_test
 ****************************************************************************/

int main_test(int argc, FAR char *argv[])
{
    // Unlock access to DWT - key according to ARM CoreSight specification
    DWT_LAR     = 0xC5ACCE55;                   // enable access - key according to ARM CoreSight specs
    NVIC_DEMCR  |= NVIC_DEMCR_TRCENA_MASK;
    DWT_CYCCNT  = 0;                            // reset the counter
    DWT_CTRL    |= DWT_CTRL_CYCCNTENA_MASK;     // enable the counter

    id_init   = task_create("init",  PRIORITY_INIT, 2048, init,  NULL);

    return 0;
}