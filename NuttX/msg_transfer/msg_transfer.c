
/****************************************************************************
 * Includes
 ****************************************************************************/
#include "nuttx/config.h"
#include "sched.h"
#include "mqueue.h"
#include "fcntl.h"
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
#define PRIORITY_INIT       (30)
#define PRIORITY_TASK_HI    (20)
#define PRIORITY_TASK_LOW   (10)
#define NUM_SAMPLES         (100000)

/* Note that NuttX limits message size to 22 bytes */
#define MESSAGE_SIZE        (8)

/*********************************************************************
 * Globals
 *********************************************************************/
/* Task PID holder */
pid_t id_init   = 0;
pid_t id_task1  = 0;
pid_t id_task2  = 0;

/* Task state */
bool  transferEnable   = false;

/*********************************************************************
 * Prototypes
 *********************************************************************/
int task1(int argc, char *argv[]);
int task2(int argc, char *argv[]);

/****************************************************************************
 * Code
 ****************************************************************************/
int init(int argc, char *argv[])
{
    /* Create message queue */
    struct mq_attr attribute = {
        .mq_maxmsg  = 1, 
        .mq_msgsize = MESSAGE_SIZE
    };
    
    mqd_t m_queue = mq_open("testQueue", O_CREAT, 0, &attribute);

    /* Create tasks that will transfer the messages. Pass false
     * to measure overhead first. */
    transferEnable = false;
    id_task1 = task_create("task1", PRIORITY_TASK_LOW, 1024, task1, NULL);
    id_task2 = task_create("task2", PRIORITY_TASK_HI,  1024, task2, NULL);

    /* Lower priority of "init" task to run task1 and task2 and measure the
     * overhead */
    uint32_t overhead_start = 0;
    uint32_t overhead_stop  = 0;
    struct sched_param prio_init = {
        .sched_priority = PRIORITY_TASK_LOW - 1
    };

    TAKE_MEASUREMENT(overhead_start);
    sched_setparam(id_init, &prio_init);
    TAKE_MEASUREMENT(overhead_stop);

    /* Restore initial priority */
    prio_init.sched_priority = PRIORITY_INIT;
    sched_setparam(id_init, &prio_init);

    /* Create new tasks but transfer messages this time. */
    transferEnable = true;
    id_task1 = task_create("task1", PRIORITY_TASK_LOW, 1024, task1, NULL);
    id_task2 = task_create("task2", PRIORITY_TASK_HI,  1024, task2, NULL);

    /* Lower priority of "init" task and measure time with shuffling.*/
    prio_init.sched_priority    = PRIORITY_TASK_LOW - 1;
    uint32_t transfer_start     = 0;
    uint32_t transfer_stop      = 0;

    TAKE_MEASUREMENT(transfer_start);
    sched_setparam(id_init, &prio_init);
    TAKE_MEASUREMENT(transfer_stop);

    /* Print results */
    uint32_t transfer_avg = ((transfer_stop - transfer_start) - 
                            (overhead_stop - overhead_start)) / NUM_SAMPLES;

    printf("------------ Inter-task message transfer test ------------\n");
    printf("Transfer time total:\t\t %lu\n", transfer_stop - transfer_start);
    printf("Overhead total:\t\t\t %lu\n",    overhead_stop - overhead_start);
    printf("Transfer time (avg):\t\t %lu\n", transfer_avg);
    printf("Done.\n");

    mq_close(m_queue);

    while (1)
    {
        /* do nothing */
    }

    return 0;
}

int task1(int argc, char *argv[])
{
    /* Low priority task sends the message */
    mqd_t m_queue = mq_open("testQueue", O_WRONLY);
    char message[MESSAGE_SIZE] = {0};

    /* Check if transfer should be performed. Shuffle only if argument was
    provided */
    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        message[0] = i % 256;

        if (transferEnable)
        {
            mq_send(m_queue, (const char *)message, MESSAGE_SIZE,
                    MQ_PRIO_MAX);
        }
    }

    /* Close queue and delete itself */
    mq_close(m_queue);
    task_delete(0);

    return 0;
}

int task2(int argc, char *argv[])
{
    /* High priority task receives the message */
    mqd_t m_queue = mq_open("testQueue", O_RDONLY);
    char message[MESSAGE_SIZE] = {0};

    /* Check if read should be performed */
    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        if (transferEnable)
        {
            mq_receive(m_queue, message, MESSAGE_SIZE, NULL);
        }
    }

    /* Close queue and delete itself */
    mq_close(m_queue);
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

    id_init   = task_create("init",  PRIORITY_INIT, 1024, init,  NULL);

    return 0;
}