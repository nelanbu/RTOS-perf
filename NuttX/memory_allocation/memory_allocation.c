/****************************************************************************
 * Includes
 ****************************************************************************/
#include "nuttx/config.h"
#include "nuttx/arch.h"
#include "sched.h"
#include <stdio.h>
#include <stdlib.h>

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
#define NUM_SAMPLES     (1000)

typedef struct test_t
{
    uint32_t size;
    uint32_t average;
    uint32_t start;
    uint32_t stop;
    uint32_t min;
    uint32_t max;
} test_t;

/*********************************************************************
 * Globals
 *********************************************************************/
/* Task PID holder */
pid_t id_init   = 0;

/****************************************************************************
 * Code
 ****************************************************************************/
void print_test_result(test_t *test)
{
    test->average = test->stop - test->start;
    printf("Test size: %4lu\t\tAverage: %lu\t"
           "Max: %lu\tMin: %lu\n",
           test->size, test->average, test->max, test->min);
}

void run_test(test_t *test)
{
    bool firstLoop = true;

    for (size_t i = 0; i < NUM_SAMPLES; ++i)
    {
        TAKE_MEASUREMENT(test->start);
        void *testPointer = malloc(test->size);
        TAKE_MEASUREMENT(test->stop);

        if (NULL == testPointer)
        {
            printf("Test failed:\tsize %lu\tloop%u", test->size, i);
        }

        free(testPointer);

        if (firstLoop)
        {
            firstLoop = false;
            test->average = test->stop - test->start;
            test->min = test->average;
            test->max = test->average;
        }
        else
        {
            const uint32_t result = test->stop - test->start;
            test->average = (test->average * i + result) / (i + 1);

            if (result < test->min)
            {
                test->min = result;
            }

            if (result > test->max)
            {
                test->max = result;
            }
        }
    }
}

int init(int argc, FAR char *argv[])
{
    test_t test0 = {.size = 8,      .average = 0};
    test_t test1 = {.size = 8,      .average = 0};
    test_t test2 = {.size = 64,     .average = 0};
    test_t test3 = {.size = 512,    .average = 0};
    test_t test4 = {.size = 1024,   .average = 0};

    run_test(&test0);
    run_test(&test1);
    run_test(&test2);
    run_test(&test3);
    run_test(&test4);

    /* USER CODE BEGIN 5 */
    printf("------------ Memory allocation test ------------\n");
    print_test_result(&test0);
    print_test_result(&test1);
    print_test_result(&test2);
    print_test_result(&test3);
    print_test_result(&test4);
    printf("Done.\n");
    
    while (true)
    {
    }
}


int main_test(int argc, FAR char *argv[])
{
    // Unlock access to DWT - key according to ARM CoreSight specification
    DWT_LAR     = 0xC5ACCE55;                   // enable access - key according to ARM CoreSight specs
    NVIC_DEMCR  |= NVIC_DEMCR_TRCENA_MASK;
    DWT_CYCCNT  = 0;                            // reset the counter
    DWT_CTRL    |= DWT_CTRL_CYCCNTENA_MASK;     // enable the counter

    id_init   = task_create("init",  PRIORITY_INIT, 4096, init,  NULL);

    return 0;
}
