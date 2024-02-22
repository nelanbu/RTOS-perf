
/****************************************************************************
 * Includes
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "nuttx/config.h"
#include "sched.h"
#include "fcntl.h"
#include "sys/boardctl.h"
#include "sys/ioctl.h"
#include "nuttx/ioexpander/gpio.h"
#include "semaphore.h"

/*********************************************************************
 * Definitions
 *********************************************************************/
/* Task settings */
#define PRIORITY_INIT           (30)
#define PRIORITY_LOW            (PRIORITY_INIT - 2)
#define PRIORITY_HIGH           (PRIORITY_INIT - 1)
#define NUM_SAMPLES             (1000)
#define USERSIGNAL_GPIO         (SIGUSR1)

#define USERGPIO_OUT_TRIGGER    "/dev/gpio10"   /* PA5 */
#define USERGPIO_OUT_FINISH     "/dev/gpio8"    /* */
#define USERGPIO_IN_INT         "/dev/gpio12"

/*********************************************************************
 * Globals
 *********************************************************************/
/* Task PID holder */
pid_t id_init   = 0;
pid_t id_task1  = 0;
pid_t id_task2  = 0;

sem_t semaphore     = {0};

/*********************************************************************
 * Prototypes
 *********************************************************************/

/****************************************************************************
 * Code
 ****************************************************************************/
int init(int argc, char *argv[])
{
    printf("------------ Task activation latency test ------------\n");

    /* Initialize semaphore */
    sem_init(&semaphore, 0, 0);

    /* Lower priority of "init" task*/
    struct sched_param prio_init = {
        .sched_priority = PRIORITY_LOW - 1
    };

    sched_setparam(id_init, &prio_init);

    while (true)
    {
    }

    return 0;
}

int task1(int argc, char *argv[])
{
    int gpioTrigger = open(USERGPIO_OUT_TRIGGER, O_RDWR);
    if (gpioTrigger < 0)
    {
        printf("ERROR: Failed to open %s: %d\n", USERGPIO_OUT_TRIGGER, errno);
    }

    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        sem_wait(&semaphore);
        int status = ioctl(gpioTrigger, GPIOC_WRITE, true);
        if (status < 0)
        {
            printf("ERROR: %d\n", errno);
        }
    }

    return 0;
}

int task2(int argc, char *argv[])
{
    /* Open pin drivers */
    int gpioTrigger = open(USERGPIO_OUT_TRIGGER, O_RDWR);
    if (gpioTrigger < 0)
    {
        printf("ERROR: Failed to open %s: %d\n", USERGPIO_OUT_TRIGGER, errno);
    }

    int gpioInterrupt = open(USERGPIO_IN_INT, O_RDWR);
    if (gpioInterrupt < 0)
    {
        printf("ERROR: Failed to open %s: %d\n", USERGPIO_IN_INT, errno);
    }

    int gpioFinish = open(USERGPIO_OUT_FINISH, O_RDWR);
    if (gpioFinish < 0)
    {
        printf("ERROR: Failed to open %s: %d\n", USERGPIO_OUT_FINISH, errno);
    }

    /* Set initial states of output GPIOs*/
    int status = ioctl(gpioTrigger, GPIOC_WRITE, false);
    if (status < 0)
    {
        printf("ERROR: %d\n", errno);
    }

    status = ioctl(gpioFinish, GPIOC_WRITE, false);
    if (status < 0)
    {
        printf("ERROR: %d\n", errno);
    }

    /* Register interrupts on GPIO */
    struct sigevent notify;
    notify.sigev_notify = SIGEV_SIGNAL;
    notify.sigev_signo = USERSIGNAL_GPIO;

    status = ioctl(gpioInterrupt, GPIOC_REGISTER, (unsigned long)&notify);
    if (status < 0)
    {
        printf("ERROR: %d\n", errno);
    }

    /* Pick signal to wait for */
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, USERSIGNAL_GPIO);

    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        /* Allow task1 to run */
        sem_post(&semaphore);

        /* Wait for signal and set GPIO high */
        sigwaitinfo(&set, NULL);
        status = ioctl(gpioFinish, GPIOC_WRITE, true);
        if (status < 0)
        {
            printf("ERROR: %d\n", errno);
        }

        /* Let signal to stay on for a short while */
        const uint32_t ms = 1000;
        usleep(100 * ms);

        /* Restore GPIO states */
        ioctl(gpioFinish,  GPIOC_WRITE, false);
        ioctl(gpioTrigger, GPIOC_WRITE, false);
    }

    printf("Check oscilloscope output.\n");
    printf("Done.\n");

    return 0;
}

/****************************************************************************
 * main_test
 ****************************************************************************/

int main_test(int argc, FAR char *argv[])
{
    /* Initialize board peripherals (GPIO) */
    int status = boardctl(BOARDIOC_INIT, 0);
    if (status < 0)
    {
        printf("Failed to initialize board peripherals!\n");

        return EXIT_FAILURE;
    }

    id_task1    = task_create("task1", PRIORITY_LOW,    1024, task1, NULL);
    id_task2    = task_create("task2", PRIORITY_HIGH,   1024, task2, NULL);
    id_init     = task_create("init",  PRIORITY_INIT,   1024, init,  NULL);

    return 0;
}