#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "ohos_init.h"

#include "air780_uart.h"

#define AIR780_CHECK_INTERVAL    3000

static int Air780CheckAlive(void)
{
    char rx[256];

    int len = Air780SendAT(
        "AT\r\n",
        rx,
        sizeof(rx),
        100
    );

    if (len <= 0)
    {
        return -1;
    }

    if (strstr(rx, "OK"))
    {
        return 0;
    }

    return -1;
}


static int Air780Recover(void)
{
    printf("\r\n");
    printf("[AIR780] recovering...\r\n");

    Air780UartDeinit();

    if (Air780UartInit() != 0)
    {
        printf("[AIR780] uart init failed\r\n");
        return -1;
    }

    osDelay(50);

    Air780Wakeup();

    if (Air780CheckAlive() == 0)
    {
        printf("[AIR780] recover success\r\n");
        return 0;
    }

    printf("[AIR780] recover failed\r\n");

    return -1;
}

static void Air780Task(void *arg)
{
    (void)arg;

    printf("\r\n");
    printf("====================================\r\n");
    printf(" AIR780 NETWORK DIAGNOSTIC START\r\n");
    printf("====================================\r\n");

    /*
     * 首次启动
     */
    while (Air780Recover() != 0)
    {
        printf("[AIR780] retry after 5 seconds...\r\n");

        osDelay(500);
    }

    /*
     * 主循环
     */
    while (1)
    {
        if (Air780CheckAlive() != 0)
        {
            printf("\r\n");
            printf("[AIR780] module offline\r\n");

            while (Air780Recover() != 0)
            {
                osDelay(500);
            }
        }
        else
        {
             printf("[AIR780] alive\r\n");
        }

        osDelay(AIR780_CHECK_INTERVAL);
    }
}

static void Air780Entry(void)
{
    osThreadAttr_t attr;

    memset(&attr, 0, sizeof(attr));

    attr.name = "Air780Task";

    attr.stack_size = 4096;

    attr.priority = osPriorityNormal;

    osThreadNew(
        Air780Task,
        NULL,
        &attr
    );
}

APP_FEATURE_INIT(Air780Entry);