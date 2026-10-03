#include "air780_uart.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "uart.h"         /* 通过 uart.h -> hal_uart.h -> platform_core.h 带入 pin_t / GPIO_0x */

/*
 * =============================================================================
 *  WS63 (Hi3863) UART 接收的关键事实（来自 SDK 源码）：
 *
 *  uapi_uart_read() 在 WS63 上：
 *    1) 直接忽略 timeout 参数 (源码里 unused(timeout));
 *    2) 调用 uart_porting_lock() 关中断;
 *    3) while(len>0) 必须凑满请求长度才返回，FIFO 空时不退出、不让出 CPU。
 *
 *  => 用它轮询不定长的 AT 响应，会在"模块还没回数据"时关中断 100% 自旋，
 *     喂不到看门狗，最终 NMI 复位（就是 Air780Task CPUP=100% 的那个崩溃）。
 *
 *  正确做法：注册中断接收回调，把字节推进自己的环形缓冲区；
 *  任务侧只从环形缓冲区取数据，用 osDelay 让出 CPU，绝不调用 uapi_uart_read。
 * =============================================================================
 */

/*
 * 环形缓冲区（单生产者=中断回调，单消费者=任务）。
 * 容量取 2 的幂，便于用位与取模。
 */
#define AIR780_RING_SIZE        2048u
#define AIR780_RING_MASK        (AIR780_RING_SIZE - 1u)

static volatile uint8_t  g_ring[AIR780_RING_SIZE];
static volatile uint32_t g_ring_head;   /* 写指针，仅中断回调修改 */
static volatile uint32_t g_ring_tail;   /* 读指针，仅任务修改 */

static volatile int g_uart_ready = 0;

/* 给 uapi_uart_init 的接收缓冲区（驱动内部用，回调的 buffer 指向这里的数据） */
static uint8_t g_uart_rx_buffer[AIR780_RX_BUFFER_SIZE];

static uart_pin_config_t g_uart_pins =
{
    .tx_pin  = GPIO_08,   /* GPIO8 -> AIR780 RX */
    .rx_pin  = GPIO_07,   /* GPIO7 <- AIR780 TX */
    .cts_pin = PIN_NONE,  /* 无硬件流控 */
    .rts_pin = PIN_NONE,
};

static uart_attr_t g_uart_attr =
{
    .baud_rate = AIR780_BAUDRATE,
    .data_bits = UART_DATA_BIT_8,
    .stop_bits = UART_STOP_BIT_1,
    .parity    = UART_PARITY_NONE,
    .flow_ctrl = UART_FLOW_CTRL_NONE,
};

static uart_extra_attr_t g_uart_extra =
{
    .tx_dma_enable    = false,
    .tx_int_threshold = 0,

    .rx_dma_enable    = false,
    .rx_int_threshold = 1,      /* 每收到 1 字节就可触发中断，配合 IDLE 条件及时回调 */
};

static uart_buffer_config_t g_uart_buffer =
{
    .rx_buffer      = g_uart_rx_buffer,
    .rx_buffer_size = sizeof(g_uart_rx_buffer),
};

/*
 * 接收中断回调：运行在中断上下文。
 * 只做"把字节塞进环形缓冲区"，不打印、不阻塞。
 */
static void Air780RxCallback(const void *buffer, uint16_t length, bool error)
{
    if (error || buffer == NULL || length == 0)
    {
        return;
    }

    const uint8_t *p = (const uint8_t *)buffer;

    for (uint16_t i = 0; i < length; i++)
    {
        uint32_t next = (g_ring_head + 1u) & AIR780_RING_MASK;

        if (next == g_ring_tail)
        {
            /* 缓冲区满，丢弃新数据，保证不覆盖未读数据、也绝不阻塞 */
            break;
        }

        g_ring[g_ring_head] = p[i];
        g_ring_head = next;
    }
}

static int Air780RingPop(uint8_t *c)
{
    if (g_ring_tail == g_ring_head)
    {
        return 0;   /* 空 */
    }

    *c = g_ring[g_ring_tail];
    g_ring_tail = (g_ring_tail + 1u) & AIR780_RING_MASK;

    return 1;
}

static void Air780RingClear(void)
{
    g_ring_tail = g_ring_head;
}

int Air780UartInit(void)
{
    errcode_t ret;

    ret = uapi_uart_init(
        AIR780_UART_BUS,
        &g_uart_pins,
        &g_uart_attr,
        &g_uart_extra,
        &g_uart_buffer
    );

    if (ret != ERRCODE_SUCC)
    {
        printf("[AIR780] uart init failed: %d\r\n", (int)ret);
        return -1;
    }

    /* 关键：注册中断接收回调，走异步接收，彻底避开 uapi_uart_read 的关中断自旋 */
    ret = uapi_uart_register_rx_callback(
        AIR780_UART_BUS,
        UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE,
        1,                      /* 阈值 1 字节 */
        Air780RxCallback
    );

    if (ret != ERRCODE_SUCC)
    {
        printf("[AIR780] register rx callback failed: %d\r\n", (int)ret);
        uapi_uart_deinit(AIR780_UART_BUS);
        return -1;
    }

    Air780RingClear();

    g_uart_ready = 1;

    printf("[AIR780] uart init ok (interrupt rx)\r\n");

    return 0;
}

void Air780UartDeinit(void)
{
    if (!g_uart_ready)
    {
        return;     /* 没初始化过就别去 deinit，避免在未就绪句柄上操作 */
    }

    g_uart_ready = 0;

    uapi_uart_unregister_rx_callback(AIR780_UART_BUS);
    uapi_uart_deinit(AIR780_UART_BUS);

    Air780RingClear();

    printf("[AIR780] uart deinit\r\n");
}

int Air780UartSend(const char *data)
{
    int len = (int)strlen(data);

    /*
     * uapi_uart_write 是 TX，FIFO 会被硬件抽空，循环能正常结束，可放心用。
     * timeout 在 WS63 同样被忽略，但 TX 不会死等，无碍。
     */
    int ret = uapi_uart_write(
        AIR780_UART_BUS,
        (const uint8_t *)data,
        (uint32_t)len,
        0
    );

    if (ret < 0)
    {
        printf("[AIR780] tx failed\r\n");
        return -1;
    }

    return ret;
}

/*
 * 从环形缓冲区读取响应。
 *
 * 注意：本板 1 个 RTOS tick = 10ms（osDelay(1) ≈ 10ms）。
 * 这里 timeout_ms 表示"连续无数据的空闲计数上限"，每个空闲计数对应一次 osDelay(1)。
 * 收到 \r\nOK\r\n / \r\nERROR\r\n 立即提前返回。
 */
int Air780UartRead(
    char *buf,
    int maxlen,
    uint32_t timeout_ms
)
{
    if (buf == NULL || maxlen <= 0)
    {
        return 0;
    }

    int total = 0;
    uint32_t idle = 0;

    memset(buf, 0, maxlen);

    while (idle < timeout_ms)
    {
        uint8_t ch;
        int got = 0;

        /*
         * 本轮把当前环形缓冲区里已有的字节一次性取空（有 maxlen 上限），
         * 但无论收没收到数据，本轮结束都会 osDelay 让出 CPU。
         * => 即使对端持续刷数据，也只是每 tick 处理一批，绝不会 100% 占满 CPU。
         */
        while (total < maxlen - 1 && Air780RingPop(&ch))
        {
            buf[total++] = (char)ch;
            got = 1;
        }
        buf[total] = '\0';

        if (strstr(buf, "OK\r\n") || strstr(buf, "ERROR\r\n"))
        {
            break;
        }

        if (total >= maxlen - 1)
        {
            break;          /* 缓冲已满仍无结束符，避免空转 */
        }

        if (got)
        {
            idle = 0;
        }
        else
        {
            idle++;
        }

        osDelay(1);         /* 每轮必让出 CPU（10ms/tick），杜绝忙等 */
    }

    return total;
}

int Air780UartWaitToken(
    const char *token,
    char *scratch,
    int cap,
    uint32_t timeout_ms
)
{
    if (token == NULL || scratch == NULL || cap <= 1)
    {
        return 0;
    }

    int total = 0;
    uint32_t ticks = 0;
    uint32_t max_ticks = timeout_ms / 10u + 1u;   /* osDelay(1) ≈ 10ms */
    int tok_len = (int)strlen(token);

    scratch[0] = '\0';

    while (ticks < max_ticks)
    {
        uint8_t ch;

        while (total < cap - 1 && Air780RingPop(&ch))
        {
            scratch[total++] = (char)ch;
        }
        scratch[total] = '\0';

        if (strstr(scratch, token))
        {
            return 1;
        }

        /*
         * 缓冲快满又没命中：保留末尾 (tok_len-1) 字节，
         * 既能跨批次拼接匹配，又不会无限增长。
         */
        if (total >= cap - 1)
        {
            int keep = tok_len > 0 ? tok_len - 1 : 0;
            if (keep > cap - 1)
            {
                keep = cap - 1;
            }
            if (keep > 0)
            {
                memmove(scratch, scratch + total - keep, keep);
            }
            total = keep;
            scratch[total] = '\0';
        }

        ticks++;
        osDelay(1);
    }

    return 0;
}

int Air780SendAT(
    const char *cmd,
    char *response,
    int response_size,
    uint32_t timeout_ms
)
{
    /* 发送前清掉上一条命令的残留（含模块上电 URC：RDY/+CPIN 等） */
    Air780RingClear();

    printf("\r\n[AIR780] TX: %s", cmd);

    if (Air780UartSend(cmd) < 0)
    {
        return -1;
    }

    int len = Air780UartRead(
        response,
        response_size,
        timeout_ms
    );

    if (len > 0)
    {
        printf("[AIR780] RX:\r\n%s\r\n", response);
    }
    else
    {
        printf("[AIR780] RX: TIMEOUT\r\n");
    }

    return len;
}

void Air780Wakeup(void)
{
    char rx[256];

    /* 上电后模块可能仍在做波特率自适应，连发几次 AT 唤醒 */
    for (int i = 0; i < 3; i++)
    {
        Air780SendAT("AT\r\n", rx, sizeof(rx), AIR780_AT_TIMEOUT);
        osDelay(10);
    }

    /* 关闭回显 */
    Air780SendAT("ATE0\r\n", rx, sizeof(rx), AIR780_AT_TIMEOUT);
}
