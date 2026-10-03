#ifndef AIR780_UART_H
#define AIR780_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * UART配置
 */
#define AIR780_UART_BUS         2

/*
 * GPIO7 <- AIR780 TX
 * GPIO8 -> AIR780 RX
 */
#define AIR780_RX_PIN           7
#define AIR780_TX_PIN           8

/*
 * UART参数
 */
#define AIR780_BAUDRATE         115200

/*
 * 缓冲区大小
 */
#define AIR780_RX_BUFFER_SIZE   1024
#define AIR780_AT_BUFFER_SIZE   512

/*
 * 超时配置(ms)
 */
#define AIR780_READ_TIMEOUT     100
#define AIR780_AT_TIMEOUT       500

/*
 * 初始化UART
 *
 * 返回：
 * 0  成功
 * -1 失败
 */
int Air780UartInit(void);

/*
 * 关闭UART
 */
void Air780UartDeinit(void);

/*
 * 发送原始字符串
 *
 * 返回：
 * >=0 实际发送长度
 * -1  失败
 */
int Air780UartSend(
    const char *data
);

/*
 * UART读取
 *
 * 返回：
 * >=0 实际读取长度
 */
int Air780UartRead(
    char *buf,
    int maxlen,
    uint32_t timeout_ms
);

/*
 * 发送AT命令并读取响应
 *
 * 返回：
 * >0 实际响应长度
 * 0  超时
 * -1 发送失败
 */
int Air780SendAT(
    const char *cmd,
    char *response,
    int response_size,
    uint32_t timeout_ms
);

/*
 * 唤醒AIR780
 *
 * 连续发送AT，
 * 然后关闭回显(ATE0)
 */
void Air780Wakeup(void);

/*
 * 等待某个 URC 关键字出现（不会像 Air780UartRead 那样遇到 OK 就截断）。
 * 用于等待 "CONNECT OK" / "CONNACK OK" 这类异步上报。
 *
 * token   : 要等待的子串
 * scratch : 调用方提供的临时缓冲区（用于累积接收数据）
 * cap     : scratch 容量
 * timeout_ms : 超时（毫秒，内部按 10ms/tick 折算）
 *
 * 返回：1 命中 token，0 超时
 */
int Air780UartWaitToken(
    const char *token,
    char *scratch,
    int cap,
    uint32_t timeout_ms
);

#ifdef __cplusplus
}
#endif

#endif