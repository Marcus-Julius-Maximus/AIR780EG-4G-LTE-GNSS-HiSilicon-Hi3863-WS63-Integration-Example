#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "ohos_init.h"

#include "air780_uart.h"
#include "air780_modem.h"
#include "air780_mqtt.h"
#include "air780_telemetry.h"

/* ============================ 配置 ============================ */

#define MQTT_HOST          "mqtt.example.com"   /* 替换为你自己的 MQTT broker 地址 */
#define MQTT_PORT          1883
#define MQTT_CLIENTID      "MK-000100"   /* 同一 broker 下必须唯一 */
#define MQTT_USER          NULL          /* 匿名 */
#define MQTT_PASS          NULL
#define MQTT_KEEPALIVE     120           /* 秒 */

#define TOPIC_MAIN         "omega/1"         /* OMEGA：专注上传真实遥测数据 */
#define TOPIC_CMD          "alpha/1/cmd"     /* ALPHA：专注接收云端下发指令 */

/* 【新增】：增加上线与状态上报的主题宏定义 */
#define TOPIC_ONLINE       "omega/1/online"  /* 设备上线通知 */
#define TOPIC_STATUS       "omega/1/status"  /* 拨轮状态上报 */

/* 预留通道的默认 topic（启用后即生效，后续按需改） */
#define TOPIC_AUX1         "omega/1/aux1"
#define TOPIC_AUX2         "omega/1/aux2"
#define TOPIC_AUX3         "omega/1/aux3"
#define TOPIC_AUX4         "omega/1/aux4"

/* 本板 1 tick = 10ms，封装一个按毫秒延时的宏 */
#define AIR780_DELAY_MS(ms) osDelay((uint32_t)((ms) / 10))

/* ============================ 内部逻辑 ============================ */

static const uint32_t g_intervals[] = {3000, 10000, 20000, 30000, 60000, 300000};
static int g_interval_idx = 3;               // 默认使用第一档 3s
static uint32_t g_upload_ms = 30000;         // 当前运行的间隔

/* 【新增】：全局上传模式变量 (0=最新值, 1=平均采样)，预留给数据中心跨文件使用 */
int g_upload_mode = 0;                       

static volatile int g_wheel_changed = 0;

void Air780ChangeInterval(int direction)
{
    if (direction > 0) 
    {
        g_interval_idx = (g_interval_idx + 1) % 6;
    } 
    else if (direction < 0) 
    {
        // 加上 6 防止负数取模出错，实现向后循环
        g_interval_idx = (g_interval_idx - 1 + 6) % 6; 
    }
    
    g_upload_ms = g_intervals[g_interval_idx];
    g_wheel_changed = 1;
    Air780TeleSetUploadInterval(g_upload_ms);
    printf("[APP] 刷新率已通过拨轮切换为: %u ms\r\n", g_upload_ms);
}
void Air780ChangeMode(void)
{
    g_upload_mode = (g_upload_mode == 0) ? 1 : 0; // 在 0 和 1 之间反转
    g_wheel_changed = 1; // 复用这个更新标志位，强制触发 MQTT 状态上报
    printf("[APP] 采样模式已通过 K2 切换为: %s\r\n", g_upload_mode ? "AVG (平均)" : "LATEST (最新)");
}

/* ============== 4G 基站时间(NITZ)同步：注入拓展板全局时间 ============== */
/* 主板（smart_wearables_main.c）提供的全局时间注入接口，跨静态库链接解析 */
extern void Set_Global_Time(uint8_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);

static int g_air780_time_synced = 0; // 只成功注入一次

/*
 * 4G 模组附网后，基站会通过 NITZ 主动下发本地时间（含时区），
 * 用 AT+CCLK? 即可零流量取到。首次调用先用 AT+CTZU=1 打开自动时区/时钟更新，
 * 提升 NITZ 成功率。取不到（年份未校准）就静默返回，主板继续以自走时运行。
 */
static void Air780SyncTime(void)
{
    if (g_air780_time_synced)
    {
        return; // 已同步过，零开销返回
    }

    char rx_buf[128] = {0};

    /* 仅在首轮启用一次自动时区/时钟更新（NITZ） */
    static int s_ctzu_set = 0;
    if (!s_ctzu_set)
    {
        Air780SendAT("AT+CTZU=1\r\n", rx_buf, sizeof(rx_buf), 100);
        s_ctzu_set = 1;
    }

    memset(rx_buf, 0, sizeof(rx_buf));
    Air780SendAT("AT+CCLK?\r\n", rx_buf, sizeof(rx_buf), 100);

    /* 返回形如：+CCLK: "26/06/23,14:31:41+32" */
    char *ptr = strstr(rx_buf, "+CCLK: \"");
    if (ptr)
    {
        int year, month, day, hour, min, sec;
        if (sscanf(ptr, "+CCLK: \"%d/%d/%d,%d:%d:%d", &year, &month, &day, &hour, &min, &sec) == 6)
        {
            /* 未校准时年份常为 1980/2004(=80/04)；仅当 24~99 视为基站已下发真实时间 */
            if (year >= 24 && year <= 99)
            {
                Set_Global_Time((uint8_t)year, (uint8_t)month, (uint8_t)day,
                                (uint8_t)hour, (uint8_t)min, (uint8_t)sec);
                g_air780_time_synced = 1;
                printf("[APP] 4G 基站时间(NITZ)同步成功并已注入拓展板!\r\n");
            }
            else
            {
                printf("[APP] 基站时间尚未校准: 20%02d-%02d-%02d，稍后重试\r\n", year, month, day);
            }
        }
    }
}
/*
 * 确保 MQTT 处于已连接状态；未连接则重连（带退避，最多尝试数次）。
 * 返回：1 已连接，0 仍未连接
 */
static int Air780EnsureMqtt(void)
{
    if (Air780MqttConnected())
    {
        return 1;
    }

    for (int attempt = 0; attempt < 3; attempt++)
    {
        printf("[APP] MQTT connecting (attempt %d)...\r\n", attempt + 1);

        if (Air780MqttConnect() == 0)
        {
            /* 【完美分离】：连接成功后，向 ALPHA 通道低头竖起耳朵！ */
            char rx[128];
            /* 利用 C 语言的字符串字面量拼接特性，自动拼入宏定义 */
            Air780SendAT("AT+MSUB=\"" TOPIC_CMD "\",0\r\n", rx, sizeof(rx), 300);
            
            /* 【核心新增】：开机主动呼叫云端，拉取最新配置！ */
            char online_msg[64];
            snprintf(online_msg, sizeof(online_msg), "{\"device_id\":\"%s\"}", MQTT_CLIENTID);
            Air780MqttPublish(TOPIC_ONLINE, online_msg);
            printf("[APP] 已发送上线通知至 %s\r\n", TOPIC_ONLINE);

            return 1;
        }

        AIR780_DELAY_MS(3000);
    }

    return 0;
}

/* 刷新真实传感器数据（GPS + 信号）到数据中心 */
static void Air780RefreshSensors(void)
{
    int csq = Air780ModemCsq();
    if (csq >= 0)
    {
        Air780TeleSetCsq(csq);
        printf("[APP] CSQ=%d\r\n", csq);
    }

    air780_gps_t g;
    if (Air780ModemGps(&g))
    {
        Air780TeleSetGps(g.lng, g.lat, g.alt, g.fix);
        if (g.fix)
        {
            printf("[APP] GPS fixed\r\n");
        }
        else
        {
            printf("[APP] GPS not fixed yet\r\n");
        }
    }
}

static void Air780SetupChannels(void)
{
    /* 主通道：omega/1，整包遥测 JSON，默认开启 */
    Air780ChannelConfig(AIR780_CH_MAIN, TOPIC_MAIN, Air780BuildMainJson);
    Air780ChannelEnable(AIR780_CH_MAIN, 1);

    Air780ChannelConfig(AIR780_CH_AUX1, TOPIC_AUX1, NULL);
    Air780ChannelConfig(AIR780_CH_AUX2, TOPIC_AUX2, NULL);
    Air780ChannelConfig(AIR780_CH_AUX3, TOPIC_AUX3, NULL);
    Air780ChannelConfig(AIR780_CH_AUX4, TOPIC_AUX4, NULL);
    Air780ChannelEnable(AIR780_CH_AUX1, 0);
    Air780ChannelEnable(AIR780_CH_AUX2, 0);
    Air780ChannelEnable(AIR780_CH_AUX3, 0);
    Air780ChannelEnable(AIR780_CH_AUX4, 0);
}

static void Air780Task(void *arg)
{
    (void)arg;
    Air780TeleSetDeviceId(MQTT_CLIENTID);
    Air780TeleSetUploadInterval(g_upload_ms);
    printf("\r\n====================================\r\n");
    printf(" AIR780 TELEMETRY UPLINK START\r\n");
    printf("====================================\r\n");

    /* 1) UART（中断接收）初始化，失败则重试 */
    while (Air780UartInit() != 0)
    {
        printf("[APP] uart init retry...\r\n");
        AIR780_DELAY_MS(1000);
    }

    /* 2) 模组业务：ATE0 + GNSS 开 + 等网络附着 */
    while (Air780ModemPrepare() != 0)
    {
        printf("[APP] modem prepare retry...\r\n");
        AIR780_DELAY_MS(3000);
    }

    /* 3) MQTT 配置 + 通道注册 */
    Air780MqttSetup(MQTT_HOST, MQTT_PORT, MQTT_CLIENTID,
                    MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE);
    Air780SetupChannels();

    /* 4) 主循环：刷新数据 -> 确保连接 -> 发布 */
    while (1)
    {
        Air780RefreshSensors();

        /* 附网后尝试用基站时间(NITZ)同步并注入拓展板；成功一次后自动停止 */
        Air780SyncTime();

        if (Air780EnsureMqtt())
        {
            if (Air780ChannelPublishAll() < 0)
            {
                /* 发布失败已在内部标记断开，下个周期会重连 */
                printf("[APP] publish error, will reconnect next cycle\r\n");
            }
        }
        else
        {
            printf("[APP] MQTT offline, retry next cycle\r\n");
        }

        /* ---------- 核心修復 1：絕對時間戳比對法 ---------- */
        uint32_t start_tick = osKernelGetTickCount();
        uint32_t target_ticks = g_upload_ms / 10; // 換算為 Tick (10ms/tick)
        int break_sleep = 0;

        // 利用絕對系統時間來判斷，徹底免疫提早返回的時間誤差！
        while ((osKernelGetTickCount() - start_tick) < target_ticks && !break_sleep)
        {
            if (g_wheel_changed)
            {
                break_sleep = 1;
                break;
            }
            char rx_buf[256];
            /* 僅做短時非阻塞輪詢，不再用它做延時依據！ */
            int len = Air780UartRead(rx_buf, sizeof(rx_buf), 10); 

            if (len > 0 && strstr(rx_buf, "+MSUB:")) 
            {
                printf("[APP] 收到 MQTT 下发指令: %s\r\n", rx_buf);
                
                /* 【核心新增】：解析上传模式 (Mode) */
                char *mode_ptr = strstr(rx_buf, "mode\":");
                if (mode_ptr) 
                {
                    g_upload_mode = atoi(mode_ptr + 6);
                    printf("[APP] 采样模式已同步为: %d (0=最新, 1=均值)\r\n", g_upload_mode);
                }

                /* 解析间隔 (Interval) */
                char *ptr = strstr(rx_buf, "interval\":");
                if (ptr) 
                {
                    int new_sec = atoi(ptr + 10);
                    int valid_ms = 0;

                    if (new_sec == 3)        valid_ms = 3000;
                    else if (new_sec == 10)  valid_ms = 10000; 
                    else if (new_sec == 20)  valid_ms = 20000; 
                    else if (new_sec == 30)  valid_ms = 30000; 
                    else if (new_sec == 60)  valid_ms = 60000; 
                    else if (new_sec == 300) valid_ms = 300000;

                    if (valid_ms > 0)
                    {
                        g_upload_ms = valid_ms;
                        Air780TeleSetUploadInterval(g_upload_ms);
                        for (int i = 0; i < 6; i++) {
                            if (g_intervals[i] == g_upload_ms) g_interval_idx = i;
                        }
                        printf("[APP] 刷新率已通过 MQTT 切换为: %u ms\r\n", g_upload_ms);
                        break_sleep = 1; 
                    }
                }
            }
        }
        
        if (g_wheel_changed)
        {
            g_wheel_changed = 0; // 清除旗標
            if (Air780MqttConnected())
            {
                char report[128];
                /* 【核心新增】：拼裝狀態 JSON，帶上 device_id 防流浪 */
                snprintf(report, sizeof(report), "{\"device_id\":\"%s\",\"current_interval\":%u,\"mode\":%d}", 
         MQTT_CLIENTID, g_upload_ms, g_upload_mode);
                Air780MqttPublish(TOPIC_STATUS, report);
                printf("[APP] 撥輪換擋成功，已實時同步至雲端 %s : %s \r\n", TOPIC_STATUS, report);
            }
        }
    }
}

static void Air780Entry(void)
{
    osThreadAttr_t attr;

    memset(&attr, 0, sizeof(attr));
    attr.name       = "Air780Task";
    attr.stack_size = 8192;
    attr.priority   = osPriorityNormal;

    osThreadNew(Air780Task, NULL, &attr);
}

APP_FEATURE_INIT(Air780Entry);