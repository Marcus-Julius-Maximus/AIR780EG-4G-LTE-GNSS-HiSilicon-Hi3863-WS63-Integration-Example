#include "air780_mqtt.h"
#include "air780_uart.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

/* 配置缓存 */
static char g_host[64]     = {0};
static int  g_port         = 1883;
static char g_clientid[64] = {0};
static char g_user[64]     = {0};
static char g_pass[64]     = {0};
static int  g_keepalive    = 120;

static int  g_connected    = 0;

static void Air780CopyStr(char *dst, int cap, const char *src)
{
    if (src == NULL)
    {
        dst[0] = '\0';
        return;
    }
    int i = 0;
    while (src[i] != '\0' && i < cap - 1)
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

void Air780MqttSetup(
    const char *host,
    int port,
    const char *clientid,
    const char *user,
    const char *pass,
    int keepalive_sec
)
{
    Air780CopyStr(g_host, sizeof(g_host), host);
    Air780CopyStr(g_clientid, sizeof(g_clientid), clientid);
    Air780CopyStr(g_user, sizeof(g_user), user);
    Air780CopyStr(g_pass, sizeof(g_pass), pass);
    g_port = (port > 0) ? port : 1883;
    g_keepalive = (keepalive_sec > 0) ? keepalive_sec : 120;
    g_connected = 0;
}

int Air780MqttConnected(void)
{
    return g_connected;
}

void Air780MqttDisconnect(void)
{
    char r[128];
    Air780SendAT("AT+MDISCONNECT\r\n", r, sizeof(r), 100);
    g_connected = 0;
}

int Air780MqttConnect(void)
{
    char cmd[256];
    char r[256];
    char scratch[256];

    g_connected = 0;

    /* 1) MCONFIG：设置 clientid（及可选用户名/密码） */
    if (g_user[0] != '\0')
    {
        snprintf(cmd, sizeof(cmd),
                 "AT+MCONFIG=\"%s\",\"%s\",\"%s\"\r\n",
                 g_clientid, g_user, g_pass);
    }
    else
    {
        snprintf(cmd, sizeof(cmd), "AT+MCONFIG=\"%s\"\r\n", g_clientid);
    }

    Air780SendAT(cmd, r, sizeof(r), 100);
    if (strstr(r, "OK") == NULL)
    {
        printf("[MQTT] MCONFIG failed\r\n");
        return -1;
    }

    /* 2) MIPSTART：建立 TCP，随后等待 URC "CONNECT OK" */
    snprintf(cmd, sizeof(cmd), "AT+MIPSTART=\"%s\",%d\r\n", g_host, g_port);
    Air780SendAT(cmd, r, sizeof(r), 100);

    if (!Air780UartWaitToken("CONNECT OK", scratch, sizeof(scratch), 15000))
    {
        if (strstr(scratch, "ALREADY CONNECT") == NULL)
        {
            printf("[MQTT] TCP connect failed\r\n");
            return -1;
        }
    }

    /* 3) MCONNECT：MQTT 会话，等待 URC "CONNACK OK" */
    snprintf(cmd, sizeof(cmd), "AT+MCONNECT=1,%d\r\n", g_keepalive);
    Air780SendAT(cmd, r, sizeof(r), 100);

    if (!Air780UartWaitToken("CONNACK OK", scratch, sizeof(scratch), 10000))
    {
        printf("[MQTT] CONNACK failed\r\n");
        return -1;
    }

    g_connected = 1;
    printf("[MQTT] connected to %s:%d\r\n", g_host, g_port);
    return 0;
}

int Air780MqttPublish(const char *topic, const char *payload)
{
    if (topic == NULL || payload == NULL)
    {
        return -1;
    }
    if (!g_connected)
    {
        return -1;
    }

    /* 转义 payload：AT+MPUB 里 " 用 \22、CR 用 \0D、LF 用 \0A、\ 用 \5C 表示 */
    static char esc[1024];
    int j = 0;
    for (int i = 0; payload[i] != '\0' && j < (int)sizeof(esc) - 4; i++)
    {
        char c = payload[i];
        if (c == '"')       { esc[j++] = '\\'; esc[j++] = '2'; esc[j++] = '2'; }
        else if (c == '\r') { esc[j++] = '\\'; esc[j++] = '0'; esc[j++] = 'D'; }
        else if (c == '\n') { esc[j++] = '\\'; esc[j++] = '0'; esc[j++] = 'A'; }
        else if (c == '\\') { esc[j++] = '\\'; esc[j++] = '5'; esc[j++] = 'C'; }
        else                { esc[j++] = c; }
    }
    esc[j] = '\0';

    static char cmd[1280];
    snprintf(cmd, sizeof(cmd), "AT+MPUB=\"%s\",0,0,\"%s\"\r\n", topic, esc);

    char r[256];
    Air780SendAT(cmd, r, sizeof(r), 300);

    if (strstr(r, "OK") != NULL)
    {
        return 0;
    }

    /* 发布失败：可能连接已断，标记断开让上层重连 */
    printf("[MQTT] publish failed -> mark disconnected\r\n");
    g_connected = 0;
    return -1;
}
