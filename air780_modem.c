#include "air780_modem.h"
#include "air780_uart.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

/* ============================ 数值解析工具 ============================ */
/*
 * 本平台精简 printf/scanf 不保证支持 %f，这里全部用整数/字符手写解析，
 * 不依赖 sscanf / atof。
 */

/* 从 s 起跳过前导空格，解析一个十进制整数（可带符号），返回值；
 * 通过 end 回传解析结束位置。无数字时返回 0。 */
static long Air780ParseLong(const char *s, const char **end)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }

    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }

    long v = 0;
    while (*s >= '0' && *s <= '9')
    {
        v = v * 10 + (*s - '0');
        s++;
    }

    if (end != NULL)
    {
        *end = s;
    }
    return neg ? -v : v;
}

/* 解析一个浮点字符串（如 "22.528000" / "-114.07"），用整数累加，不用 %f。 */
static double Air780ParseDouble(const char *s)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }

    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }

    double v = 0.0;
    while (*s >= '0' && *s <= '9')
    {
        v = v * 10.0 + (double)(*s - '0');
        s++;
    }

    if (*s == '.')
    {
        s++;
        double frac = 0.0;
        double scale = 1.0;
        while (*s >= '0' && *s <= '9')
        {
            frac = frac * 10.0 + (double)(*s - '0');
            scale *= 10.0;
            s++;
        }
        v += frac / scale;
    }

    return neg ? -v : v;
}

/*
 * 取以逗号分隔的第 idx 个字段（0 基）拷贝到 out。
 * line 应指向第一个字段起始（例如 "+CGNSINF:" 冒号之后）。
 * 返回：1 成功，0 没有该字段。
 */
static int Air780GetCsvField(const char *line, int idx, char *out, int cap)
{
    int field = 0;
    const char *p = line;

    /* 定位到第 idx 个字段起点 */
    while (field < idx && *p != '\0')
    {
        if (*p == ',')
        {
            field++;
        }
        p++;
    }

    if (field != idx)
    {
        return 0;
    }

    int n = 0;
    while (*p != '\0' && *p != ',' && *p != '\r' && *p != '\n' && n < cap - 1)
    {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return 1;
}

/* ============================ 业务接口 ============================ */

int Air780ModemAttached(void)
{
    char r[128];

    Air780SendAT("AT+CGATT?\r\n", r, sizeof(r), 100);

    /* 期望 "+CGATT: 1" */
    char *p = strstr(r, "+CGATT:");
    if (p == NULL)
    {
        return 0;
    }

    long v = Air780ParseLong(p + 7, NULL);
    return (v == 1) ? 1 : 0;
}

int Air780ModemCsq(void)
{
    char r[128];

    Air780SendAT("AT+CSQ\r\n", r, sizeof(r), 80);

    char *p = strstr(r, "+CSQ:");
    if (p == NULL)
    {
        return -1;
    }

    long rssi = Air780ParseLong(p + 5, NULL);
    return (int)rssi;
}

int Air780ModemGps(air780_gps_t *g)
{
    if (g == NULL)
    {
        return 0;
    }

    g->fix = 0;
    g->lat = 0.0;
    g->lng = 0.0;
    g->alt = 0.0;

    char r[256];
    Air780SendAT("AT+CGNSINF\r\n", r, sizeof(r), 150);

    char *p = strstr(r, "+CGNSINF:");
    if (p == NULL)
    {
        return 0;
    }
    p += (int)strlen("+CGNSINF:");

    char field[32];

    /*
     * +CGNSINF: <run>,<fix>,<utc>,<lat>,<lng>,<alt>,...
     * 字段索引：1=fix，3=lat，4=lng，5=alt
     */
    if (Air780GetCsvField(p, 1, field, sizeof(field)))
    {
        g->fix = (Air780ParseLong(field, NULL) == 1) ? 1 : 0;
    }

    if (Air780GetCsvField(p, 3, field, sizeof(field)) && field[0] != '\0')
    {
        g->lat = Air780ParseDouble(field);
    }

    if (Air780GetCsvField(p, 4, field, sizeof(field)) && field[0] != '\0')
    {
        g->lng = Air780ParseDouble(field);
    }

    if (Air780GetCsvField(p, 5, field, sizeof(field)) && field[0] != '\0')
    {
        g->alt = Air780ParseDouble(field);
    }

    return 1;
}

int Air780ModemPrepare(void)
{
    char r[256];

    /* 唤醒 + 关回显（内部已连发 AT 并 ATE0） */
    Air780Wakeup();

    /* 打开 GNSS */
    Air780SendAT("AT+CGNSPWR=1\r\n", r, sizeof(r), 200);
    if (strstr(r, "OK") == NULL)
    {
        printf("[MODEM] CGNSPWR=1 no OK (GNSS 可能未就绪，可继续)\r\n");
    }

    /* 等待 SIM + 网络附着，最多约 60 秒 */
    for (int i = 0; i < 30; i++)
    {
        if (Air780ModemAttached())
        {
            printf("[MODEM] network attached\r\n");

            /*
             * 关键：使能辅助定位（AGNSS）。
             * Air780EG 纯冷启动很难/极慢定位（日志里可视卫星仅 2 颗、Fix 恒为 0）；
             * 说明书 18.6 节要求打开 GPS 后再发 AT+CGNSAID=<mode>,<time>,<epo>,<loc>，
             * 三项辅助全部使能后约 2~10 秒即可定位。
             * EPO 星历需要联网下载，故放在网络附着成功之后执行。
             */
            Air780SendAT("AT+CGNSAID=31,1,1,1\r\n", r, sizeof(r), 200);
            if (strstr(r, "OK") == NULL)
            {
                printf("[MODEM] CGNSAID no OK (辅助定位未使能，仍可继续等待定位)\r\n");
            }
            else
            {
                printf("[MODEM] AGNSS aiding enabled\r\n");
            }

            return 0;
        }
        printf("[MODEM] waiting network... (%d)\r\n", i);
        osDelay(200);   /* ~2s */
    }

    printf("[MODEM] network attach timeout\r\n");
    return -1;
}
