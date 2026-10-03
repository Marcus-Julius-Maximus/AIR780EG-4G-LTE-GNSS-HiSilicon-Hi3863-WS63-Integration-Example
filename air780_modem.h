#ifndef AIR780_MODEM_H
#define AIR780_MODEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GNSS 定位结果。
 * fix=1 表示已定位，此时 lat/lng/alt 才有效。
 */
typedef struct
{
    int    fix;     /* 0 未定位 / 1 已定位 */
    double lat;     /* 纬度  ±dd.dddddd */
    double lng;     /* 经度  ±ddd.dddddd */
    double alt;     /* 海拔(m) */
} air780_gps_t;

/*
 * 模组业务初始化：关回显(ATE0) -> 打开 GNSS(CGNSPWR=1) -> 等待 SIM/网络附着
 *                -> 使能辅助定位(CGNSAID=31,1,1,1，需联网下载 EPO 星历)。
 *
 * 返回：0 成功（已附着网络），-1 失败/超时
 */
int Air780ModemPrepare(void);

/*
 * 读取信号质量 AT+CSQ。
 * 返回：rssi(0~31)，99=未知，-1=读取失败
 */
int Air780ModemCsq(void);

/*
 * 读取 GNSS 信息 AT+CGNSINF，解析到 g。
 * 返回：1 成功解析（不代表已定位，看 g->fix），0 失败
 */
int Air780ModemGps(air780_gps_t *g);

/*
 * 查询是否已附着数据网络 AT+CGATT?。
 * 返回：1 已附着，0 未附着/失败
 */
int Air780ModemAttached(void);

#ifdef __cplusplus
}
#endif

#endif
