#ifndef AIR780_MQTT_H
#define AIR780_MQTT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 保存 MQTT 连接配置（不发起连接）。
 * user/pass 传 NULL 或空字符串表示匿名（mosquitto allow_anonymous）。
 */
void Air780MqttSetup(
    const char *host,
    int port,
    const char *clientid,
    const char *user,
    const char *pass,
    int keepalive_sec
);

/*
 * 发起完整连接：AT+MCONFIG -> AT+MIPSTART(等 CONNECT OK) -> AT+MCONNECT(等 CONNACK OK)。
 * 返回：0 成功，-1 失败
 */
int Air780MqttConnect(void);

/*
 * 断开 MQTT（AT+MDISCONNECT）。
 */
void Air780MqttDisconnect(void);

/*
 * 是否处于已连接状态（基于内部状态 + 发布结果维护）。
 * 返回：1 已连接，0 未连接
 */
int Air780MqttConnected(void);

/*
 * 发布一条消息到指定 topic（QoS0、retain0）。
 * payload 内部会自动做 AT+MPUB 的转义（" -> \22, \r -> \0D, \n -> \0A, \ -> \5C）。
 * 发布失败会把内部连接状态置为断开，便于上层触发重连。
 *
 * 返回：0 成功，-1 失败
 */
int Air780MqttPublish(const char *topic, const char *payload);

#ifdef __cplusplus
}
#endif

#endif
