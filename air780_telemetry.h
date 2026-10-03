#ifndef AIR780_TELEMETRY_H
#define AIR780_TELEMETRY_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 *  遥测数据中心 + JSON 构造 + 上传通道注册表
 *
 *  - 任何模块（GPS、NTC、IMU、电量…）通过下面的 setter 往中心写数据；
 *  - 主通道 omega/1 的 JSON 由 Air780BuildMainJson() 按 json.txt 的结构生成；
 *  - 通道表共 5 个槽位：0 = 主通道，1~4 = 预留通道，方便后续挂接新数据。
 * ============================================================================
 */
/* ====================== 数据读取接口（供 OLED 等 UI 使用） ====================== */
// ... 原本的讀取接口 ...
uint32_t Air780TeleGetUploadInterval(void); // 【新增】讀取刷新率

/* ====================== 数据写入接口（线程内调用） ====================== */
// ... 原本的寫入接口 ...
void Air780TeleSetDeviceId(const char *id); // 【新增】動態注入設備 ID
void Air780TeleSetUploadInterval(uint32_t ms); // 【新增】寫入刷新率
double Air780TeleGetNtcTemp(void);
void Air780TeleGetImuAcc(double *ax, double *ay, double *az);
double Air780TeleGetTemp(void);
double Air780TeleGetHumidity(void);
/* ---- 通道索引 ---- */
#define AIR780_CH_MAIN   0      /* 主通道：omega/1，整包遥测 JSON */
#define AIR780_CH_AUX1   1      /* 预留 1 */
#define AIR780_CH_AUX2   2      /* 预留 2 */
#define AIR780_CH_AUX3   3      /* 预留 3 */
#define AIR780_CH_AUX4   4      /* 预留 4 */
#define AIR780_CH_MAX    5

/*
 * 通道 payload 生成回调：把要发布的字符串写入 out（容量 cap），
 * 返回写入长度(>0)；返回 <=0 表示本次不发布。
 */
typedef int (*air780_payload_fn)(char *out, int cap);

/* ====================== 数据写入接口（线程内调用） ====================== */

void Air780TeleSetGps(double lng, double lat, double alt, int fix);
void Air780TeleSetCsq(int csq);                 /* 0~31，99 未知 */
void Air780TeleSetImu(double ax, double ay, double az,
                      double gx, double gy, double gz);
void Air780TeleSetBattery(int battery_pct);
void Air780TeleSetNtcTemp(double ntctemp);
void Air780TeleSetTemp(double temp);
void Air780TeleSetHumidity(double humidity);

/* 读取当前信号（供日志/预留通道使用） */
int  Air780TeleGetCsq(void);

/* ====================== JSON 构造 ====================== */

/*
 * 生成主通道 JSON（json.txt 结构：gps/imu/base），单行、无换行。
 * 全程整数定点拼接，不使用 %f。
 * 返回写入长度。
 */
int Air780BuildMainJson(char *out, int cap);

/*
 * 预置的“信号/状态”JSON：{"csq":..,"fix":..}。
 * 默认未挂到任何通道；需要上报信号时，把它配到某个预留通道即可（见 .c 注释）。
 */
int Air780BuildStatusJson(char *out, int cap);

/* ====================== 通道注册表 ====================== */

/* 配置某通道的 topic 与 payload 生成回调（不改变启用状态） */
void Air780ChannelConfig(int idx, const char *topic, air780_payload_fn fn);

/* 启用/停用某通道 */
void Air780ChannelEnable(int idx, int enable);

/*
 * 遍历所有“已启用且已配置回调”的通道，逐个生成 payload 并发布。
 * 返回：成功发布的通道数；若发布过程中出现失败返回 -1（上层据此重连）。
 */
int Air780ChannelPublishAll(void);

#ifdef __cplusplus
}
#endif

#endif
