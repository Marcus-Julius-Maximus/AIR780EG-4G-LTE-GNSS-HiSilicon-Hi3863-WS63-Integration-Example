#include "air780_telemetry.h"
#include "air780_mqtt.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* 引入来自 air780_app.c 的全局上传模式标志 */
extern int g_upload_mode; // 0=最新值, 1=平均采样

/* ============================ 数据中心 ============================ */
typedef struct
{
    char device_id[32]; 
    struct { double lng, lat, alt; int fix; } gps;
    struct { double ax, ay, az, gx, gy, gz; } imu;
    struct { int battery; double ntctemp, temp, humidity; uint32_t upload_interval; } base; 
    int csq;
} air780_tele_t;

static air780_tele_t g_tele =
{
    .device_id = "UNKNOWN", 
    .gps  = { .lng = 114.07, .lat = 22.528, .alt = 20.5, .fix = 0 },
    .imu  = { .ax = 10.5, .ay = 2.1, .az = 9.8, .gx = 0.05, .gy = 0.12, .gz = -0.03 },
    .base = { .battery = 59, .ntctemp = 39.2, .temp = 39.2, .humidity = 45.0, .upload_interval = 30000 },
    .csq  = 99,
};

/* ============================ 边缘计算累加器 ============================ */
static int count_ntc = 0;   static double sum_ntc = 0;
static int count_temp = 0;  static double sum_temp = 0;
static int count_humi = 0;  static double sum_humi = 0;

static int count_imu = 0;   
static int active_ticks = 0; // 记录运动帧数
static double sum_delta_g = 0; // 净动能累加
static double max_delta_g = 0; // 净动能峰值
static double sum_gyro = 0;    // 角速度累加
static double max_gyro = 0;    // 角速度峰值

/* ============================== Setter ============================== */
void Air780TeleSetDeviceId(const char *id)
{
    if (id) {
        strncpy(g_tele.device_id, id, sizeof(g_tele.device_id) - 1);
        g_tele.device_id[sizeof(g_tele.device_id) - 1] = '\0';
    }
}

uint32_t Air780TeleGetUploadInterval(void) { return g_tele.base.upload_interval; }
void Air780TeleSetUploadInterval(uint32_t ms) { g_tele.base.upload_interval = ms; }
double Air780TeleGetTemp(void) { return g_tele.base.temp; }
double Air780TeleGetHumidity(void) { return g_tele.base.humidity; }
double Air780TeleGetNtcTemp(void) { return g_tele.base.ntctemp; }

void Air780TeleGetImuAcc(double *ax, double *ay, double *az) {
    if(ax) *ax = g_tele.imu.ax;
    if(ay) *ay = g_tele.imu.ay;
    if(az) *az = g_tele.imu.az;
}

void Air780TeleSetGps(double lng, double lat, double alt, int fix)
{
    g_tele.gps.lng = lng; 
    g_tele.gps.lat = lat; 
    g_tele.gps.alt = alt; 
    g_tele.gps.fix = fix;
}

void Air780TeleSetCsq(int csq)                  { g_tele.csq = csq; }
void Air780TeleSetBattery(int b)                { g_tele.base.battery = b; }
int  Air780TeleGetCsq(void)                     { return g_tele.csq; }

void Air780TeleSetNtcTemp(double t)             
{ 
    sum_ntc += t; count_ntc++;
    g_tele.base.ntctemp = t; 
}
void Air780TeleSetTemp(double t)                
{ 
    sum_temp += t; count_temp++;
    g_tele.base.temp = t; 
}
void Air780TeleSetHumidity(double h)            
{ 
    sum_humi += h; count_humi++;
    g_tele.base.humidity = h; 
}

void Air780TeleSetImu(double ax, double ay, double az, double gx, double gy, double gz)
{
    // 1. 实时更新供 OLED 屏幕读取
    g_tele.imu.ax = ax; g_tele.imu.ay = ay; g_tele.imu.az = az;
    g_tele.imu.gx = gx; g_tele.imu.gy = gy; g_tele.imu.gz = gz;

    // 2. 边缘计算逻辑：提取每一帧的绝对向量
    double inst_mag = sqrt(ax * ax + ay * ay + az * az);
    
    // 手动计算绝对差值，剥离 9.80665 的背景重力，获取纯粹动能
    double delta_g = (inst_mag > 9.80665) ? (inst_mag - 9.80665) : (9.80665 - inst_mag);
    double inst_gyro = (gx > 0 ? gx : -gx) + (gy > 0 ? gy : -gy) + (gz > 0 ? gz : -gz);

    // 动能累加与峰值记录
    sum_delta_g += delta_g;
    if (delta_g > max_delta_g) max_delta_g = delta_g;

    sum_gyro += inst_gyro;
    if (inst_gyro > max_gyro) max_gyro = inst_gyro;

    // 记录活跃帧数 (偏离重力1.2G 或 陀螺仪抖动>25)
    if (delta_g > 1.2 || inst_gyro > 25) {
        active_ticks++;
    }
    count_imu++;
}

/* ======================= 定点数字格式化 ======================= */
static int Air780AppendStr(char *out, int pos, int cap, const char *s)
{
    while (*s != '\0' && pos < cap - 1) { out[pos++] = *s++; }
    out[pos] = '\0'; return pos;
}

static int Air780AppendInt(char *out, int pos, int cap, long v)
{
    if (v < 0) { if (pos < cap - 1) out[pos++] = '-'; v = -v; }
    char rev[16]; int n = 0;
    if (v == 0) { rev[n++] = '0'; }
    else { while (v > 0 && n < (int)sizeof(rev)) { rev[n++] = (char)('0' + (v % 10)); v /= 10; } }
    while (n > 0 && pos < cap - 1) { out[pos++] = rev[--n]; }
    out[pos] = '\0'; return pos;
}

static int Air780AppendFixed(char *out, int pos, int cap, double v, int dec)
{
    if (v < 0) { if (pos < cap - 1) out[pos++] = '-'; v = -v; }
    long mul = 1;
    for (int i = 0; i < dec; i++) mul *= 10;
    long ipart = (long)v;
    double fracd = (v - (double)ipart) * (double)mul + 0.5;   
    long fpart = (long)fracd;
    if (fpart >= mul) { ipart += 1; fpart -= mul; }
    pos = Air780AppendInt(out, pos, cap, ipart);
    if (dec > 0)
    {
        if (pos < cap - 1) out[pos++] = '.';
        char fb[16];
        for (int i = 0; i < dec && i < (int)sizeof(fb); i++) fb[i] = '0';
        int idx = dec - 1; long x = fpart;
        while (x > 0 && idx >= 0) { fb[idx--] = (char)('0' + (x % 10)); x /= 10; }
        for (int i = 0; i < dec && pos < cap - 1; i++) { out[pos++] = fb[i]; }
        out[pos] = '\0';
    }
    return pos;
}

/* ============================ JSON 构造 ============================ */
int Air780BuildMainJson(char *out, int cap)
{
    if (out == NULL || cap < 8) return 0;

    /* 默认使用最新值 */
    double json_ntc = g_tele.base.ntctemp;
    double json_temp = g_tele.base.temp;
    double json_humi = g_tele.base.humidity;
    double json_ax = g_tele.imu.ax;
    double json_ay = g_tele.imu.ay;
    double json_az = g_tele.imu.az;
    double json_gx = g_tele.imu.gx;
    double json_gy = g_tele.imu.gy;
    double json_gz = g_tele.imu.gz;

    /* 如果模式开启，且计数有效，则换用平均采样值 */
    if (g_upload_mode == 1 && count_imu > 0) {
        if (count_ntc > 0)  json_ntc = sum_ntc / count_ntc;
        if (count_temp > 0) json_temp = sum_temp / count_temp;
        if (count_humi > 0) json_humi = sum_humi / count_humi;
        
        // 【核心封装】：将平均动能、最大动能、活跃比例打包
        json_ax = sum_delta_g / count_imu;  
        json_ay = max_delta_g;
        json_az = (double)active_ticks / count_imu;

        json_gx = sum_gyro / count_imu;
        json_gy = max_gyro;
        json_gz = 0.0;
    }

    /* 清空累加器，迎接下一周期 */
    sum_ntc = 0; count_ntc = 0; 
    sum_temp = 0; count_temp = 0; 
    sum_humi = 0; count_humi = 0;
    sum_delta_g = 0; max_delta_g = 0; 
    sum_gyro = 0; max_gyro = 0; 
    count_imu = 0; active_ticks = 0;

    int p = 0;
    
    /* 彻底展开的 JSON 拼接，保证可读性 */
    p = Air780AppendStr(out, p, cap, "{\"device_id\":\"");
    p = Air780AppendStr(out, p, cap, g_tele.device_id);
    
    p = Air780AppendStr(out, p, cap, "\",\"mode\":"); 
    p = Air780AppendInt(out, p, cap, g_upload_mode);
    
    p = Air780AppendStr(out, p, cap, ",\"gps\":{\"lng\":");
    p = Air780AppendFixed(out, p, cap, g_tele.gps.lng, 6);
    
    p = Air780AppendStr(out, p, cap, ",\"lat\":");
    p = Air780AppendFixed(out, p, cap, g_tele.gps.lat, 6);
    
    p = Air780AppendStr(out, p, cap, ",\"alt\":");
    p = Air780AppendFixed(out, p, cap, g_tele.gps.alt, 1);
    
    p = Air780AppendStr(out, p, cap, "},\"imu\":{\"ax\":");
    p = Air780AppendFixed(out, p, cap, json_ax, 2);
    
    p = Air780AppendStr(out, p, cap, ",\"ay\":");
    p = Air780AppendFixed(out, p, cap, json_ay, 2);
    
    p = Air780AppendStr(out, p, cap, ",\"az\":");
    p = Air780AppendFixed(out, p, cap, json_az, 3);
    
    p = Air780AppendStr(out, p, cap, ",\"gx\":");
    p = Air780AppendFixed(out, p, cap, json_gx, 2);
    
    p = Air780AppendStr(out, p, cap, ",\"gy\":");
    p = Air780AppendFixed(out, p, cap, json_gy, 2);
    
    p = Air780AppendStr(out, p, cap, ",\"gz\":");
    p = Air780AppendFixed(out, p, cap, json_gz, 2);
    
    p = Air780AppendStr(out, p, cap, "},\"base\":{\"battery\":");
    p = Air780AppendInt(out, p, cap, g_tele.base.battery);
    
    p = Air780AppendStr(out, p, cap, ",\"ntctemp\":");
    p = Air780AppendFixed(out, p, cap, json_ntc, 1);
    
    p = Air780AppendStr(out, p, cap, ",\"temp\":");
    p = Air780AppendFixed(out, p, cap, json_temp, 1);
    
    p = Air780AppendStr(out, p, cap, ",\"humidity\":");
    p = Air780AppendFixed(out, p, cap, json_humi, 1);
    
    p = Air780AppendStr(out, p, cap, "}}");
    
    return p;
}

int Air780BuildStatusJson(char *out, int cap)
{
    if (out == NULL || cap < 8) return 0;
    int p = 0;
    p = Air780AppendStr(out, p, cap, "{\"device_id\":\"");
    p = Air780AppendStr(out, p, cap, g_tele.device_id);
    p = Air780AppendStr(out, p, cap, "\",\"csq\":");
    p = Air780AppendInt(out, p, cap, g_tele.csq);
    p = Air780AppendStr(out, p, cap, ",\"fix\":");
    p = Air780AppendInt(out, p, cap, g_tele.gps.fix);
    p = Air780AppendStr(out, p, cap, "}");
    return p;
}

/* ============================ 通道注册表 ============================ */
typedef struct
{
    const char        *topic;
    air780_payload_fn  build;
    int                enabled;
} air780_channel_t;

static air780_channel_t g_channels[AIR780_CH_MAX];

void Air780ChannelConfig(int idx, const char *topic, air780_payload_fn fn) {
    if (idx >= 0 && idx < AIR780_CH_MAX) { 
        g_channels[idx].topic = topic; 
        g_channels[idx].build = fn; 
    }
}

void Air780ChannelEnable(int idx, int enable) {
    if (idx >= 0 && idx < AIR780_CH_MAX) { 
        g_channels[idx].enabled = enable ? 1 : 0; 
    }
}

int Air780ChannelPublishAll(void)
{
    static char buf[768];
    int sent = 0;
    for (int i = 0; i < AIR780_CH_MAX; i++) {
        if (!g_channels[i].enabled || g_channels[i].build == NULL || g_channels[i].topic == NULL) {
            continue;
        }
        int len = g_channels[i].build(buf, sizeof(buf));
        if (len <= 0) continue;
        
        if (Air780MqttPublish(g_channels[i].topic, buf) == 0) {
            sent++;
            printf("[CH%d] pub %s ok\r\n", i, g_channels[i].topic);
        } else {
            printf("[CH%d] pub %s FAIL\r\n", i, g_channels[i].topic);
            return -1; 
        }
    }
    return sent;
}