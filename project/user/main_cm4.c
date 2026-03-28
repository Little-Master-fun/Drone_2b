/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "drivers/driver_sch16tk01.h"
#include "estimator/attitude_estimator_6axis.h"

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define SCH16TK01_DEMO_DT_S                       (0.02f)
#define SCH16TK01_ACC_SCALE_MSS_PER_LSB           (1.0f / 3200.0f)
#define SCH16TK01_GYRO_SCALE_REV1_DPS_PER_LSB     (1.0f / 1600.0f)
#define SCH16TK01_GYRO_SCALE_REV2_DPS_PER_LSB     (1.0f / 100.0f)
#define SCH16TK01_DEG_TO_RAD                      (M_PI / 180.0f)

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

typedef struct
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float acc_x_mss;
    float acc_y_mss;
    float acc_z_mss;
    float temp_deg;
} app_sch16tk01_scaled_data_struct;

typedef struct
{
    driver_sch16tk01_status_struct status;
    driver_sch16tk01_data_struct imu;
    app_sch16tk01_scaled_data_struct scaled;
    attitude_estimator_6axis_state_struct attitude;
    uint32 update_count;
    uint8 init_done;
    uint8 read_ok;
    int8 text[128];
} app_sch16tk01_demo_data_struct;

app_sch16tk01_demo_data_struct g_sch16tk01_data;

static void sch16tk01_demo_init (void)
{
    memset(&g_sch16tk01_data, 0, sizeof(g_sch16tk01_data));
    attitude_estimator_6axis_init();

    if (0U == driver_sch16tk01_init())
    {
        g_sch16tk01_data.init_done = 1U;
        g_sch16tk01_data.read_ok = 1U;
        zf_sprintf(g_sch16tk01_data.text, "sch16tk01 init ok");
    }
    else
    {
        g_sch16tk01_data.read_ok = 0U;
        zf_sprintf(g_sch16tk01_data.text, "sch16tk01 init failed");
    }

    g_sch16tk01_data.status = driver_sch16tk01_get_status();
    g_sch16tk01_data.init_done = g_sch16tk01_data.status.initialized;
    g_sch16tk01_data.attitude = attitude_estimator_6axis_get_state();
}

static void sch16tk01_demo_update (void)
{
    uint8 ret = 0U;
    float gyro_scale_dps_per_lsb = SCH16TK01_GYRO_SCALE_REV1_DPS_PER_LSB;

    g_sch16tk01_data.status = driver_sch16tk01_get_status();
    g_sch16tk01_data.init_done = g_sch16tk01_data.status.initialized;

    if (!g_sch16tk01_data.init_done)
    {
        g_sch16tk01_data.read_ok = 0U;
        zf_sprintf(g_sch16tk01_data.text,
                   "sch not ready detect:%d asic:0x%04x comp:0x%04x",
                   (int32)g_sch16tk01_data.status.detect_status,
                   (int32)g_sch16tk01_data.status.asic_id,
                   (int32)g_sch16tk01_data.status.comp_id);
        return;
    }

    ret = driver_sch16tk01_read(&g_sch16tk01_data.imu);
    if (0U != ret)
    {
        g_sch16tk01_data.read_ok = 0U;
        zf_sprintf(g_sch16tk01_data.text, "sch16tk01 read error:%d", ret);
        return;
    }

    if (g_sch16tk01_data.status.chip_version == DRIVER_SCH16TK01_CHIP_VERSION_REV2)
    {
        gyro_scale_dps_per_lsb = SCH16TK01_GYRO_SCALE_REV2_DPS_PER_LSB;
    }

    g_sch16tk01_data.scaled.gyro_x_dps = (float)g_sch16tk01_data.imu.gyro_x_raw * gyro_scale_dps_per_lsb;
    g_sch16tk01_data.scaled.gyro_y_dps = (float)g_sch16tk01_data.imu.gyro_y_raw * gyro_scale_dps_per_lsb;
    g_sch16tk01_data.scaled.gyro_z_dps = (float)g_sch16tk01_data.imu.gyro_z_raw * gyro_scale_dps_per_lsb;
    g_sch16tk01_data.scaled.acc_x_mss = (float)g_sch16tk01_data.imu.acc_x_raw * SCH16TK01_ACC_SCALE_MSS_PER_LSB;
    g_sch16tk01_data.scaled.acc_y_mss = (float)g_sch16tk01_data.imu.acc_y_raw * SCH16TK01_ACC_SCALE_MSS_PER_LSB;
    g_sch16tk01_data.scaled.acc_z_mss = (float)g_sch16tk01_data.imu.acc_z_raw * SCH16TK01_ACC_SCALE_MSS_PER_LSB;
    g_sch16tk01_data.scaled.temp_deg = (float)g_sch16tk01_data.imu.temp_cdeg / 100.0f;

    (void)attitude_estimator_6axis_update(g_sch16tk01_data.scaled.gyro_x_dps * SCH16TK01_DEG_TO_RAD,
                                          g_sch16tk01_data.scaled.gyro_y_dps * SCH16TK01_DEG_TO_RAD,
                                          g_sch16tk01_data.scaled.gyro_z_dps * SCH16TK01_DEG_TO_RAD,
                                          g_sch16tk01_data.scaled.acc_x_mss,
                                          g_sch16tk01_data.scaled.acc_y_mss,
                                          g_sch16tk01_data.scaled.acc_z_mss,
                                          SCH16TK01_DEMO_DT_S);
    g_sch16tk01_data.attitude = attitude_estimator_6axis_get_state();

    g_sch16tk01_data.update_count += 1U;
    g_sch16tk01_data.read_ok = 1U;
    zf_sprintf(g_sch16tk01_data.text,
               "r:%d p:%d y:%d gx:%d gy:%d gz:%d",
               (int32)g_sch16tk01_data.attitude.roll_deg,
               (int32)g_sch16tk01_data.attitude.pitch_deg,
               (int32)g_sch16tk01_data.attitude.yaw_deg,
               (int32)g_sch16tk01_data.scaled.gyro_x_dps,
               (int32)g_sch16tk01_data.scaled.gyro_y_dps,
               (int32)g_sch16tk01_data.scaled.gyro_z_dps);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    sch16tk01_demo_init();              // SCH16TK01 初始化，查看 g_sch16tk01_data 即可观察原始 IMU 数据

    for(;;)
    {
        sch16tk01_demo_update();        // 持续刷新 SCH16TK01 数据结构体
        system_delay_ms(20);            // 50Hz 刷新，便于调试观察
    }
}

// **************************** 代码区域 ****************************
