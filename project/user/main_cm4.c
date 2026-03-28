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
#include "drivers/driver_vl53l1x.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

typedef struct
{
    driver_vl53l1x_status_struct status;
    driver_vl53l1x_data_struct distance;
    uint32 update_count;
    uint8 init_done;
    uint8 read_ok;
    int8 text[96];
} app_vl53l1x_demo_data_struct;

app_vl53l1x_demo_data_struct g_vl53l1x_distance_data;

static void vl53l1x_demo_init (void)
{
    memset(&g_vl53l1x_distance_data, 0, sizeof(g_vl53l1x_distance_data));

    if (0U == driver_vl53l1x_init())
    {
        g_vl53l1x_distance_data.init_done = 1U;
        g_vl53l1x_distance_data.read_ok = 1U;
        zf_sprintf(g_vl53l1x_distance_data.text, "vl53l1x init ok");
    }
    else
    {
        g_vl53l1x_distance_data.read_ok = 0U;
        zf_sprintf(g_vl53l1x_distance_data.text, "vl53l1x init failed");
    }

    g_vl53l1x_distance_data.status = driver_vl53l1x_get_status();
    g_vl53l1x_distance_data.init_done = g_vl53l1x_distance_data.status.initialized;
}

static void vl53l1x_demo_update (void)
{
    uint8 ret = 0U;

    g_vl53l1x_distance_data.status = driver_vl53l1x_get_status();
    g_vl53l1x_distance_data.init_done = g_vl53l1x_distance_data.status.initialized;

    if (!g_vl53l1x_distance_data.init_done)
    {
        g_vl53l1x_distance_data.read_ok = 0U;
        zf_sprintf(g_vl53l1x_distance_data.text,
                   "vl53l1x not ready detect:%d model:0x%02x",
                   (int32)g_vl53l1x_distance_data.status.detect_status,
                   (int32)g_vl53l1x_distance_data.status.model_id);
        return;
    }

    ret = driver_vl53l1x_read(&g_vl53l1x_distance_data.distance);
    if (0U != ret)
    {
        g_vl53l1x_distance_data.read_ok = 0U;
        zf_sprintf(g_vl53l1x_distance_data.text, "vl53l1x read error:%d", ret);
        return;
    }

    g_vl53l1x_distance_data.update_count += 1U;
    g_vl53l1x_distance_data.read_ok = 1U;
    zf_sprintf(g_vl53l1x_distance_data.text,
               "dist:%dmm status:%d ready:%d",
               (int32)g_vl53l1x_distance_data.distance.distance_mm,
               (int32)g_vl53l1x_distance_data.distance.range_status,
               (int32)g_vl53l1x_distance_data.distance.data_ready);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    vl53l1x_demo_init();                // VL53L1X 初始化，查看 g_vl53l1x_distance_data 即可观察测距数据

    for(;;)
    {
        vl53l1x_demo_update();          // 持续刷新测距数据结构体
        system_delay_ms(20);            // 50Hz 刷新，便于调试观察
    }
}

// **************************** 代码区域 ****************************
