<!--
 * @Author: Littlemaster littlemasterfun@gmail.com
 * @Date: 2026-03-28 20:04:02
 * @LastEditors: Littlemaster littlemasterfun@gmail.com
 * @LastEditTime: 2026-03-29 09:53:00
 * @FilePath: /mcu/Drone_2b/pins.md
 * @Description: 
 * 
 * Copyright (c) 2026 by LittleMaster, All Rights Reserved. 
-->
| 设备 | 通信方式 | 接口号/总线 | MCU 引脚 | 说明 |
|---|---|---|---|---|
| IMU - SCH16TK01 | SPI | SPI1 | `P06.2 / SCK` `P06.1 / MOSI` `P06.0 / MISO` `P06.3 / CS` | 当前主 IMU，输出陀螺和加速度数据 |
| 光流 - PMW3901 | SPI | SPI3 | `P13.2 / SCK` `P13.1 / MOSI` `P13.0 / MISO` `P13.3 / CS` | 向下光流传感器 |
| 测距 - VL53L1X / DL1B | I2C | Soft I2C | `P11.1 / SCL` `P11.2 / SDA` | 当前高度测距输入 |
| 无线串口模块 | UART | UART2 | `P07.0 / RX` `P07.1 / TX` `P07.2 / RTS` | 地面站通信、参数和遥测链路 |
| 4in1 电调 | DShot600 | Motor1~4 | `P18.0 / M1` `P18.1 / M2` `P18.2 / M3` `P18.3 / M4` | 当前飞控输出使用并行 DShot600，`P18.4` 预留未使用 |
| 调试串口 | UART | Debug UART | 见 `debug_init()` 默认配置 | 用于下载后串口调试输出 |
| 板载 LED | GPIO | LED | `P08.1` | 状态指示、任务闪灯测试 |

## 备注

- 当前飞控最小传感器组合为 `SCH16TK01 + PMW3901 + VL53L1X`。
- 电机输出协议当前为 `DShot600`，不再使用旧的 `PWM` 输出方案。
- `driver_motor_pwm.c/.h` 仍保留在仓库里做参考，但当前工程未编译它。
- 若后续接入磁力计、气压计、接收机或电调遥测，建议继续在本文件补充。
