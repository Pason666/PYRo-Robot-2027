#ifndef __PYRO_UART_MESSAGE_H__
#define __PYRO_UART_MESSAGE_H__

#include <stdint.h>

namespace pyro
{

struct frame_header
{
    uint8_t sof;
} __attribute__((packed));

struct frame_tailer
{
    uint16_t crc16;
} __attribute__((packed));

struct frame_enter
{
    uint8_t enter;
} __attribute__((packed));

// ========== 视觉 → MCU 消息（OutputData）==========
typedef struct
{
    uint8_t fire;           // 是否开火
    float shoot_yaw;        // 目标 Yaw 轴角度（rad）
    float shoot_pitch;      // 目标 Pitch 轴角度（rad）
    float avg_speed;        // 平均弹速（m/s）
    uint8_t food;           // 目标 ID 
} __attribute__((packed)) aim2mcu_data_t;

typedef struct
{
    frame_header header;
    aim2mcu_data_t data;
    frame_tailer tailer;
} __attribute__((packed)) aim2mcu_msg_t;

// ========== MCU → 视觉 消息（InputData）==========
typedef struct
{
    float curr_yaw;         // 当前 Yaw 轴 IMU 角度（rad）
    float curr_pitch;       // 当前 Pitch 轴 IMU 角度（rad）
    uint8_t state;          // 机器人状态
    uint8_t autoaim;        // 自瞄使能标志
    uint8_t fire;           // 开火标志（预留）
    uint8_t enemy_color;    // 敌方颜色（0=蓝 1=红）
    float curr_speed;       // 当前弹速（m/s）
    uint16_t shoot_delay;   // 射击延迟（ms）
} __attribute__((packed)) mcu2aim_data_t;

typedef struct
{
    frame_header header{};
    mcu2aim_data_t data{};
    frame_tailer tailer{};
    frame_enter enter;
} __attribute__((packed)) mcu2aim_msg_t;

} // namespace pyro

#endif // __PYRO_UART_MESSAGE_H__
