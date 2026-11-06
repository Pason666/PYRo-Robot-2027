#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_bsp_uart.h"
#include "pyro_uart_comm.h"
#include "pyro_uart_message.h"
#include "pyro_crc.h"
#include "pyro_ins.h"
#include <cmath>

using namespace pyro;

// =========================================================
// Booster 任务句柄和事件定义（用于自瞄开火）
// =========================================================
extern "C" TaskHandle_t* get_booster_task_handle();
constexpr uint32_t EVENT_BIT_FIRE = (1 << 1);

// =========================================================
// 阶跃测试配置（调试用）
// =========================================================
constexpr bool ENABLE_STEP_TEST = false;         // 是否启用阶跃测试（改为 false 启用真正自瞄）
constexpr uint32_t STEP_INTERVAL_MS = 2000;      // 跳变间隔（2秒）
constexpr float STEP_PITCH_TARGET_1_RAD = 0.2f;  // Pitch 目标位置 1
constexpr float STEP_PITCH_TARGET_2_RAD = -0.6f; // Pitch 目标位置 2
constexpr float STEP_YAW_TARGET_1_RAD = 0.0f;    // Yaw 目标位置 1
constexpr float STEP_YAW_TARGET_2_RAD = 0.5f;    // Yaw 目标位置 2（约 28.6°）
// =========================================================

// =========================================================
// 调试用全局变量
// =========================================================
float g_debug_pitch_target_rad = 0.0f;      // Pitch 目标角度（rad）
float g_debug_pitch_feedback_rad = 0.0f;    // Pitch 反馈角度（rad）
float g_debug_pitch_imu_rad = 0.0f;         // IMU Pitch 角度（rad）
float g_debug_yaw_target_rad = 0.0f;        // Yaw 目标角度（rad）
float g_debug_yaw_feedback_rad = 0.0f;      // Yaw 反馈角度（rad）
bool g_debug_has_calibrated = false;        // 是否已标定

// ========== 自瞄调试变量 ==========
float g_debug_aim_pitch_received = 0.0f;    // 接收到的视觉 Pitch
float g_debug_aim_yaw_received = 0.0f;      // 接收到的视觉 Yaw
uint8_t g_debug_aim_food = 0;               // 接收到的 food 值
uint8_t g_debug_aim_fire = 0;               // 接收到的 fire 值

// ========== IMU 姿态角（实时） ==========
float g_debug_imu_yaw_rad = 0.0f;           // IMU Yaw（rad）
float g_debug_imu_pitch_rad = 0.0f;         // IMU Pitch（rad）
float g_debug_imu_roll_rad = 0.0f;          // IMU Roll（rad）
// =========================================================

// =========================================================
// Autoaim 相关全局变量（框架已搭建，待 UART 通信库后启用）
// =========================================================
bool autoaim_enabled = false;               // 自瞄总使能标志
uint8_t auto_fire_flag = 0;                 // 自动开火标志
float aim_yaw_rad = 0.0f;                   // 调试：自瞄 Yaw 角度
float aim_pitch_rad = 0.0f;                 // 调试：自瞄 Pitch 角度
float bullet_speed = 16.3f;                 // 当前弹速（m/s）
uint8_t enemy_color = 0;                    // 敌方颜色（0=蓝 1=红）

// UART 通信相关
static pyro::uart_comm_t *uart_comm_ptr = nullptr;
__attribute__((section(".dma_heap"))) pyro::mcu2aim_msg_t mcu2aim_msg;
__attribute__((section(".dma_heap"))) pyro::aim2mcu_msg_t aim2mcu_msg;
// =========================================================

static TaskHandle_t gimbal_task_handle                = nullptr;
static pyro::screw_gimbal_t *screw_gimbal_ptr         = nullptr;
static pyro::screw_gimbal_cmd_t *screw_gimbal_cmd_ptr = nullptr;
static pyro::screw_gimbal_deps_t *screw_gimbal_deps   = nullptr;

static void gimbal_dr162cmd();
static void deps_init();

// ========== Autoaim 相关函数（框架已搭建，待 UART 支持后启用） ==========

/**
 * @brief 处理从视觉接收的自瞄数据
 * TODO: 待 UART 通信库添加后启用
 */
static void aim2mcu_process()
{
    // 调试：记录接收到的原始数据
    g_debug_aim_pitch_received = aim2mcu_msg.data.shoot_pitch;
    g_debug_aim_yaw_received = aim2mcu_msg.data.shoot_yaw;
    g_debug_aim_food = aim2mcu_msg.data.food;
    g_debug_aim_fire = aim2mcu_msg.data.fire;

    // food == 9 表示无目标
    if (aim2mcu_msg.data.food == 9)
    {
        screw_gimbal_cmd_ptr->is_aiming = false;
    }
    else
    {
        screw_gimbal_cmd_ptr->is_aiming = true;
    }

    auto_fire_flag = aim2mcu_msg.data.fire;

    // 角度合法性检查
    if (std::fabs(aim2mcu_msg.data.shoot_yaw) <= PI)
    {
        screw_gimbal_cmd_ptr->aim_imu_yaw_rad = aim2mcu_msg.data.shoot_yaw;
        aim_yaw_rad = aim2mcu_msg.data.shoot_yaw;
    }

    if (std::fabs(aim2mcu_msg.data.shoot_pitch) <= PI / 2.0f)
    {
        screw_gimbal_cmd_ptr->aim_imu_pitch_rad = aim2mcu_msg.data.shoot_pitch;
        aim_pitch_rad = aim2mcu_msg.data.shoot_pitch;
    }
}

/**
 * @brief 发送 MCU 状态给视觉
 * TODO: 待 UART 通信库添加后启用
 */
static void mcu2aim_process()
{
    // 获取 IMU 姿态
    float yaw_deg, pitch_deg, roll_deg;
    pyro::ins_drv_t *ins = pyro::ins_drv_t::get_instance();
    ins->get_angles_b(&yaw_deg, &pitch_deg, &roll_deg);

    // 转换为弧度
    float yaw_rad = yaw_deg / 180.0f * PI;
    float pitch_rad = pitch_deg / 180.0f * PI;
    float roll_rad = roll_deg / 180.0f * PI;

    // 更新调试变量（方便监视）
    g_debug_imu_yaw_rad = yaw_rad;
    g_debug_imu_pitch_rad = pitch_rad;
    g_debug_imu_roll_rad = roll_rad;

    // 填充消息数据（匹配 InputData 结构）
    mcu2aim_msg.data.curr_yaw = yaw_rad;
    mcu2aim_msg.data.curr_pitch = pitch_rad;
    mcu2aim_msg.data.state = 0;                    // 机器人状态
    mcu2aim_msg.data.autoaim = autoaim_enabled;    // 自瞄使能
    mcu2aim_msg.data.fire = 0;                     // 开火标志（预留）
    mcu2aim_msg.data.enemy_color = enemy_color;    // 敌方颜色
    mcu2aim_msg.data.curr_speed = bullet_speed;    // 当前弹速
    mcu2aim_msg.data.shoot_delay = 0;              // 射击延迟

    // 添加 CRC16 校验
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(&mcu2aim_msg),
                                 sizeof(mcu2aim_msg) - 1);

    // 通过 UART 发送
    if (uart_comm_ptr != nullptr)
    {
        uart_comm_ptr->write(mcu2aim_msg);
    }
}

// ========================================

extern "C"
{
    void engineer_gimbal_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            // 从视觉接收自瞄数据
            if (uart_comm_ptr != nullptr)
            {
                uart_comm_ptr->read(aim2mcu_msg);
            }

            // 独立运行模式：直接检查遥控器
            if (dr16_drv_t::instance().check_online())
            {
                gimbal_dr162cmd();
            }
            else
            {
                screw_gimbal_cmd_ptr->mode =
                    pyro::cmd_base_t::mode_t::PASSIVE;
            }

            screw_gimbal_ptr->set_command(*screw_gimbal_cmd_ptr);

            // 发送 MCU 状态给视觉
            mcu2aim_process();

            // 更新调试全局变量
            g_debug_pitch_target_rad = screw_gimbal_ptr->get_pitch_target_rad();
            g_debug_pitch_feedback_rad = screw_gimbal_ptr->get_pitch_feedback_rad();
            g_debug_pitch_imu_rad = screw_gimbal_ptr->get_pitch_imu_rad();
            g_debug_has_calibrated = screw_gimbal_ptr->get_has_calibrated();

            vTaskDelay(1);
        }
    }

    void engineer_gimbal_init(void *argument)
    {
        screw_gimbal_cmd_ptr = new pyro::screw_gimbal_cmd_t();
        screw_gimbal_ptr     = pyro::screw_gimbal_t::instance();

        deps_init();
        screw_gimbal_ptr->configure(*screw_gimbal_deps);
        screw_gimbal_ptr->start();

        // ========== 初始化 UART 通信（用于视觉自瞄） ==========
        uart_comm_ptr = new pyro::uart_comm_t(7, 0x01);  // UART7

        // 初始化消息帧头
        mcu2aim_msg.header.sof = 0xA5;
        mcu2aim_msg.enter.enter = '\n';

        // 注册接收消息类型
        uint8_t header = 0xA5;
        uart_comm_ptr->register_msg_type<pyro::aim2mcu_msg_t>(&header, sizeof(pyro::aim2mcu_msg_t::header));
        // =====================================================

        xTaskCreate(engineer_gimbal_thread, "engineer_gimbal_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);

        vTaskDelete(nullptr);
    }
}

void gimbal_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    // 右拨杆逻辑：
    // 上档(UP) - 无力模式（PASSIVE）
    // 中档(MID) - 正常模式（ACTIVE/Normal）
    // 下档(DOWN) - 自瞄模式（ACTIVE/Autoaim，预留）

    switch (vrc.switches.right.current_pos)
    {
        case pyro::sw_pos_t::UP:
            // 无力模式
            screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
            screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
            screw_gimbal_cmd_ptr->autoaim_mode      = false;
            break;

        case pyro::sw_pos_t::MID:
            // 正常模式
            screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
            screw_gimbal_cmd_ptr->pitch_delta_angle = -vrc.axes.ry * 0.0025f;
            screw_gimbal_cmd_ptr->yaw_delta_angle   = -vrc.axes.rx * 0.0035f;
            screw_gimbal_cmd_ptr->autoaim_mode      = false;
            break;

        case pyro::sw_pos_t::DOWN:
            // 自瞄模式
            screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
            screw_gimbal_cmd_ptr->autoaim_mode      = true;
            autoaim_enabled = true;

            // 处理视觉数据
            aim2mcu_process();

            // ========== 自瞄开火逻辑 ==========
            // 检测视觉开火信号（上升沿触发）
            static uint8_t last_auto_fire_flag = 0;
            if (auto_fire_flag && !last_auto_fire_flag)
            {
                // 触发拨弹盘射击
                TaskHandle_t* booster_handle = get_booster_task_handle();
                if (booster_handle != nullptr && *booster_handle != nullptr)
                {
                    xTaskNotify(*booster_handle, EVENT_BIT_FIRE, eSetBits);
                }
            }
            last_auto_fire_flag = auto_fire_flag;
            // ===================================

            // 如果启用阶跃测试，使用测试目标；否则使用视觉数据
            if (ENABLE_STEP_TEST)
            {
                static uint32_t last_switch_tick = 0;
                static bool use_target_1 = true;

                uint32_t current_tick = xTaskGetTickCount();

                // 每隔 STEP_INTERVAL_MS 切换一次目标
                if (current_tick - last_switch_tick >= pdMS_TO_TICKS(STEP_INTERVAL_MS))
                {
                    use_target_1 = !use_target_1;
                    last_switch_tick = current_tick;
                }

                // 设置目标绝对角度（阶跃测试）
                screw_gimbal_cmd_ptr->target_pitch = use_target_1 ? STEP_PITCH_TARGET_1_RAD : STEP_PITCH_TARGET_2_RAD;
                screw_gimbal_cmd_ptr->target_yaw = use_target_1 ? STEP_YAW_TARGET_1_RAD : STEP_YAW_TARGET_2_RAD;
            }
            else
            {
                // 使用视觉提供的自瞄目标角度
                screw_gimbal_cmd_ptr->target_pitch = screw_gimbal_cmd_ptr->aim_imu_pitch_rad;
                screw_gimbal_cmd_ptr->target_yaw = screw_gimbal_cmd_ptr->aim_imu_yaw_rad;
            }
            break;

        default:
            screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            break;
    }
}

void deps_init()
{
    screw_gimbal_deps = new pyro::screw_gimbal_deps_t();

    // 1. 初始化电机（使用 new，与 chassis_app.cpp 一致）
    // Pitch motor - M3508 on CAN3
    screw_gimbal_deps->motor_deps.pitch =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can3);

    // Yaw motor - GM6020 on CAN1
    screw_gimbal_deps->motor_deps.yaw =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1);

    // 2. 初始化 PID 控制器
    // Pitch 轴串级 PID
    screw_gimbal_deps->pid_deps.pitch_pos =
        new pid_t(11.5f, 0.108f, 0.01f, 0.5f, 10.0f, 40, 1, 10, 1, 4);
    screw_gimbal_deps->pid_deps.pitch_spd =
        new pid_t(11000.0f, 51.0f, 7.0f, 500.0f, 11000.0f, 20, 1, 10, 1, 4);

    // Pitch 自瞄专用 PID
    screw_gimbal_deps->pid_deps.pitch_auto_pos =
        new pid_t(12.5f, 0.027f, 0.01f, 0.1f, 10.0f, 15, 1, 5, 1, 4);
    screw_gimbal_deps->pid_deps.pitch_auto_spd =
        new pid_t(11500.0f, 25.0f, 7.0f, 300.0f, 10000.0f, 10, 1, 5, 1, 4);

    // Yaw 轴串级 PID
    screw_gimbal_deps->pid_deps.yaw_pos =
        new pid_t(8.2f, 0.01f, 0.22f, 0.8f, 10.0f, 100, 1, 50, 1, 4);
    screw_gimbal_deps->pid_deps.yaw_spd =
        new pid_t(5.0f, 0.0003f, 0.0001f, 0.2f, 3.0f, 100, 1, 50, 1, 4);

    // Yaw 吊射模式相对角 PID
    screw_gimbal_deps->pid_deps.yaw_relative_pos =
        new pid_t(5.0f, 0.0f, 0.0f, 0.3f, 3.0f, 60, 2, 20, 3, 4);
    screw_gimbal_deps->pid_deps.yaw_relative_spd =
        new pid_t(12.0f, 0.0f, 0.0f, 0.3f, 3.0f, 60, 1, 10, 2, 4);

    // LESO 观测器（用于吊射模式扰动补偿）
    screw_gimbal_deps->pid_deps.yaw_pos_leso =
        new leso_t<3>(40, 12.221f, 20.0f);
    screw_gimbal_deps->pid_deps.yaw_spd_leso =
        new leso_t<2>(50, 12.221f, 20.0f);
    screw_gimbal_deps->pid_deps.yaw_pos_imu_leso =
        new leso_t<3>(50, 12.221f, 20.0f);
    screw_gimbal_deps->pid_deps.yaw_spd_imu_leso =
        new leso_t<2>(60, 12.221f, 20.0f);
}
