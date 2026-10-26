#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"

using namespace pyro;

static TaskHandle_t gimbal_task_handle                = nullptr;
static pyro::screw_gimbal_t *screw_gimbal_ptr         = nullptr;
static pyro::screw_gimbal_cmd_t *screw_gimbal_cmd_ptr = nullptr;
static pyro::screw_gimbal_deps_t *screw_gimbal_deps   = nullptr;

static void gimbal_dr162cmd();
static void deps_init();

extern "C"
{
    void engineer_gimbal_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

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

        xTaskCreate(engineer_gimbal_thread, "engineer_gimbal_thread", 128, nullptr,
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
            // 自瞄模式（预留）
            screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
            screw_gimbal_cmd_ptr->autoaim_mode      = true;
            // 自瞄模式下也允许手动微调
            screw_gimbal_cmd_ptr->pitch_delta_angle = -vrc.axes.ry * 0.0025f;
            screw_gimbal_cmd_ptr->yaw_delta_angle   = -vrc.axes.rx * 0.0035f;
            // TODO: 从视觉系统获取目标角度
            // screw_gimbal_cmd_ptr->target_pitch = ...;
            // screw_gimbal_cmd_ptr->target_yaw = ...;
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
