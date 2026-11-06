#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_quad_booster.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_bsp_can.h"

namespace pyro
{

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE         = (1 << 0);
constexpr uint32_t EVENT_BIT_FIRE                = (1 << 1);
constexpr uint32_t EVENT_BIT_TRIGGER_RESET       = (1 << 5);
constexpr uint32_t EVENT_BIT_ANTI_JAM_START      = (1 << 7);
constexpr uint32_t EVENT_BIT_ANTI_JAM_STOP       = (1 << 8);

static TaskHandle_t booster_task_handle           = nullptr;

// ========== 导出给其他模块使用 ==========
extern "C"
{
    TaskHandle_t* get_booster_task_handle()
    {
        return &booster_task_handle;
    }
}
static pyro::quad_booster_t *quad_booster_ptr     = nullptr;
static pyro::quad_booster_cmd_t *quad_booster_cmd_ptr = nullptr;
static pyro::quad_deps_t *quad_deps_ptr           = nullptr;

static void deps_init();

extern "C"
{
    void booster_dr162cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        // 右上档：完全无力模式
        if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
            quad_booster_cmd_ptr->anti_jam = false;
            return;
        }

        quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        // 左上档：强制关闭摩擦轮
        if (pyro::sw_pos_t::UP == vrc.switches.left.current_pos)
        {
            quad_booster_cmd_ptr->fric_on = false;
            quad_booster_cmd_ptr->anti_jam = false;
            return;
        }

        // 左拨杆上档切中档：摩擦轮开关
        if (notify_val & EVENT_BIT_FRIC_TOGGLE)
        {
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
            quad_booster_cmd_ptr->anti_jam = false;
        }

        // 左拨杆下档切中档：单发
        if (notify_val & EVENT_BIT_FIRE)
        {
            quad_booster_cmd_ptr->fire_count++;
        }
    }

    void engineer_booster_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            if (dr16_drv_t::instance().check_online())
            {
                booster_dr162cmd(notify_val);
            }
            else
            {
                quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
                quad_booster_cmd_ptr->anti_jam = false;
            }

            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void engineer_booster_init(void *argument)
    {
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();

        deps_init();
        quad_booster_ptr->configure(*quad_deps_ptr);

        xTaskCreate(engineer_booster_thread, "booster_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);

        auto &vrc = pyro::rc_drv_t::read();

        // --- DR16 拨杆绑定 ---
        // 左拨杆：上档切中档 = 摩擦轮开关
        pyro::sw_broker::subscribe(&vrc.switches.left,
                                   pyro::sw_event_t::UP_TO_MID,
                                   booster_task_handle, EVENT_BIT_FRIC_TOGGLE);

        // 左拨杆：下档切中档 = 单发
        pyro::sw_broker::subscribe(&vrc.switches.left,
                                   pyro::sw_event_t::DOWN_TO_MID,
                                   booster_task_handle, EVENT_BIT_FIRE);

        quad_booster_ptr->start();
        vTaskDelete(nullptr);
    }
}

void deps_init()
{
    quad_deps_ptr = new pyro::quad_deps_t();

    // 四摩擦轮电机（M3508，CAN2）
    quad_deps_ptr->motor_deps.fric_wheels[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1, bsp_can::can2);
    quad_deps_ptr->motor_deps.fric_wheels[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2, bsp_can::can2);
    quad_deps_ptr->motor_deps.fric_wheels[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3, bsp_can::can2);
    quad_deps_ptr->motor_deps.fric_wheels[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4, bsp_can::can2);

    // 拨弹轮电机（DM4310，CAN1）
    quad_deps_ptr->motor_deps.trigger_wheel =
        new pyro::dm_motor_drv_t(0x51, 0x61, bsp_can::can1);

    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_torque_range(-7.0f, 7.0f);

    // 摩擦轮 PID 参数
    quad_deps_ptr->pid_deps.fric_pid[0] =
        new pid_t(6.4f, 0.02f, 0.02f, 2.5f, 20.0f, pid_t::INTEGRAL_LIMIT);
    quad_deps_ptr->pid_deps.fric_pid[1] =
        new pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20.0f, pid_t::INTEGRAL_LIMIT);
    quad_deps_ptr->pid_deps.fric_pid[2] =
        new pid_t(6.4f, 0.02f, 0.02f, 2.5f, 20.0f, pid_t::INTEGRAL_LIMIT);
    quad_deps_ptr->pid_deps.fric_pid[3] =
        new pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20.0f, pid_t::INTEGRAL_LIMIT);

    // 拨弹轮 PID 参数
    quad_deps_ptr->pid_deps.trigger_pos_pid =
        new pid_t(12.5f, 1.0f, 0.1f, 2.0f, 12.0f, pid_t::INTEGRAL_LIMIT);
    quad_deps_ptr->pid_deps.trigger_spd_pid =
        new pid_t(1.0f, 1.0f, 0.003f, 1.5f, 7.0f, pid_t::INTEGRAL_LIMIT);
}
} // namespace pyro
