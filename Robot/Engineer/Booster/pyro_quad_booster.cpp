#include "pyro_quad_booster.h"
#include "pyro_algo_common.h"
#include "pyro_bsp_uart.h"
#include "pyro_dwt_drv.h"
#include <cmath>
#include "quad_config.h"
#include <algorithm>

namespace pyro
{

quad_booster_t::quad_booster_t() : module_base_t("quad_booster")
{
}

status_t quad_booster_t::_init()
{
    _ctx.motor              = _module_deps.motor_deps;
    _ctx.pid                = _module_deps.pid_deps;
    _ctx.pid.ball_speed_pid = new pid_t(0.6f, 0.0f, 0.005f, 0.0f, 2.0f);

    return PYRO_OK;
}

float quad_booster_t::_normalize_angle(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

bool quad_booster_t::_is_trigger_located(float trigger_rad)
{
    constexpr float TRIGGER_SLOT_RAD = PI / 3.0f;
    const float delta = _normalize_angle(trigger_rad - TRIGGER_OFFSET);
    const float nearest_slot_delta =
        delta - std::round(delta / TRIGGER_SLOT_RAD) * TRIGGER_SLOT_RAD;
    return std::fabs(nearest_slot_delta) < TRIGGER_LOCATED_THRESHOLD_RAD;
}

float quad_booster_t::_get_next_trigger_preset(float trigger_rad,
                                               float min_advance_rad)
{
    constexpr float TRIGGER_SLOT_RAD = PI / 3.0f;
    const float delta        = _normalize_angle(trigger_rad - TRIGGER_OFFSET);
    const bool feed_positive = TRIGGER_FEED_DIR > 0.0f;
    float preset_index  = feed_positive ? std::ceil(delta / TRIGGER_SLOT_RAD)
                                        : std::floor(delta / TRIGGER_SLOT_RAD);
    const float advance = feed_positive
                              ? preset_index * TRIGGER_SLOT_RAD - delta
                              : delta - preset_index * TRIGGER_SLOT_RAD;

    if (advance <= min_advance_rad)
    {
        preset_index += feed_positive ? 1.0f : -1.0f;
    }

    return _normalize_angle(TRIGGER_OFFSET + preset_index * TRIGGER_SLOT_RAD);
}

void quad_booster_t::_update_feedback()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_torque[i] =
            _ctx.motor.fric_wheels[i]->get_current_torque();
    }
    _ctx.data.current_fric_mps[0] =
        _ctx.motor.fric_wheels[0]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[1] =
        _ctx.motor.fric_wheels[1]->get_current_rotate() * FRIC1_RADIUS;
    _ctx.data.current_fric_mps[2] =
        _ctx.motor.fric_wheels[2]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[3] =
        _ctx.motor.fric_wheels[3]->get_current_rotate() * FRIC1_RADIUS;

    for (int i = 0; i < 4; i++)
    {
        _ctx.data.abs_current_fric_mps[i] = abs(_ctx.data.current_fric_mps[i]);
    }

    _ctx.motor.trigger_wheel->update_feedback();
    _ctx.data.current_trig_radps =
        _ctx.motor.trigger_wheel->get_current_rotate();
    _ctx.data.current_trig_torque =
        _ctx.motor.trigger_wheel->get_current_torque();
    _ctx.data.current_trig_rad =
        _ctx.motor.trigger_wheel->get_current_position();
    _ctx.data.trigger_located = _is_trigger_located(_ctx.data.current_trig_rad);

    // 板间通信已移除，默认设置为允许部署
    _ctx.data.deploy_mode = true;
}

void quad_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

#include <cstdint>

__attribute__((section(".dma_heap"))) char shoot_speed[10];

/**
 * @brief 轻量级浮点数转字符函数（保留5位小数）
 * @param value 要转换的浮点数
 * @param buffer 输出的字符数组
 * @param max_len 数组最大长度（防止越界）
 */
void float_to_char_5_decimals(float value, char *buffer, int max_len)
{
    int idx = 0;

    // 1. 处理符号
    if (value < 0)
    {
        if (idx < max_len - 1)
            buffer[idx++] = '-';
        value = -value;
    }

    // 2. 分离整数和小数部分
    int int_part  = (int)value;
    // 加 0.5f 用于实现最后一位的四舍五入
    int frac_part = (int)((value - (float)int_part) * 100000.0f + 0.5f);

    // 处理四舍五入导致的进位
    if (frac_part >= 100000)
    {
        int_part++;
        frac_part -= 100000;
    }

    // 3. 计算整数部分的位数
    int temp       = int_part;
    int num_digits = 0;
    do
    {
        num_digits++;
        temp /= 10;
    } while (temp > 0);

    // 4. 边界安全检查：符号位 + 整数位数 + 小数点(1) + 5位小数 + 结束符(1)
    if (idx + num_digits + 1 + 5 + 1 > max_len)
    {
        // 如果越界（例如弹速异常到了三位数），默认安全返回全0
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    // 5. 提取整数部分（逆序写入）
    for (int i = num_digits - 1; i >= 0; i--)
    {
        buffer[idx + i] = '0' + (int_part % 10);
        int_part /= 10;
    }
    idx += num_digits;

    // 6. 写入小数点
    buffer[idx++] = '.';

    // 7. 提取小数部分（固定提取5位）
    for (int i = 4; i >= 0; i--)
    {
        buffer[idx + i] = '0' + (frac_part % 10);
        frac_part /= 10;
    }
    idx += 5;

    // 8. 添加字符串结束符
    buffer[idx]     = '\n';
    buffer[idx + 1] = '\0';
}

void quad_booster_t::_speed_control()
{
    // 统一使用高速模式
    auto &shoot_data = _ctx.shoot_data;
    _ctx.data.target_shoot_speed = shoot_data.target_speed;

    // TODO: 如需实现弹速反馈控制，可以在这里添加
    // 暂时禁用弹速反馈相关代码
}

void quad_booster_t::_launch_delay_calculate()
{
    auto &shoot_data = _ctx.shoot_data;

    _ctx.data.fresh_timer++;

    if (shoot_data.fric1_mps - std::abs(_ctx.data.current_fric_mps[1]) > 0.8f &&
        shoot_data.fric1_mps - std::abs(_ctx.data.current_fric_mps[3]) > 0.8f &&
        std::abs(_ctx.data.current_fric_torque[1]) > 3.0f &&
        std::abs(_ctx.data.current_fric_torque[2]) > 3.0f &&
        _ctx.data.fresh_timer > 220)
    {
        _ctx.data.launch_delay_timer[2] = _ctx.data.launch_delay_timer[1];
        _ctx.data.launch_delay_timer[1] = _ctx.data.launch_delay_timer[0];
        _ctx.data.launch_delay_timer[0] =
            (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer > 200.0f)
                ? _ctx.data.avg_launch_delay
                : (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer +
                   20.0f);

        _ctx.data.avg_launch_delay = 0.7f * _ctx.data.launch_delay_timer[0] +
                                     0.2f * _ctx.data.launch_delay_timer[1] +
                                     0.1f * _ctx.data.launch_delay_timer[2];
        _ctx.data.fresh_timer = 0;
        _ctx.data.fire_count++;
    }
}

bool quad_booster_t::_use_deploy_data() const
{
    return _ctx.data.deploy_mode ||
           (_ctx.cmd != nullptr && _ctx.cmd->force_deploy);
}

void quad_booster_t::_reset_active_shoot_data()
{
    auto &shoot_data = _ctx.shoot_data;
    shoot_data.reset();
}

void quad_booster_t::_fric_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void quad_booster_t::_anti_jam_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.target_fric_mps[i] = 0.0f;
        _ctx.data.out_fric_torque[i] = 0.0f;
    }

    _ctx.data.out_fric_torque[1] = -FRIC1_ANTI_JAM_REVERSE_TORQUE;
    _ctx.data.out_fric_torque[3] = FRIC1_ANTI_JAM_REVERSE_TORQUE;
    _ctx.data.target_trig_rad    = _ctx.data.current_trig_rad;
    _ctx.data.target_trig_radps  = 0.0f;
    _ctx.data.out_trig_torque    = 0.0f;
}

void quad_booster_t::_trigger_position_control()
{
    float error = _ctx.data.target_trig_rad - _ctx.data.current_trig_rad;
    error       = _normalize_angle(error);

    _ctx.data.target_trig_radps =
        _ctx.pid.trigger_pos_pid->calculate(error, 0.0f);

    static float ff_torque                 = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 1.0f;
    constexpr float TRIG_FF_TORQUE         = 0.505f;

    const float feed_speed = _ctx.data.target_trig_radps * TRIGGER_FEED_DIR;
    if (feed_speed > TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = TRIGGER_FEED_DIR * TRIG_FF_TORQUE;
    }
    else if (feed_speed < 0.0f)
    {
        ff_torque = 0.0f;
    }

    _ctx.data.out_trig_torque =
        _ctx.pid.trigger_spd_pid->calculate(_ctx.data.target_trig_radps,
                                            _ctx.data.current_trig_radps) +
        ff_torque;

    _ctx.data.out_trig_torque =
        std::clamp(_ctx.data.out_trig_torque, -7.0f, 7.0f);
}

void quad_booster_t::_trigger_speed_control()
{
    float ff_torque                        = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 0.5f;
    constexpr float TRIG_FF_TORQUE         = 0.505f;

    if (_ctx.data.target_trig_radps * TRIGGER_FEED_DIR > TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = TRIGGER_FEED_DIR * TRIG_FF_TORQUE;
    }

    _ctx.data.out_trig_torque =
        _ctx.pid.trigger_spd_pid->calculate(_ctx.data.target_trig_radps,
                                            _ctx.data.current_trig_radps) +
        ff_torque;
}

void quad_booster_t::_send_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(
            _ctx.data.out_fric_torque[i] +
            0.08f * _ctx.data.current_fric_torque[i]);
    }
}

void quad_booster_t::_send_raw_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i]);
    }
}

void quad_booster_t::_send_trigger_command() const
{
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

} // namespace pyro
