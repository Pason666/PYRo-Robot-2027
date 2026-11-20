#include "pyro_screw_gimbal.h"
#include "screw_config.h"
#include "pyro_algo_common.h"
#include <algorithm>
#include <cmath>

namespace pyro
{

// 计算软限位阻尼系数（越靠近极限位置，增益越小）
static float calculate_damping_factor(float current_angle, float min_limit, float max_limit, float damping_zone)
{
    float dist_to_min = current_angle - min_limit;
    float dist_to_max = max_limit - current_angle;

    float damping_factor = 1.0f; // 默认无阻尼

    // 接近下限
    if (dist_to_min < damping_zone && dist_to_min > 0.0f)
    {
        damping_factor = std::min(damping_factor, dist_to_min / damping_zone);
    }

    // 接近上限
    if (dist_to_max < damping_zone && dist_to_max > 0.0f)
    {
        damping_factor = std::min(damping_factor, dist_to_max / damping_zone);
    }

    // 保证最小增益，避免完全失控
    return std::max(damping_factor, PITCH_MIN_GAIN_RATIO);
}

void screw_gimbal_t::fsm_active_t::normal_state_t::enter(owner *owner)
{
    // 切换到 Normal 模式时，对齐目标防止跳变
    owner->_ctx.data.target_yaw_rad = owner->_ctx.data.yaw_imu_rad;
    owner->_ctx.data.target_pitch_rad =
        owner->_ctx.data.current_pitch_motor_rad;

    // 清除切入切出时可能残留的 PID 积分，防止由于历史积分导致的猛烈弹跳
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();
    owner->_ctx.pid.pitch_pos->clear();
    owner->_ctx.pid.pitch_spd->clear();
}

void screw_gimbal_t::fsm_active_t::normal_state_t::execute(owner *owner)
{
    // ==========================================
    // 1. 计算当前位置的阻尼系数
    // ==========================================
    float damping_factor = calculate_damping_factor(
        owner->_ctx.data.current_pitch_motor_rad,
        PITCH_SOFT_MIN_RAD,
        PITCH_SOFT_MAX_RAD,
        PITCH_DAMPING_ZONE_RAD
    );

    // ==========================================
    // 2. 应用阻尼后的增量控制
    // ==========================================
    owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle * damping_factor;
    owner->_ctx.data.target_yaw_rad += owner->_ctx.cmd->yaw_delta_angle;

    // ==========================================
    // 3. Pitch 软限位（留有缓冲区，避免触碰硬限位）
    // ==========================================
    owner->_ctx.data.target_pitch_rad = std::clamp(
        owner->_ctx.data.target_pitch_rad,
        PITCH_SOFT_MIN_RAD,
        PITCH_SOFT_MAX_RAD);

    // ==========================================
    // 4. 在极限位置时清除 Pitch PID 积分（防止积分饱和）
    // ==========================================
    if (damping_factor < 0.5f) // 当阻尼系数小于 50% 时，认为接近极限
    {
        owner->_ctx.pid.pitch_pos->clear();
        owner->_ctx.pid.pitch_spd->clear();
    }

    // ==========================================
    // 5. Yaw 环化限幅（360° 无限位）
    // ==========================================
    owner->_ctx.data.target_yaw_rad =
        pyro::loop_fp32_constrain(owner->_ctx.data.target_yaw_rad, -PI, PI);

    // ==========================================
    // 6. 执行底层控制与发送指令
    // ==========================================
    owner->_gimbal_control();
    _send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::normal_state_t::exit(owner *owner)
{
    // 保存当前目标角度，供进入自瞄状态时使用
    owner->_ctx.data.last_target_pitch_before_autoaim = owner->_ctx.data.target_pitch_rad;
    owner->_ctx.data.last_target_yaw_before_autoaim = owner->_ctx.data.target_yaw_rad;
}
} // namespace pyro
