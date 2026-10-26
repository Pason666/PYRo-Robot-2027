#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"
#include "quad_config.h"

namespace pyro
{
void quad_booster_t::fsm_active_t::state_ready_t::enter(owner *owner)
{
    // 进入 ready 状态时，强制同步内部计数器与外部命令，
    // 清除在非 ready 状态期间（如 interim, busy,
    // stall）累积的所有误触发开火指令。
    owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
    owner->_ctx.data.fric_err            = false;
    owner->_ctx.data.ready_state_flag    = true;
    _fric_unready_start_time             = 0.0f;
}

void quad_booster_t::fsm_active_t::state_ready_t::execute(owner *owner)
{
    constexpr float FRIC_SWITCH_BUFFER_MS = 50.0f;
    const float now_ms                    = dwt_drv_t::get_timeline_ms();

    // 检查命令计数器是否与内部追踪计数器不一致，不一致说明有新的开火请求
    if (owner->_ctx.cmd->fire_count != owner->_ctx.data.internal_fire_count)
    {
        // 立即同步计数器，防止重复发弹或连发
        owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;

        owner->_ctx.data.signal_timer        = dwt_drv_t::get_timeline_ms();

        // 板间通信已移除，默认允许射击（不检查热量）
        bool heat_ok = true;

        if (heat_ok)
        {
            owner->_ctx.data.target_trig_rad =
                quad_booster_t::_get_next_trigger_preset(
                    owner->_ctx.data.current_trig_rad,
                    TRIGGER_PRESET_MIN_ADVANCE_RAD);

            request_switch(&owner->_state_active._busy_state);
        }
    }

    // 循环判断摩擦轮转速，不符合要求则退回interim状态
    bool fric_unready = false;
    for (int i = 0; i < 4; i++)
    {
        if (abs(owner->_ctx.data.current_fric_mps[i] -
                owner->_ctx.data.target_fric_mps[i]) > 1.0f)
        {
            fric_unready = true;
            break;
        }
    }
    if (fric_unready)
    {
        if (_fric_unready_start_time == 0.0f)
        {
            _fric_unready_start_time = now_ms;
        }
        else if (now_ms - _fric_unready_start_time >= FRIC_SWITCH_BUFFER_MS)
        {
            request_switch(&owner->_state_active._interim_state);
        }
    }
    else
    {
        _fric_unready_start_time = 0.0f;
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_ready_t::exit(owner *owner)
{
    owner->_ctx.data.ready_state_flag = false;
}
} // namespace pyro
