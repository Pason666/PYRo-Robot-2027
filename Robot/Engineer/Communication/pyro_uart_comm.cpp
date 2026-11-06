/**
 * @file pyro_uart_comm.cpp
 * @brief UART 通信应用层二次封装实现
 */

#include "pyro_uart_comm.h"
#include "pyro_bsp_uart.h"
#include "string.h"

namespace pyro
{

uart_comm_t::uart_comm_t(uint8_t uart_id, uint32_t owner_id, size_t msg_buf_size)
    : _drv(nullptr), _owner_id(owner_id), _msg_config_count(0)
{
    // 1. 创建 FreeRTOS 消息缓冲区
    _msg_buffer = xMessageBufferCreate(msg_buf_size);
    configASSERT(_msg_buffer != nullptr);

    // 2. 根据 uart_id 获取对应的底层驱动实例
    switch (uart_id)
    {
        case 1:
            _drv = &bsp_uart::get_uart1();
            break;
        case 5:
            _drv = &bsp_uart::get_uart5();
            break;
        case 7:
            _drv = &bsp_uart::get_uart7();
            break;
        case 10:
            _drv = &bsp_uart::get_uart10();
            break;
        default:
            _drv = nullptr;
            break;
    }

    if (_drv)
    {
        // 3. 将本类的 internal_rx_callback 通过 Lambda 表达式注册到底层 ISR
        _drv->add_rx_event_callback(
            [this](uint8_t *p, uint16_t size, BaseType_t &xHigherTaskWoken) -> bool {
                return this->internal_rx_callback(p, size, xHigherTaskWoken);
            },
            _owner_id);

        // 4. 开启底层 DMA 双缓冲接收
        _drv->enable_rx_dma();
    }
}

uart_comm_t::~uart_comm_t()
{
    // 从底层注销回调
    if (_drv)
    {
        _drv->remove_rx_event_callback(_owner_id);
    }

    // 销毁消息缓冲区
    if (_msg_buffer != nullptr)
    {
        vMessageBufferDelete(_msg_buffer);
        _msg_buffer = nullptr;
    }
}

// 注册重载 1：无帧头校验，仅校验长度
void uart_comm_t::register_msg_type(uint16_t length)
{
    if (_msg_config_count >= MAX_MSG_TYPES) return; // 防止数组越界

    msg_config_t cfg;
    cfg.length = length;
    cfg.header_len = 0; // 显式置 0，代表不校验帧头

    _msg_configs[_msg_config_count++] = cfg;
}

// 注册重载 2：有帧头校验
void uart_comm_t::register_msg_type(uint16_t length, const uint8_t* header, uint8_t header_len)
{
    if (_msg_config_count >= MAX_MSG_TYPES) return; // 防止数组越界

    msg_config_t cfg;
    cfg.length = length;
    cfg.header_len = (header_len > 8) ? 8 : header_len; // 限制最大帧头长度防溢出

    if (header != nullptr && cfg.header_len > 0)
    {
        memcpy(cfg.header, header, cfg.header_len);
    }
    else
    {
        cfg.header_len = 0; // 安全降级为无帧头
    }

    _msg_configs[_msg_config_count++] = cfg;
}

// 【核心逻辑】ISR 内部滑动窗口防粘包/错位解析算法
bool uart_comm_t::internal_rx_callback(uint8_t *p, uint16_t size, BaseType_t &xHigherPriorityTaskWoken)
{
    // 若未注册任何消息白名单，或传入数据为空，直接返回 false (丢弃数据并要求底层勿翻转)
    if (_msg_config_count == 0 || size == 0)
    {
        return false;
    }

    uint16_t idx = 0;
    bool found_valid_msg = false;

    // 滑动窗口机制：在接收到的字节流中搜索所有合法的数据包
    while (idx < size)
    {
        bool packet_matched = false;

        // 遍历所有已注册的消息规则
        for (uint8_t i = 0; i < _msg_config_count; i++)
        {
            const msg_config_t& cfg = _msg_configs[i];
            // 1. 长度校验：窗口剩余数据量必须大于等于该类型包长
            if ((idx + cfg.length) <= size)
            {
                bool header_ok = true;

                // 2. 帧头校验（如果注册时指定了帧头）
                if (cfg.header_len > 0)
                {
                    for (uint8_t i = 0; i < cfg.header_len; ++i)
                    {
                        if (p[idx + i] != cfg.header[i])
                        {
                            header_ok = false;
                            break;
                        }
                    }
                }

                // 3. 如果长度和帧头都匹配，写入消息缓冲区
                if (header_ok)
                {
                    size_t sent_size = xMessageBufferSendFromISR(_msg_buffer, &p[idx], cfg.length, &xHigherPriorityTaskWoken);

                    if (sent_size == cfg.length)
                    {
                        // 写入成功，移动滑动窗口
                        idx += cfg.length;
                        packet_matched = true;
                        found_valid_msg = true;
                        break; // 跳出 for 循环，继续下一轮 while
                    }
                    else
                    {
                        // 消息缓冲区满了，丢弃该包，移动窗口继续
                        idx += cfg.length;
                        packet_matched = true;
                        break;
                    }
                }
            }
        }

        // 如果没有任何规则匹配当前位置，则跳过一个字节继续扫描
        if (!packet_matched)
        {
            ++idx;
        }
    }

    // 返回 true 表示至少解析出一个合法包，底层可以继续翻转缓冲区
    return found_valid_msg;
}

} // namespace pyro
