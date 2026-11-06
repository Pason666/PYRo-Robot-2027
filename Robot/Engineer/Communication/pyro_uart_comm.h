/**
 * @file pyro_uart_comm.h
 * @brief UART 通信应用层二次封装
 */

#ifndef __PYRO_UART_COMM_H__
#define __PYRO_UART_COMM_H__

#include "pyro_uart_drv.h"
#include "pyro_mutex.h"
#include "message_buffer.h"
#include "string.h"

namespace pyro
{

struct msg_config_t
{
    uint16_t length;
    uint8_t  header[8];
    uint8_t  header_len;
};

class uart_comm_t
{
  public:
    uart_comm_t(uint8_t uart_id, uint32_t owner_id, size_t msg_buf_size = 1024);
    ~uart_comm_t();

    uart_comm_t(const uart_comm_t&) = delete;
    uart_comm_t& operator=(const uart_comm_t&) = delete;

    void register_msg_type(uint16_t length);
    void register_msg_type(uint16_t length, const uint8_t* header, uint8_t header_len);

    template <typename T>
    void register_msg_type() { register_msg_type(sizeof(T)); }

    template <typename T>
    void register_msg_type(const uint8_t* header, uint8_t header_len)
    { register_msg_type(sizeof(T), header, header_len); }

    template <typename T>
    bool write(const T& packet)
    {
        scoped_mutex_t lock(_tx_mutex);
        return (_drv->write(reinterpret_cast<const uint8_t*>(&packet), sizeof(T)) == PYRO_OK);
    }

    template <typename T>
    bool write_blocking(const T& packet, TickType_t timeout_ticks)
    {
        scoped_mutex_t lock(_tx_mutex, timeout_ticks);
        if (!lock.is_locked()) return false;
        return (_drv->write(reinterpret_cast<const uint8_t*>(&packet), sizeof(T), timeout_ticks) == PYRO_OK);
    }

    template <typename T>
    bool read(T& out_packet, TickType_t timeout_ticks = 0)
    {
        scoped_mutex_t lock(_rx_mutex);
        size_t rx_len = xMessageBufferReceive(_msg_buffer, &out_packet, sizeof(T), timeout_ticks);
        return (rx_len == sizeof(T));
    }

  private:
    static constexpr uint8_t MAX_MSG_TYPES = 8;

    uart_drv_t *_drv;
    uint32_t _owner_id;
    mutex_t _tx_mutex;
    mutex_t _rx_mutex;
    MessageBufferHandle_t _msg_buffer;
    msg_config_t _msg_configs[MAX_MSG_TYPES];
    uint8_t _msg_config_count;

    bool internal_rx_callback(uint8_t *p, uint16_t size, BaseType_t &xHigherPriorityTaskWoken);
};

} // namespace pyro

#endif // __PYRO_UART_COMM_H__
