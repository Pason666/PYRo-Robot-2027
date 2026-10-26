#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"

namespace pyro
{
extern "C"
{
    can_drv_t *can1_drv;
    can_drv_t *can2_drv;
    can_drv_t *can3_drv;
    ins_drv_t *ins_drv;

    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        // CAN 初始化（使用 bsp_can，与 chassis 一致）
        can1_drv = &bsp_can::get_can1();
        can1_drv->init();
        can1_drv->start();

        can2_drv = &bsp_can::get_can2();
        can2_drv->init();
        can2_drv->start();

        can3_drv = &bsp_can::get_can3();
        can3_drv->init();
        can3_drv->start();

        // INS 初始化
        ins_drv = ins_drv_t::get_instance();
        ins_config_t ins_config = {};
        ins_config.direct = ins_config_t::imu_direct_t::DIRECT_4;
        ins_drv->init(ins_config);

#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                        UART_PARITY_EVEN);
        DR16_UART.enable_rx_dma();
#endif

        vTaskDelete(nullptr);
    }
}
} // namespace pyro
