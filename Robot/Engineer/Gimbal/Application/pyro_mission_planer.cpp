#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void engineer_gimbal_init(void *argument);
    extern void engineer_booster_init(void *argument);

    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

#if BOARD == GIMBAL_BOARD
        xTaskCreate(engineer_gimbal_init, "engineer_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(engineer_booster_init, "engineer_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        vTaskDelay(10);
#endif

        vTaskDelete(nullptr);
    }
}
