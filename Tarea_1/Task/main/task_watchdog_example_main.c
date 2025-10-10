
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include <projdefs.h>

#define TWDT_TIMEOUT_MS         3000
#define TASK_RESET_PERIOD_MS    2000
#define MAIN_DELAY_MS           10000

static volatile bool run_loop;
static esp_task_wdt_user_handle_t func_a_twdt_user_hdl;
static esp_task_wdt_user_handle_t func_b_twdt_user_hdl;

static void func_a(void)
{
    esp_task_wdt_reset_user(func_a_twdt_user_hdl);
}

static void func_b(void)
{
    esp_task_wdt_reset_user(func_b_twdt_user_hdl);
}

void task_func(void *arg)
{
    
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    
    ESP_ERROR_CHECK(esp_task_wdt_add_user("func_a", &func_a_twdt_user_hdl));
    ESP_ERROR_CHECK(esp_task_wdt_add_user("func_b", &func_b_twdt_user_hdl));

    printf("Subscribed to TWDT\n");

    while (run_loop) {
        
        esp_task_wdt_reset();
        func_a();
        func_b();

        vTaskDelay(pdMS_TO_TICKS(TASK_RESET_PERIOD_MS));
    }

    
    ESP_ERROR_CHECK(esp_task_wdt_delete_user(func_a_twdt_user_hdl));
    ESP_ERROR_CHECK(esp_task_wdt_delete_user(func_b_twdt_user_hdl));
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));

    printf("Unsubscribed from TWDT\n");

   
    xTaskNotifyGive((TaskHandle_t)arg);
    vTaskDelete(NULL);
}

void app_main(void)
{
#if !CONFIG_ESP_TASK_WDT_INIT
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_MS,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = false,
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    printf("TWDT initialized\n");
#endif 

    
    run_loop = true;
    xTaskCreatePinnedToCore(task_func, "task", 2048, xTaskGetCurrentTaskHandle(), 10, NULL, 0);

    
    printf("Delay for %d seconds\n", MAIN_DELAY_MS/1000);
    vTaskDelay(pdMS_TO_TICKS(MAIN_DELAY_MS));

    
    run_loop = false;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if !CONFIG_ESP_TASK_WDT_INIT
    
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    printf("TWDT deinitialized\n");
#endif 
    printf("Example complete\n");
}
