#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t state = 0;
uint64_t c_value = 0;
uint32_t interruption_counter = 0;

void handler_test(void* args) {
    if(state == 0){
        c_value = time_reference_get_current_time_us();
        state = 1;
    } else {
        c_value = time_reference_get_current_time_us() - c_value;
        state = 0;
        interruption_counter++;
    }
    gpio_set_level(32,state);
}

void init_interrupts(void){
    gpio_config_t init_conf;
    init_conf.pin_bit_mask = 1ULL << 25;
    init_conf.pull_up_en = 0;
    init_conf.pull_down_en = 0;
    init_conf.intr_type = GPIO_INTR_ANYEDGE;
    init_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&init_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(25,handler_test,NULL);
}



void app_main(void)
{   uint32_t prev = 0;
    gpio_set_direction(32,GPIO_MODE_OUTPUT);
    init_interrupts();
    time_reference_init();
    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(prev != interruption_counter){
        ESP_LOGI("timer","value (us): %lu",c_value / 5);
        prev = interruption_counter;
        }
    }
}
