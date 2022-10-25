#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint32_t state = 0;
uint32_t c_value = 0;
uint32_t interruption_counter = 0;

void init_timer(void){
    
     timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = false,
        .auto_reload = true,
        .clk_src = TIMER_SRC_CLK_APB
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
}

void handler_test(void* args) {
    if(state == 0){
        timer_set_counter_value(TIMER_GROUP_0,TIMER_0,0);
        c_value = 0;
        state = 1;
        interruption_counter++;
    } else {
        timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&c_value);
        state = 0;
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
    init_timer();
    timer_start(TIMER_GROUP_0,TIMER_0);
    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(prev != interruption_counter){
        ESP_LOGI("timer","value (us): %lu",c_value / 5);
        prev = interruption_counter;
        }
    }
}
