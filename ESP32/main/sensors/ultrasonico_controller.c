#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_structures.h"
#include "time_reference.h"

#define PIN_ULTRASONICO_ECHO 34
#define PIN_ULTRASONICO_TRIGGER 23
#define MIN_DANGER_DISTANCE 9

#define STACK_SIZE 2048
#define TASK_ULTRASONIC_PRIORITY 9

uint8_t state = 0;
uint64_t c_value = 0;
robot_state_t* robot_state;

double calculate_distance(uint64_t echo_time){
    return (double)echo_time/58.0f;
}

void ISR_handler_ultrasonic(void* args) {
    if(state == 0){
        c_value = time_reference_get_current_time_us();
        state = 1;
    } else {
        c_value = time_reference_get_current_time_us() - c_value;
        state = 0;
        double distance = calculate_distance(c_value);
        if(distance > MIN_DANGER_DISTANCE) robot_state->falling_risk = 1;
        else robot_state->falling_risk = 0;
    }
}

void init_interrupts(void){
    gpio_config_t init_conf;
    init_conf.pin_bit_mask = 1ULL << PIN_ULTRASONICO_ECHO;
    init_conf.pull_up_en = 0;
    init_conf.pull_down_en = 0;
    init_conf.intr_type = GPIO_INTR_ANYEDGE;
    init_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&init_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ULTRASONICO_ECHO,ISR_handler_ultrasonic,NULL);
}

void send_trigger(void){
    uint64_t instant_to_stop_trigger = time_reference_get_current_time_us() + 10;
    gpio_set_level(PIN_ULTRASONICO_TRIGGER,1);
    while(time_reference_get_current_time_us() < instant_to_stop_trigger);
    gpio_set_level(PIN_ULTRASONICO_TRIGGER,0);
    state = 0;
}

void main_task(void* args){
    time_reference_init();
    init_interrupts();
    gpio_set_direction(PIN_ULTRASONICO_TRIGGER,GPIO_MODE_OUTPUT);

    for(;;){
        send_trigger();
        ESP_LOGI("ultrasonico","trigger sent!");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void init_task_ultrasonico(robot_state_t* estado_robot){
    robot_state = estado_robot;
    xTaskCreate(main_task,                   // Pointer to the task entry function.
                "ultrasonic",                // A descriptive name for the task.
                STACK_SIZE,                  // the size of the task stack specified as the number of bytes.
                NULL,                        // No parameters
                TASK_ULTRASONIC_PRIORITY,    // The priority at which the task should run.
                NULL);                       // No delete

}
