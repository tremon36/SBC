#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_structures.h"
#include "time_reference.h"
#include "driver/timer.h"
#include "tof.h"

#define STACK_SIZE 2048
#define TASK_TOF_PRIORITY 10

#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 50 Hz


#define PIN_PWM 5
#define TOF_SCL_PIN 19
#define TOF_SDA_PIN 18
#define TOF_IRQ_PIN 22

robot_state_t* robot_state;

float last_tof_measurement = 0;
int current_servo_angle = 0;
float* _distances_array;


void tof_interrupts_handler(void* args){
    last_tof_measurement = tof_get_last_measurement();
    _distances_array[(current_servo_angle/20)] = last_tof_measurement;
}

void servo_init(){

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_PWM,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

}

void set_servo_angle(int angle){
    int duty_cycle = (int)( ((((float)angle/180.0f) * 0.1f) + (0.0222f)) * 8191.0f);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void main_servo_task(void* args){
    int servo_going_up = 1;
    servo_init();
    tof_init(TOF_SCL_PIN,TOF_SDA_PIN,TOF_IRQ_PIN);
    tof_init_interrupts(tof_interrupts_handler,NULL);
    
    for(;;){
        
        if(robot_state->pause == 1) goto task_end;
        
        current_servo_angle = servo_going_up ? current_servo_angle+20 : current_servo_angle-20;
        if(current_servo_angle == 180) servo_going_up = 0;
        else if(current_servo_angle == 0) servo_going_up = 1;

        set_servo_angle(current_servo_angle);

        //servo speed is 0.1s/60º, wait 0.1/6 s

        vTaskDelay(pdMS_TO_TICKS(1000 * (0.1f/6.0f)));

        tof_request_measurement();

        // period of the task

        task_end: vTaskDelay(pdMS_TO_TICKS(100));

    }
    
}

void init_task_tof_servo(float* distances_array,robot_state_t* estado) { //index of the array is the angle. Should have 19 elements [0-18], where 0 is 0º and 18 is 180
    _distances_array = distances_array;
    robot_state = estado;
    gpio_set_direction(TOF_SCL_PIN,GPIO_MODE_OUTPUT);

    xTaskCreate(main_servo_task,                   // Pointer to the task entry function.
                "servo_and_tof",                   // A descriptive name for the task.
                STACK_SIZE,                        // the size of the task stack specified as the number of bytes.
                NULL,                              // No parameters
                TASK_TOF_PRIORITY,                 // The priority at which the task should run.
                NULL);                             // No delete

}

