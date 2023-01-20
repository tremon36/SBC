/* MAPPER PROGRAM FOR M.A.P.I.T.O, main executable*/

// Generic libraries
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// Include libraries of sensors
#include "sensors/motors_controller.h"
#include "sensors/ultrasonico_controller.h"
#include "sensors/tof_controller.h"

// Data structures
#include "data_structures/data_structures.h"

//networking
#include "networks/mqtt_publisher.h"
#include "networks/tcp_client.h"


// Global variables
#define LED_STATE_RED 4
#define LED_STATE_GREEN 2
#define LED_STATE_BLUE 15

const int go_forward = 1;
const int stop = 5;
int desired_rotation = 0;
int rotate = 4;
int count = 0;
float distances_array[19];

void init_led_state();
void set_led_state(int r, int g, int b);


void app_main(void){

    QueueHandle_t motor_command_queue;
    SemaphoreHandle_t robot_state_semaphore;

    robot_state_t robot_state;
    robot_state.position = malloc(2*sizeof(int));
    robot_state.position[0] = 0;
    robot_state.position[1] = 0;
    robot_state.rotation = 0;
    robot_state.pause = 1;

    set_led_state(1,0,0);

    /*Networking initialization*/
    
    init_task_mqtt();
    init_tcp_client_task();

    set_led_state(0,0,1);

    /*Motor initialization*/

    init_GPIO();
    init_Queues(&motor_command_queue);
    init_Semaphore(&robot_state_semaphore);
    init_task(&robot_state);

    /*Ultrasonic initialization*/

    init_task_ultrasonico(&robot_state);

    /*Servo and ToF initialization*/
    
    init_task_tof_servo(distances_array,&robot_state);

    xQueueOverwrite(motor_command_queue,&go_forward);

    for(;;){
        ESP_LOGI("MAIN: ","position = [%f,%f], rotation = [%d], falling_risk = [%d]", robot_state.position[0],robot_state.position[1],robot_state.rotation,robot_state.falling_risk);
        xSemaphoreTake(robot_state_semaphore,(TickType_t)300);
        mqtt_set_data_to_send(robot_state.falling_risk);
        if(robot_state.pause == 1){
         xQueueOverwrite(motor_command_queue,&stop);
         set_led_state(0,0,1);
        }
        else {
        set_led_state(0,1,0);
        if(desired_rotation > robot_state.rotation){
            xQueueOverwrite(motor_command_queue,&rotate);
        } else xQueueOverwrite(motor_command_queue,&go_forward);
        }
        xSemaphoreGive(robot_state_semaphore);
        if(count % 40000 == 0){
            desired_rotation = (desired_rotation + 90) % 360;
        }
        count += 100;
        
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

void init_led_state(){
    gpio_set_direction(LED_STATE_RED,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_STATE_GREEN,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_STATE_BLUE,GPIO_MODE_OUTPUT);
}
void set_led_state(int r, int g, int b){
    gpio_set_level(LED_STATE_RED,r);
    gpio_set_level(LED_STATE_GREEN,g);
    gpio_set_level(LED_STATE_BLUE,b);
}

