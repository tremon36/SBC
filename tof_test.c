#include <stdio.h>
#include "tof.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int interrupt_counter = 0;
int test = 0;
void handler(void* args){
interrupt_counter++;
test = ((int*) args) [1];
}

void app_main(void)
{
    tof_init(27,32,25);
    int a[3] = {1,2,3};
    tof_init_interrupts(handler,(void*)a);
    tof_request_measurement();
    float lecture = 0;
    int prevcount= 0;
    //tof_start_calibration(2.0);
    for(;;){
    while(prevcount == interrupt_counter) vTaskDelay(10 / portTICK_PERIOD_MS);
    lecture = tof_get_last_measurement();
    ESP_LOGI("distance: ","%f %d",lecture,test);
    prevcount = interrupt_counter;
    tof_request_measurement();
    }
}
