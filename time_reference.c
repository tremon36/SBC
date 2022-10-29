#include "driver/timer.h"

void time_reference_init(){
    
     timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = false,
        .auto_reload = true,
        .clk_src = TIMER_SRC_CLK_APB
    }; // default clock source is APB

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0,TIMER_0);
}

uint64_t time_reference_get_current_time_us(void){
    uint64_t current_time;
    timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&current_time);
    return current_time;
}