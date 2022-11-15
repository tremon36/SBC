#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/adc.h"

#define SWITCH_PIN 14
#define LED_PIN  12
#define LOG_COLOR_RED     "31"


esp_err_t set_adc (void)
{   

  gpio_set_direction(33, GPIO_MODE_INPUT);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // es el gpio 33
  adc1_config_width(ADC_WIDTH_BIT_12);
  return ESP_OK;
}

int read_LDR(void)
{
    return adc1_get_raw(ADC1_CHANNEL_5);

}