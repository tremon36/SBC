#ifndef _TOF_H
#define _TOF_H

void tof_init(int scl_pin,int sda_pin,int irq_pin);
void tof_init_interrupts(void(*isr_handler)(void*),void* args);
void tof_request_read();
float tof_get_last_measurement();
uint8_t tof_start_calibration(double actual_distance);

#endif
