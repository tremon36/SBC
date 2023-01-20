#ifndef _motor_controller_h
#define _motor_controller_h

#include "data_structures.h"
/*  Types of instructions:

        stepperInstruction = 1 : The robot goes forwards.
        stepperInstruction = 2 : The robot goes backwards .
        stepperInstruction = 3 : The robot turns clockwise.
        stepperInstruction = 4 : The robot turns counterclockwise.
        stepperInstruction = 5 : The robot stops.
*/

esp_err_t init_Queues(QueueHandle_t* get_motor_instruction_queue);
esp_err_t init_task(robot_state_t* coordinates_and_angle);
esp_err_t init_Semaphore(SemaphoreHandle_t* get_semaphore);
esp_err_t init_GPIO(void);

#endif