
#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "data_structures.h"
#define STACK_SIZE 1024 * 2
#define MOTOR_TASK_PRIORITY 9

#define Q1 GPIO_NUM_26 // yellow
#define Q2 GPIO_NUM_25 // orange 
#define Q3 GPIO_NUM_33 // brown 
#define Q4 GPIO_NUM_32 // black 

#define Q5 GPIO_NUM_13 // yellow
#define Q6 GPIO_NUM_12 // orange
#define Q7 GPIO_NUM_14 // brown
#define Q8 GPIO_NUM_27 // black

static const char *TAG = "TAG";
robot_state_t* _coordinates_and_angle;

/*  Tipes of instructions:
        stepperInstruction = 1 : The robot goes forwards.
        stepperInstruction = 2 : The robot goes backwards .
        stepperInstruction = 3 : The robot turns clockwise.
        stepperInstruction = 4 : The robot turns counterclockwise.
        stepperInstruction = 5 : The robot stops.
*/
QueueHandle_t instructionQueue = NULL;        // Queue for comunication between the master function and the Stepper State Maching
uint8_t stepperInstructionSend = 0;    // Last instructiond sended register.
uint8_t stepperInstructionRecived = 0; // Last instructiond recived register.

SemaphoreHandle_t xSemaphore_cooRobot = NULL; // binary semaphore for *cooRobot[2].

esp_err_t init_Queues(QueueHandle_t* get_motor_instruction_queue);
esp_err_t init_task(robot_state_t* coordinates_and_angle);
esp_err_t init_Semaphore(SemaphoreHandle_t* get_semaphore);
esp_err_t init_GPIO(void);

void vTask_StepperStateMachine(void *instruction);
void toggle_state(uint8_t *currentState, uint8_t nextState);
void calculate_robot_position(uint8_t stepperInstructionRecived);

esp_err_t init_Queues(QueueHandle_t* get_motor_instruction_queue)
{

     instructionQueue = xQueueCreate(1, sizeof(uint8_t));
     *get_motor_instruction_queue = instructionQueue;

    if (instructionQueue == 0)
        ESP_LOGE(TAG, "Error al crear la cola----------------------------------\n");
    else
    {
        ESP_LOGW(TAG, "Cola creada correctamente----------------------------------\n");
        stepperInstructionSend = 5;
        if (xQueueSend(instructionQueue, (void *)&stepperInstructionSend, (TickType_t)10) != pdPASS)
            ESP_LOGE(TAG, "Error al introducir datos en la cola\n");
        else
        {
            xQueuePeek(instructionQueue, &stepperInstructionRecived, 0);
            ESP_LOGI(TAG, "La cola tine la instución %d ahora mismo\n", stepperInstructionRecived);
        }
    }

    return ESP_OK;
}

esp_err_t init_task(robot_state_t* coordinates_and_angle)
{
    _coordinates_and_angle = coordinates_and_angle;

    TaskHandle_t xHandle = NULL;

    xTaskCreatePinnedToCore(vTask_StepperStateMachine,   // Pointer to the task entry function.
                            "vTask_StepperStateMachine", // A descriptive name for the task.
                            STACK_SIZE,                  // the size of the task stack specified as the number of bytes.
                            NULL,                        // No parameters
                            MOTOR_TASK_PRIORITY,         // The priority at which the task should run.
                            &xHandle,                    // Use the handle to delete the task.
                            1);

    configASSERT(xHandle);

    return ESP_OK;
}

esp_err_t init_Semaphore(SemaphoreHandle_t* get_semaphore)
{   
    xSemaphore_cooRobot = xSemaphoreCreateBinary();
    *get_semaphore = xSemaphore_cooRobot;

    if (xSemaphore_cooRobot != NULL)
        ESP_LOGW(TAG, "the semaphore xSemaphore_cooRobot was created successfully.\n");
    else
        ESP_LOGE(TAG, "ERROR creating the  xSemaphore_cooRobot.\n");
    

    xSemaphoreGive( xSemaphore_cooRobot );
    return ESP_OK;
}

esp_err_t init_GPIO(void)
{
    gpio_reset_pin(Q1);
    // esp_err_t gpio_set_direction(gpio_num_t Q1, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(Q2);
    // esp_err_t gpio_set_direction(gpio_num_t Q2, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(Q3);
    // esp_err_t gpio_set_direction(gpio_num_t Q3, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(Q4);
    // esp_err_t gpio_set_direction(gpio_num_t Q4, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q4, GPIO_MODE_OUTPUT);

    gpio_reset_pin(Q5);
    // esp_err_t gpio_set_direction(gpio_num_t Q5, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q5, GPIO_MODE_OUTPUT);
    gpio_reset_pin(Q6);
    // esp_err_t gpio_set_direction(gpio_num_t Q6, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q6, GPIO_MODE_OUTPUT);
    gpio_reset_pin(Q7);
    // esp_err_t gpio_set_direction(gpio_num_t Q7, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q7, GPIO_MODE_OUTPUT);
    gpio_reset_pin(Q8);
    // esp_err_t gpio_set_direction(gpio_num_t Q8, gpio_mode_t GPIO_PULLUP_ENABLE);
    gpio_set_direction(Q8, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

void vTask_StepperStateMachine(void *parameters)
{
    uint8_t state_left = 1, state_right = 1;                                                   // state of the state machine for each mottor.
    uint8_t motorCoils_state_Left[4] = {1, 0, 1, 0}, motorCoils_state_Right[4] = {1, 0, 1, 0}; // state of the 4 coper coils of each mottor (on or off).
    for (;;)
    {
        xQueuePeek(instructionQueue, &stepperInstructionRecived, 0);

        switch (stepperInstructionRecived)
        {
        case 1: // go forward.
            state_left++;
            state_right++;
            break;
        case 2: // go backwards.
            state_left++;
            state_right++;
            break;
        case 3: // turn clockwise.
            state_left++;
            state_right--;
            break;
        case 4: // turn counterclockwise.
            state_left--;
            state_right++;
            break;
        default: // stoped.
            break;
        }

        if (state_left <= 0)
            state_left = 4;
        if (state_left >= 5)
            state_left = 1;

        if (state_right <= 0)
            state_right = 4;
        if (state_right >= 5)
            state_right = 1;

        toggle_state(motorCoils_state_Left, state_left);
        toggle_state(motorCoils_state_Right, state_right);

        gpio_set_level(Q1, motorCoils_state_Left[0]);
        gpio_set_level(Q2, motorCoils_state_Left[1]);
        gpio_set_level(Q3, motorCoils_state_Left[2]);
        gpio_set_level(Q4, motorCoils_state_Left[3]);

        gpio_set_level(Q5, motorCoils_state_Right[0]);
        gpio_set_level(Q6, motorCoils_state_Right[1]);
        gpio_set_level(Q7, motorCoils_state_Right[2]);
        gpio_set_level(Q8, motorCoils_state_Right[3]);

        calculate_robot_position(stepperInstructionRecived);

        /*ESP_LOGI(TAG, "instruccion : %d\nEstado motor izquierdo | %d  | %d  | %d  | %d  |",
                 (int)stepperInstructionRecived,
                 (int)motorCoils_state_Left[0],
                 (int)motorCoils_state_Left[1],
                 (int)motorCoils_state_Left[2],
                 (int)motorCoils_state_Left[3]);*/
                

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void toggle_state(uint8_t *currentState, uint8_t nextState)
{

    /*   States machine :
State | Q1 | Q2 | Q3 | Q4 |
 1    | 1  | 0  | 1  | 0  |
 2    | 1  | 0  | 0  | 1  |
 3    | 0  | 1  | 0  | 1  |
 4    | 0  | 1  | 1  | 0  |
*/

    switch (nextState)
    {
    case 1:
        currentState[0] = 1;
        currentState[1] = 0;
        currentState[2] = 1;
        currentState[3] = 0;
        break;

    case 2:
        currentState[0] = 1;
        currentState[1] = 0;
        currentState[2] = 0;
        currentState[3] = 1;
        break;

    case 3:
        currentState[0] = 0;
        currentState[1] = 1;
        currentState[2] = 0;
        currentState[3] = 1;
        break;

    case 4:
        currentState[0] = 0;
        currentState[1] = 1;
        currentState[2] = 1;
        currentState[3] = 0;
        break;

    default:
        break;
    }
}

void calculate_robot_position(uint8_t stepperInstructionRecived)
{
    float distancePerStep = 0.5236; // this is the distance that the weel advances when the motor taques a step (cm).
    uint16_t aux_anggle = _coordinates_and_angle->rotation;

    if (stepperInstructionRecived == 2) // in reality the robot doesnt change the angle where is pointing but the heading vector is the oposite. Used to calculate positionval
    {
        aux_anggle =  (aux_anggle + 180)%360; 
    }

    if (xSemaphore_cooRobot != NULL)
    {
        if (xSemaphoreTake(xSemaphore_cooRobot, (TickType_t)100) == pdTRUE)
        {

            if (stepperInstructionRecived == 1 || stepperInstructionRecived == 2) // if the robot advances forward
            {
                _coordinates_and_angle->position[0] = _coordinates_and_angle->position[0] + distancePerStep * (float)cos(((double)aux_anggle * (M_PI / 180)));
                _coordinates_and_angle->position[1] = _coordinates_and_angle->position[1] + distancePerStep * (float)sin(((double)aux_anggle * (M_PI / 180)));       
            }
            if (stepperInstructionRecived == 3) // if the robot turns clockwise
            {
                _coordinates_and_angle->rotation -= 2;
                if (_coordinates_and_angle->rotation < 0)
                    _coordinates_and_angle->rotation += 360;
            }

            if (stepperInstructionRecived == 4) // if the robot turns counterclockwise
            {
                _coordinates_and_angle->rotation += 2;
                _coordinates_and_angle->rotation = _coordinates_and_angle->rotation % 360;
            }

            xSemaphoreGive( xSemaphore_cooRobot );
        }
        else 
            ESP_LOGE(TAG, "ERROR updating the coordiantes of the robot, semaphore blocked for too mutch time");
        
    }

    //ESP_LOGI(TAG, "coordenadas robot x: %f, y: %f\n",_coordinates_and_angle->position[0],  _coordinates_and_angle->position[1]);
}

