#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SENSOR_ADDR                 0x57                       /*!< Slave address of the sensor */
#define EEPROM_ADDR                 0x50                       /*!< Slave address of the eeprom */

int _scl_pin;
int _sda_pin;
int _irq_pin;


static esp_err_t _tof_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t _eeprom_address_read(uint8_t addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, EEPROM_ADDR, &addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t _tof_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t _i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = _sda_pin,
        .scl_io_num = _scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// SS pin always to GND (use command register). Necessary to call this to init the library.

void tof_init(int scl_pin,int sda_pin,int irq_pin) {
    _scl_pin = scl_pin;
    _sda_pin = sda_pin;
    _irq_pin = irq_pin;
    _i2c_master_init();
      
    //INICIALIZAR VALORES DE LOS REGISTROS DE CONTROL COMO DICE DIGILENT

    _tof_register_write_byte(0x10,0x04);
    _tof_register_write_byte(0x11,0x6E);
    _tof_register_write_byte(0x13,0x71);
    _tof_register_write_byte(0x60,0x01);
    _tof_register_write_byte(0x18,0x22);
    _tof_register_write_byte(0x19,0x22);
    _tof_register_write_byte(0x90,0x0F);
    _tof_register_write_byte(0x91,0xFF);
    
}


void tof_init_interrupts(void(*isr_handler)(void*),void* args){
    gpio_config_t init_conf;
    init_conf.pin_bit_mask = 1ULL << _irq_pin;
    init_conf.pull_up_en = 1;
    init_conf.pull_down_en = 0;
    init_conf.intr_type = GPIO_INTR_NEGEDGE;
    init_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&init_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(_irq_pin,isr_handler,args);
}

void tof_request_read() {
    uint8_t trash;
    _tof_register_write_byte(0x13,0x7D); // mode single shot
    _tof_register_write_byte(0x60,0x01); // interrupts enabled
    _tof_register_read(0x69, &trash, 1); // clear interrupt flags
    _tof_register_write_byte(0xB0,0x49); // send soft start command
}

float tof_get_last_measurement() {
    uint8_t trash,high_value,low_value;
    uint32_t distance;
    _tof_register_read(0xD1,&high_value,1);                                             // read distance result
    _tof_register_read(0xD2,&low_value,1);                                              // read distance result
    distance = ((((float)high_value) * 256.0f + (float)low_value) / 65536.0f) * 33.31f; //calculate distance
    _tof_register_read(0x60,&trash,1);                                                  // clear interrupt register
    return distance;
}


// calibration functions

void CALIB_perform_magnitude_calibration();
void CALIB_perform_crosstalk_calibration();
void CALIB_perform_distance_calibration(double actual_dist);
double _3bytes_to_double(uint8_t exp, uint8_t msb, uint8_t lsb);
double _2bytes_to_double(uint8_t msb, uint8_t lsb);
void double_to_3bytes(double dNum, uint8_t* exp, uint8_t* msb, uint8_t* lsb);
void double_to_2bytes(double dNum, uint8_t* msb, uint8_t* lsb);

void CALIB_initiate_calibration_measurement()
{
    _tof_register_write_byte(0xB0,0x49);
}

uint8_t tof_start_calibration(double actual_distance)
{
    ESP_LOGI("Starting"," Calibration");
    vTaskDelay(2000/portTICK_PERIOD_MS);
    
    ESP_LOGI("Starting magnitude calibration... You have 5 sec to prepare the device","");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    CALIB_perform_magnitude_calibration();

    ESP_LOGI("Starting crosstalk calibration... You have 10 sec to prepare the device","");
    vTaskDelay(10000/portTICK_PERIOD_MS);
    CALIB_perform_crosstalk_calibration();

    ESP_LOGI("Starting distance calibration...  You have 10 sec to prepare the device","");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    CALIB_perform_distance_calibration(actual_distance);

    ESP_LOGI("Calibration done","");
    return 0;

}


/* ------------------------------------------------------------ */
/** CALIB_perform_magnitude_calibration()
**  Parameters:
**      none
**  Return Value:
**      none
**
**  Description:
**  Function for performing magnitude calibration. It implements the steps described in the Firmware Routines documentation(an1724.pdf).
**  It is called by tof_start_calibration function.
**/
void CALIB_perform_magnitude_calibration()
{
    /* WRITE REG */
    uint8_t reg0x13_data = 0x61;
    uint8_t reg0x60_data = 0x01;
    /* READ REG */
    uint8_t regs[3];
    uint8_t unused;

    _tof_register_write_byte(0x13, reg0x13_data);
    _tof_register_write_byte(0x60, reg0x60_data);
    _tof_register_read(0x69, &unused, 1);
    CALIB_initiate_calibration_measurement();
	//waits for IRQ
    while(gpio_get_level(25) == 1) vTaskDelay(10/portTICK_PERIOD_MS);
    _tof_register_read(0xF6, regs, 3);
    _tof_register_write_byte(0x2C, regs[0]);
    _tof_register_write_byte(0x2D, regs[1]);
    _tof_register_write_byte(0x2E, regs[2]);
    reg0x13_data= 0x7D;
    _tof_register_write_byte(0x13, reg0x13_data);
    reg0x60_data=0x00;
    _tof_register_write_byte(0x60, reg0x60_data);
}

/* ------------------------------------------------------------ */
/** CALIB_perform_crosstalk_calibration()
**  Parameters:
**      none
**  Return Value:
**      none
**
**  Description:
**  Function for performing crosstalk calibration.
**  It implements the steps described in the Firmware Routines documentation(an1724.pdf).
**  It is called by tof_start_calibration function.
**/
void CALIB_perform_crosstalk_calibration()
{
    int N= 100;
    /* WRITE REG */
    uint8_t reg0x13_data = 0x7D;
    uint8_t reg0x60_data = 0x01;
    /* READ REG */
    uint8_t regs[14];
    uint8_t unused;
    uint8_t i_exp_calib;
    uint8_t i_msb_calib;
    uint8_t i_lsb_calib;
    double i_sum=0;
    double i_avg;
    uint8_t q_exp_calib;
    uint8_t q_msb_calib;
    uint8_t q_lsb_calib;
    double q_sum=0;
    double q_avg;
    uint8_t gain_msb_calib;
    uint8_t gain_lsb_calib;
    double gain_sum=0;
    double gain_avg;


    _tof_register_write_byte(0x13, reg0x13_data);
    _tof_register_write_byte(0x60, reg0x60_data);

    for(int i=0; i < N;i++)
    {
        _tof_register_read(0x69, &unused, 1);
        CALIB_initiate_calibration_measurement();

		//waits for IRQ
        while(gpio_get_level(25) == 1) vTaskDelay(10/portTICK_PERIOD_MS);

        _tof_register_read(0xDA, regs, 14);
        double I = _3bytes_to_double(regs[0],regs[1],regs[2]);
        i_sum += I;
        double Q = _3bytes_to_double(regs[3],regs[4],regs[5]);
        q_sum += Q;
        double gain = _2bytes_to_double(regs[12],regs[13]);
        gain_sum += gain;

    }
    i_avg= i_sum / N;
    q_avg= q_sum / N;
    gain_avg= gain_sum / N;

    double_to_3bytes(i_avg, &i_exp_calib, &i_msb_calib, &i_lsb_calib);
    double_to_3bytes(q_avg, &q_exp_calib, &q_msb_calib, &q_lsb_calib);
    double_to_2bytes(gain_avg, &gain_msb_calib, &gain_lsb_calib);
    _tof_register_write_byte(0x24, i_exp_calib);
    _tof_register_write_byte(0x25, i_msb_calib);
    _tof_register_write_byte(0x26, i_lsb_calib);
    _tof_register_write_byte(0x27, q_exp_calib);
    _tof_register_write_byte(0x28, q_msb_calib);
    _tof_register_write_byte(0x29, q_lsb_calib);
    _tof_register_write_byte(0x2A, gain_msb_calib);
    _tof_register_write_byte(0x2B, gain_lsb_calib);

}


/* ------------------------------------------------------------ */
/** void CALIB_perform_distance_calibration(double actual_dist)
**  Parameters:
**      actual_distance - the actual measuring distance (in meters) corresponding to the calibration setup.
**  Return Value:
**      none
**
**  Description:
**  Function for performing distance calibration.
**  It implements the steps described in the Firmware Routines documentation(an1724.pdf).
**  It is called by tof_start_calibration function.
**/
void CALIB_perform_distance_calibration(double actual_dist)
{
    int N= 100;
    /* WRITE REG */
    uint8_t reg0x13_data = 0x7D;
    uint8_t reg0x60_data = 0x01;


    /* READ REG */
    uint8_t regs[2];
    uint8_t unused;
    double phase_sum=0;
    double phase_avg;
    double dist_calib;
    uint8_t dist_msb_calib;
    uint8_t dist_lsb_calib;


    _tof_register_write_byte(0x13, reg0x13_data);
    _tof_register_write_byte(0x60, reg0x60_data);
    for(int i=0; i < N;i++)
    {
        _tof_register_read(0x69, &unused, 1);
        CALIB_initiate_calibration_measurement();

		//waits for IRQ
        while(gpio_get_level(25) == 1) vTaskDelay(10/portTICK_PERIOD_MS);

        _tof_register_read(0xD8, regs, 2);
        double phase = _2bytes_to_double(regs[0],regs[1]);
        phase_sum += phase;

    }
    phase_avg= phase_sum / N;
    dist_calib = phase_avg - (actual_dist/33.31*65536);
    double_to_2bytes(dist_calib, &dist_msb_calib, &dist_lsb_calib);
    _tof_register_write_byte(0x2F, dist_msb_calib);
    _tof_register_write_byte(0x30, dist_lsb_calib);

}


double _3bytes_to_double(uint8_t exp, uint8_t msb, uint8_t lsb)
{
    double result = 0;
    int iMantissa = 0;
    // flag for negative numbers
    uint8_t negative = 0u;

    // negative number
    if (msb > 127)
        negative = 1u;

    iMantissa = msb << 8;
    iMantissa |= lsb;
    if (negative) {
        // convert from 2's complement
        iMantissa = ((iMantissa - 1) ^ 0xFFFF);
        // combine mantissa and exponent
        result = -iMantissa * pow(2, exp);
    } else {
        result = iMantissa * pow(2, exp);
    }

    return result;
}

double _2bytes_to_double(uint8_t msb, uint8_t lsb)
{
    return (double)(unsigned short)(((int)(msb) << 8) | (int)(lsb));
}

void double_to_3bytes(double dNum, uint8_t* exp, uint8_t* msb, uint8_t* lsb)
{
    double dNumLog = 0;
    double dMantissa = 0;
    int iMantissa = 0;
    uint8_t negative = 0u;
    uint8_t first_exp, new_exp = 0;
    uint8_t a;

    // handle negative numbers
    if (dNum < 0) {
        // set negative flag
        negative = 1u;
        // convert to positive
        dNum = fabs(dNum);
    }

    // log base 2 of input
    dNumLog = log2(dNum);
    // exponent of the double
    first_exp = (uint8_t) dNumLog;
    // log of mantissa
    dMantissa = (dNumLog - (double) first_exp);
    // convert mantissa to double
    dMantissa = pow(2, dMantissa);
    // start new exponent as the original exponent
    new_exp = first_exp;

    // it might seem like 15 shifts is the correct number but it's 14.
    // Doing 15 shifts into the sign bit making it a negative number.
    // convert mantissa to whole number
    for (a = 1; a <= first_exp && a < 15; ++a) {
        // double the mantissa
        dMantissa = dMantissa * 2;
        // decrement the exponent
        --new_exp;
    }

    if (negative) {
        // take 2's complement, convert to short
        iMantissa = (int) (-dMantissa);
    } else {
        // convert to short
        iMantissa = (int) (dMantissa);
    }

    *exp = new_exp;
    *msb = (uint8_t) ((iMantissa & 0xFF00) >> 8);
    *lsb = (uint8_t) (iMantissa & 0x00FF);
}

void double_to_2bytes(double dNum, uint8_t* msb, uint8_t* lsb)
{
    int iNum = (int) dNum;
    *msb = (uint8_t) ((iNum & 0x0000FF00) >> 8);
    *lsb = (uint8_t) (iNum & 0x000000FF);
}
