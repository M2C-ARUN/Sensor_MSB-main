
#include "lsm6dsl_reg.h"
#include "nrf_drv_twi.h"

#define MOTOR_PIN1 22
#define MOTOR_PIN2 27

typedef struct
{
    float acceleration_mg[3];
    float gyro_mdps[3];
    double pitch;
    double roll;
    double yaw;
    double mean;
    uint8_t GyData_ready;
    float GY_X;
    float GY_Y;
    float GY_Z;
    double magni_mean;

} accData_t;

extern accData_t accData;
extern bool conn_flag;
extern bool his_notify;
extern bool gyr_noti_flag;
extern uint8_t hook_conn;
extern char adv_data[10];
extern uint16_t delay_counter;

// X-axis
extern bool xless_flag;
extern bool xg_flag;

// Y-axis
extern bool yl_flag;
extern bool yg_flag;

// Z-axis
extern bool zl_flag;
extern bool zg_flag;

// Average magnitude
extern bool avg_lt_flag;
extern bool avg_gt_flag;

// rssi value
extern bool rssi_lt_flag;
extern bool rssi_gt_flag;

ret_code_t acc_init(void);
void getAccData(void);
void setThreshold(uint8_t *threshold);
void AccIdleProcess(void);
void AccActiveProcess(void);
void temperature_dataread(void);

void pwm_init(void);
void pwm_set_duty_cycle(uint16_t duty_cycle);
void pwm_stop(void);
void fade_led(void);
