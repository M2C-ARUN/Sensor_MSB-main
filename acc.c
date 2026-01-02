#include "main.h"
#include "acc.h"
#include "nrf_drv_gpiote.h"
#include "pwr.h"
#include "flash_func.h"
#include "ble_func.h"
#include "calendar.h"
#include "nrf_drv_pwm.h"
#include "ble_gap.h"
#include "float.h"

/* =========================================================
 * BLE / Scan Related
 * ========================================================= */
extern ble_gap_addr_t gap_addr;

extern bool scan_hook_stats;
extern bool is_UUID;
extern bool is_url;
extern uint8_t device_detect_cnt;

/* =========================================================
 * iButton / Hook Control
 * ========================================================= */
bool iB_start_flag = false;
bool iB_stop_flag  = false;

uint8_t iB_start_cnt = 0;
uint8_t iB_stop_cnt  = 0;

/* =========================================================
 * Test / Trigger / Condition Flags
 * ========================================================= */
int  test_beep_cnt     = 0;
bool triggered         = false;
bool ymax_ans_cond     = false;
int  ymax_ans_cond_cnt = 0;

/* =========================================================
 * PWM / Motor Control
 * ========================================================= */
#define PWM_PIN     27
#define PWM_PERIOD  20000

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t seq_values;

static nrf_pwm_sequence_t const seq =
{
    .values.p_individual = &seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

/* =========================================================
 * TWI / IMU Configuration
 * ========================================================= */
#define TWI_INSTANCE_ID  0
#define LSM6DSL_ADDR     LSM6DSL_ID

/* =========================================================
 * Time / Math Constants
 * ========================================================= */
#define MINUTE_IN_HOUR   60
#define HOUR_IN_DAY     24
#define RAD_TO_DEG      57.295
#define M_PI            3.1415
#define DT              0.01

/* =========================================================
 * Motor Direction & Orientation
 * ========================================================= */
uint32_t motor_clk_cnt     = 0;
uint32_t motor_antclk_cnt  = 0;

int direction_angle;
const char *cardinal_direction;

/* =========================================================
 * Timers
 * ========================================================= */
APP_TIMER_DEF(MOTOR_TIMER);
APP_TIMER_DEF(TIMER_ACC);
APP_TIMER_DEF(TIMER_HEALTH);
APP_TIMER_DEF(TIMER_1SEC);
APP_TIMER_DEF(TIMER_5MIT);
APP_TIMER_DEF(TIMER_EVENT_USER);

/* =========================================================
 * Step Counter Logic
 * ========================================================= */
static uint16_t step_count = 0;

#define step_threshold   1100
#define step_hysteresis  20
#define stable_baseline  515

static uint8_t step_flag = 0;

/* =========================================================
 * Device Configuration & Flags
 * ========================================================= */
extern configuration_t m_device_cfg;
extern bool bit1_on;
extern bool bit2_on;
extern bool bit3_on;

/* =========================================================
 * TWI Transfer State
 * ========================================================= */
static volatile bool m_xfer_done = false;

/* =========================================================
 * Buffers & Packets
 * ========================================================= */
uint8_t temp1[250];
uint8_t temp2[250];
uint8_t rotation_packet[512];

/* =========================================================
 * Health / Power Mode
 * ========================================================= */
bool health_flag = false;
bool in_lp_mode  = false;

volatile double mean_HIGH = 518;
volatile double mean_LOW  = 518;

double mean_HIGH_check = 518;
double mean_LOW_check  = 518;

int array_size = 3;

/* =========================================================
 * Alert Thresholds & Codes
 * ========================================================= */
double array_threshold[6] = {1200, 1500, 100, 1700, 2100, 2200};
char   array_AlertCh[6]   = {'T', 'A', 'F', 'X', 'Y', 'Z'};

/* =========================================================
 * Hook / Disconnect States
 * ========================================================= */
bool self_hook_disconnect = false;
bool bhd_flag             = false;

uint16_t hd_cnt  = 0;
uint16_t hc_cnt  = 0;
uint16_t rcc_cnt = 0;
uint16_t rac_cnt = 0;

/* =========================================================
 * System / Delay / Advertising
 * ========================================================= */
uint16_t delay_counter = 0;
bool     msd_set_flag  = false;

uint8_t hook_conn     = 0;
uint8_t batt_adv_data = 0;
char    adv_data[10]  = "";

/* =========================================================
 * IMU Raw & Angle Data
 * ========================================================= */
int16_t data_raw_angular_rate[3];

float AccXangle, AccYangle, AccZangle, TAccYangle;
float gyroXangle = 0, gyroYangle = 0, gyroZangle = 0;
float CFangleX   = 0, CFangleY   = 0, CFangleZ   = 0;

/* =========================================================
 * Rotation Tracking
 * ========================================================= */
int flag_tick = 1, flag_tick1 = 1;

uint16_t Last_RotationF = 0;
uint16_t Last_RotationR = 0;
uint16_t RotationF      = 0;
uint16_t RotationR      = 0;
uint16_t lastangle      = 0;

/* =========================================================
 * Buzzer Control
 * ========================================================= */
uint16_t buz_cnt              = 0;
uint16_t buz_en               = 0;
uint16_t buz_stop             = 0;
uint16_t buz_10s_cnt           = 0;
uint16_t buz_change_on_off_cnt = 10;

uint8_t  beep_counter = 0;
volatile bool buzzer_on = false;

/* =========================================================
 * Sensor / Fall / System State
 * ========================================================= */
uint16_t sensor_detect = 0;
uint16_t FTCT          = 0;
uint16_t recunt        = 2;

bool S_ok  = false;
bool FF_EN = false;

/* =========================================================
 * System Counters
 * ========================================================= */
uint16_t S_O_C = 1;
uint16_t S_C_C = 1;

/* =========================================================
 * Time Tracking
 * ========================================================= */
time_t t1, last_time, current_time;

uint8_t timespm[20];
uint8_t timestr[20];

/* =========================================================
 * Event / Countdown Control
 * ========================================================= */
volatile uint16_t cc_time      = 0;
uint16_t fix_time              = 8;
volatile uint8_t cc_time_enble = 0;
volatile uint8_t event_task    = 0;

/* =========================================================
 * Battery / Hook Status
 * ========================================================= */
uint8_t battery_alert_cnt = 0;

bool hook_cn_stat  = false;
bool hook_dis_stat = false;

/* =========================================================
 * Miscellaneous
 * ========================================================= */
uint8_t hm_cmdex_buzcnt = 0;
float   temperature_celsius;

/* =========================================================
 * TWI Driver Instance
 * ========================================================= */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
stmdev_ctx_t dev_ctx;

uint8_t whoamI[1], rst;

/* =========================================================
 * Data Structures
 * ========================================================= */
typedef union
{
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union
{
    int16_t i16bit;
    uint8_t u8bit[2];
} axis1bit16_t;

/* =========================================================
 * Accelerometer / Gyro Data
 * ========================================================= */
accData_t accData, gyrData;

#define ACC_X accData.acceleration_mg[0]
#define ACC_Y accData.acceleration_mg[1]
#define ACC_Z accData.acceleration_mg[2]

#define GYR_X gyrData.gyro_mdps[0]
#define GYR_Y gyrData.gyro_mdps[1]
#define GYR_Z gyrData.gyro_mdps[2]

static axis3bit16_t data_raw_gyro;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_ge;

/* =========================================================
 * Flags & Thresholds
 * ========================================================= */
uint8_t fa_flag, test_flag, ff_flag;
uint8_t bit_0, bit_1, bit_2, bit_3, bit_4, bit_5;

double fall_threshold = FALL_THRESHOLD;
double test_threshold = TEST_THRESHOLD;
double wake_threshold = WAKE_THRESHOLD;

/* =========================================================
 * Alert / Logging
 * ========================================================= */
static char AlertCh = ' ';
__ALIGN(4) char AlertStr[250] = "";

static char uid[20] = "";
static char ts[22]  = "";

/* =========================================================
 * Axis Min / Max Tracking
 * ========================================================= */
typedef struct
{
    float min;
    float max;
    float delta;
} AxisMinMax;

AxisMinMax accel_x   = {.min = FLT_MAX, .max = -FLT_MAX};
AxisMinMax accel_y   = {.min = FLT_MAX, .max = -FLT_MAX};
AxisMinMax accel_z   = {.min = FLT_MAX, .max = -FLT_MAX};
AxisMinMax accel_mag = {.min = FLT_MAX, .max = -FLT_MAX};

/* =========================================================
 * Delta / Direction Flags
 * ========================================================= */
bool x_min = false, x_max = false;
bool y_min = false, y_max = false;
bool z_min = false, z_max = false;
bool net_min = false, net_max = false;

bool dX_flag = false, dY_flag = false;
bool dZ_flag = false, dA_flag = false;

/* =========================================================
 * Pattern Detection Window
 * ========================================================= */
static char prev_timespm[32] = {0};

#define WINDOW_SIZE   6
#define MAX_LOG_STR   512

typedef struct
{
    char  log_str[MAX_LOG_STR];
    float y_delta;
    float x_delta;
} log_entry_t;

static log_entry_t log_window[WINDOW_SIZE];
static uint8_t sec_counter = 0;

bool pattern_flag = false;
char window_packet[512];

/**
 * @brief Initialize PWM for buzzer control.
 */
void pwm_init(void)
{
    ret_code_t err_code;

    // Configure PWM
    nrf_drv_pwm_config_t config =
        {
            .output_pins =
                {
                    PWM_PIN | NRF_DRV_PWM_PIN_INVERTED, // Channel 0
                    NRF_DRV_PWM_PIN_NOT_USED,           // Channel 1
                    NRF_DRV_PWM_PIN_NOT_USED,           // Channel 2
                    NRF_DRV_PWM_PIN_NOT_USED,           // Channel 3
                },
            .irq_priority = APP_IRQ_PRIORITY_LOWEST,
            .base_clock = NRF_PWM_CLK_1MHz,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value = PWM_PERIOD,
            .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
            .step_mode = NRF_PWM_STEP_AUTO};

    // Initialize PWM
    err_code = nrf_drv_pwm_init(&m_pwm0, &config, NULL);
    APP_ERROR_CHECK(err_code);

    // Set initial duty cycle to 0
    seq_values.channel_0 = 0;
    seq_values.channel_1 = 0;
    seq_values.channel_2 = 0;
    seq_values.channel_3 = 0;

    // Start PWM generation
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    printf("pwm init\n");
}
/**
 * @brief Set PWM duty cycle for buzzer.
 * @param duty_cycle Duty cycle percentage (0-100).
 */
void pwm_set_duty_cycle(uint16_t duty_cycle)
{
    if (duty_cycle > 100)
    {
        duty_cycle = 100;
    }

    uint16_t pwm_value = (duty_cycle * PWM_PERIOD) / 100;
    seq_values.channel_0 = pwm_value;

    // Apply changes properly
    nrf_pwm_sequence_t updated_seq =
        {
            .values.p_individual = &seq_values,
            .length = NRF_PWM_VALUES_LENGTH(seq_values),
            .repeats = 0,
            .end_delay = 0};

    nrf_drv_pwm_stop(&m_pwm0, false);
    nrf_drv_pwm_simple_playback(&m_pwm0, &updated_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}
/** 
 * @brief Stop PWM buzzer.
 */
void pwm_stop(void)
{
    nrf_drv_pwm_stop(&m_pwm0, true);
}
/** 
 * @brief  write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to buffer that store the data write
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    ret_code_t err_code;
    uint8_t add[2] = {reg, 0};
    add[1] = bufp[0];

    err_code = nrf_drv_twi_tx(&m_twi, LSM6DSL_ADDR, add, sizeof(add), false); // Function for sending data to a TWI slave.
    APP_ERROR_CHECK(err_code);
    return 0;
}

/** 
 * @brief  Read generic device register (platform dependent)
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&m_twi, LSM6DSL_ADDR, &reg, 1, false); // Function for sending data to a TWI slave.
    // APP_ERROR_CHECK(err_code);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = nrf_drv_twi_rx(&m_twi, LSM6DSL_ADDR, bufp, len); // Function for reading data from a TWI slave.
    return err_code;
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    switch (p_event->type)
    {
    case NRF_DRV_TWI_EVT_DONE:
        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
        {
            NRF_LOG_INFO("Data Ready");
        }
        m_xfer_done = true;
        break;
    default:
        break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lsm6sd3_config = {
        .scl = ARDUINO_SCL_PIN,
        .sda = ARDUINO_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false};

    err_code = nrf_drv_twi_init(&m_twi, &twi_lsm6sd3_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Accelerometer INT PIN handler
 */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    getAccData(); // in this function we'r getting the accelerometer data (x,y,z angle & roll/pitch/yard data)
}

/**
 * @brief Handler for timer events.
 */
void event_user_timer_handler(void *p_context)
{
    if (cc_time_enble == 0)
    {
        cc_time++;
    }

    if (cc_time > fix_time)
    {
        cc_time_enble = 1;
    }
}
/**
 * @brief Reset accelerometer min/max values.
 */
void reset_acc_minmax(void)
{
    accel_x.min = (int)fabs(ACC_X);
    accel_x.max = (int)fabs(ACC_X);
    accel_y.min = (int)fabs(ACC_Y);
    accel_y.max = (int)fabs(ACC_Y);
    accel_z.min = (int)fabs(ACC_Z);
    accel_z.max = (int)fabs(ACC_Z);
    accel_mag.min = (int)accData.magni_mean;
    accel_mag.max = (int)accData.magni_mean;
}
/**
 * @brief Check if timestamp has changed.
 * @return true if timestamp has changed, false otherwise.
 */
static bool check_if_second_change()
{
    // Check if timespm is different from prev_timespm
    if (strcmp(timespm, prev_timespm) != 0)
    {
        // Update prev_timespm with the new value
        strncpy(prev_timespm, timespm, sizeof(prev_timespm) - 1);
        prev_timespm[sizeof(prev_timespm) - 1] = '\0'; // Ensure null-termination
        return true;
    }
    else
    {
        return false;
    }
}
/**
 * @brief Reset min/max data if timestamp changes.
 * This function checks if the timestamp has changed
 * and resets the min/max data accordingly.
 */
void reset_the_min_max_data()
{
    // Check if timespm is different from prev_timespm

    // Timestamp changed → Reset min/max or print a reset message
    // printf("RESET: Timestamp changed to %s\n", timespm);
    // Optionally reset min/max values here (if needed)
    reset_acc_minmax(); // (Assuming you have this function)

    // // Update prev_timespm with the new value
    // strncpy(prev_timespm, timespm, sizeof(prev_timespm) - 1);
    // prev_timespm[sizeof(prev_timespm) - 1] = '\0'; // Ensure null-termination
}
/**
 * @brief Update axis min/max/delta values.
 * @param axis Pointer to AxisMinMax structure.
 * @param axis_val New axis value to consider.
 */
void update_axis_data(AxisMinMax *axis, float axis_val)
{
    int abs_val = (int)fabs(axis_val);

    if (abs_val < axis->min)
        axis->min = abs_val;

    if (abs_val > axis->max)
        axis->max = abs_val;

    axis->delta = axis->max - axis->min; // Calculate delta for axis
}
/**
 * @brief Update magnitude min/max/delta values.
 * @param mag Pointer to AxisMinMax structure.
 * @param magni_val New magnitude value to consider.
 */
void update_magnitude_data(AxisMinMax *mag, int magni_val)
{
    if (magni_val < mag->min)
        mag->min = magni_val;

    if (magni_val > mag->max)
        mag->max = magni_val;

    mag->delta = mag->max - mag->min; // Calculate delta for magnitude
}
/**
 * @brief Store data in a FIFO log window.
 * @param window_packet Data packet to store.
 * @param x_val X-axis value to store.
 */
void store_data(char *window_packet, float x_val)
{
    if (sec_counter < WINDOW_SIZE) // buffer not full yet
    {
        strncpy(log_window[sec_counter].log_str, window_packet, MAX_LOG_STR - 1);
        log_window[sec_counter].log_str[MAX_LOG_STR - 1] = '\0';
        // log_window[sec_counter].y_delta = y_val;
        log_window[sec_counter].x_delta = x_val;
        sec_counter++;
    }
    else // buffer full → shift FIFO
    {
        for (int i = 0; i < WINDOW_SIZE - 1; i++)
        {
            log_window[i] = log_window[i + 1];
        }
        strncpy(log_window[WINDOW_SIZE - 1].log_str, window_packet, MAX_LOG_STR - 1);
        log_window[WINDOW_SIZE - 1].log_str[MAX_LOG_STR - 1] = '\0';
        // log_window[WINDOW_SIZE - 1].y_delta = y_val;
        log_window[WINDOW_SIZE - 1].x_delta = x_val;
    }
}
/**
 * @brief Check if axis value crosses threshold.
 * @param value Axis value to check.
 */
bool check_axis(float value, int threshold)
{
    if (threshold > 0)
    {
        return value > threshold; // threshold > 0 → greater than
    }
    else if (threshold < 0)
    {
        return value < -threshold; // threshold < 0 → less than
    }
    return false; // 0 → disabled
}
/**
 * @brief Check if value crosses threshold.
 * @param value Value to check.
 * @param threshold Threshold to compare against.
 * @return true if threshold is non-zero and value >= threshold, false otherwise.
 */
bool check_threshold(float value, int threshold)
{
    return (threshold != 0 && value >= threshold);
}
/**
 * @brief Check for specific pattern in logged data.
 * This function checks if the logged data in the window
 * matches a predefined pattern based on high and low thresholds.
 * If the pattern is detected, it triggers an alert (e.g., buzzer).
 */
void check_pattern(void)
{  
    if (sec_counter < WINDOW_SIZE)
        return; // not enough samples yet

    bool ans = true;
    bool condition = false;
    for (int i = 0; i < 6; i++)
    {
        if (m_device_cfg.lockth[i] == 0)
        {
            continue;
        }
        if (m_device_cfg.lockth[i] > 0)
        {
            ans = ans && (log_window[i].x_delta > m_device_cfg.lockth[i]);
            condition = true;
        }

        if (m_device_cfg.lockth[i] < 0)
        {
            ans = ans && (log_window[i].x_delta < abs(m_device_cfg.lockth[i]));
            condition = true;
        }
    }

    if (ans && condition &&
        (check_threshold(accel_x.max, m_device_cfg.testX_beep_th) ||
         check_threshold(accel_y.max, m_device_cfg.testY_beep_th) ||
         check_threshold(accel_z.max, m_device_cfg.testZ_beep_th) ||
         check_threshold(accel_mag.max, m_device_cfg.testA_beep_th))) // before

    {
        // Send packet
        // live_alert_update(window_packet);
        pattern_flag = true;
        nrf_gpio_pin_set(BUZZER_PIN);
        printf("Pattern detected! Beep!\n");
        // printf("check: [%0.f,%0.f,%0.f,%0.f,%0.f,%0.f] th(low=%0.f, high=%0.f)\n",
        //        log_window[5].y_delta, log_window[4].y_delta, log_window[3].y_delta,
        //        log_window[2].y_delta, log_window[1].y_delta, log_window[0].y_delta,
        //        low_th, high_th);

        char check_packet[256]; // buffer for packet
        memset(check_packet, 0x00, sizeof(check_packet));
        // snprintf(check_packet, sizeof(check_packet),
        //          "{mac:%X%X%X,time:%s,y_deltas:[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f],xM:%d,yM:%d,zM:%d,M:%d}",
        //          gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
        //          log_window[5].y_delta,
        //          log_window[4].y_delta,
        //          log_window[3].y_delta,
        //          log_window[2].y_delta,
        //          log_window[1].y_delta,
        //          log_window[0].y_delta,
        //          (int)fabs(accel_x.max), (int)fabs(accel_y.max), (int)fabs(accel_z.max), (int)accel_mag.max);
        snprintf(check_packet, sizeof(check_packet),
                 "{mac:%X%X%X,time:%s,y_deltas:[xM:%d,yM:%d,zM:%d,M:%d]}",
                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
                 (int)fabs(accel_x.max), (int)fabs(accel_y.max), (int)fabs(accel_z.max), (int)accel_mag.max);
        live_alert_update(check_packet);
        uart_trasmit_str(check_packet);
    }

    // --- Pattern execution only when enabled ---
    if (pattern_flag)
    {
        beep_counter++;

        if (beep_counter >= 5)
        {
            buzzer_on = !buzzer_on; // toggle ON/OFF state
            beep_counter = 0;

            if (buzzer_on)
            {
                nrf_gpio_pin_set(BUZZER_PIN);
            }
            else
            {
                nrf_gpio_pin_clear(BUZZER_PIN);
            }
        }

        printf("beep_counter: %d, buzzer_on: %d\n", beep_counter, buzzer_on);
    }

    if (pattern_flag &&
        (accel_x.delta >= m_device_cfg.test_high_beep_th ||
         accel_y.delta >= m_device_cfg.test_high_beep_th ||
         accel_z.delta >= m_device_cfg.test_high_beep_th))
    {
        pattern_flag = false;
        nrf_gpio_pin_clear(BUZZER_PIN);
    }
}
/**
 * @brief 1-second timer handler.
 */
void timer_1sec_handler(void *p_context)
{
    // in this function we are checking the which type of alert & how buzzer works with alert
    // array_threshold[6] = { 1200,1500,100,1700,2100,2200};  <- in the above this value is declared
    // array_AlertCh[6]   = {'T','A','F','X','Y','Z'};
    // F = free fall alert , A = fall alert , T = test alert etc... ,  using its threshold value
    batt_adv_data = pwrSense.SOC;

    int ck = 0;
    mean_HIGH_check = mean_HIGH;
    mean_HIGH = 518;
    mean_LOW_check = mean_LOW;
    mean_LOW = 518;

    int alert_index = -1;       // Initialize to an invalid index
    bool hook_mod_flag = false; // Default state

    // printf("Initial alert_index: %d\n", alert_index);

    // printf("%c %c %c\n", array_AlertCh[0], array_AlertCh[1], array_AlertCh[2]);
    // printf("%0.2f %0.2f %0.2f\n", array_threshold[0], array_threshold[1], array_threshold[2]);

    // Only toggle hook_mode after a certain period or count to prevent fast toggling
    // if (sensor_detect == 1)
    // {
    //     hook_mode_toggle_count++;
    //     if (hook_mode_toggle_count >= 5) // Add a delay count (adjust as needed)
    //     {
    //         hook_mod_flag = !hook_mod_flag; // Toggle hook_mode
    //         // printf("hook_mod_flag: %s", hook_mod_flag ? "ON" : "OFF\n");
    //         hook_mode_toggle_count = 0; // Reset toggle count
    //     }
    // }

    // // Update X-axis
    // if ((int)fabs(ACC_X) < accel_x.min)
    //     accel_x.min = (int)fabs(ACC_X);

    // if ((int)fabs(ACC_X) > accel_x.max)
    //     accel_x.max = (int)fabs(ACC_X);

    // // Update Y-axis
    // if ((int)fabs(ACC_Y) < accel_y.min)
    //     accel_y.min = (int)fabs(ACC_Y);

    // if ((int)fabs(ACC_Y) > accel_y.max)
    //     accel_y.max = (int)fabs(ACC_Y);

    // // Update Z-axis
    // if ((int)fabs(ACC_Z) < accel_z.min)
    //     accel_z.min = (int)fabs(ACC_Z);

    // if ((int)fabs(ACC_Z) > accel_z.max)
    //     accel_z.max = (int)fabs(ACC_Z);

    // // Update Magnitude
    // if ((int)accData.magni_mean < accel_mag.min)
    //     accel_mag.min = (int)accData.magni_mean;

    // if ((int)accData.magni_mean > accel_mag.max)
    //     accel_mag.max = (int)accData.magni_mean;

    // if (gyr_noti_flag)
    // {
    //     memset(rotation_packet, 0x00, sizeof(rotation_packet));
    //     sprintf(rotation_packet, "{mac:%X%X%X,time:%s,x:%d,y:%d,z:%d,m:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean);
    //     raw_data_update(rotation_packet);
    // }

    // if (check_if_second_change())
    // {

    //     accel_x.delta = accel_x.max - accel_x.min;
    //     accel_y.delta = accel_y.max - accel_y.min;
    //     accel_z.delta = accel_z.max - accel_z.min;
    //     accel_mag.delta = accel_mag.max - accel_mag.min;
    //     reset_the_min_max_data();

    //     // Store and check continuously
    //     store_data(window_packet, accel_y.delta);
    //     check_pattern();

    //     // memset(window_packet, 0x00, sizeof(window_packet));
    //     snprintf(window_packet, sizeof(window_packet),
    //              "{mac:%X%X%X,time:%s,y_deltas:[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]}",
    //              gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
    //              log_window[5].y_delta,
    //              log_window[4].y_delta,
    //              log_window[3].y_delta,
    //              log_window[2].y_delta,
    //              log_window[1].y_delta,
    //              log_window[0].y_delta);

    //     // send data

    //     uart_trasmit_str(window_packet);
    //     live_alert_update(window_packet);
    // }

    // // ✅ Take one fresh reading of accelerometer
    // update_axis_data(&accel_x, ACC_X);
    // update_axis_data(&accel_y, ACC_Y);
    // update_axis_data(&accel_z, ACC_Z);
    // update_magnitude_data(&accel_mag, accData.magni_mean);

    // memset(window_packet, 0x00, sizeof(window_packet));
    // snprintf(window_packet, sizeof(window_packet),
    //          "{mac:%X%X%X,time:%s,y_deltas:[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]}",
    //          gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
    //          log_window[5].y_delta,
    //          log_window[4].y_delta,
    //          log_window[3].y_delta,
    //          log_window[2].y_delta,
    //          log_window[1].y_delta,
    //          log_window[0].y_delta);

    // uart_trasmit_str(window_packet);
    // live_alert_update(window_packet);
    // reset_the_min_max_data();

    // Send rotation_packet over UART + update alert
    // uart_trasmit_str(window_packet);
    // live_alert_update(window_packet);

    for (ck = 0; ck < array_size; ++ck)
    {
        if (FF_EN == true)
        {
            if (ck == 0)
            {
                if (mean_LOW_check < array_threshold[0])
                {
                    AlertCh = array_AlertCh[0];
                    // printf("alertch1: %c, array_thrs1: %.2f\n", AlertCh, array_threshold[0]);
                    S_ok = true;
                    alert_index = 0; // Capture the index for the threshold
                    // printf("Alert  triggered, ck: %d, alert_index: %0.2f, threshold: %0.02f\n", ck, alert_index, array_threshold[alert_index]);
                }
            }
            if (ck > 0)
            {
                if (mean_HIGH_check > array_threshold[ck])
                {
                    AlertCh = array_AlertCh[ck];
                    // printf("alertch2: %c, array_thrs2: %.2f\n", AlertCh, array_threshold[ck]);
                    S_ok = true;
                    alert_index = ck; // Capture the index for the threshold
                    // printf("Alert triggered, ck: %d, alert_index: %d, threshold: %0.02f\n", ck, alert_index, array_threshold[alert_index]);
                }
            }
        }
        else
        {
            if (mean_HIGH_check > array_threshold[ck])
            {
                AlertCh = array_AlertCh[ck];
                // printf("alertch3: %c, array_thrs3: %.2f\n", AlertCh, array_threshold[ck]);
                S_ok = true;
                alert_index = ck; // Capture the index for the threshold
                // printf("Alert triggered, ck: %d, alert_index: %d, threshold: %0.02f\n", ck, alert_index, array_threshold[alert_index]);
            }
        }
    }

    nrf_cal_get_epoch_time_string(&t1);
    sprintf(timespm, "%lu", t1);
    if (cc_time_enble == 1)
    {
        if (event_task == 0)
        {
            event_task = 1;
            if (S_ok == true && alert_index >= 0)
            {
                memset(temp1, 0, sizeof(temp1));
                if (AlertCh == 'F')
                {
                    sprintf(temp1, "G=%0.02f", mean_LOW_check);
                    // sprintf(temp1, "Current G=%0.02f, Crossed Threshold=%0.02f", mean_LOW_check, array_threshold[alert_index]);
                    // printf("temp data1: %s\n", temp1);
                    NRF_LOG_INFO("%s", temp1);
                }
                else
                {
                    sprintf(temp1, "G=%0.02f", mean_HIGH_check);
                    // sprintf(temp1, "Current G=%0.02f, Crossed Threshold=%0.02f", mean_HIGH_check, array_threshold[alert_index]);
                    // printf("temp data_1: %s\n", temp1);
                    NRF_LOG_INFO("%s", temp1);
                }

                // alert_count++; // Increment the alert count each time an alert is generated

                // getBattLevel();
                getUID(uid);
                sprintf(AlertStr, "{mac:%X%X%X,time:%s,uid:%s,alert:%c,batt:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, uid, AlertCh, pwrSense.SOC);
                // sprintf(AlertStr, "{time:%s,uid:%s,alert:%c,threshold:%0.02f,batt:%d,cth:%s,aidx:%d}", timespm, uid, AlertCh, array_threshold[alert_index], pwrSense.SOC, temp1, alert_index + 1);
                NRF_LOG_INFO("%s", AlertStr);

                // printf("alert_count: %d\n", alert_count);

                // if (conn_flag == true)
                // {
                //     live_alert_update(AlertStr);
                // }

                // if (!his_notify || !conn_flag)
                // {
                //     AddLog(AlertStr);
                // }
                // uart_trasmit_str(AlertStr);
            }

            event_task = 0;
            cc_time_enble = 0;
            cc_time = 0;
            S_ok = false;
        }
    }

    // according varun sir

    // condition 1
    // if (((fabs(ACC_Y) >= m_device_cfg.testY_beep_th && m_device_cfg.testY_beep_th != 0 && fabs(ACC_Z) >= m_device_cfg.testZ_beep_th && m_device_cfg.testZ_beep_th != 0) ||
    //      (fabs(ACC_Z) >= m_device_cfg.testY_beep_th && m_device_cfg.testY_beep_th != 0 && fabs(ACC_Y) >= m_device_cfg.testZ_beep_th && m_device_cfg.testZ_beep_th != 0)) &&
    //     (accData.magni_mean >= m_device_cfg.testA_beep_th && m_device_cfg.testA_beep_th != 0) && (fabs(ACC_X) < m_device_cfg.testX_beep_th && m_device_cfg.testX_beep_th != 0))

    // condition 2
    // if (fabs(ACC_X) <= m_device_cfg.testX_beep_th && fabs(ACC_Y) <= m_device_cfg.testY_beep_th && fabs(ACC_Z) >= m_device_cfg.testZ_beep_th)

    // condition 3
    // if ((fabs(ACC_Y) >= m_device_cfg.testY_beep_th && m_device_cfg.testY_beep_th != 0 && fabs(ACC_Z) >= m_device_cfg.testZ_beep_th && m_device_cfg.testZ_beep_th != 0) && fabs(ACC_X) >= m_device_cfg.testX_beep_th && m_device_cfg.testX_beep_th != 0)
    // {
    //     memset(window_packet, 0x00, sizeof(window_packet));
    //     snprintf(window_packet, sizeof(window_packet),
    //              "{mac:%X%X%X,alert:BEEP_LT,time:%s,ax:%.0f,ay:%.0f,az:%.0f,xv:%d,yv:%d,zv:%d,av:%d}",
    //              gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
    //              fabs(ACC_X), fabs(ACC_Y), fabs(ACC_Z), m_device_cfg.testX_beep_th,
    //              m_device_cfg.testY_beep_th, m_device_cfg.testZ_beep_th, m_device_cfg.testA_beep_th);

    //     pattern_flag = true;
    //     nrf_gpio_pin_set(BUZZER_PIN);
    //     uart_trasmit_str(window_packet);
    //     live_alert_update(window_packet);
    //     printf("<<----- pattern detected ! beep...beep ----->>\n");
    // }
    // else
    // {
    //     memset(window_packet, 0x00, sizeof(window_packet));
    //     snprintf(window_packet, sizeof(window_packet),
    //              "{mac:%X%X%X,time:%s,ax:%.0f,ay:%.0f,az:%.0f,xv:%d,yv:%d,zv:%d,av:%d}",
    //              gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
    //              fabs(ACC_X), fabs(ACC_Y), fabs(ACC_Z), m_device_cfg.testX_beep_th,
    //              m_device_cfg.testY_beep_th, m_device_cfg.testZ_beep_th, m_device_cfg.testA_beep_th);

    //     uart_trasmit_str(window_packet);
    // }

    // if (pattern_flag)
    // {
    //     nrf_gpio_pin_clear(BUZZER_PIN);
    // }

    // if (accData.mean >= m_device_cfg.lock_test_th && m_device_cfg.lock_test_th != 0 && (fabs(ACC_X) >= m_device_cfg.x_axis_gt && m_device_cfg.x_axis_gt != 0 && fabs(ACC_X) <= m_device_cfg.x_axis_lt && m_device_cfg.x_axis_lt != 0)) // changes accr. to @md_sir
    // {
    //     // printf("lock_threshold crossed of range:%d\n", m_device_cfg.lock_test_th);
    //     sprintf(AlertStr, "{mac:%X%X%X,time:%s,lock:TL,x:%3.02f,y:%3.02f,z:%3.02f,r:%0.0f,p:%0.0f,w:%0.0f,rec_id:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, ACC_X, ACC_Y, ACC_Z, m_device_cfg.lock_test_th); // changes accr. to @md_sir
    //     if (conn_flag == true)
    //     {
    //         live_alert_update(AlertStr);
    //     }
    //     if (!his_notify || !conn_flag)
    //     {
    //         AddLog(AlertStr);
    //     }
    //     uart_trasmit_str(AlertStr);
    //     nrf_gpio_pin_set(BUZZER_PIN);
    //     printf("<<--- lock detceted ---->>\n");
    // }
    // else
    // {
    //     nrf_gpio_pin_clear(BUZZER_PIN);
    //     sprintf(AlertStr, "{mac:%X%X%X,time:%s,x:%.0f,y:%.0f,z:%.0f,m:%.0f,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, fabs(ACC_X), fabs(ACC_Y), fabs(ACC_Z), accData.magni_mean, m_device_cfg.lock_test_th); // changes accr. to @md_sir
    //     uart_trasmit_str(AlertStr);
    // }
    // else if (accData.mean <= m_device_cfg.fall_test_th) // changes accr. to @md_sir
    // {
    //     // printf("fall_threshold down less than:%d\n", m_device_cfg.fall_test_th);
    //     sprintf(AlertStr, "{mac:%X%X%X,time:%s,lock:TF,x:%3.02f,y:%3.02f,z:%3.02f,r:%0.0f,p:%0.0f,w:%0.0f,rec_id:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, ACC_X, ACC_Y, ACC_Z, AccXangle, AccYangle, AccZangle, m_device_cfg.fall_test_th); // changes accr. to @md_sir
    //     if (conn_flag == true)
    //     {
    //         live_alert_update(AlertStr);
    //     }
    //     if (!his_notify || !conn_flag)
    //     {
    //         AddLog(AlertStr);
    //     }
    //     uart_trasmit_str(AlertStr);
    // }

    // if (m_device_cfg.ht_flag_start == 1 && !hook_test_ok && fabs(ACC_Z) >= m_device_cfg.z_axis_gt)
    // if (m_device_cfg.zl_flag_start == 1 && fabs(ACC_Z) < m_device_cfg.z_axis_lt && m_device_cfg.xg_flag_start == 1 && fabs(ACC_X) > m_device_cfg.x_axis_gt && !hook_test_ok)
    // if (m_device_cfg.yl_flag_start == 1 && fabs(ACC_Y) < m_device_cfg.y_axis_lt && m_device_cfg.avg_flag_start == 1 && (int)accData.magni_mean > m_device_cfg.xyz_avg_gt && !hook_test_ok)
    // {
    //     // buzzer_toggle();
    //     hook_test_ok = true;
    //     // m_device_cfg.zl_flag_start = 0;
    //     // m_device_cfg.xg_flag_start = 0;
    //     nrf_gpio_pin_set(BUZZER_PIN); // ON
    //     // printf("buzzer on...\n");

    //     sprintf(AlertStr, "{mac:%X%X%X,time:%s,lock:FF,x:%d,y:%d,z:%d,zl:%d,xg:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), m_device_cfg.z_axis_lt, m_device_cfg.x_axis_gt); // changes accr. to @md_sir

    //     if (conn_flag == true)
    //     {
    //         live_alert_update(AlertStr);
    //     }
    //     if (!his_notify || !conn_flag)
    //     {
    //         AddLog(AlertStr);
    //     }
    //     uart_trasmit_str(AlertStr);
    // }

    // if (m_device_cfg.yl_flag_start == 1)
    // {
    //        nrf_gpio_pin_set(BUZZER_PIN); // ON
    // }

    // else if(hook_test_ok)
    // {
    //     hook_test_ok = false;
    //     nrf_gpio_pin_clear(BUZZER_PIN); // OFF
    //     printf("buzzer off...\n");
    // }
    // else if (!hook_test_ok)
    // {
    //     // buzzer_toggle();
    // }

    // if (fabs(ACC_Z) <= m_device_cfg.z_axis_lt && fabs(ACC_X) <= m_device_cfg.x_axis_gt)
    // // if (m_device_cfg.ht_flag_start == 1 && hook_test_ok && fabs(ACC_Z) <= m_device_cfg.z_axis_lt && fabs(ACC_Z) <= m_device_cfg.x_axis_gt)
    // {
    //     sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:Fall,x:%d,y:%d,z:%d,thz:%d,thx:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), m_device_cfg.z_axis_lt, m_device_cfg.x_axis_gt); // changes accr. to @md_sir

    //     if (conn_flag == true)
    //     {
    //         live_alert_update(AlertStr);
    //     }
    //     if (!his_notify || !conn_flag)
    //     {
    //         AddLog(AlertStr);
    //     }

    //     uart_trasmit_str(AlertStr);
    // }

    // if (xg_flag)
    // {
    //     if (fabs(ACC_X) > m_device_cfg.x_axis_gt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,ACC_X:%d,greater:%d}", timespm, (int)fabs(ACC_X), m_device_cfg.x_axis_gt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:xg,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.x_axis_gt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("ACC_X axis greater crossed threshold: %d\n", m_device_cfg.x_axis_gt);
    //     }
    // }

    // if (xless_flag)
    // {
    //     if (fabs(ACC_X) < m_device_cfg.x_axis_lt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,ACC_X:%d,less:%d}", timespm, (int)fabs(ACC_X), m_device_cfg.x_axis_lt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:xl,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.x_axis_lt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("ACC_X axis less crossed threshold: %d\n", m_device_cfg.x_axis_lt);
    //     }
    // }

    // if (yg_flag)
    // {
    //     if (fabs(ACC_Y) > m_device_cfg.y_axis_gt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,ACC_Y:%d,greater:%d}", timespm, (int)fabs(ACC_Y), m_device_cfg.y_axis_gt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:yg,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.y_axis_gt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("ACC_Y axis greater crossed threshold: %d\n", m_device_cfg.y_axis_gt);
    //     }
    // }

    // if (yl_flag)
    // {
    //     if (fabs(ACC_Y) < m_device_cfg.y_axis_lt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,ACC_Y:%d,less:%d}", timespm, (int)fabs(ACC_Y), m_device_cfg.y_axis_lt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:yl,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.y_axis_lt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("ACC_Y axis less crossed threshold: %d\n", m_device_cfg.y_axis_lt);
    //     }
    // }

    // if (zg_flag)
    // {
    //     if (fabs(ACC_Z) > m_device_cfg.z_axis_gt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,ACC_Z:%d,greater:%d}", timespm, (int)fabs(ACC_Z), m_device_cfg.z_axis_gt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:zg,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.z_axis_gt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("ACC_Z axis greater crossed threshold: %d\n", m_device_cfg.z_axis_gt);
    //     }
    // }

    // if (zl_flag)
    // {
    //     if (fabs(ACC_Z) < m_device_cfg.z_axis_lt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,ACC_Z:%d,less:%d}", timespm, (int)fabs(ACC_Z), m_device_cfg.z_axis_lt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:zl,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.z_axis_lt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("ACC_Z axis less crossed threshold: %d\n", m_device_cfg.z_axis_lt);
    //     }
    // }

    // if (avg_gt_flag)
    // {
    //     if (accData.magni_mean > m_device_cfg.xyz_avg_gt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,avg:%d,greater:%d}", timespm, (int)accData.magni_mean, m_device_cfg.xyz_avg_gt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:ag,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.xyz_avg_gt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("AVG greater crossed threshold: %d\n", m_device_cfg.xyz_avg_gt);
    //     }
    // }

    // if (avg_lt_flag)
    // {
    //     if (accData.magni_mean < m_device_cfg.xyz_avg_lt)
    //     {
    //         // sprintf(AlertStr, "{time:%s,avg:%d,less:%d}", timespm, (int)accData.magni_mean, m_device_cfg.xyz_avg_lt);
    //         sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:al,x:%d,y:%d,z:%d,av:%d,th:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean, m_device_cfg.xyz_avg_lt);
    //         if (conn_flag == true)
    //         {
    //             live_alert_update(AlertStr);
    //         }
    //         if (!his_notify || !conn_flag)
    //         {
    //             AddLog(AlertStr);
    //         }
    //         uart_trasmit_str(AlertStr);
    //         // printf("AVG less crossed threshold: %d\n", m_device_cfg.xyz_avg_lt);
    //     }
    // }

    // if (m_device_cfg.master_device_flag != 1 && m_device_cfg.slave_device_flag != 1)
    // {
    //     buzzer_toggle();
    // }
    // else if (!msd_set_flag && (m_device_cfg.master_device_flag == 1 || m_device_cfg.slave_device_flag == 1))
    // {
    //     msd_set_flag = true;
    //     nrf_gpio_pin_clear(BUZZER_PIN);
    // }

    if (buz_hmcmd_ex && m_device_cfg.hook_mode == 1 || buz_hmcmd_ex && m_device_cfg.hook_mode == 0)
    {
        hm_cmdex_buzcnt++;
        nrf_gpio_pin_set(BUZZER_PIN);

        nrf_gpio_pin_clear(BATTERY_ALERT_PIN);
        if (hm_cmdex_buzcnt >= 3)
        {
            hm_cmdex_buzcnt = 0;
            buz_hmcmd_ex = false;
            nrf_gpio_pin_clear(BUZZER_PIN);
            nrf_gpio_pin_set(BATTERY_ALERT_PIN);
            // is_url = false;
        }
        // printf("bz_cnt:%d\n", hm_cmdex_buzcnt);
    }

    // for beacon

    // if (is_target)
    // {
    //     nrf_ble_scan_stop();

    //     if (m_device_cfg.hook_mode == 0 && device_detect_cnt == 1)
    //     {
    //         // nrf_ble_scan_stop();
    //         start_device_found = true;
    //         buz_en = 1;
    //         buz_hmcmd_ex = true;
    //         m_device_cfg.hook_mode = 1;
    //         m_device_cfg.buzzer_onoff_enable = 1;
    //         is_target = false;
    //         printf(" Beacon Detect 1st time...\n");
    //     }
    //     else if (device_detect_cnt == 2 || device_detect_cnt == 1 && buz_en > 0 && is_target)
    //     {
    //         // nrf_ble_scan_stop();
    //         start_device_found = true;
    //         buz_en = 0;
    //         buz_hmcmd_ex = true;
    //         m_device_cfg.hook_mode = 0;
    //         m_device_cfg.buzzer_onoff_enable = 0;
    //         is_target = false;
    //         device_detect_cnt = 0;
    //         nrf_gpio_pin_set(BUZZER_LED);
    //         printf(" 2nd time detect...  ||  Already on now stopped...\n");
    //     }
    //     else if (device_detect_cnt > 2)
    //     {
    //         // blescan_start();
    //         start_device_found = true;
    //         buz_en = 0;
    //         buz_hmcmd_ex = true;
    //         m_device_cfg.hook_mode = 0;
    //         m_device_cfg.buzzer_onoff_enable = 0;
    //         is_target = false;
    //         device_detect_cnt = 0;
    //         nrf_gpio_pin_set(BUZZER_LED);
    //         printf(" Detect more than 2times...\n");
    //     }
    // }

    if (is_UUID && m_device_cfg.hook_mode == 0)
    {

        // if (device_detect_cnt == 1)
        // {
        printf(" --------- hello test 1----------\n");
        nrf_ble_scan_stop();
        iB_start_flag = true;
        is_UUID = false;

        buz_en = 1;
        buz_hmcmd_ex = true;
        m_device_cfg.hook_mode = 1;
        m_device_cfg.buzzer_onoff_enable = 1;
        nrf_gpio_pin_set(BUZZER_LED);

        sprintf(AlertStr, "{mac:%X%X%X,time:%s,alerthm:START}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        AddLog(AlertStr);
        uart_trasmit_str(AlertStr);
        // printf(" UUID key detect device started.\n");
        // }
    }
    else if (is_UUID && m_device_cfg.hook_mode == 1)
    {
        // if (device_detect_cnt == 2)
        // {
        printf(" --------- hello test 2 ----------\n");
        nrf_ble_scan_stop();
        iB_stop_flag = true;
        is_UUID = false;

        buz_en = 0;
        buz_hmcmd_ex = true;
        m_device_cfg.hook_mode = 0;
        m_device_cfg.buzzer_onoff_enable = 0;
        nrf_gpio_pin_set(BUZZER_LED);

        sprintf(AlertStr, "{mac:%X%X%X,time:%s,alerthm:STOP}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        AddLog(AlertStr);
        uart_trasmit_str(AlertStr);
        // printf(" UUID key detect device stopped.\n");
        // }
    }
    else if (is_url && m_device_cfg.hook_mode == 1)
    {
        printf("------------ hello test 3 -----------\n");
        nrf_ble_scan_stop();
        is_url = false;
        iB_stop_flag = true;
        is_UUID = false;
        device_detect_cnt = 0;

        buz_en = 0;
        buz_hmcmd_ex = true;
        m_device_cfg.hook_mode = 0;
        m_device_cfg.buzzer_onoff_enable = 0;
        nrf_gpio_pin_set(BUZZER_LED);

        sprintf(AlertStr, "{mac:%X%X%X,time:%s,alerthm:STOP}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        AddLog(AlertStr);
        uart_trasmit_str(AlertStr);
        // printf("url_detected device stopped\n");
    }
    // else if (device_detect_cnt > 2)
    // {
    //     blescan_start();
    // }

    if (iB_start_flag)
    {
        iB_start_cnt++;
        printf("start_stop counter:%d\n", iB_start_cnt);

        if (iB_start_cnt >= m_device_cfg.scanwait_time)
        {
            iB_start_cnt = 0;
            iB_start_flag = false;
            blescan_start();
        }
    }

    if (iB_stop_flag)
    {
        iB_stop_cnt++;
        printf("start_stop counter:%d\n", iB_stop_cnt);

        if (iB_stop_cnt >= m_device_cfg.scanwait_time)
        {
            iB_stop_cnt = 0;
            iB_stop_flag = false;
            blescan_start();
        }
    }

    if (m_device_cfg.master_device_flag == 1 && m_device_cfg.slave_device_flag == 0) // && m_device_cfg.nm_lan_flag == 0)
    {
        if (m_device_cfg.hook_mode == 0 || m_device_cfg.hook_mode == 1)
        {
            // for master
            sprintf(adv_data, "%d%d", m_device_cfg.hook_mode, (int)self_hook_disconnect);
            update_advertising_data(adv_data);
        }
    }

    if (m_device_cfg.slave_device_flag == 1 && m_device_cfg.nm_lan_flag == 0)
    {
        if (scan_hook_stats && m_device_cfg.hook_mode == 0) // for slave
        {
            printf("slave hook enable according to master status ... hello_1\n");
            buz_en = 1;
            buz_hmcmd_ex = true;
            m_device_cfg.hook_mode = 1;
            m_device_cfg.buzzer_onoff_enable = 1;
        }
        else if (!scan_hook_stats && m_device_cfg.hook_mode == 1) // for slave
        {
            printf("slave hook disable according to master status ... hello_2\n");
            buz_en = 0;
            buz_hmcmd_ex = true;
            m_device_cfg.hook_mode = 0;
            m_device_cfg.buzzer_onoff_enable = 0;
            nrf_gpio_pin_set(BUZZER_LED);
        }
    }

    // if (m_device_cfg.slave_device_flag == 1 && m_device_cfg.nm_lan_flag == 0 && m_device_cfg.hm_stopflag == 1)
    // {
    //     printf("hello_3\n");
    //     buz_en = 0;
    //     buz_hmcmd_ex = false;
    //     m_device_cfg.hook_mode = 0;
    //     m_device_cfg.buzzer_onoff_enable = 0;
    // }

    if (m_device_cfg.hm_startflag == 1)
    {
        delay_counter++;
        printf("hm_start_counter:%d\n", delay_counter);

        if (delay_counter >= m_device_cfg.delaytimer_threshold)
        {
            delay_counter = 0;
            m_device_cfg.hm_startflag = 0;
            buz_en = 1;
            m_device_cfg.buzzer_onoff_enable = 1;

            // memset(temp2, 0, sizeof(temp2));
            sprintf(AlertStr, "{mac:%X%X%X,time:%s,alerthm:START}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);

            if (conn_flag == true)
            {
                // sprintf(temp2, "{ct:%s,alerthm:start}", timespm); // save hook history log in mem after disconn.
                live_alert_update(AlertStr);
                // AddLog(temp2);
            }
            if (!his_notify)
            {
                // sprintf(temp2, "{lt:%s,ct:%s,alerthm:start}", timestr, timespm); // save hook history log in mem after disconn.
                AddLog(AlertStr);
            }

            uart_trasmit_str(AlertStr);
            printf("<<----- THE HOOK BUZZER IS ENABLED ----->>\n");
        }
        else
        {
            // buz_en = 0;
            // m_device_cfg.buzzer_onoff_enable = 0;
        }

        // printf("delay counter:%d\n", delay_counter);
    }

    // static int buzzer_low_count = 0; // Counter for BUZZER_PIN low state

    // if (nrf_gpio_pin_out_read(BUZZER_PIN) == 1)
    // {
    //     buzzer_high_count++;  // Increment the high count
    //     buzzer_low_count = 0; // Reset the low count since BUZZER_PIN is high

    //     // If BUZZER_PIN remains high for 5 seconds
    //     if (buzzer_high_count >= 5)
    //     {
    //         // Run the motor in the clockwise direction
    //         nrf_gpio_pin_set(MOTOR_PIN1);   // MOTOR_PIN High
    //         nrf_gpio_pin_clear(MOTOR_PIN2); // MOTOR_PIN2 Low
    //         // printf("Motor running clockwise\n");
    //     }
    // }
    // else
    // {
    //     buzzer_low_count++;    // Increment the low count
    //     buzzer_high_count = 0; // Reset the high count since BUZZER_PIN is low

    //     // If BUZZER_PIN remains low for 5 seconds
    //     if (buzzer_low_count >= 5)
    //     {
    //         // Run the motor in the anticlockwise direction
    //         nrf_gpio_pin_clear(MOTOR_PIN1); // MOTOR_PIN Low
    //         nrf_gpio_pin_set(MOTOR_PIN2);   // MOTOR_PIN2 High
    //         // printf("Motor running anticlockwise after 5 seconds of BUZZER_PIN low state\n");
    //     }
    // }

    if (buz_en > 0)
    {
        if (sensor_detect == 1)
        {
            buz_10s_cnt++;
            if (buz_10s_cnt > buz_change_on_off_cnt)
            {
                if (m_device_cfg.buzzer_onoff_enable == 1)
                {
                    if (nrf_gpio_pin_out_read(BUZZER_PIN) == 1)
                    {
                        // printf("hey1\n");
                        nrf_gpio_pin_clear(BUZZER_PIN); // buzzer off
                        nrf_gpio_pin_set(BUZZER_LED);   // Buzzer LDE off
                        NRF_LOG_INFO("stop buzzer OFF");
                        buz_change_on_off_cnt = m_device_cfg.hook_beeptime; // before hardcoded 3
                    }
                    else
                    {
                        // printf("hey2\n");
                        nrf_gpio_pin_set(BUZZER_PIN);   // buzzer on
                        nrf_gpio_pin_clear(BUZZER_LED); // Buzzer LDE oN
                        // printf("green on 1\n");
                        NRF_LOG_INFO("stop buzzer ON");
                        sprintf(timestr, "%d", last_time);
                        // memset(temp2, 0, sizeof(temp2));

                        // sprintf(temp2,"{lt:%s,ct:%s,alerthm:hm,cd:0}",timestr,timespm); // no hook log save in mem by this
                        // NRF_LOG_INFO("%s",temp2);
                        // live_alert_update(temp2);
                        // uart_trasmit_str(temp2);

                        // hook_cn_stat = false;
                        // hook_dis_stat = true;

                        // printf("hello\n");
                        // sprintf(AlertStr, "{lt:%s,ct:%s,alerthm:hm,cd:0}", timestr, timespm); // changed this to save hook log in memory after disconn.
                        // NRF_LOG_INFO("%s", AlertStr);

                        if (conn_flag == true && !hook_dis_stat)
                        {
                            // printf("hello\n");
                            self_hook_disconnect = true;
                            hd_cnt++;
                            sprintf(AlertStr, "{mac:%X%X%X,lt:%s,ct:%s,hdc:%d,alerthm:hm,cd:0}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hd_cnt); // changed this to save hook log in memory after disconn.

                            live_alert_update(AlertStr);
                            // AddLog(AlertStr);
                            hook_dis_stat = true;
                            hook_cn_stat = false;
                            uart_trasmit_str(AlertStr);
                        }

                        else if (!his_notify && !hook_dis_stat)
                        {
                            // printf("hello1\n");
                            self_hook_disconnect = true;
                            hd_cnt++;
                            sprintf(AlertStr, "{mac:%X%X%X,lt:%s,ct:%s,hdc:%d,alerthm:hm,cd:0}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hd_cnt);

                            AddLog(AlertStr);
                            hook_dis_stat = true;
                            hook_cn_stat = false;
                            uart_trasmit_str(AlertStr);
                        }

                        // uart_trasmit_str(AlertStr);
                        buz_change_on_off_cnt = m_device_cfg.buzzertimer_threshold; // hardcoded 10

                        hook_conn = 0;
                        // sprintf(adv_data, "%d%d", hook_conn, batt_adv_data);
                        sprintf(adv_data, "%d%d%d", hook_conn, batt_adv_data, m_device_cfg.hook_mode);
                        // printf("ADV_DATA:%s\n", adv_data);
                        // update_advertising_data(adv_data);
                    }
                }
                else
                {
                    if (buz_change_on_off_cnt == m_device_cfg.buzzertimer_threshold) // hardcoded 10
                    {
                        // printf("hey3\n");
                        nrf_gpio_pin_set(BUZZER_LED);                       // Buzzer LDE off
                        buz_change_on_off_cnt = m_device_cfg.hook_beeptime; // harcoded 3
                    }
                    else if (buz_change_on_off_cnt == m_device_cfg.hook_beeptime) // hardcoded 3
                    {
                        // printf("hello2\n");
                        nrf_gpio_pin_clear(BUZZER_LED); // Buzzer LDE oN
                        // printf("green on 2\n");
                        sprintf(timestr, "%d", last_time);
                        hd_cnt++;
                        self_hook_disconnect = true;
                        memset(AlertStr, 0, sizeof(AlertStr));
                        sprintf(AlertStr, "{mac:%X%X%X,lt:%s,ct:%s,hdc:%d,alerthm:hm,cd:0}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hd_cnt);
                        NRF_LOG_INFO("%s", AlertStr);
                        AddLog(AlertStr);
                        live_alert_update(AlertStr);

                        if (!his_notify)
                        {
                            AddLog(temp2);
                        }

                        uart_trasmit_str(temp2);
                        buz_change_on_off_cnt = m_device_cfg.buzzertimer_threshold; // hardcoded 10

                        hook_conn = 0;
                        // sprintf(adv_data, "%d%d", hook_conn, batt_adv_data);
                        sprintf(adv_data, "%d%d%d", hook_conn, batt_adv_data, m_device_cfg.hook_mode);
                        // printf("ADV_DATA:%s\n", adv_data);
                        // update_advertising_data(adv_data);
                    }
                }
                buz_10s_cnt = 0;
            }
        }
    }

    temperature_dataread();
}

/**
 * @brief Handler for timer events.
 */
void timer_5mit_handler(void *p_context)
{
    // Hook mode check run on 1 sec
    // buzzer working on the basis of hook sensor status
    batt_adv_data = pwrSense.SOC;

    if (m_device_cfg.cal_mode == 0)
    {
        if (buz_en == 1)
        {
            FTCT++;
            if (FTCT > 5)
            {
                if (nrf_gpio_pin_read(Hook_Sensor) == 0)
                {
                    // printf("hey4\n");
                    if (m_device_cfg.buzzer_onoff_enable == 1)
                        nrf_gpio_pin_clear(BUZZER_PIN);
                    nrf_gpio_pin_set(BUZZER_LED); // Buzzer LDE off
                    NRF_LOG_INFO("buzzer OFF");
                    sensor_detect = 0;
                    last_time = t1;
                    sprintf(timestr, "%d", last_time);
                    memset(temp2, 0, sizeof(temp2));
                    // sprintf(temp2, "{lt:%s,ct:%s,alerthm:hm,cd:1}", timestr, timespm);
                    // NRF_LOG_INFO("%s", temp2);
                    // live_alert_update(temp2);

                    // if (!conn_flag)
                    // {
                    //     AddLog(temp2);
                    // }
                    bhd_flag = false;
                    self_hook_disconnect = false;
                    hc_cnt++;
                    sprintf(temp2, "{mac:%X%X%X,lt:%s,ct:%s,hcc:%d,alerthm:hm,cd:1}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hc_cnt);
                    NRF_LOG_INFO("%s", temp2);
                    // live_alert_update(temp2);
                    if (conn_flag == true)
                    {
                        live_alert_update(temp2);
                    }
                    if (!his_notify)
                    {
                        AddLog(temp2);
                    }
                    uart_trasmit_str(temp2);
                    printf("hello3\n");
                    hook_conn = 1;
                    // sprintf(adv_data, "%d%d", hook_conn, batt_adv_data);
                    sprintf(adv_data, "%d%d%d", hook_conn, batt_adv_data, m_device_cfg.hook_mode);
                    // printf("ADV_DATA:%s\n", adv_data);
                    // update_advertising_data(adv_data);
                }
                else
                {
                    // printf("hey5\n");
                    if (m_device_cfg.buzzer_onoff_enable == 1)
                        nrf_gpio_pin_set(BUZZER_PIN);
                    nrf_gpio_pin_clear(BUZZER_LED); // Buzzer LDE on
                    // printf("green on 3\n");
                    NRF_LOG_INFO("buzzer ON");
                    sensor_detect = 1;
                    current_time = t1;
                    if (S_C_C == 1)
                    {
                        hd_cnt++;
                        self_hook_disconnect = true;
                        sprintf(timestr, "%d", last_time);
                        memset(AlertStr, 0, sizeof(AlertStr));
                        sprintf(AlertStr, "{mac:%X%X%X,lt:%s,ct:%s,hdc:%d,alerthm:hm,cd:0}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hd_cnt);
                        NRF_LOG_INFO("%s", temp2);
                        AddLog(AlertStr);
                        live_alert_update(AlertStr);
                        uart_trasmit_str(AlertStr);

                        if (!his_notify)
                        {
                            AddLog(temp2);
                        }

                        S_C_C = 2;
                        // printf("hello4\n");
                        hook_conn = 0;
                        // sprintf(adv_data, "%d%d", hook_conn, batt_adv_data);
                        sprintf(adv_data, "%d%d%d", hook_conn, batt_adv_data, m_device_cfg.hook_mode);
                        // printf("ADV_DATA:%s\n", adv_data);
                        // update_advertising_data(adv_data);
                    }
                }
                buz_en = 2;
            }
        }
        else if (buz_en == 2)
        {
            buz_cnt++;
            if (buz_cnt > 1)
            {
                buz_cnt = 0;
                if (nrf_gpio_pin_read(Hook_Sensor) == 0)
                {
                    // printf("hey6\n");
                    sensor_detect = 0;
                    if (m_device_cfg.buzzer_onoff_enable == 1)
                        nrf_gpio_pin_clear(BUZZER_PIN); // buzzer off
                    nrf_gpio_pin_set(BUZZER_LED);       // Buzzer LDE off
                    last_time = t1;
                    buz_cnt = 0;
                    buz_stop = 0;
                    buz_10s_cnt = 0;
                    S_C_C = 1;
                    buz_change_on_off_cnt = m_device_cfg.hook_beeptime; // hardcoded 3
                    if (S_O_C == 1)
                    {
                        sprintf(timestr, "%d", last_time);
                        memset(temp2, 0, sizeof(temp2));

                        // sprintf(temp2,"{lt:%s,ct:%s,alerthm:hm,cd:1}",timestr,timespm); // no hook log save in mem by this
                        // live_alert_update(temp2);
                        // NRF_LOG_INFO("%s",temp2);
                        // live_alert_update(temp2);
                        // uart_trasmit_str(temp2);

                        // hook_dis_stat = false;
                        // hook_cn_stat = true;

                        // printf("hello5\n");
                        // sprintf(AlertStr, "{lt:%s,ct:%s,alerthm:hm,cd:1}", timestr, timespm); // save hook history log in mem after disconn.

                        if (conn_flag == true && !hook_cn_stat)
                        {
                            printf("hello5\n");
                            self_hook_disconnect = false;
                            bhd_flag = false;
                            hc_cnt++;
                            sprintf(AlertStr, "{mac:%X%X%X,lt:%s,ct:%s,hcc:%d,alerthm:hm,cd:1}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hc_cnt); // save hook history log in mem after disconn.

                            live_alert_update(AlertStr);
                            // AddLog(AlertStr);
                            hook_cn_stat = true;
                            hook_dis_stat = false;
                            uart_trasmit_str(AlertStr);
                        }
                        else if (!his_notify && !hook_cn_stat)
                        {
                            printf("hello6\n");
                            self_hook_disconnect = false;
                            bhd_flag = false;
                            hc_cnt++;
                            sprintf(AlertStr, "{mac:%X%X%X,lt:%s,ct:%s,hcc:%d,alerthm:hm,cd:1}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timestr, timespm, hc_cnt); // save hook history log in mem after disconn.
                            AddLog(AlertStr);

                            hook_dis_stat = false;
                            hook_cn_stat = true;
                            uart_trasmit_str(AlertStr);
                        }

                        // uart_trasmit_str(AlertStr);
                        S_O_C = 2;

                        hook_conn = 1;
                        // sprintf(adv_data, "%d%d", hook_conn, batt_adv_data);
                        sprintf(adv_data, "%d%d%d", hook_conn, batt_adv_data, m_device_cfg.hook_mode);
                        // printf("ADV_DATA:%s\n", adv_data);
                        // update_advertising_data(adv_data);
                    }
                }
                else
                {
                    S_O_C = 1;
                    sensor_detect = 1;
                }
            }
            else
            {
                current_time = t1;
            }
        }
    }
}

void timer_health_handler(void *p_context)
{
    ret_code_t err_code;
    getTimestamp(ts); // in this function we'r getting the timestamp
    getUID(uid);      // get device ID
    err_code = lsm6dsl_device_id_get(&dev_ctx, (uint8_t *)&whoamI);
    if (whoamI[0] != LSM6DSL_ID)
    {
        AlertCh = 'S';
        NRF_LOG_ERROR("Device Not found %x\r\n", whoamI[0]);
        return err_code;
    }
    else
    {
        AlertCh = 'H';
        NRF_LOG_INFO("Device found %x\r\n", whoamI[0]);
    }
    sprintf(AlertStr, "{mac:%X%X%X,time:%s,uid:%s,alert:%c,batt:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, uid, AlertCh, pwrSense.SOC);
    // AddLog(AlertStr);
    uart_trasmit_str(AlertStr);
    health_flag = false;
}

/**
 * @brief Handler for timer events.
 */
void timer_acc_handler(void *p_context)
{
    // in this function we getting the rotation movement..
    batt_adv_data = pwrSense.SOC;

    if (AccZangle > (lastangle + 10))
    {
        // if (AccZangle > 300 && AccZangle < 360)
        if ((AccZangle >= 0 && AccZangle < 30) || (AccZangle >= 300)) // do it by 360 && do the 30 to the 60
        {
            if (flag_tick == 0)
            {
                RotationF++;
                flag_tick = 1;
                flag_tick1 = 1;
            }
        }
        else if (AccZangle > 130 && AccZangle < 180)
        {
            flag_tick = 0;
        }
        lastangle = AccZangle;
    }
    else if (AccZangle < (lastangle - 10))
    {
        // if (AccZangle > 300 && AccZangle < 360)
        if ((AccZangle >= 330 && AccZangle < 360) || (AccZangle <= 30))
        {
            if (flag_tick1 == 0)
            {
                RotationR++;
                flag_tick1 = 1;
                flag_tick = 1;
                printf("Counterclockwise Count: %d\n", RotationR);
            }
        }
        // else if (AccZangle > 130 && AccZangle < 180)
        else if (AccZangle > 180 && AccZangle < 230)
        {
            flag_tick1 = 0;
        }
        lastangle = AccZangle;
    }
    if (cc_time_enble == 1)
    {
        if (event_task == 0)
        {
            event_task = 1;
            if (RotationF != 0)
            {
                rcc_cnt++;
                memset(rotation_packet, 0x00, sizeof(rotation_packet));

                // print the accelerometer data...s
                sprintf(rotation_packet, "{mac:%X%X%X,time:%s,rcc:%d,x:%3.02f,y:%3.02f,z:%3.02f,r:%0.0f,p:%0.0f,w:%0.0f,cl:%d,acl:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, rcc_cnt, ACC_X, ACC_Y, ACC_Z, AccXangle, AccYangle, AccZangle, 1, 0);
                NRF_LOG_INFO("%s", rotation_packet);
                RotationF--;
                if (RotationF < 0)
                {
                    RotationF = 0;
                }
                if (conn_flag == true)
                {
                    live_alert_update(rotation_packet);
                    save_cnt_mode(RotationF, RotationR);
                }
                if (!his_notify)
                {
                    AddLog(rotation_packet);
                    save_cnt_mode(RotationF, RotationR);
                }
                uart_trasmit_str(rotation_packet);
            }
            if (RotationR != 0)
            {
                rac_cnt++;
                memset(rotation_packet, 0x00, sizeof(rotation_packet));
                // sprintf(rotation_packet, "{time:%s,x:%3.02f,y:%3.02f,z:%3.02f,r:%0.0f,p:%0.0f,w:%0.0f,cl:%d,acl:%d}", timespm, ACC_X, ACC_Y, ACC_Z, AccXangle, AccYangle, AccZangle, 0, 1);
                sprintf(rotation_packet, "{mac:%X%X%X,time:%s,rac:%d,x:%3.02f,y:%3.02f,z:%3.02f,r:%0.0f,p:%0.0f,w:%0.0f,cl:%d,acl:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, rac_cnt, ACC_X, ACC_Y, ACC_Z, AccXangle, AccYangle, AccZangle, 0, 1);
                NRF_LOG_INFO("%s", rotation_packet);
                RotationR--;
                if (RotationR < 0)
                {
                    RotationR = 0;
                }
                if (conn_flag == true)
                {
                    live_alert_update(rotation_packet);
                    save_cnt_mode(RotationF, RotationR);
                }
                if (!his_notify)
                {
                    AddLog(rotation_packet);
                    save_cnt_mode(RotationF, RotationR);
                }
                uart_trasmit_str(rotation_packet);
            }
            event_task = 0;
            cc_time_enble = 0;
            cc_time = 0;
        }
    }
    // if (gyr_noti_flag == true)
    // {
    //     memset(rotation_packet, 0x00, sizeof(rotation_packet));
    //     // sprintf(rotation_packet, "{time:%s,AX:%3.02f,AY:%3.02f,AZ:%3.02f,GX:%3.02f,GY:%3.02f,GZ:%3.02f}", timespm, ACC_X, ACC_Y, ACC_Z, accData.GY_X, accData.GY_Y, accData.GY_Z);
    //     // sprintf(rotation_packet, "{time:%s,x:%3.02f,y:%3.02f,z:%3.02f,r:%0.0f,p:%0.0f,w:%0.0f,Tp:%0.0f,T:%0.2f}", timespm, ACC_X, ACC_Y, ACC_Z, AccXangle, AccYangle, AccZangle, TAccYangle, temperature_celsius);
    //     sprintf(rotation_packet, "{mac:%X%X%X,time:%s,x:%d,y:%d,z:%d,r:%0.0f,p:%0.0f,w:%0.0f,m:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), AccXangle, AccYangle, AccZangle, (int)accData.magni_mean);
    //     NRF_LOG_INFO("%s", rotation_packet);
    //     raw_data_update(rotation_packet);
    // }

    if (gyr_noti_flag)
    {
        memset(rotation_packet, 0x00, sizeof(rotation_packet));
        // sprintf(rotation_packet, "{mac:%X%X%X,time:%s,x:%d,y:%d,z:%d,m:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean);
        sprintf(rotation_packet,
                "{time:%s,xm:%d,xM:%d,ym:%d,yM:%d,zm:%d,zM:%d,m:%d,M:%d}", timespm,
                (int)fabs(accel_x.min), (int)fabs(accel_x.max),
                (int)fabs(accel_y.min), (int)fabs(accel_y.max),
                (int)fabs(accel_z.min), (int)fabs(accel_z.max),
                (int)fabs(accel_mag.min), (int)fabs(accel_mag.max));
        raw_data_update(rotation_packet);
    }

    if (check_if_second_change())
    {

        accel_x.delta = accel_x.max - accel_x.min;
        accel_y.delta = accel_y.max - accel_y.min;
        accel_z.delta = accel_z.max - accel_z.min;
        accel_mag.delta = accel_mag.max - accel_mag.min;
        // reset_the_min_max_data();
        reset_acc_minmax();

        // Store and check continuously
        store_data(window_packet, accel_x.delta);
        check_pattern();

        // memset(window_packet, 0x00, sizeof(window_packet));
        // snprintf(window_packet, sizeof(window_packet), // store data
        //          "{mac:%X%X%X,time:%s,std:[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]}",
        //          gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
        //          log_window[5].x_delta,
        //          log_window[4].x_delta,
        //          log_window[3].x_delta,
        //          log_window[2].x_delta,
        //          log_window[1].x_delta,
        //          log_window[0].x_delta);

        // send data
        // uart_trasmit_str(window_packet); // 1pkt/sec
        // live_alert_update(window_packet);
    }
    // uart_trasmit_str(window_packet);  // 5pkt/sec

    // ✅ Take one fresh reading of accelerometer
    update_axis_data(&accel_x, ACC_X);
    update_axis_data(&accel_y, ACC_Y);
    update_axis_data(&accel_z, ACC_Z);
    update_magnitude_data(&accel_mag, accData.magni_mean);

    // reset_the_min_max_data();

    // // Update min/max for each axis and calculate delta
    // // update_axis_data(&accel_x, ACC_X);
    // // update_axis_data(&accel_y, ACC_Y);
    // // update_axis_data(&accel_z, ACC_Z);
    // // update_magnitude_data(&accel_mag, accData.magni_mean);

    // // Update X-axis
    // if ((int)fabs(ACC_X) < accel_x.min)
    //     accel_x.min = (int)fabs(ACC_X);

    // if ((int)fabs(ACC_X) > accel_x.max)
    //     accel_x.max = (int)fabs(ACC_X);

    // // Update Y-axis
    // if ((int)fabs(ACC_Y) < accel_y.min)
    //     accel_y.min = (int)fabs(ACC_Y);

    // if ((int)fabs(ACC_Y) > accel_y.max)
    //     accel_y.max = (int)fabs(ACC_Y);

    // // Update Z-axis
    // if ((int)fabs(ACC_Z) < accel_z.min)
    //     accel_z.min = (int)fabs(ACC_Z);

    // if ((int)fabs(ACC_Z) > accel_z.max)
    //     accel_z.max = (int)fabs(ACC_Z);

    // // Update Magnitude
    // if ((int)accData.magni_mean < accel_mag.min)
    //     accel_mag.min = (int)accData.magni_mean;

    // if ((int)accData.magni_mean > accel_mag.max)
    //     accel_mag.max = (int)accData.magni_mean;

    // accel_x.delta = accel_x.max - accel_x.min;
    // accel_y.delta = accel_y.max - accel_y.min;
    // accel_z.delta = accel_z.max - accel_z.min;
    // accel_mag.delta = accel_mag.max - accel_mag.min;

    // // memset(rotation_packet, 0x00, sizeof(rotation_packet));

    // // print the accelerometer data...s
    // // sprintf(rotation_packet, "{mac:%X%X%X,time:%s,y_delta:%0.0f}",
    // //         gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, accel_y.delta);

    // // AddLog(rotation_packet);
    // // handle_packet(rotation_packet); // rolling

    // memset(window_packet, 0x00, sizeof(window_packet));
    // snprintf(window_packet, sizeof(window_packet),
    //          "{mac:%X%X%X,time:%s,y_deltas:[%.0f,%.0f,%.0f,%.0f,%.0f]}",
    //          gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
    //          log_window[0].y_delta,
    //          log_window[1].y_delta,
    //          log_window[2].y_delta,
    //          log_window[3].y_delta,
    //          log_window[4].y_delta);

    // // Store and check continuously
    // store_data(window_packet, accel_y.delta);
    // check_pattern();

    // uart_trasmit_str(window_packet);
    // live_alert_update(window_packet);
    // uart_trasmit_str(window_packet);

    // build_window_packet(window_packet, sizeof(window_packet));
    // uart_trasmit_str(window_packet);
    // printf("Window Packet: %s\n", window_packet);

    // // Store 5 data into buffer
    // strncpy(log_window[sec_counter], rotation_packet, MAX_LOG_STR - 1);
    // log_window[sec_counter][MAX_LOG_STR - 1] = '\0'; // ensure null terminator

    // sec_counter++;

    // // After 5 seconds (5 samples)
    // if (sec_counter >= WINDOW_SIZE)
    // {

    //     for (int i = 0; i < WINDOW_SIZE; i++)
    //     {
    //         printf("records:%s\n", log_window[i]);
    //     }

    //     // process_window(); // analyze the 5 logs
    //     sec_counter = 0; // reset for next 5-sec cycle
    // }

    // if (accel_x.delta > m_device_cfg.testX_beep_th && m_device_cfg.testX_beep_th != 0)
    // {
    //     dX_flag = true;

    // sprintf(AlertStr, "{mac:%X%X%X,alert:x_del}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    // live_alert_update(AlertStr);
    // uart_trasmit_str(AlertStr);
    // }

    // if (accel_y.delta > m_device_cfg.testY_beep_th && m_device_cfg.testY_beep_th != 0)
    // {
    //     dY_flag = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:y_del}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }

    // if (accel_z.delta > m_device_cfg.testZ_beep_th && m_device_cfg.testZ_beep_th != 0)
    // {
    //     dZ_flag = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:z_del}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }

    // if (accel_mag.delta > m_device_cfg.testA_beep_th && m_device_cfg.testA_beep_th != 0)
    // {
    //     dA_flag = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:a_del}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }

    // if (dX_flag && dY_flag && dZ_flag && dA_flag)
    // {
    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:AT}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    //     // nrf_gpio_pin_set(BUZZER_PIN);
    // }

    // // Compute deltas
    // accel_x.delta = accel_x.max - accel_x.min;
    // accel_y.delta = accel_y.max - accel_y.min;
    // accel_z.delta = accel_z.max - accel_z.min;
    // accel_mag.delta = accel_mag.max - accel_mag.min;

    // int deltas[4] = {accel_x.delta, accel_y.delta, accel_z.delta, accel_mag.delta};
    // int thresholds[4] = {m_device_cfg.testX_beep_th,
    //                      m_device_cfg.testY_beep_th,
    //                      m_device_cfg.testZ_beep_th,
    //                      m_device_cfg.testA_beep_th};

    // const char *labels[4] = {"x", "y", "z", "a"};

    // for (int i = 0; i < 4; i++)
    // {
    //     if (thresholds[i] != 0 && deltas[i] > thresholds[i])
    //     {
    //         triggered = true;

    //         sprintf(AlertStr, "{mac:%X%X%X,alert:%s_del,xd:%0.0f,yd:%0.0f,zd:%0.0f,ad:%0.0f}",
    //                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], labels[i],
    //                 accel_x.delta, accel_y.delta, accel_z.delta, accel_mag.delta);
    //         live_alert_update(AlertStr);
    //         uart_trasmit_str(AlertStr);
    //         // nrf_gpio_pin_set(BUZZER_PIN);
    //     }
    // }

    // if (triggered)
    // {
    // sprintf(AlertStr, "{mac:%X%X%X,alert:del_XYZ}",
    //         gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    // live_alert_update(AlertStr);
    // uart_trasmit_str(AlertStr);
    //     nrf_gpio_pin_set(BUZZER_PIN);
    // }

    // if (triggered)
    // {
    //     nrf_gpio_pin_set(BUZZER_PIN);
    //     test_beep_cnt++;
    //     if (test_beep_cnt >= 50)
    //     {
    //         test_beep_cnt = 0;
    //         triggered = false;
    //         nrf_gpio_pin_clear(BUZZER_PIN);
    //     }
    //     printf("test_beep_cnt:%d\n", test_beep_cnt);
    // }

    // if (accel_x.delta < m_device_cfg.x_axis_lt)
    // {

    // }
    // if (accel_x.delta > m_device_cfg.x_axis_gt)
    // {

    // }
    // if (accel_y.delta < m_device_cfg.y_axis_lt)
    // {

    // }
    // if (accel_y.delta < m_device_cfg.y_axis_gt)
    // {
    // }
    // if (accel_z.delta < m_device_cfg.z_axis_lt)
    // {
    // }
    // if (accel_z.delta < m_device_cfg.z_axis_gt)
    // {
    // }
    // if (accel_mag.delta < m_device_cfg.xyz_avg_lt)
    // {
    // }
    // if (accel_mag.delta < m_device_cfg.xyz_avg_gt)
    // {
    // }

    // if (accel_x.min < m_device_cfg.x_axis_lt && m_device_cfg.x_axis_lt != 0)
    // {
    //     // printf("x_less than x_min\n");
    //     x_min = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:x_min}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     x_min = false;
    // }
    // if (accel_x.max > m_device_cfg.x_axis_gt && m_device_cfg.x_axis_gt != 0)
    // {
    //     // printf("x_greater than x_max\n");
    //     x_max = true;

    //     //     sprintf(AlertStr, "{mac:%X%X%X,alert:x_max}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     //     live_alert_update(AlertStr);
    //     //     uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     x_max = false;
    // }
    // if (accel_y.min < m_device_cfg.y_axis_lt && m_device_cfg.y_axis_lt != 0)
    // {
    //     // printf("y_less than y_min\n");
    //     y_min = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:y_min}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     y_min = false;
    // }
    // if (accel_y.max > m_device_cfg.y_axis_gt && m_device_cfg.y_axis_gt != 0)
    // {
    //     // printf("y_greater than y_max\n");
    //     y_max = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:y_max}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     y_max = false;
    // }
    // if (accel_z.min < m_device_cfg.z_axis_lt && m_device_cfg.z_axis_lt != 0)
    // {
    //     // printf("z_less than z_min\n");
    //     z_min = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:z_min}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     z_min = false;
    // }
    // if (accel_z.max > m_device_cfg.z_axis_gt && m_device_cfg.z_axis_gt != 0)
    // {
    //     // printf("z_greater than z_max\n");
    //     z_max = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:z_max}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     z_max = false;
    // }
    // if (accel_mag.min < m_device_cfg.xyz_avg_lt && m_device_cfg.xyz_avg_lt != 0)
    // {
    //     // printf("magni_mean less than mag_min\n");
    //     net_min = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:net_min}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     net_min = false;
    // }
    // if (accel_mag.max > m_device_cfg.xyz_avg_gt && m_device_cfg.xyz_avg_gt != 0)
    // {
    //     // printf("magni_mean greater than mag_max\n");
    //     net_max = true;

    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:net_max}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    // }
    // else
    // {
    //     net_max = false;
    // }

    // if (x_min && x_max && y_min && y_max && z_min && z_max && net_min && net_max)
    // {
    //     // sprintf(AlertStr, "{mac:%X%X%X,alert:ACT}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     // live_alert_update(AlertStr);
    //     // uart_trasmit_str(AlertStr);
    //     // nrf_gpio_pin_set(BUZZER_PIN);
    // }

    // if (x_min || x_max || y_min || y_max || z_min || z_max || net_min || net_max)
    // {
    //     sprintf(AlertStr, "{mac:%X%X%X,alert:BT}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     live_alert_update(AlertStr);
    //     uart_trasmit_str(AlertStr);
    //     nrf_gpio_pin_set(BUZZER_PIN);
    // }

    // if (m_device_cfg.test_beep_th != 0 &&
    //     (accel_x.min < m_device_cfg.test_beep_th ||
    //      accel_x.max > m_device_cfg.test_beep_th ||
    //      accel_y.min < m_device_cfg.test_beep_th ||
    //      accel_y.max > m_device_cfg.test_beep_th ||
    //      accel_z.min < m_device_cfg.test_beep_th ||
    //      accel_z.max > m_device_cfg.test_beep_th ||
    //      accel_mag.min < m_device_cfg.test_beep_th ||
    //      accel_mag.max > m_device_cfg.test_beep_th))
    // {
    //     sprintf(AlertStr, "{mac:%X%X%X,alert:BA}",
    //             gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    //     live_alert_update(AlertStr);
    //     uart_trasmit_str(AlertStr);
    //     nrf_gpio_pin_set(BUZZER_PIN);
    // }

    // else
    // {
    //     memset(rotation_packet, 0x00, sizeof(rotation_packet));
    //     sprintf(rotation_packet, "{time:%s,x:%3.02f,y:%3.02f,z:%3.02f}", timespm, ACC_X, ACC_Y, ACC_Z);
    //     printf("raw data: %s\n", rotation_packet);
    // }

    // if (pwrSense.inputPwr <= 3.0)
    if (batt_adv_data <= m_device_cfg.battery_perc_value)
    {
        battery_alert_cnt++;
        if (battery_alert_cnt > 15)
        {
            battery_alert_cnt = 0;
            // nrf_gpio_pin_clear(BATTERY_ALERT_PIN); // blue led on
            // AccIdleProcess();
            // uint32_t pwrconn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            // printf("<<--- pwrlow_disconnceted: %d links. --->>\n", pwrconn_count);
            printf("<<----- BATTERY LOW! PLEASE RECHARGE THE BATTERY ----->>\n");

            memset(temp2, 0, sizeof(temp2));
            sprintf(temp2, "{mac:%X%X%X,time:%s,alert:BL}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
            if (conn_flag)
            {
                live_alert_update(temp2);
            }
            if (!his_notify)
            {
                AddLog(temp2);
            }
            uart_trasmit_str(temp2);

            btr_pwrdn = true;

            sprintf(adv_data, "%d%d", hook_conn, batt_adv_data);
            // update_advertising_data(adv_data);
        }
    }
    // led_brightness_control();
    // fade_led();
}
/**
 * @brief Handler for motor timer events.
 */
void motor_timer_handler()
{
    if (m_device_cfg.motorclk_flag == 1)
    {
        nrf_gpio_pin_set(MOTOR_PIN1);   // MOTOR_PIN1_HIGH
        nrf_gpio_pin_clear(MOTOR_PIN2); // MOTOR_PIN2_LOW

        // motor_clk_cnt++;
        motor_clk_cnt = motor_clk_cnt + 100;

        if (motor_clk_cnt >= m_device_cfg.mclk_en)
        {
            motor_clk_cnt = 0;
            nrf_gpio_pin_clear(MOTOR_PIN1); // MOTOR_PIN1 Low
            nrf_gpio_pin_clear(MOTOR_PIN2); // MOTOR_PIN2 Low
            motor_reset();
            printf("motor cl stopped\n");
        }
        printf("motor_clkwisecounter:%d\n", motor_clk_cnt);
    }

    else if (m_device_cfg.motoraclk_flag == 1)
    {
        nrf_gpio_pin_set(MOTOR_PIN2);   // MOTOR_PIN2_HIGH
        nrf_gpio_pin_clear(MOTOR_PIN1); // MOTOR_PIN1_LOW

        // motor_antclk_cnt++;
        motor_antclk_cnt = motor_antclk_cnt + 100;

        if (motor_antclk_cnt >= m_device_cfg.manc_en)
        {
            // m_device_cfg.motoraclk_flag = 0;
            motor_antclk_cnt = 0;
            nrf_gpio_pin_clear(MOTOR_PIN1); // MOTOR_PIN1 Low
            nrf_gpio_pin_clear(MOTOR_PIN2); // MOTOR_PIN2 Low
            motor_reset();
            printf("motor anc stopped\n");
        }
        printf("motor_anticlkwisecounter:%d\n", motor_antclk_cnt);
    }
    else
    {
        motor_reset();
    }

    // if (scan_hook_stats && m_device_cfg.hook_mode == 0)
    // {
    // sen_stat_detect = true;
    // scan_hook_stats = false;
    // buz_en = 1;
    // buz_hmcmd_ex = true;
    // m_device_cfg.hook_mode = 1;
    // m_device_cfg.buzzer_onoff_enable = 1;
    // nrf_gpio_pin_set(MOTOR_PIN2); // MOTOR_PIN2_HIGH
    // nrf_gpio_pin_set(BUZZER_PIN); // BUZZER PIN
    // }
}
/**
 * @brief Function for initializing the accelerometer.
 * @return NRF_SUCCESS on successful initialization, otherwise an error code.
 */
ret_code_t acc_init(void)
{

    // pwm_init();
    // this is for accelerometer intialization

    ret_code_t err_code;
    lsm6dsl_int1_route_t int_1_reg;
    int i = 0, j = 0, temp_val = 0;
    char temp_char;
    bit_0 = m_device_cfg.th_status & 0x01; // fall arrest
    bit_1 = m_device_cfg.th_status & 0x02; // test
    bit_2 = m_device_cfg.th_status & 0x04; // free fall

    /* changes */

    bit_3 = m_device_cfg.th_status & 0x08; // th1
    bit_4 = m_device_cfg.th_status & 0x10; // th2
    bit_5 = m_device_cfg.th_status & 0x20; // th3
    array_size = 0;

    // printf("bit_0: %d, bit_1: %d, bit_2: %d, bit_3: %d, bit_4: %d, bit_5: %d, th_status: %d\n",
    //        bit_0, bit_1, bit_2, bit_3, bit_4, bit_5, m_device_cfg.th_status);

    if (bit_0 == 1)
    {
        array_threshold[array_size] = m_device_cfg.fall_threshold;
        // printf("fall value set: %d\n", m_device_cfg.fall_threshold);
        array_AlertCh[array_size] = 'A';
        NRF_LOG_INFO("bit_0 array_threshold[%d]: %d\n", array_size, array_threshold[array_size]);
        // printf("bit_0 array_threshold[%d]: %d\n", array_size,array_threshold[array_size]);
        array_size += 1;
    }
    if (bit_1 == 2)
    {
        array_threshold[array_size] = m_device_cfg.test_threshold;
        // printf("test value set: %d\n", m_device_cfg.test_threshold);
        array_AlertCh[array_size] = 'T';
        NRF_LOG_INFO("bit_1 array_threshold[%d]: %d\n", array_size, array_threshold[array_size]);
        // printf("bit_1 array_threshold[%d]: %d\n", array_size,array_threshold[array_size]);
        array_size += 1;
    }
    if (bit_2 == 4)
    {

        array_threshold[array_size] = m_device_cfg.ff_threshold;
        // printf("ff value set: %d\n", m_device_cfg.ff_threshold);
        array_AlertCh[array_size] = 'F';
        NRF_LOG_INFO("bit_2 array_threshold[%d]: %d\n", array_size, array_threshold[array_size]);
        // printf("bit_2 array_threshold[%d]: %d\n",array_size, array_threshold[array_size]);
        array_size += 1;
        FF_EN = true;
    }

    /* changes */

    if (bit_3 == 8)
    {
        printf("bit_3 is ON\n");
        array_threshold[array_size] = m_device_cfg.th1_threshold;
        // printf("th1 value set: %d\n", m_device_cfg.th1_threshold);
        array_AlertCh[array_size] = 'X';
        NRF_LOG_INFO("bit_3 array_threshold[%d]: %d\n", array_size, array_threshold[array_size]);
        // printf( "bit_3 array_threshold[%d]: %d\n",array_size, array_threshold[array_size]);
        array_size += 1;
    }
    if (bit_4 == 16)
    {
        // printf("bit_4 is ON\n");
        array_threshold[array_size] = m_device_cfg.th2_threshold;
        // printf("th2 value set: %d\n", m_device_cfg.th2_threshold);
        array_AlertCh[array_size] = 'Y';
        NRF_LOG_INFO("bit_4 array_threshold[%d]: %d\n", array_size, array_threshold[array_size]);
        // printf( "bit_4 array_threshold[%d]: %d\n",array_size, array_threshold[array_size]);
        array_size += 1;
    }
    if (bit_5 == 32)
    {
        // printf("bit_5 is ON\n");
        array_threshold[array_size] = m_device_cfg.th3_threshold;
        // printf("th3 value set: %d\n", m_device_cfg.th3_threshold);
        array_AlertCh[array_size] = 'Z';
        NRF_LOG_INFO("bit_5 array_threshold[%d]: %d\n", array_size, array_threshold[array_size]);
        // printf( "bit_5 array_threshold[%d]: %d\n",array_size, array_threshold[array_size]);
        array_size += 1;
    }

    NRF_LOG_INFO("array_size = %d\r\n", array_size);
    for (i = 0; i < array_size; ++i)
    {
        for (j = i + 1; j < array_size; ++j)
        {
            if (array_threshold[i] > array_threshold[j])
            {
                temp_char = array_AlertCh[i];
                temp_val = array_threshold[i];
                array_AlertCh[i] = array_AlertCh[j];
                array_threshold[i] = array_threshold[j];
                array_AlertCh[j] = temp_char;
                array_threshold[j] = temp_val;
            }
        }
    }
    for (i = 0; i < array_size; ++i)
    {
        NRF_LOG_INFO("array_threshold[%d]: %d  array_AlertCh[%d]: %c \n", i, array_threshold[i], i, array_AlertCh[i]);
    }
    dev_ctx.read_reg = platform_read;
    dev_ctx.write_reg = platform_write;

    twi_init();

    err_code = app_timer_create(&TIMER_ACC, APP_TIMER_MODE_REPEATED, timer_acc_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&TIMER_HEALTH, APP_TIMER_MODE_REPEATED, timer_health_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(TIMER_HEALTH, APP_TIMER_TICKS(HOUR_IN_DAY * MINUTE_IN_HOUR * 60 * 1000), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&TIMER_1SEC, APP_TIMER_MODE_REPEATED, timer_1sec_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&TIMER_5MIT, APP_TIMER_MODE_REPEATED, timer_5mit_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&MOTOR_TIMER, APP_TIMER_MODE_REPEATED, motor_timer_handler);
    APP_ERROR_CHECK(err_code);

    if (m_device_cfg.cal_mode == 0)
    {
        err_code = app_timer_start(TIMER_1SEC, APP_TIMER_TICKS(980), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(TIMER_ACC, APP_TIMER_TICKS(200), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(TIMER_5MIT, APP_TIMER_TICKS(1000), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_create(&TIMER_EVENT_USER, APP_TIMER_MODE_REPEATED, event_user_timer_handler);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(TIMER_EVENT_USER, APP_TIMER_TICKS(30), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(MOTOR_TIMER, APP_TIMER_TICKS(100), NULL);
        APP_ERROR_CHECK(err_code);
    }

    nrf_delay_ms(20);

    err_code = lsm6dsl_device_id_get(&dev_ctx, (uint8_t *)&whoamI);
    if (whoamI[0] != LSM6DSL_ID)
    {
        NRF_LOG_ERROR("Device Not found %x\r\n", whoamI[0]);
        return err_code;
    }
    NRF_LOG_INFO("Device found %x\r\n", whoamI[0]);

    /* Restore default configuration */
    lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6dsl_reset_get(&dev_ctx, &rst);
    } while (rst);

    /*
     *  Enable Block Data Update
     */
    lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /*
     * Set full scale
     */
    err_code = lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_4g);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Full scale xl Failed\r\n");
        return err_code;
    }

    /*
     * Set Output Data Rate for Acc and Gyro
     */
    if (m_device_cfg.cal_mode == 0)
    {
        err_code = lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_208Hz);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Data rate xl Failed\r\n");
            return err_code;
        }
        /*
         * Configure filtering chain(No aux interface)
         */
        /* Accelerometer - analog filter */
        lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
        lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9);
        lsm6dsl_auto_increment_set(&dev_ctx, 1);
        /* Gyroscope - filtering chain */
        lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);

        /*
         * Configure buzzer and sensor
         */
        nrf_gpio_cfg_input(Hook_Sensor, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_output(BUZZER_PIN);
        nrf_gpio_cfg_output(BUZZER_LED);
        nrf_gpio_cfg_output(BATTERY_ALERT_PIN);
        nrf_gpio_pin_set(BUZZER_LED);        // Buzzer LDE off
        nrf_gpio_pin_set(BATTERY_ALERT_PIN); // Blue LDE off
        nrf_gpio_cfg_output(MOTOR_PIN1);
        nrf_gpio_cfg_output(MOTOR_PIN2);
        nrf_gpio_pin_clear(MOTOR_PIN1); // MOTOR_PIN1 Low
        nrf_gpio_pin_clear(MOTOR_PIN2); // MOTOR_PIN2 Low

        // sprintf(adv_data, "%d%d%d", hook_conn, batt_adv_data, m_device_cfg.hook_mode);
        // update_advertising_data(adv_data);
        // printf("init adv_data:%s\n", adv_data);

        if (m_device_cfg.hook_mode == 1)
        {
            NRF_LOG_INFO("init hook_mode: ON");
            printf("init hook_mode: ON\n");
            nrf_gpio_pin_clear(BUZZER_PIN);
            buz_en = 1;
            FTCT = 0;
            buz_cnt = 0;
            recunt = 2;
            m_device_cfg.buzzer_onoff_enable = 0;
            m_device_cfg.hm_startflag = 1;
        }
        if (m_device_cfg.hook_mode == 0)
        {
            NRF_LOG_INFO("init hook_mode: OFF");
            printf("init hook_mode: OFF\n");
            nrf_gpio_pin_clear(BUZZER_PIN);
            buz_en = 0;
        }
        NRF_LOG_INFO("*--buzzer_onoff_enable %d", m_device_cfg.buzzer_onoff_enable);
    }
    else
    {
        err_code = lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Data rate xl Failed\r\n");
            return err_code;
        }
    }

    lsm6dsl_int_notification_set(&dev_ctx, LSM6DSL_INT_PULSED);

    hc_cnt = 0;
    hd_cnt = 0;

    // for gyroscope...
    if (m_device_cfg.cal_mode == 0)
    {
        nrf_delay_ms(500);
        lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
        lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_208Hz);
    }

    Last_RotationF = RotationF;
    Last_RotationR = RotationR;

    lsm6dsl_ff_dur_set(&dev_ctx, 1);
    lsm6dsl_motion_sens_set(&dev_ctx, PROPERTY_ENABLE);

    uint16_t sm_th = {m_device_cfg.wake_threshold};
    uint16_t hmstop_test = {m_device_cfg.hm_stopth};
    lsm6dsl_ff_dur_set(&dev_ctx, 1);
    lsm6dsl_ff_threshold_set(&dev_ctx, sm_th);

    NRF_LOG_INFO("sm_th : %d", sm_th);

    err_code = lsm6dsl_pin_int1_route_get(&dev_ctx, &int_1_reg);
    int_1_reg.int1_drdy_xl = PROPERTY_ENABLE;
    int_1_reg.int1_ff = PROPERTY_DISABLE;
    int_1_reg.int1_sign_mot = PROPERTY_DISABLE;
    err_code = lsm6dsl_pin_int1_route_set(&dev_ctx, int_1_reg);
    lsm6dsl_6d_threshold_set(&dev_ctx, LSM6DSL_DEG_50);
    nrf_delay_ms(50);

    uint8_t data_ready = 0;
    lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &data_ready);

    if (m_device_cfg.cal_mode == 0)
    {
        if (data_ready)
        {
            /*
             * Read acceleration field data
             */
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

            AccXangle = (float)(atan2(data_raw_acceleration.i16bit[1], data_raw_acceleration.i16bit[2]) + M_PI) * RAD_TO_DEG;
            AccYangle = (float)(atan2(data_raw_acceleration.i16bit[2], data_raw_acceleration.i16bit[0]) + M_PI) * RAD_TO_DEG;
            AccZangle = (float)(atan2(data_raw_acceleration.i16bit[1], data_raw_acceleration.i16bit[0]) + M_PI) * RAD_TO_DEG;
            TAccYangle = (float)atan2(-data_raw_acceleration.i16bit[0], sqrt(data_raw_acceleration.i16bit[1] * data_raw_acceleration.i16bit[1] + data_raw_acceleration.i16bit[2] * data_raw_acceleration.i16bit[2])) * RAD_TO_DEG;
            lastangle = AccZangle;
            NRF_LOG_INFO("lastangle : %d", lastangle);
        }
    }

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    err_code = nrf_drv_gpiote_in_init(LSM6DSL_INT1_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(LSM6DSL_INT1_PIN, true);

    return err_code;
}
/**
 * @brief  Deactivate accelerometer process
 * Put the accelerometer into idle mode to save power
 * 
 */
void AccIdleProcess(void)
{
    in_lp_mode = true;
    lsm6dsl_int1_route_t int_1_reg;
    ret_code_t err_code;
    app_timer_stop(TIMER_1SEC);
    app_timer_stop(TIMER_ACC);
    app_timer_stop(TIMER_5MIT);
    app_timer_stop(TIMER_EVENT_USER);
    nrf_gpio_pin_clear(BUZZER_PIN); // Buzzer off
    nrf_gpio_pin_set(BUZZER_LED);   // Buzzer LDE off
    err_code = lsm6dsl_pin_int1_route_get(&dev_ctx, &int_1_reg);
    if (err_code != NRF_SUCCESS) // Or use another appropriate error macro
    {
        printf("lsm6dsl_pin_int1_route_get Error: %ld\n", err_code);
        return;
    }

    int_1_reg.int1_drdy_xl = PROPERTY_DISABLE;
    err_code = lsm6dsl_pin_int1_route_set(&dev_ctx, int_1_reg);

    if (err_code != NRF_SUCCESS) // Or use another appropriate error macro
    {
        printf("lsm6dsl_pin_int1_route_set Error: %ld\n", err_code);
        return;
    }

    lsm6dsl_wkup_dur_set(&dev_ctx, 0x02);
    /* Set duration for Inactivity detection */

    // /* Set Activity/Inactivity threshold to 62.5 mg */
    lsm6dsl_wkup_threshold_set(&dev_ctx, 0x02);
    /* Inactivity configuration: acc to 12.5 LP, gyro to Power-Down */
    lsm6dsl_act_mode_set(&dev_ctx, LSM6DSL_XL_12Hz5_GY_PD);
    /* Enable interrupt generation on Inactivity INT1 pin */

    lsm6dsl_pin_int1_route_get(&dev_ctx, &int_1_reg);
    int_1_reg.int1_inact_state = PROPERTY_ENABLE;
    lsm6dsl_pin_int1_route_set(&dev_ctx, int_1_reg);

    lsm6dsl_6d_threshold_set(&dev_ctx, LSM6DSL_DEG_50);
    err_code = lsm6dsl_pin_int1_route_get(&dev_ctx, &int_1_reg);
    int_1_reg.int1_6d = PROPERTY_ENABLE;
    int_1_reg.int1_ff = PROPERTY_ENABLE;
    err_code = lsm6dsl_pin_int1_route_set(&dev_ctx, int_1_reg);
    // DisableBattLevel();
    nrf_delay_ms(1000);
}
/**
 * @brief  Activate accelerometer process
 * Reactivate the accelerometer from idle mode
 */
void AccActiveProcess(void)
{
    lsm6dsl_int1_route_t int_1_reg;
    ret_code_t err_code;
    lsm6dsl_wkup_threshold_set(&dev_ctx, 0x00);

    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_208Hz);
    lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
    lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9);
    lsm6dsl_auto_increment_set(&dev_ctx, 1);

    lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_208Hz);

    err_code = lsm6dsl_pin_int1_route_get(&dev_ctx, &int_1_reg);
    int_1_reg.int1_inact_state = PROPERTY_DISABLE;
    int_1_reg.int1_drdy_xl = PROPERTY_ENABLE;
    int_1_reg.int1_6d = PROPERTY_DISABLE;
    int_1_reg.int1_ff = PROPERTY_DISABLE;
    int_1_reg.int1_sign_mot = PROPERTY_DISABLE;
    err_code = lsm6dsl_pin_int1_route_set(&dev_ctx, int_1_reg);

    if (m_device_cfg.cal_mode == 0)
    {
        err_code = app_timer_start(TIMER_1SEC, APP_TIMER_TICKS(990), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(TIMER_ACC, APP_TIMER_TICKS(200), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(TIMER_5MIT, APP_TIMER_TICKS(1000), NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(TIMER_EVENT_USER, APP_TIMER_TICKS(30), NULL);
        APP_ERROR_CHECK(err_code);
    }
    getBattLevel();
}

/**
 * @brief  Read temperature data
 * Read temperature data register 
 * Read the temperature data from the LSM6DSL sensor
 * and convert it to Celsius 
 */
void temperature_dataread(void)
{
    uint8_t buf[2];
    int16_t raw_temperature;

    int32_t ret = platform_read(NULL, 0x20, buf, 2);

    if (ret != NRF_SUCCESS)
    {
        printf("Error reading temperature data: 0x%X\n", ret);
        return;
    }

    // Combine bytes (signed)
    raw_temperature = (int16_t)((buf[1] << 8) | buf[0]);

    // Convert to Celsius
    temperature_celsius = lsm6dsl_from_lsb_to_celsius(raw_temperature);

    // Debug (optional)
    //  printf("Temperature: %.2f °C\n", temperature_celsius);
}
/**
 * @brief  Update step count based on accelerometer magnitude
 * Implements step detection logic with hysteresis
 */
static void update_step_count(int acc_magnitude)
{
    static char alert_buffer[10]; // Adjust size as needed

    // Step detection logic with hysteresis
    if ((acc_magnitude > step_threshold) && (step_flag == 0))
    {
        step_count++;
        step_flag = 1; // Mark step detected
        printf("Steps: %d\n", step_count);
        sprintf(alert_buffer, "{steps:%d}", step_count);
        // step_data_update(alert_buffer);
        // nrf_delay_ms(500);
    }
    else if (acc_magnitude < (step_threshold - step_hysteresis))
    {
        step_flag = 0; // Reset for next step
    }
    // printf("Steps: %d\n", step_count);
}
/**
 * @brief  Read gyroscope data
 * Read gyroscope data register 
 * Read the gyroscope data from the LSM6DSL sensor
 * and convert it to mdps 
 */
void acc_gyro_data()
{
    ret_code_t err_code;
    static uint8_t GYRO_MODE = 0;

    switch (GYRO_MODE)
    {
    case 0: // Set full scale
        err_code = lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_500dps);
        if (err_code != 0)
        {
            printf("Full scale gyro Failed\r\n");
            return;
        }
        GYRO_MODE++;
        break; // ✅ FIX: Prevents fall-through

    case 1: // Set data rate
        err_code = lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_208Hz);
        if (err_code != 0)
        {
            printf("Data rate gyro Failed\r\n");
            return;
        }
        GYRO_MODE++;
        break;

    case 2: // Set analog filter
        err_code = lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
        if (err_code != 0)
        {
            printf("Filter analog set Failed\r\n");
            return;
        }
        GYRO_MODE++;
        break;

    case 3: // Set Bandwidth
        err_code = lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9);
        if (err_code != 0)
        {
            printf("Bandwidth set Failed\r\n");
            return;
        }
        GYRO_MODE++;
        break;

    case 4: // Set Auto Increment
        err_code = lsm6dsl_auto_increment_set(&dev_ctx, 1);
        if (err_code != 0)
        {
            printf("Auto Increment set Failed\r\n");
            return;
        }
        GYRO_MODE++;
        break;

    case 5: // Set Notification alert type
        err_code = lsm6dsl_int_notification_set(&dev_ctx, LSM6DSL_INT_PULSED);
        if (err_code != 0)
        {
            printf("Notification set Failed\r\n");
            return;
        }
        GYRO_MODE++;
        break;

    case 6: // Check Acceleration Data Ready Or Not
        err_code = lsm6dsl_gy_flag_data_ready_get(&dev_ctx, &accData.GyData_ready);
        if (err_code != 0)
        {
            printf("Gyro Data Not Ready\r\n");
            return;
        }
        // printf("Gyroscope data is ready!\n");
        GYRO_MODE++;
        break;

    case 7: // Convert Raw Value To Mean
        if (accData.GyData_ready)
        {
            memset(data_raw_gyro.u8bit, 0x00, sizeof(data_raw_gyro)); // ✅ FIX: Properly clear

            err_code = lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_gyro.i16bit);
            if (err_code != 0)
            {
                printf("Gyro Data unable to read\r\n");
                return;
            }

            // Print raw values
            // printf("Raw Gyroscope Data: X=%d, Y=%d, Z=%d\n",
            //        data_raw_gyro.i16bit[0], data_raw_gyro.i16bit[1], data_raw_gyro.i16bit[2]);

            accData.GY_X = lsm6dsl_from_fs500dps_to_mdps(data_raw_gyro.i16bit[0]);
            accData.GY_Y = lsm6dsl_from_fs500dps_to_mdps(data_raw_gyro.i16bit[1]);
            accData.GY_Z = lsm6dsl_from_fs500dps_to_mdps(data_raw_gyro.i16bit[2]);

            // printf("GX: %.2f, GY: %.2f, GZ: %.2f\n", accData.GY_X, accData.GY_Y, accData.GY_Z);
        }
        break; // ✅ FIX: Prevents unintended execution of future cases

        // default:
        //     printf("Unknown GYRO_MODE: %d\n", GYRO_MODE);
        //     break;
    }
}
/**
 * @brief  Read accelerometer data
 * Read accelerometer data register 
 * Read the accelerometer data from the LSM6DSL sensor
 * and convert it to mg
 */
void getAccData(void)
{

    // in this function we are getting the accelerometer data/values
    uint8_t data_ready = 0;
    // uint8_t data_ready1 = 0;
    // double mean;
    // ret_code_t err_code;
    lsm6dsl_all_sources_t all_sources;
    lsm6dsl_all_sources_get(&dev_ctx, &all_sources);
    // memset(temp2, 0, sizeof(temp2));

    if (all_sources.d6d_src.d6d_ia)
    {
        NRF_LOG_INFO("Motion Detected");
        AccActiveProcess();
        ble_adv_start(false);
        lastangle = AccZangle;
        blescan_start();

        memset(AlertStr, 0, sizeof(AlertStr));
        sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:DW}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        // AddLog(AlertStr);
        // uart_trasmit_str(AlertStr);
        // hook_test_ok = false;
        // m_device_cfg.zl_flag_start = 1;
        // m_device_cfg.xg_flag_start = 1;
    }
    else if (all_sources.wake_up_src.wu_ia)
    {
        NRF_LOG_INFO("FreeFall Detected");
        AccActiveProcess();
        ble_adv_start(false);
        blescan_start();

        memset(AlertStr, 0, sizeof(AlertStr));
        sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:DW}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        // AddLog(AlertStr);
        // uart_trasmit_str(AlertStr);
        // hook_test_ok = false;
        // m_device_cfg.zl_flag_start = 1;
        // m_device_cfg.xg_flag_start = 1;
    }

    // getAccGyroData();
    acc_gyro_data();

    lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &data_ready);
    if (data_ready)
    {
        /*
         * Read acceleration field data
         */
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

        if (m_device_cfg.cal_mode == 0)
        {
            AccXangle = (float)(atan2(data_raw_acceleration.i16bit[1], data_raw_acceleration.i16bit[2]) + M_PI) * RAD_TO_DEG;
            AccYangle = (float)(atan2(data_raw_acceleration.i16bit[2], data_raw_acceleration.i16bit[0]) + M_PI) * RAD_TO_DEG;
            AccZangle = (float)(atan2(data_raw_acceleration.i16bit[1], data_raw_acceleration.i16bit[0]) + M_PI) * RAD_TO_DEG;
            TAccYangle = (float)atan2(-data_raw_acceleration.i16bit[0], sqrt(data_raw_acceleration.i16bit[1] * data_raw_acceleration.i16bit[1] + data_raw_acceleration.i16bit[2] * data_raw_acceleration.i16bit[2])) * RAD_TO_DEG;
            float inclination = 180.0 * acos(data_raw_acceleration.i16bit[2] / sqrt(data_raw_acceleration.i16bit[0] * data_raw_acceleration.i16bit[0] + data_raw_acceleration.i16bit[1] * data_raw_acceleration.i16bit[1] + data_raw_acceleration.i16bit[2] * data_raw_acceleration.i16bit[2])) / M_PI;
            if (TAccYangle < 0)
            {
                TAccYangle += 360.0f;
            }
            // printf("AccXangle: %f, AccYangle: %f, AccZangle: %f\n", AccXangle, AccYangle, AccZangle);
        }

        ACC_X = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
        ACC_Y = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
        ACC_Z = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

        // process_direction(ACC_X, ACC_Y, ACC_Z);

        // if (gyr_noti_flag)
        // {
        //     memset(rotation_packet, 0x00, sizeof(rotation_packet));
        //     // sprintf(rotation_packet, "{time:%s,x:%3.02f,y:%3.02f,z:%3.02f}", timespm, ACC_X, ACC_Y, ACC_Z);
        //     sprintf(rotation_packet, "{mac:%X%X%X,time:%s,x:%d,y:%d,z:%d,m:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, (int)fabs(ACC_X), (int)fabs(ACC_Y), (int)fabs(ACC_Z), (int)accData.magni_mean);
        //     raw_data_update(rotation_packet);
        // }
        // printf("ACC_X: %.2f, ACC_Y: %.2f, ACC_Z: %.2f\n", ACC_X, ACC_Y, ACC_Z);

        // Calculate acceleration magnitude
        // float acc_magnitude = sqrt(ACC_X * ACC_X + ACC_Y * ACC_Y + ACC_Z * ACC_Z);

        // if (m_device_cfg.step_en == 1)
        // {
        //     // Update step count logic
        //     update_step_count(acc_magnitude);
        // }

        /*
        Tilt alert according to angle ACC_X
        */

        if (bit1_on == true)
        {
            if (m_device_cfg.th1_threshold >= 0)
            {
                if (AccXangle > m_device_cfg.th1_threshold)
                {
                    //    printf("Alert: Threshold 1 (X-axis) exceeded!\n");
                    nrf_gpio_pin_set(BUZZER_PIN);
                }
                else
                {
                    nrf_gpio_pin_clear(BUZZER_PIN);
                }
            }
            else
            {
                if (AccXangle <= m_device_cfg.th1_threshold)
                {
                    // printf("Alert: Threshold 1 (X-axis) exceeded!\n");
                    nrf_gpio_pin_set(BUZZER_PIN);
                }
                else
                {
                    nrf_gpio_pin_clear(BUZZER_PIN);
                }
            }
        }
        // done & working for x-axis

        /*
        Tilt alert according to angle ACC_Y
        */

        if (bit2_on == true)
        {
            if (m_device_cfg.th2_threshold >= 0)
            {
                if (AccYangle > m_device_cfg.th2_threshold)
                {
                    //    printf("Alert: Threshold 2 (Y-axis) exceeded!\n");
                    nrf_gpio_pin_set(BUZZER_PIN);
                }
                else
                {
                    nrf_gpio_pin_clear(BUZZER_PIN);
                }
            }
            else
            {
                if (AccYangle <= m_device_cfg.th2_threshold)
                {
                    // printf("Alert: Threshold 2 (Y-axis) exceeded!\n");
                    nrf_gpio_pin_set(BUZZER_PIN);
                }
                else
                {
                    nrf_gpio_pin_clear(BUZZER_PIN);
                }
            }
        }
        // done & working for y-axis

        /*
        Tilt alert according to angle ACC_X
        */

        if (bit3_on == true)
        {
            if (m_device_cfg.th3_threshold >= 0)
            {
                if (AccZangle > m_device_cfg.th3_threshold)
                {
                    //    printf("Alert: Threshold 3 (Z-axis) exceeded!\n");
                    nrf_gpio_pin_set(BUZZER_PIN);
                }
                else
                {
                    nrf_gpio_pin_clear(BUZZER_PIN);
                }
            }
            else
            {
                if (AccZangle <= m_device_cfg.th3_threshold)
                {
                    // printf("Alert: Threshold 3 (Z-axis) exceeded!\n");
                    nrf_gpio_pin_set(BUZZER_PIN);
                }
                else
                {
                    nrf_gpio_pin_clear(BUZZER_PIN);
                }
            }
        }

        /*
        Tilt alert over here
        */

        /* changes */

        accData.pitch = atan(ACC_X / (sqrt(ACC_Y * ACC_Y + ACC_Z * ACC_Z)));
        accData.roll = atan(ACC_Y / (sqrt(ACC_X * ACC_X + ACC_Z * ACC_Z)));
        accData.yaw = atan(ACC_Z / (sqrt(ACC_Y * ACC_Y + ACC_X * ACC_X)));

        // accData.mean = sqrt(pow(ACC_X, 2) + pow(ACC_Y, 2) + pow(ACC_Z, 2));
        // printf("the mean val is : %0.02f\n", accData.mean);

        if ((ACC_Y * ACC_Y + ACC_Z * ACC_Z) != 0)
        {
            accData.pitch = atan(ACC_X / sqrt(ACC_Y * ACC_Y + ACC_Z * ACC_Z));
        }
        if ((ACC_X * ACC_X + ACC_Z * ACC_Z) != 0)
        {
            accData.roll = atan(ACC_Y / sqrt(ACC_X * ACC_X + ACC_Z * ACC_Z));
        }
        if ((ACC_Y * ACC_Y + ACC_X * ACC_X) != 0)
        {
            accData.yaw = atan(ACC_Z / sqrt(ACC_Y * ACC_Y + ACC_X * ACC_X));
        }

        float highest = fmax(fmax(fabs(ACC_X), fabs(ACC_Y)), fabs(ACC_Z));
        accData.mean = highest; // Replaced with the highest axis value

        // Calculate acceleration magnitude
        accData.magni_mean = sqrt(ACC_X * ACC_X + ACC_Y * ACC_Y + ACC_Z * ACC_Z);

        // float acc_magnitude = sqrt(ACC_X * ACC_X + ACC_Y * ACC_Y + ACC_Z * ACC_Z);

        if (m_device_cfg.step_en == 1)
        {
            // Update step count logic
            update_step_count(accData.magni_mean);
        }

        if (m_device_cfg.cal_mode == 0)
        {
            if (accData.mean > mean_HIGH)
            {
                mean_HIGH = accData.mean;
            }
            else if (accData.mean < mean_LOW)
            {
                mean_LOW = accData.mean;
            }
        }

        if (m_device_cfg.cal_mode == 1)
        {
            sprintf(temp1, "Mean=%0.02f", accData.mean);
            sendMean(temp1);
        }
#if 0   
        sprintf(temp,"xa=%0.02f;ya=%0.02f;za=%0.02f;mean=%0.02f",ACC_X,ACC_Y,ACC_Z,accData.mean);
        NRF_LOG_INFO("%s",temp);
#endif
    }
}
