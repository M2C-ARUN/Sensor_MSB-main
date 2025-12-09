/**
 * @file pwr.c
 * @author Aakash Singh (development.a@m2cloud.in)
 * @brief File contains Power and battery managment related function
 * @version 0.1
 * @date 2021-07-14
 *
 *
 *
 */
#include "main.h"
#include "pwr.h"
#include "ble_func.h"

pwrSense_t pwrSense;

#define PWRSENSEENPIN 30
#define INPUTPWRPIN NRF_SAADC_INPUT_AIN4 // Pin 0.28

#define SAMPLES_IN_BUFFER 10

#define MULTI_FACTOR 8.7
#define ADD_FACTOR 0.062
#define V_REF 4.2
#define V_MIN 3.3

// Assuming ADC_MAX_VALUE is 1023 for a 10-bit ADC
#define ADC_MAX_VALUE 1023
bool btr_pwrdn = false;
bool pwrdwn_bzr = false;
uint8_t pwrdwn_cnt = 0;

volatile uint8_t state = 1;

static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter;

// #define ADC_RESOLUTION 4095.0f   // 12-bit SAADC
// #define SAADC_REF_VOLTAGE 0.6f   // Internal reference
// #define SAADC_GAIN (1.0f / 6.0f) // Gain setting = 1/6
// #define DIVIDER_FACTOR 8.7f      // Your resistor divider ratio
// #define VBAT_MAX 4.0f            // Full battery voltage
// #define VBAT_MIN 2.5f            // Empty battery voltage

// float get_battery_voltage(uint16_t adc_raw)
// {
//     // Convert raw ADC to SAADC input voltage
//     float v_input = ((float)adc_raw / ADC_RESOLUTION) * SAADC_REF_VOLTAGE / SAADC_GAIN;

//     // Undo resistor divider to get actual battery voltage
//     float v_batt = v_input * DIVIDER_FACTOR;

//     return v_batt;
// }

// uint8_t battery_voltage_to_soc(float v_batt)
// {
//     if (v_batt >= 4.0f)
//         return 100;
//     else if (v_batt >= 3.9f)
//         return 90;
//     else if (v_batt >= 3.8f)
//         return 80;
//     else if (v_batt >= 3.7f)
//         return 70;
//     else if (v_batt >= 3.6f)
//         return 50;
//     else if (v_batt >= 3.5f)
//         return 30;
//     else if (v_batt >= 3.4f)
//         return 15;
//     else if (v_batt >= 3.3f)
//         return 5;

//     else
//         return 0;
// }

// Battery percentage lookup based on voltage (4.0V → 100%, 3.3V → 0%)
// static uint8_t battery_voltage_to_soc(float v_batt)
// {
//     if (v_batt >= 4.0f)
//         return 100;
//     else if (v_batt >= 3.9f)
//         return 90;
//     else if (v_batt >= 3.8f)
//         return 80;
//     else if (v_batt >= 3.7f)
//         return 70;
//     else if (v_batt >= 3.6f)
//         return 50;
//     else if (v_batt >= 3.5f)
//         return 30;
//     else if (v_batt >= 3.4f)
//         return 15;
//     else if (v_batt >= 3.3f)
//         return 5;
//     else
//         return 0;
// }

/**
 * @brief Construct a new app timer ==> Battery Timer
 *
 */
APP_TIMER_DEF(TIMER_ADC);

/**
 * @brief Handler for timer events.
 */
void timer_adc_handler(void *p_context)
{

    nrf_drv_saadc_sample();
}

/**
 * @brief ADC Callback
 *
 * @param p_event ADC Event
 */
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    float batt_voltage;
    uint8_t batt_soc;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        batt_voltage = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            batt_voltage += p_event->data.done.p_buffer[i];

            // Print raw ADC value for debugging
            // printf("Raw ADC Value: %d\n", p_event->data.done.p_buffer[i]);
        }

        batt_voltage = batt_voltage / SAMPLES_IN_BUFFER;
        // batt_voltage = (((batt_voltage * V_REF) / 1023) + ADD_FACTOR) * MULTI_FACTOR;

        // if (batt_voltage > 3.7)
        //     batt_voltage = 3.7;

        // uint16_t batt_voltage_raw = 0;
        // for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        // {
        //     batt_voltage_raw += p_event->data.done.p_buffer[i];
        // }
        // batt_voltage_raw /= SAMPLES_IN_BUFFER; // average raw ADC

        // float batt_voltage = get_battery_voltage(batt_voltage_raw);
        // uint8_t batt_soc = battery_voltage_to_soc(batt_voltage);
        // battery_level_update(batt_soc);

        // Convert ADC value to voltage
        batt_voltage = (batt_voltage * V_REF) / ADC_MAX_VALUE; // Assuming 10-bit ADC
        batt_voltage = batt_voltage + ADD_FACTOR;              // Apply any necessary offset
        batt_voltage = batt_voltage * MULTI_FACTOR;            // Apply any necessary scaling

        // Clamp the voltage to the maximum value
        if (batt_voltage > V_REF)
            batt_voltage = V_REF;
        // printf("battery_voltage2: %.2f\n", batt_voltage);

        batt_soc = ((batt_voltage - V_MIN) * 100) / (V_REF - V_MIN);
        // printf("battery_soc: %d\n", batt_soc);
        if (batt_soc > 100)
            batt_soc = 100;
        else if (batt_soc < 0)
            batt_soc = 0;
        // printf("battery_soc: %d\n", batt_soc);

        // batt_soc = battery_voltage_to_soc(batt_voltage);
        // printf("battery_soc: %d\n", batt_soc);

        pwrSense.inputPwr = batt_voltage;
        pwrSense.SOC = batt_soc;
        // if (pwrSense.SOC > 100)
        //     pwrSense.SOC = 100;
        battery_level_update(pwrSense.SOC);
        m_adc_evt_counter++;
        pwrSense.pwrSenseEn = false;

        // if (!pwrdwn_bzr && btr_pwrdn)
        if (btr_pwrdn)
        {
            pwrdwn_cnt++;
            printf("pwr_cnt: %d\n", pwrdwn_cnt);
            nrf_gpio_pin_set(BUZZER_PIN); // buzzer on

            if (pwrdwn_cnt >= 5)
            {
                btr_pwrdn = false;
                // pwrdwn_bzr = true;
                pwrdwn_cnt = 0;
                nrf_gpio_pin_clear(BUZZER_PIN); // buzzer off
                nrf_gpio_pin_set(BATTERY_ALERT_PIN);
            }

            // nrf_gpio_pin_set(BUZZER_PIN); // buzzer on
            // nrf_delay_ms(500);
            // nrf_gpio_pin_clear(BUZZER_PIN); // buzzer off
            // nrf_delay_ms(500);
        }
        // else if (btr_pwrdn && pwrSense.inputPwr >= 3.1)
        // {
        //     btr_pwrdn = false;
        //     pwrdwn_bzr = false;
        //     AccActiveProcess();
        //     nrf_gpio_pin_set(BATTERY_ALERT_PIN); // blue led off
        // }
    }
}

/**
 * @brief Adc Initializtion
 *
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief PWR Managment Initlization
 *
 * @return ret_code_t Error Code
 */
ret_code_t pwr_sense_init()
{
    uint32_t time_ms = 500; // Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    ret_code_t err_code = NRF_SUCCESS;

    nrf_gpio_cfg_output(PWRSENSEENPIN);

    err_code = app_timer_create(&TIMER_ADC, APP_TIMER_MODE_REPEATED, timer_adc_handler);
    APP_ERROR_CHECK(err_code);

    saadc_init();
    getBattLevel();
    return err_code;
}

/**
 * @brief Get the Battery Level
 *
 * @return ret_code_t Error Code
 */
ret_code_t getBattLevel()
{
    ret_code_t err_code = NRF_SUCCESS;

    pwrSense.pwrSenseEn = true;

    nrf_gpio_pin_write(PWRSENSEENPIN, 1);

    err_code = app_timer_start(TIMER_ADC, APP_TIMER_TICKS(200), NULL);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

/**
 * @brief Diasble Bsttery mesurement
 *
 * @return ret_code_t
 */
ret_code_t DisableBattLevel()
{
    ret_code_t err_code = NRF_SUCCESS;

    nrf_gpio_pin_write(PWRSENSEENPIN, 0);

    err_code = app_timer_stop(TIMER_ADC);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

/**
 * @brief Get the Battery Level
 *
 * @return ret_code_t Error Code
 */
ret_code_t getCurrBattLevel()
{

    //    this function we getting the current battery level status

    ret_code_t err_code = NRF_SUCCESS;
    float batt_voltage = 0.0;
    uint8_t batt_soc = 0;

    nrf_saadc_value_t value;

    nrf_gpio_pin_write(PWRSENSEENPIN, 1);

    for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
    {
        nrf_drv_saadc_sample_convert(0, &value);
        batt_voltage += value;
    }
    batt_voltage = batt_voltage / SAMPLES_IN_BUFFER;
    batt_voltage = (((batt_voltage * V_REF) / 1023) + ADD_FACTOR) * MULTI_FACTOR;
    batt_soc = ((batt_voltage - V_MIN) * 100) / (V_REF - V_MIN);
    pwrSense.inputPwr = batt_voltage;
    pwrSense.SOC = batt_soc;

    nrf_gpio_pin_write(PWRSENSEENPIN, 0);

    return err_code;
}
