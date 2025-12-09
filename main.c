/**
 * @file main.c
 * @author Vivek Kumar (development.b@m2cloud.in)
 * @brief
 * @version 3.0
 * @date 2021-09-16
 */
#include "main.h"
#include "acc.h"
#include "ble_func.h"
#include "pwr.h"
#include "bsp.h"
#include "calendar.h"
#include "flash_func.h"

bool acc_init_flag = true;
extern bool time_written;
bool restart = false;

#define NRF52_ONRAM1_OFFRAM1 POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos | POWER_RAM_POWER_S0RETENTION_On << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_On << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM1_OFFRAM0 POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos | POWER_RAM_POWER_S0RETENTION_Off << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_Off << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM0_OFFRAM0 POWER_RAM_POWER_S0POWER_Off << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_Off << POWER_RAM_POWER_S1POWER_Pos;
void configure_ram_retention(void)
{
#ifdef NRF51
    // Configure nRF51 RAM retention parameters. Set for System On 16kB RAM retention
    NRF_POWER->RAMON = POWER_RAMON_ONRAM0_RAM0On << POWER_RAMON_ONRAM0_Pos | POWER_RAMON_ONRAM1_RAM1On << POWER_RAMON_ONRAM1_Pos | POWER_RAMON_OFFRAM0_RAM0Off << POWER_RAMON_OFFRAM0_Pos | POWER_RAMON_OFFRAM1_RAM1Off << POWER_RAMON_OFFRAM1_Pos;
    NRF_POWER->RAMONB = POWER_RAMONB_ONRAM2_RAM2Off << POWER_RAMONB_ONRAM2_Pos | POWER_RAMONB_ONRAM3_RAM3Off << POWER_RAMONB_ONRAM3_Pos | POWER_RAMONB_OFFRAM2_RAM2Off << POWER_RAMONB_OFFRAM2_Pos | POWER_RAMONB_OFFRAM3_RAM3Off << POWER_RAMONB_OFFRAM3_Pos;
#endif // NRF51

#ifdef NRF52

    // Configure nRF52 RAM retention parameters. Set for System On 64kB RAM retention
    NRF_POWER->RAM[0].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[1].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[2].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[3].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[4].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[5].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[6].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[7].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_LOG_INFO("Final Touch for Init \r\n");
#endif // NRF52
}

void uart_error_handle(app_uart_evt_t *p_event)
{
}
void uart_trasmit_str(uint8_t *ch)
{
#if !debug_enable
    while ((*ch) != '\0')
    {
        if (app_uart_put(*ch) == NRF_SUCCESS)
        {
            // NRF_LOG_INFO("Fial to transmi");
        }
        ch++;
    }
    app_uart_put('\n');
#endif
}

/**
 * @brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_init(BSP_INIT_LEDS, NULL); // in bsp_init() function we intializing the led
}
/**
 * @brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init(); //  in app_timer_init() function we intializing the timer
    APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
/**
 * @brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
}
/**
 * @brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
void calendar_updated()
{
    __NOP();
}

extern bool conn_flag;
uint32_t cnt11 = 0;
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    // Initialize the async  SVCI interface to bootloader before any interrupts are enabled.
    // NOTE: Set USE_BOOTLOADER_FOTA 1 when using FOTA and Bootlaader
#if USE_BOOTLOADER_FOTA
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
#endif
//  select hare debug / serial
//  if debug enable 1 -> work as uart debug port,
//  if debug enable 0 -> work as serial data transfor
#if debug_enable
    // Initialize log .
    log_init();
#else
    const app_uart_comm_params_t comm_params = {USB_RX_PIN, USB_TX_PIN, RTS_PIN_NUMBER, CTS_PIN_NUMBER, false, false, NRF_UARTE_BAUDRATE_115200};
    APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_error_handle, APP_IRQ_PRIORITY_LOWEST, err_code);
    APP_ERROR_CHECK(err_code);
    // printf("UART Init Status: 0x%X\n", err_code); // Debug print
    // uart_trasmit_str("UART Reset\n");
#endif

    // Initialize leds for BLE status
    leds_init();
    NRF_LOG_INFO("leds_init-> Case-1\r\n");
    // Initialize time for event
    timers_init();
    NRF_LOG_INFO("timers_init-> Case pass-2\r\n");
    // Initialize Power Management
    power_management_init();
    NRF_LOG_INFO("power_management_init-> Case-3\r\n");
    // Initialize calentar
    nrf_cal_init();
    NRF_LOG_INFO("nrf_cal_init-> Case-4\r\n");
    nrf_cal_set_callback(calendar_updated, 4);
    NRF_LOG_INFO("nrf_cal_set_callback-> Case-5\r\n");
    // Initialize data record memory
    flash_init();
    NRF_LOG_INFO("flash_init-> Case-6\r\n");
    config_init();
    NRF_LOG_INFO("config_init-> Case-7\r\n");
    // Initialize BLE
    ble_init();
    NRF_LOG_INFO("ble_init-> Case-8\r\n");
    // Initialize Accelerometer
    err_code = acc_init();
    NRF_LOG_INFO("acc_init-> Case-9\r\n");
    if (err_code != NRF_SUCCESS)
    {
        acc_init_flag = false;
        NRF_LOG_ERROR("Accelerometer Initialization Failed!");
    }
    else
    {
        NRF_LOG_ERROR("Accelerometer Initialization successfully!");
    }
    pwr_sense_init();
    NRF_LOG_INFO("pwr_sense_init-> Case-10\r\n");
    NRF_LOG_INFO("Device:Fall Arrest\r\nManufacturer:M2Cloud\r\nVersion:8.2 ARCH\r\n");
    system_ram_retention();
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

/**
 * @brief Get the ram_retention
 *
 * @param[out] i select power secter
 * @return uint8_t
 */
void system_ram_retention(void)
{
    for (int i = 0; i < 4; i++)
    {
        sd_power_ram_power_set(i,
                               (POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos) | (POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos) | (POWER_RAM_POWER_S0RETENTION_On << POWER_RAM_POWER_S0RETENTION_Pos) | (POWER_RAM_POWER_S1RETENTION_On << POWER_RAM_POWER_S1RETENTION_Pos));
    }
}
/**
 * @brief Get the Controller UID
 *
 * @param[out] uid UID for the controller
 * @return uint8_t
 */
uint8_t getUID(char *uid)
{

    // print device ID
    sprintf(uid, "%08X%08X", (unsigned int)NRF_FICR->DEVICEID[0], (unsigned int)NRF_FICR->DEVICEID[1]);
    return 1;
}
/**
 * @brief Get the Timestamp
 *
 * @param[out] ts Timestamp
 * @return uint8_t
 */
uint8_t getTimestamp(char *ts)
{

    // print timestamp

    sprintf(ts, "%s", nrf_cal_get_time_string(false));
    return 1;
}
/**
 * @brief Reboot Device
 *
 */
void rebootDevice()
{
    NRF_LOG_INFO("Reboot Device");
    NRF_LOG_FINAL_FLUSH();
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);
}
