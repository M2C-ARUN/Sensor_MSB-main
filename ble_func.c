#include "main.h"
#include "ble_func.h"
#include "pwr.h"
#include "bsp.h"
#include "flash_func.h"
#include "calendar.h"
#include "acc.h"
#include "nrf_ble_scan.h"
#include "ble_gap.h"

ble_gap_addr_t gap_addr;

#define APP_ADV_INTERVAL 64                                    /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_ADV_FAST_INTERVAL 64     /**< Fast advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL 0x0C80 /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */

#define APP_ADV_FAST_DURATION 3000  /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION 18000 /**< The advertising duration of slow advertising in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                        /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SEC_PARAM_TIMEOUT 30                           /**< Time-out for pairing request or security request (in seconds). */
#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection requirement. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data availability. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */
#define SCAN_INTERVAL 0x00A0 // 100 ms = 160 × 0.625 ms
#define SCAN_WINDOW 0x00A0   // 100 ms = same as interval for continuous scan
#define SCAN_DURATION 0      // 0 = scan until explicitly stopped
#define SCAN_PHY BLE_GAP_PHY_1MBPS

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)  /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)      /**< Determines supervision time-out in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

extern configuration_t m_device_cfg; // Device Configuration
bool is_UUID = false, is_url = false;
bool live_notify = false, his_notify = false;
uint8_t device_detect_cnt = 0, th1[100];
uint8_t bit0[10], bit1[10], bit2[10];
uint8_t bit3[10], bit4[10], bit5[10],
    wt[10], bzt[10], delaytm[10], hm_stp[10];
BLE_CUS_DATA_DEF(m_data);
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_CUS_SETTINGS_DEF(m_settings);         /**< Settings for the Custom User Service. */
BLE_CTS_C_DEF(m_cts_c);                   /**< Current Time service instance. */
BLE_BAS_DEF(m_bas);                       /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                 /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                   /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);       /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery); /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                 /**< Scan module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE); /**< BLE GATT Queue instance. */

APP_TIMER_DEF(TIMER_HISTORY);

static pm_peer_id_t m_peer_id;                                           /**< Device reference handle to the current bonded central. */
static uint16_t m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;             /**< Handle of the current connection. */
static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT]; /**< List of peers currently in the whitelist. */
static uint32_t m_whitelist_peer_cnt;                                    /**< Number of peers currently in the whitelist. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */
static uint8_t m_device_name[20] = DEVICE_NAME;                         /**< Name of device. Will be included in the advertising data. */
bool gyr_noti_flag = false, conn_flag = false;


/**< Scanning module instance. */
static char const m_target_periph_name[] = "KARE-D0B58"; /**< Name of the device we try to connect to. This name is searched in the scan report data*/

bool scan_hook_stats = false;
extern bool self_hook_disconnect;
extern bool bhd_flag;
uint16_t both_hook_disconnect = 0;

// Define your target UUID (LSB first as it appears in the advertising packet)
uint8_t TARGET_UUID[] = {0x1E, 0x69, 0x1D, 0x30, 0x7E, 0x08, 0x47, 0xF0,
                         0x8B, 0x3E, 0xA6, 0x35, 0xF3, 0x50, 0x0E, 0x6A};

static char const *day_of_week[] =
    {
        "Unknown",
        "Monday",
        "Tuesday",
        "Wednesday",
        "Thursday",
        "Friday",
        "Saturday",
        "Sunday"};
/**
 * @brief Month of year strings.
 */
static char const *month_of_year[] =
    {
        "Unknown",
        "January",
        "February",
        "March",
        "April",
        "May",
        "June",
        "July",
        "August",
        "September",
        "October",
        "November",
        "December"};

/**
 * @brief Universally unique service identifiers.
 */
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {
        {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_CURRENT_TIME_SERVICE, BLE_UUID_TYPE_BLE}};

/**
 * @brief Advertising data.
 * @note: Both the advertising data and scan response data are limited
 *      to BLE_GAP_ADV_SET_DATA_SIZE_MAX = 31 bytes.
 */

static ble_gap_adv_data_t m_adv_data =
    {
        .adv_data =
            {
                .p_data = m_enc_advdata,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
        .scan_rsp_data =
            {
                .p_data = m_enc_scan_response_data,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

            }};

/**
 * @brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    NRF_LOG_INFO("nrf_callback error case 1\r\n");
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Fast advertising");
        break;

    case BLE_ADV_EVT_SLOW:
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Slow advertising");
        break;

    case BLE_ADV_EVT_FAST_WHITELIST:
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Fast advertising with WhiteList");
        break;

    case BLE_ADV_EVT_SLOW_WHITELIST:
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Slow advertising with WhiteList");
        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_ADV_EVT_IDLE:
        // if (m_device_cfg.hook_mode == 0)
        // {
        NRF_LOG_INFO("BLE idle state");
        NRF_LOG_FINAL_FLUSH();
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
        AccIdleProcess();
        nrf_ble_scan_stop();                 // stop scanning...
        nrf_gpio_pin_set(BATTERY_ALERT_PIN); // blue led off
        nrf_gpio_pin_clear(BUZZER_PIN);      // buzzer off
        m_device_cfg.zl_flag_start = 0;
        m_device_cfg.xg_flag_start = 0;
        printf("<<----- BLE idle state ----->>\n");
        // }
        // System_OFF_Mode();
        break;

    case BLE_ADV_EVT_WHITELIST_REQUEST:
    {
        ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
        ble_gap_irk_t whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
        uint32_t addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
        uint32_t irk_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

        err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                    whitelist_irks, &irk_cnt);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                      addr_cnt,
                      irk_cnt);

        // Apply the whitelist.
        err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                   whitelist_addrs,
                                                   addr_cnt,
                                                   whitelist_irks,
                                                   irk_cnt);
        APP_ERROR_CHECK(err_code);
    }
    break;

    default:
        break;
    }
}
/**
 * @brief Function to enter System OFF mode (this function will not return;
  wakeup will cause a reset).
 *
 */
void System_OFF_Mode(void)
{
    int32_t ret = sd_power_system_off();
    // get here if softdevice is not running
    NRF_LOG_DEBUG("sd_power_system_off() == %d", ret);
    NRF_POWER->TASKS_LOWPWR = 1;
    NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
}
/**
 * @brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("nrf_qwr_error_handler error case 2\r\n");
    APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void current_time_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("current_time_error_handler error case 3\r\n");
    APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Function for printing the current time received from the Current Time Service.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t *p_evt)
{
    uint32_t year, month, day, hour, minute, second;
    year = p_evt->params.current_time.exact_time_256.day_date_time.date_time.year;
    month = p_evt->params.current_time.exact_time_256.day_date_time.date_time.month;
    day = p_evt->params.current_time.exact_time_256.day_date_time.date_time.day;
    hour = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
    minute = p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
    second = p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;
    // NRF_LOG_INFO("\n **************** year %d %d %d %d %d %d\n",year, month, day, hour, minute,second);

    nrf_cal_set_time(year, month - 1, day, hour, minute, second);
    NRF_LOG_INFO("Uncalibrated time:\t%s\r\n", nrf_cal_get_time_string(false));
    NRF_LOG_INFO("Calibrated time:\t%s\r\n", nrf_cal_get_time_string(true));

#if 1
    NRF_LOG_INFO("\tDay of week   %s", (uint32_t)day_of_week[p_evt->params.current_time.exact_time_256.day_date_time.day_of_week]);

    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.day == 0)
    {
        NRF_LOG_INFO("\tDay of month  Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tDay of month  %i",
                     p_evt->params.current_time.exact_time_256.day_date_time.date_time.day);
    }

    NRF_LOG_INFO("\tMonth of year %s",
                 (uint32_t)month_of_year[p_evt->params.current_time.exact_time_256.day_date_time.date_time.month]);
    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.year == 0)
    {
        NRF_LOG_INFO("\tYear          Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tYear          %i",
                     p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
    }
    NRF_LOG_INFO("\r\nTime:");
    NRF_LOG_INFO("\tHours     %i",
                 p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours);
    NRF_LOG_INFO("\tMinutes   %i",
                 p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes);
    NRF_LOG_INFO("\tSeconds   %i",
                 p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);
    NRF_LOG_INFO("\tFractions %i/256 of a second",
                 p_evt->params.current_time.exact_time_256.fractions256);

    NRF_LOG_INFO("\r\nAdjust reason:\r");
    NRF_LOG_INFO("\tDaylight savings %x",
                 p_evt->params.current_time.adjust_reason.change_of_daylight_savings_time);
    NRF_LOG_INFO("\tTime zone        %x",
                 p_evt->params.current_time.adjust_reason.change_of_time_zone);
    NRF_LOG_INFO("\tExternal update  %x",
                 p_evt->params.current_time.adjust_reason.external_reference_time_update);
    NRF_LOG_INFO("\tManual update    %x",
                 p_evt->params.current_time.adjust_reason.manual_time_update);
#endif
}

/**
 * @brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to
 *          the application.
 *
 * @param[in]   p_bas  Battery Service structure.
 * @param[in]   p_evt  Event received from the Battery Service.
 */
static void nrf_bas_evt_handler(ble_bas_t *p_bas, ble_bas_evt_t *p_evt)
{

    switch (p_evt->evt_type)
    {
    case BLE_BAS_EVT_NOTIFICATION_ENABLED: /**< Battery value notification enabled event. */
        NRF_LOG_INFO("BAS Notifiaction Enable");
        getBattLevel(); // enable to get battery level
        break;
    case BLE_BAS_EVT_NOTIFICATION_DISABLED: /**< Battery value notification disabled event. */
        NRF_LOG_INFO("BAS Notifiaction Disable");
        DisableBattLevel(); // disable get battery level
        break;
    }
}

/**
 * @brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t *p_cts, ble_cts_c_evt_t *p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
    case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("Current Time Service discovered on server.");
        printf("Current Time Service discovered on server.\n");
        err_code = ble_cts_c_handles_assign(&m_cts_c,
                                            p_evt->conn_handle,
                                            &p_evt->params.char_handles);
        APP_ERROR_CHECK(err_code);
        get_cts();
        break;

    case BLE_CTS_C_EVT_DISCOVERY_FAILED:
        NRF_LOG_INFO("Current Time Service not found on server. ");
        printf("Current Time Service not found on server. ");
        // CTS not found in this case we just disconnect. There is no reason to stay
        // in the connection for this simple app since it all wants is to interact with CT
        if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BLE_CTS_C_EVT_DISCONN_COMPLETE:
        NRF_LOG_INFO("Disconnect Complete.");
        printf("Disconnection Complete.");
        break;

    case BLE_CTS_C_EVT_CURRENT_TIME:
        NRF_LOG_INFO("Current Time received.");
        current_time_print(p_evt);
        break;

    case BLE_CTS_C_EVT_INVALID_TIME:
        NRF_LOG_INFO("Invalid Time received.");
        break;

    default:
        break;
    }
}

/**
 * @brief History Alert to BLE
 * @param[in] p_context Context.
 */
void timer_history_handler(void *p_context)
{
    uint8_t log[256];
    uint8_t log_len;
    NRF_LOG_INFO("History Sending");
    // printf("History Sending\n");
    log_len = GetLog(log);
    NRF_LOG_INFO("Log length %d", log_len);
    // printf("Log length %d\n", log_len);
    if (log_len != 0)
    {
        NRF_LOG_INFO("LOG Alert:%s", log);
        printf("LOG Alert:%s\n", log);
        ble_data_history_value_update(&m_data, log);
        uart_trasmit_str(log);
    }
    else
    {
        GetLogReset();
        ClearLog();
        NRF_LOG_INFO("History timer Stop");
        // printf("History timer Stop\n");
        app_timer_stop(TIMER_HISTORY);
    }
}

/**
 * @brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 */

static void on_cus_data_evt(ble_cus_data_t *p_cus_service,ble_cus_data_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_CUS_DATA_LIVE_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("live noti en");
        // printf("live notification enabled\n");
        live_notify = true;
        // conn_flag=true;
        break;

    case BLE_CUS_DATA_LIVE_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("live noti dis");
        // printf("live noti dis\n");
        live_notify = false;
        break;

    case BLE_CUS_DATA_HISTORY_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("history noti en");
        // printf("history noti en\n");
        his_notify = true;
        SendHistory();
        break;

    case BLE_CUS_DATA_HISTORY_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("history noti dis");
        // printf("history noti dis\n");
        his_notify = false;
        break;
    case BLE_CUS_ACC_GYR_RAW_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("acc gyr raw noti en");
        gyr_noti_flag = true;
        break;
    case BLE_CUS_ACC_GYR_RAW_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("acc gyr raw noti dis");
        gyr_noti_flag = false;
        break;

    case BLE_CUS_DATA_HISTORY_EVT_WRITE:

        break;
    case BLE_CUS_DATA_EVT_CONNECTED:
        NRF_LOG_INFO("CON");
        break;

    case BLE_CUS_DATA_EVT_DISCONNECTED:
        NRF_LOG_INFO("DIS");
        break;

    default:
        // No implementation needed.
        break;
    }
}
/**
 * @brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_cus_settings_evt(ble_cus_settings_t *p_cus_service,ble_cus_settings_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_CUS_SETTINGS_EVT_NOTIFICATION_ENABLED:
        break;

    case BLE_CUS_SETTINGS_EVT_NOTIFICATION_DISABLED:
        break;

    case BLE_CUS_SETTINGS_EVT_CONNECTED:
        break;

    case BLE_CUS_SETTINGS_EVT_DISCONNECTED:
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**
 * @brief Function for handling the data received over BLE.
 * @details This function processes the data received from the Nordic UART Service.
 * @snippet [Handling the data received over BLE]
 * */
static void nus_data_handler(ble_nus_evt_t *p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
    }
}

/**
 * @brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_cur_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("conn_params_error handler error case 4\r\n");
    APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Handler for shutdown preparation.
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 * @param[in]   event   Power manager event.
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
        NRF_LOG_INFO("Power management wants to reset to DFU mode.");
        // YOUR_JOB: Get ready to reset into DFU mode
        //
        // If you aren't finished with any ongoing tasks, return "false" to
        // signal to the system that reset is impossible at this stage.
        //
        // Here is an example using a variable to delay resetting the device.
        //
        // if (!m_ready_for_reset)
        // {
        //      return false;
        // }
        // else
        //{
        //
        //    // Device ready to enter
        //    uint32_t err_code;
        //    err_code = sd_softdevice_disable();
        //    APP_ERROR_CHECK(err_code);
        //    err_code = app_timer_stop_all();
        //    APP_ERROR_CHECK(err_code);
        //}
        break;

    default:
        // YOUR_JOB: Implement any of the other events available from the power management module:
        //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
        //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
        //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
        return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

// lint -esym(528, m_app_shutdown_handler)
/**
 * @brief Register application shutdown handler with priority 0 (highest).
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

// lint -esym(552, buttonless_dfu_sdh_state_observer)
/** @brief SoftDevice state observer.
 *
 * @details This function will be called when the SoftDevice changes state.
 *
 * @param[in] state     SoftDevice state.
 * @param[in] p_context Context.
 */
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        // Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
    {
        .handler = buttonless_dfu_sdh_state_observer,
};
/**
 * @brief Function for getting the advertising configuration.
 */
static void advertising_config_get(ble_adv_modes_config_t *p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout = APP_ADV_DURATION;
}

// disconnect all connected bonded devices before going into DFU mode
/**
 * @brief Function for disconnecting a given connection handle.
 *
 * @param[in] conn_handle  Connection handle to be disconnected.
 * @param[in] p_context   Context.
 */
void disconnect(uint16_t conn_handle, void *p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**
 * @brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
    {
        NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

        // Prevent device from advertising on disconnect.
        ble_adv_modes_config_t config;
        advertising_config_get(&config);
        config.ble_adv_on_disconnect_disabled = true;
        ble_advertising_modes_config_set(&m_advertising, &config);

        // Disconnect all other bonded devices that currently are connected.
        // This is required to receive a service changed indication
        // on bootup after a successful (or aborted) Device Firmware Update.
        uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
        NRF_LOG_INFO("Disconnected %d links.", conn_count);
        break;
    }

    case BLE_DFU_EVT_BOOTLOADER_ENTER:
        // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
        //           by delaying reset by reporting false in app_shutdown_handler
        NRF_LOG_INFO("Device will enter bootloader mode.");
        break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
        NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        break;

    case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
        NRF_LOG_ERROR("Request to send a response to client failed.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        APP_ERROR_CHECK(false);
        break;

    default:
        NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
        break;
    }
}
/**
 * @brief Scan event handler.
 */
static void scan_evt_handler(scan_evt_t const *p_scan_evt)
{
    switch (p_scan_evt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_NOT_FOUND:
    {
        ble_gap_evt_adv_report_t const *p_adv = p_scan_evt->params.p_not_found;
        uint8_t *p_data = p_adv->data.p_data;
        uint16_t length = p_adv->data.len;
        bool is_config_mode = false;
        // Check device name and manufacturer data
        for (uint8_t i = 0; i < length;)
        {
            if ((i + 1) >= length)
                break;

            uint8_t field_length = p_data[i];
            uint8_t field_type = p_data[i + 1];

            if (field_length == 0 || (i + field_length) >= length)
                break;

            // Check device name
            if (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME ||
                field_type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
            {
                char name[32] = {0};
                memcpy(name, &p_data[i + 2], field_length - 1);
                if (strstr(name, m_device_cfg.sensor_name_set) != NULL)
                {
                    // for slave
                    is_config_mode = true;
                    // printf("Found target device name: %s\n", name);
                }
            }
            // Check for iBeacon data (config mode)
            if (field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
            {
                // for master
                if (buz_en == 0 && m_device_cfg.master_device_flag == 1 || buz_en > 0 && m_device_cfg.master_device_flag == 1)
                    parse_ibeacon_data(&p_data[i + 2], field_length - 1);
            }
            if (field_type == BLE_GAP_AD_TYPE_SERVICE_DATA)
            {
                // for master
                if (buz_en > 0 && m_device_cfg.master_device_flag == 1)
                    parse_eddystone_url(&p_data[i + 2], field_length - 1);
            }
            i += field_length + 1;
        }

        // Only print details if it's our target device
        if (is_config_mode && m_device_cfg.slave_device_flag == 1)
        {

            for (uint8_t i = 0; i < length;)
            {
                uint8_t field_length = p_data[i];
                uint8_t field_type = p_data[i + 1];

                switch (field_type)
                {
                case BLE_GAP_AD_TYPE_FLAGS:
                    // printf("Flags: 0x%02X\n", p_data[i + 2]);
                    break;

                case BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE:
                    // printf("Service UUID: 0x%02X%02X\n", p_data[i + 3], p_data[i + 2]);
                    break;

                case BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA:
                    printf("Manufacturer Data: ");
                    for (uint8_t j = 0; j < field_length - 1; j++)
                    {
                        printf("%02X ", p_data[i + 2 + j]);
                    }
                    printf("\n");

                    // Define relative positions for clarity
                    const uint8_t POS_HCMD_STATS = 2; // 3rd byte
                    const uint8_t POS_HM_STATS = 3;   // 4th byte

                    char m_hookcmd_stats = p_data[i + 2 + POS_HCMD_STATS];
                    char m_hd_stats = p_data[i + 2 + POS_HM_STATS];

                    // Convert from ASCII '0'..'9' to decimal 0..9
                    uint8_t master_hcmd_status = m_hookcmd_stats - '0';
                    uint8_t master_hd_status = m_hd_stats - '0';

                    // printf("Master Hook Command Status: %d\n", master_hcmd_status);
                    // printf("Master Hook Disconnect Status: %d\n", master_hd_status);

                    if (master_hd_status == 0)
                    {
                        bhd_flag = false;
                    }

                    if (m_device_cfg.slave_device_flag == 1 && !bhd_flag && master_hd_status == 1 && self_hook_disconnect == true) // working for both hook disconnect
                    {
                        both_hook_disconnect++;
                        bhd_flag = true;
                        printf("<<----- Both Hook Disconnect: %d ----->>\n", both_hook_disconnect);
                    }
                    if (!scan_hook_stats && master_hcmd_status == 1)
                    {
                        printf("<<-------- hello the status is 1 --------->>\n");
                        scan_hook_stats = true;
                    }
                    if (scan_hook_stats && master_hcmd_status == 0)
                    {
                        printf("<<-------- hello the status is 0 --------->>\n");
                        scan_hook_stats = false;
                    }
                    break;
                }
                i += field_length + 1;
            }

            break;
        }
    }
    }
}
/**
 * @brief Function to parse iBeacon data from advertisement payload.
 * @param[in] data Pointer to the advertisement data.
 * @param[in] len Length of the advertisement data.
 */
void parse_ibeacon_data(uint8_t *data, uint8_t len)
{
    // Verify minimum length and iBeacon header
    if (len < 25)
        return;
    if (data[0] != 0x4C || data[1] != 0x00 ||
        data[2] != 0x02 || data[3] != 0x15)
        return;

    // Check UUID matches our target
    bool uuid_matches = true;
    for (int i = 0; i < 16; i++)
    {
        if (data[4 + i] != m_device_cfg.beacon_uuid[i])
        {
            uuid_matches = false;
            break;
        }
    }

    if (!uuid_matches)
        return;

    // Extract and print the beacon data

    is_UUID = true;
    device_detect_cnt++;
    printf("Target Beacon Found:\n");
    printf("Beacon_found_counter:%d\n", device_detect_cnt);

    // Print UUID
    printf("  UUID: ");
    for (uint8_t i = 0; i < 16; i++)
    {
        printf("%02X", data[4 + i]);
    }
    printf("\n");

    uint16_t major = (data[20] << 8) | data[21]; // Major (big-endian)
    uint16_t minor = (data[22] << 8) | data[23]; // Minor (big-endian)
    int8_t tx_power = (int8_t)data[24];          // TX Power

    printf("  Major: %u (0x%04X)\n", major, major);
    printf("  Minor: %u (0x%04X)\n", minor, minor);
    printf("  TX Power: %ddBm\n", tx_power);

    // Raw data printing
    printf("  Raw Data: ");
    for (uint8_t i = 0; i < len && i < 25; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
}
/**
 * @brief Function to parse Eddystone-URL data from advertisement payload.
 * @param[in] data Pointer to the advertisement data.
 * @param[in] len Length of the advertisement data.
 */
void parse_eddystone_url(uint8_t *data, uint8_t len)
{
    // Minimum Eddystone-URL frame is 4 bytes
    if (len < 4)
        return;

    // Check for Eddystone UUID (0xFEAA) and URL frame type (0x10)
    uint16_t service_uuid = (data[1] << 8) | data[0];
    if (service_uuid != 0xFEAA || data[2] != 0x10)
        return;

    // TX Power (1 byte)
    int8_t tx_power = (int8_t)data[3];

    // URL Scheme Prefix
    const char *prefixes[] = {
        "http://www.", "https://www.",
        "http://", "https://",
        "urn:uuid:"};
    uint8_t prefix_idx = data[4] & 0x0F;

    // Buffer to reconstruct the full URL
    char url[256] = {0};
    strcpy(url, prefixes[prefix_idx]);

    // URL Encoding Map
    const char *encodings[] = {
        ".com/", ".org/", ".edu/", ".net/", ".info/",
        ".biz/", ".gov/", ".com", ".org", ".edu",
        ".net", ".info", ".biz", ".gov"};

    // Parse URL suffix
    for (uint8_t i = 5; i < len; i++)
    {
        uint8_t c = data[i];
        if (c <= 0x0D)
        {
            strcat(url, encodings[c]);
        }
        else if (c == 0x0E)
        {
            strcat(url, ".org/");
        }
        else if (c == 0x0F)
        {
            strcat(url, ".com/");
        }
        else
        {
            // Append single character
            size_t len = strlen(url);
            url[len] = c;
            url[len + 1] = '\0';
        }
    }

    // Check if this is our target URL
    if (strstr(url, "arresto.in") != NULL)
    {
        if (!is_url)
            is_url = true;
        printf("\n=== ARRESTO URL DETECTED ===\n");
        printf("  Full URL: %s\n", url);
        printf("  TX Power: %ddBm\n", tx_power);
    }
    // else
    // {
    //     // For other URLs, use normal output
    //     printf("Eddystone-URL Detected:\n");
    //     printf("  TX Power: %ddBm\n", tx_power);
    //     printf("  URL: %s\n", url);
    // }
}
/**
 * @brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t err_code;
    nrf_ble_scan_init_t init_scan;
    ble_gap_scan_params_t scan_params;

    memset(&scan_params, 0, sizeof(scan_params));
    memset(&init_scan, 0, sizeof(init_scan));

    // Set continuous scan parameters
    scan_params.active = 0; // Passive scan (change to 1 for active scan)
    scan_params.interval = SCAN_INTERVAL;
    scan_params.window = SCAN_WINDOW;
    scan_params.timeout = SCAN_DURATION;
    scan_params.scan_phys = BLE_GAP_PHY_1MBPS;
    scan_params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;

    init_scan.p_scan_param = &scan_params;
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // IMPORTANT: Do NOT enable filters unless you use them!
    // Filters are disabled here to catch all devices

    printf("Scanning initialized.\n");
}
/**
 * @brief Function for starting scanning.
 */
void blescan_start(void)
{
    ret_code_t err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    printf("Scan started.\n");
}
/**
 * @brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code;

    // pm_handler_secure_on_connection(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected.");
        printf("device connected with bluetooth\n");
        nrf_ble_scan_stop();
        conn_flag = true;

        // blescan_start();
        // sprintf(AlertStr,
        //         "{KEY_ID:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X}",
        //         m_device_cfg.beacon_uuid[0], m_device_cfg.beacon_uuid[1],
        //         m_device_cfg.beacon_uuid[2], m_device_cfg.beacon_uuid[3],
        //         m_device_cfg.beacon_uuid[4], m_device_cfg.beacon_uuid[5],
        //         m_device_cfg.beacon_uuid[6], m_device_cfg.beacon_uuid[7],
        //         m_device_cfg.beacon_uuid[8], m_device_cfg.beacon_uuid[9],
        //         m_device_cfg.beacon_uuid[10], m_device_cfg.beacon_uuid[11],
        //         m_device_cfg.beacon_uuid[12], m_device_cfg.beacon_uuid[13],
        //         m_device_cfg.beacon_uuid[14], m_device_cfg.beacon_uuid[15]);
        memset(AlertStr, 0, sizeof(AlertStr));
        // sprintf(AlertStr, "{mac:%X%X%X,time:%s,alert:DC}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        snprintf(AlertStr, sizeof(AlertStr),
                 "{mac:%X%X%X,time:%s,alert:DC}",
                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
        uart_trasmit_str(AlertStr);
        AddLog(AlertStr);

        // live_alert_update(AlertStr);
        // else
        // {
        //     AddLog(AlertStr);
        // }
        // uart_trasmit_str(AlertStr);

        if (m_device_cfg.hook_mode != 1 || m_device_cfg.master_device_flag == 1 && m_device_cfg.hook_mode != 1 || m_device_cfg.slave_device_flag == 1 && m_device_cfg.hook_mode != 1)
        {
            // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            // APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_clear(BATTERY_ALERT_PIN); // blue led on
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            nrf_gpio_pin_set(BATTERY_ALERT_PIN); // blue led off
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
        }
        // else
        // {
        // nrf_gpio_pin_clear(BATTERY_ALERT_PIN); // blue led(green) on
        // err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        // APP_ERROR_CHECK(err_code);
        // }

        m_cur_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_cur_conn_handle);
        APP_ERROR_CHECK(err_code);
        err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
        APP_ERROR_CHECK(err_code);

        if (btr_pwrdn)
        {
            nrf_gpio_pin_set(BUZZER_LED); // buzzer led(green) off
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        // printf("device disconnected with bluetooth 0x%x", p_ble_evt->evt.gap_evt.params.disconnected.reason);
        printf("device disconnected with bluetooth\n");
        blescan_start();
        conn_flag = false;
        his_notify = false;
        live_notify = false;
        m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;
        if (p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)
        {
            m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
        }
        // conn_flag=false;
        nrf_gpio_pin_set(BUZZER_LED);
        nrf_gpio_pin_set(BATTERY_ALERT_PIN);

        if (btr_pwrdn)
        {
            nrf_gpio_pin_clear(BATTERY_ALERT_PIN); // buzzer led on
        }
        break; // BLE_GAP_EVT_DISCONNECTED

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**
 * @brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
    ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
}

/**
 * @brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
    case PM_EVT_CONN_SEC_SUCCEEDED:
    {
        m_peer_id = p_evt->peer_id;

        // Discover peer's services.
    }
    break;

    case PM_EVT_PEERS_DELETE_SUCCEEDED:
    {
        advertising_start(false);
    }
    break;

    case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
    {
        // Note: You should check on what kind of white list policy your application should use.
        if (p_evt->params.peer_data_update_succeeded.flash_changed && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
        {
            NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
            NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                          m_whitelist_peer_cnt + 1,
                          BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

            if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
            {
                // Bonded to a new peer, add it to the whitelist.
                m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                // The whitelist has been modified, update it in the Peer Manager.
                err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                if (err_code != NRF_ERROR_NOT_SUPPORTED)
                {
                    APP_ERROR_CHECK(err_code);
                }

                err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                APP_ERROR_CHECK(err_code);
            }
        }
    }
    break;

    default:
        break;
    }
}

/**
 * @brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    // ble_gap_addr_t gap_addr;
    char temp[15];
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_addr_get(&gap_addr);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("MAC:%X:%X:%X:%X:%X:%X", gap_addr.addr[5], gap_addr.addr[4], gap_addr.addr[3], gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);

    sprintf(m_device_name, "%s-%X%X%X", m_device_cfg.name_prefix, gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0]);
    NRF_LOG_INFO("Device Name : %s", m_device_name)
    sprintf(temp, "\nDevice Name: %s", m_device_name);
    uart_trasmit_str(temp);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_device_name,
                                          strlen(m_device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled = false;
    init.config.ble_adv_fast_enabled = true;

    // init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    // init.config.ble_adv_fast_timeout = APP_ADV_FAST_DURATION;

    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    // if (m_device_cfg.hook_mode == 1)
    // {
    //     init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    //     init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    // }
    // else if (m_device_cfg.hook_mode == 0)
    // {
    //     init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    //     init.config.ble_adv_fast_timeout = APP_ADV_FAST_DURATION;
    // }

    init.config.ble_adv_slow_enabled = true;
    init.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout = APP_ADV_SLOW_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
/**
 * @brief Function to update advertising data based on hook connection status.
 * @param new_hook_conn New hook connection status (1 for connected, 0 for disconnected).
 */
void update_advertising_data(uint8_t new_hook_conn)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    // Correctly declare and copy adv_data into vendors_adv_data
    uint8_t vendors_adv_data[sizeof(adv_data) + 1];
    memcpy(vendors_adv_data, adv_data, sizeof(adv_data));
    // printf("ble_advertising_data: %s\n", (char *)vendors_adv_data);

    ble_advdata_manuf_data_t m_spec_data;

    // Initialize the values in the struct
    m_spec_data.company_identifier = 0x0059;
    m_spec_data.data.p_data = vendors_adv_data;
    m_spec_data.data.size = sizeof(vendors_adv_data);

    ble_advdata_t advdata;
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &m_spec_data;

    init.config.ble_adv_fast_enabled = true;
    // init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    // init.config.ble_adv_fast_timeout = APP_ADV_FAST_DURATION;

    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    // if (m_device_cfg.hook_mode == 1)
    // {
    //     init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    //     init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    // }
    // else if (m_device_cfg.hook_mode == 0)
    // {
    //     init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    //     init.config.ble_adv_fast_timeout = APP_ADV_FAST_DURATION;
    // }

    init.config.ble_adv_slow_enabled = true;
    init.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout = APP_ADV_SLOW_DURATION;

    // Update the advertising data using the provided function
    err_code = ble_advertising_advdata_update(&m_advertising, &advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Send History on BLE
 * @note This function starts a timer to send historical data over BLE at regular intervals.
 */
void SendHistory(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("History Send Timer started");
    err_code = app_timer_create(&TIMER_HISTORY, APP_TIMER_MODE_REPEATED, timer_history_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(TIMER_HISTORY, APP_TIMER_TICKS(200), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for initializing services that will be used by the application.
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
void get_mac_address(uint8_t *mac_addr)
{
    ble_gap_addr_t addr;
    sd_ble_gap_addr_get(&addr);     // Get device BLE address
    memcpy(mac_addr, addr.addr, 6); // Copy the MAC address
}
/**
 * @brief Function for initializing services that will be used by the application.
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
void services_init(void)
{

    ret_code_t err_code;
    ble_cus_data_init_t data_init;
    ble_cus_settings_init_t settings_init;
    ble_bas_init_t bas_init;
    ble_nus_init_t nus_init;
    ble_cts_c_init_t cts_init = {0};
    ble_dis_init_t dis_init = {0};
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler = nrf_bas_evt_handler;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    uint8_t mac_addr[6];
    get_mac_address(mac_addr);

    char mac_str[18]; // String to store MAC
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac_addr[5], mac_addr[4], mac_addr[3],
            mac_addr[2], mac_addr[1], mac_addr[0]); // Convert to string

    printf("MAC_ID NUMBER: %s\n", mac_str);

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FW_VERSION);
    // ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)m_device_cfg.block_serial);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize CTS.
    cts_init.evt_handler = on_cts_c_evt;
    cts_init.error_handler = current_time_error_handler;
    cts_init.p_gatt_queue = &m_ble_gatt_queue;
    err_code = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&data_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&data_init.custom_value_char_attr_md.write_perm);

    // Initialize CUS Service init structure to zero.
    if (m_device_cfg.cal_mode == 0)
    {
        memset(&data_init, 0, sizeof(data_init));

        data_init.evt_handler = on_cus_data_evt;
        err_code = ble_cus_data_init(&m_data, &data_init);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        memset(&nus_init, 0, sizeof(nus_init));

        nus_init.data_handler = nus_data_handler;

        err_code = ble_nus_init(&m_nus, &nus_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialize CUS Service init structure to zero.
    memset(&settings_init, 0, sizeof(settings_init));
    settings_init.evt_handler = on_cus_settings_evt;
    err_code = ble_cus_settings_init(&m_settings, &settings_init);
    APP_ERROR_CHECK(err_code);
    uint8_t length, bit_0, bit_1, bit_2, bit_3, bit_4, bit_5;

    bit_0 = m_device_cfg.th_status & 0x01; // fall arrest
    bit_1 = m_device_cfg.th_status & 0x02; // test
    bit_2 = m_device_cfg.th_status & 0x04; // free fall

    /* changes */

    bit_3 = m_device_cfg.th_status & 0x08; // th1
    bit_4 = m_device_cfg.th_status & 0x10; // th2
    bit_5 = m_device_cfg.th_status & 0x20; // th3

    memset(th1, 0, 100);
    if (bit_0 == 1)
    {
        memset(bit0, 0, sizeof(bit0));
        sprintf(bit0, "F:%d", m_device_cfg.fall_threshold);
        strcat(th1, bit0);
        length = strlen(th1);
        // length = sprintf(th1,"FA:%d,WT:%d",m_device_cfg.fall_threshold,m_device_cfg.wake_threshold);
    }
    if (bit_1 == 2)
    {
        memset(bit1, 0, sizeof(bit1));
        sprintf(bit1, ",T:%d", m_device_cfg.test_threshold);
        strcat(th1, bit1);
        length = strlen(th1);
        // length = sprintf(th1,"FA:%d;T:%d,WT:%d",m_device_cfg.fall_threshold,m_device_cfg.test_threshold,m_device_cfg.wake_threshold);
    }

    if (bit_2 == 4)
    {
        memset(bit2, 0, sizeof(bit2));
        sprintf(bit2, ",F:%d", m_device_cfg.ff_threshold);
        strcat(th1, bit2);
        length = strlen(th1);
        // length = sprintf(th1,"FA:%d;T:%d;FF:%d,WT:%d",m_device_cfg.fall_threshold,m_device_cfg.test_threshold,m_device_cfg.ff_threshold,m_device_cfg.wake_threshold);
    }

    /* changes */

    if (bit_3 == 8)
    {
        memset(bit3, 0, sizeof(bit3));
        sprintf(bit3, ",TH1:%d", m_device_cfg.th1_threshold);
        strcat(th1, bit3);
        length = strlen(th1);
        // length = sprintf(th1,"FA:%d;T:%d;FF:%d,WT:%d,TH1:%d",m_device_cfg.fall_threshold,m_device_cfg.test_threshold,m_device_cfg.ff_threshold,m_device_cfg.wake_threshold,m_device_cfg.th1_threshold);
    }

    if (bit_4 == 16)
    {
        memset(bit4, 0, sizeof(bit4));
        sprintf(bit4, ",TH2:%d", m_device_cfg.th2_threshold);
        strcat(th1, bit4);
        length = strlen(th1);
        // length = sprintf(th1,"FA:%d;T:%d;FF:%d,WT:%d,TH1:%d,TH2:%d",m_device_cfg.fall_threshold,m_device_cfg.test_threshold,m_device_cfg.ff_threshold,m_device_cfg.wake_threshold,m_device_cfg.th1_threshold,m_device_cfg.th2_threshold);
    }

    if (bit_5 == 32)
    {
        memset(bit5, 0, sizeof(bit5));
        sprintf(bit5, ",TH3:%d", m_device_cfg.th3_threshold);
        strcat(th1, bit5);
        length = strlen(th1);
        // length = sprintf(th1,"FA:%d;T:%d;FF:%d,WT:%d,TH1:%d,TH2:%d,TH3:%d",m_device_cfg.fall_threshold,m_device_cfg.test_threshold,m_device_cfg.ff_threshold,m_device_cfg.wake_threshold,m_device_cfg.th1_threshold,m_device_cfg.th2_threshold,m_device_cfg.th3_threshold);
    }

    memset(wt, 0, sizeof(wt));
    sprintf(wt, ",WT:%d", m_device_cfg.wake_threshold);
    strcat(th1, wt);
    length = strlen(th1);
    //   length = sprintf(th1,"WT:%d",m_device_cfg.wake_threshold);
    NRF_LOG_INFO("th = %s Size = %d", th1, length);
    th_settings_update(th1, length);

    memset(bzt, 0, sizeof(bzt));
    sprintf(bzt, ",bb:%d", m_device_cfg.buzzertimer_threshold); // buzzer beep time set
    strcat(th1, bzt);
    length = strlen(th1);
    //   length = sprintf(th1,"WT:%d",m_device_cfg.wake_threshold);
    NRF_LOG_INFO("th = %s Size = %d", th1, length);
    th_settings_update(th1, length);

    memset(delaytm, 0, sizeof(delaytm));
    sprintf(delaytm, ",bd:%d", m_device_cfg.delaytimer_threshold); // hook buzzer delay time set
    strcat(th1, delaytm);
    length = strlen(th1);
    // length = sprintf(th1,"WT:%d",m_device_cfg.wake_threshold);
    // printf("default th = %s Size = %d\n", th1, length);
    th_settings_update(th1, length);

    memset(hm_stp, 0, sizeof(hm_stp));
    sprintf(hm_stp, ",hs:%d", m_device_cfg.hm_stopth); // hook buzzer delay time set
    strcat(th1, hm_stp);
    length = strlen(th1);
    //   length = sprintf(th1,"WT:%d",m_device_cfg.wake_threshold);
    // printf("th: %s Size = %d\n", th1, length);
    th_settings_update(th1, length);
}

/**
 * @brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
void peer_list_get(pm_peer_id_t *p_peers, uint32_t *p_size)
{
    pm_peer_id_t peer_id;
    uint32_t peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ? *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}

/**
 * @brief Clear bond information from persistent storage.
 */
void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
/**
 * @brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t ret;

        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(ret);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (ret != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(ret);
        }

        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}
/**
 * @brief Function to get Current Time from CTS Service
 */
void get_cts()
{
    ret_code_t err_code;
    err_code = ble_cts_c_current_time_read(&m_cts_c);
    if (err_code == NRF_ERROR_NOT_FOUND)
    {
        NRF_LOG_INFO("Current Time Service is not discovered.");
    }
}

/**
 * @brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**
 * @brief Function for initializing the Database Discovery Module.
 * @details This function initializes the database discovery module.
 */
void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}
/**
 * @brief Function for initializing the Peer Manager.
 * /
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    // sec_param.bond           = SEC_PARAM_BOND;
    // sec_param.mitm           = SEC_PARAM_MITM;
    // sec_param.lesc           = SEC_PARAM_LESC;
    // sec_param.keypress       = SEC_PARAM_KEYPRESS;
    // sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    // sec_param.oob            = SEC_PARAM_OOB;
    // sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    // sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    // sec_param.kdist_own.enc  = 1;
    // sec_param.kdist_own.id   = 1;
    // sec_param.kdist_peer.enc = 1;
    // sec_param.kdist_peer.id  = 1;

    sec_param.bond = false;
    sec_param.mitm = false;
    sec_param.lesc = 0;
    sec_param.keypress = 0;
    sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
    sec_param.oob = false;
    sec_param.min_key_size = 7;
    sec_param.max_key_size = 16;
    sec_param.kdist_own.enc = 0;
    sec_param.kdist_own.id = 0;
    sec_param.kdist_peer.enc = 0;
    sec_param.kdist_peer.id = 0;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief BLE Stack Initialization
 * @details Initializes the BLE stack and related components.
 */
void ble_init(void)
{
    bool erase_bonds = false;
    ble_stack_init();               // Initialize BLE stack
    gap_params_init();              // Initialize GAP parameters
    gatt_init();                    // Initialize GATT module
    db_discovery_init();            // Initialize Database Discovery module
    advertising_init();             //  Initialize Advertising functionality
    services_init();                // Initialize services
    conn_params_init();             // Initialize Connection Parameters module
    peer_manager_init();            // Initialize Peer Manager
    advertising_start(erase_bonds); //  Start advertising
    scan_init();                    // Initialize scanning
    nrf_delay_ms(1000);
    blescan_start(); // Start scanning
}

/**
 * @brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 * @param[in] battery_level Battery level value to be updated.
 */
void battery_level_update(uint8_t battery_level)
{
    ret_code_t err_code;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        NRF_LOG_INFO("battery_level_update error case 5\r\n");
        APP_ERROR_HANDLER(err_code);
    }
}

/**
 * @brief Live Alert to BLE
 * @param[in] alert Alerts
 */
ret_code_t live_alert_update(uint8_t *alert)
{
    ret_code_t err_code;

    err_code = ble_data_live_value_update(&m_data, alert);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        NRF_LOG_INFO("rerr_code-> %d\n\r", err_code);
        NRF_LOG_INFO("live_alert_update error case 6\r\n");
        APP_ERROR_HANDLER(err_code);
    }
    return err_code;
}

/**
 * @brief Threshold Setting to BLE
 * @param[in] th Threshold
 */
void th_settings_update(uint8_t *th, size_t len)
{
    ret_code_t err_code;

    err_code = ble_settingss_th_value_update(&m_settings, th, len);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        NRF_LOG_INFO("th_settings_update error case 7\r\n");
        printf("th_settings_update error case: %d\n", err_code);
        APP_ERROR_HANDLER(err_code);
    }
}

/**
 * @brief Send Acc Mean to BLE
 * @param[in] mean
 */
void sendMean(uint8_t *mean)
{
    ret_code_t err_code;

    uint16_t length = strlen(mean);
    err_code = ble_nus_data_send(&m_nus, mean, &length, m_cur_conn_handle);
    if ((err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_NOT_FOUND))
    {
        APP_ERROR_CHECK(err_code);
    }
}
/**
 * @brief Wake Up BLE Advertising
 *
 */
void wakeUpBLE(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
/**
 * @brief Raw Data to BLE
 * * @param[in] alert Alerts
 */
void raw_data_update(uint8_t *alert)
{
    ret_code_t err_code;

    err_code = ble_data_acc_gya_row_value_update(&m_data, alert);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        NRF_LOG_INFO("rerr_code-> %d\n\r", err_code);
        NRF_LOG_INFO("raw data error case 6\r\n");
        APP_ERROR_HANDLER(err_code);
    }
    return err_code;
}
