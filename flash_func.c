/**

 * @brief This function is used to set the hook mode, buzzer mode, get the axis threshold string packet and get the device stats.
 *
 * @param hook pointer to the string which contains the command.
 *
 * @details This function takes a pointer to a string which contains the command. The function is used to set the hook mode, buzzer mode,
 * get the axis threshold string packet and get the device stats. The command can be "SET_HM:<value>", "SET_BE:<value>",
 * "STEP_EN:<value>", "GET_STR", "GET_KEY", "GET_DS", "GET_HS", "GET_THV", "GET_RD" or "1".
 *
 * If the command is "SET_HM:<value>", it sets the hook mode to the given value. If the value is 1, it enables the hook mode and
 * if the value is 0, it disables the hook mode. If the hook mode is enabled, it sets the buzzer to on and the blue led to off.
 * If the hook mode is disabled, it sets the buzzer to off and the blue led to on.
 *
 * If the command is "SET_BE:<value>", it sets the buzzer mode to the given value. If the value is 1, it enables the buzzer and
 * if the value is 0, it disables the buzzer.
 *
 * If the command is "STEP_EN:<value>", it sets the step enable to the given value. If the value is 1, it enables the step and
 * if the value is 0, it disables the step.
 *
 * If the command is "GET_STR", it gets the axis threshold string packet. The packet is of the form
 * "{bp:<value>,bt:<value>,wp:<value>,MC:<value>,MA:<value>,xl:<value>,xg:<value>,yl:<value>,yg:<value>,zl:<value>,zg:<value>,al:<value>,ag:<value>,rl:<value>,rg:<value>}".
 *
 * If the command is "GET_KEY", it gets the key id. The key id is of the form
 * "{KEY_ID:<value><value><value><value><value><value><value><value><value><value><value><value><value><value><value><value>}".
 *
 * If the command is "GET_DS", it gets the device stats. The device stats is of the form
 * "{mac:<value><value><value>,DS_Master:<value>,key:<value><value><value><value><value><value><value><value><value><value><value><value><value><value><value>}".
 *
 * If the command is "GET_HS", it gets the hook stats. The hook stats is of the form
 * "{mac:<value><value><value>,HS:<value>}".
 *
 * If the command is "GET_THV", it gets the threshold values. The threshold values are of the form
 * "{mac:<value><value><value>,time:<value>,xv:<value>,yv:<value>,zv:<value>,av:<value>}".
 *
 * If the command is "GET_RD", it gets the rotation data. The rotation data is of the form
 * "{mac:<value><value><value>,time:<value>,ccr:<value>,acr:<value>}".
 *
 * If the command is "1", it enters the dfu mode.
 */

#include "main.h"
#include "flash_func.h"
#include "ble_func.h"
#include "calendar.h"
#include "acc.h"
#include "bsp.h"
#include "ble_gap.h"

/* =========================================================
 * BLE / Scan / Restart Control
 * ========================================================= */
extern ble_gap_addr_t gap_addr;
extern bool scan_hook_stats;
// extern bool hook_test_ok;

extern bool restart;

/* =========================================================
 * Logging
 * ========================================================= */
#define MAX_LOG_LEN 256

/* =========================================================
 * Bit / Command Flags
 * ========================================================= */
bool bit1_on = false;
bool bit2_on = false;
bool bit3_on = false;

bool buz_hmcmd_ex = false;

/* =========================================================
 * Axis Threshold Flags (Instant Detection)
 * ========================================================= */
bool xless_flag   = false;   // X-axis less-than threshold crossed
bool xg_flag      = false;   // X-axis greater-than threshold crossed

bool yl_flag      = false;   // Y-axis less-than threshold crossed
bool yg_flag      = false;   // Y-axis greater-than threshold crossed

bool zl_flag      = false;   // Z-axis less-than threshold crossed
bool zg_flag      = false;   // Z-axis greater-than threshold crossed

bool avg_lt_flag  = false;   // Average magnitude less-than threshold
bool avg_gt_flag  = false;   // Average magnitude greater-than threshold

bool rssi_lt_flag = false;   // RSSI less-than threshold crossed
bool rssi_gt_flag = false;   // RSSI greater-than threshold crossed

/* =========================================================
 * Hook / Connection Counters (External)
 * ========================================================= */
extern uint16_t rcc_cnt;
extern uint16_t rac_cnt;

extern uint16_t both_hook_disconnect;
extern uint16_t hc_cnt;
extern uint16_t hd_cnt;

/* =========================================================
 * Threshold State Flags (External – Window/Pattern Logic)
 * ========================================================= */
extern bool x_min;
extern bool x_max;

extern bool y_min;
extern bool y_max;

extern bool z_min;
extern bool z_max;

extern bool net_min;
extern bool net_max;

extern bool pattern_flag;
extern bool ymax_ans_cond;

/* =========================================================
 * Time / HM Stop Control
 * ========================================================= */
bool time_written = false;

APP_TIMER_DEF(TIMER_HMSTOP);
APP_TIMER_DEF(reset_timer_id);

uint32_t hm_stop_cnt = 0;

/* =========================================================
 * Threshold Data Packet
 * ========================================================= */
uint8_t threshold_data_packet[250];

static bool volatile m_fds_initialized;

configuration_t m_device_cfg =
    {
        .name_prefix = DEVICE_NAME_PREFIX,
        .sensor_name_set = SENSOR_NAME_PREFIX,
        .block_serial = "",
        .fall_threshold = FALL_THRESHOLD,
        .ff_threshold = FF_THRESHOLD,
        .test_threshold = TEST_THRESHOLD,
        .wake_threshold = WAKE_THRESHOLD,
        /* changes */

        .th1_threshold = THRESHOLD1,
        .th2_threshold = THRESHOLD2,
        .th3_threshold = THRESHOLD3,

        .th_status = TH_STATUS,
        .rtc_time = 0,
        .cal_mode = 0,
        .hook_mode = HOOK_STATUS,
        .buzzer_onoff_enable = BUZZER_STATUS,
        .buzzertimer_threshold = BUZZERTIMERTHRESHOLD,
        .delaytimer_threshold = DELAYTIMERTHRESHOLD,
        .hm_stopth = HM_STOPTH,
        .hm_startflag = DELAYTIMER_FLAG,
        .hm_stopflag = HM_STOPFLAG,
        .step_en = step_en_status,
        .mclk_en = MCLK_EN,
        .manc_en = MANC_EN,
        .mstop_en = 0,
        .motorclk_flag = 0,
        .motoraclk_flag = 0,
        .beacon_uuid = {0x1E, 0x69, 0x1D, 0x30, 0x7E, 0x08, 0x47, 0xF0,
                        0x8B, 0x3E, 0xA6, 0x35, 0xF3, 0x50, 0x0E, 0x6E},
        .master_device_flag = MASTER_DEVICE_FLAG,
        .slave_device_flag = SLAVE_DEVICE_FLAG,
        .scanwait_time = SCANWAIT_TIME,
        .slave_name = SLAVE_NAME,
        .nm_lan_flag = NM_LAN_FLAG,
        .battery_perc_value = BATTERY_PERC_VALUE,
        .th_data = TH_DATA,
        .lock_test_th = LT_THRESHOLD,
        .fall_test_th = FT_THRESHOLD,
        .x_axis_lt = X_AXIS_LT,
        .x_axis_gt = X_AXIS_GT,
        .y_axis_lt = Y_AXIS_LT,
        .y_axis_gt = Y_AXIS_GT,
        .z_axis_lt = Z_AXIS_LT,
        .z_axis_gt = Z_AXIS_GT,
        .xyz_avg_lt = XYZ_AVG_LT,
        .xyz_avg_gt = XYZ_AVG_GT,
        .rssi_lt = RSSI_LT,
        .rssi_gt = RSSI_GT,
        .hook_beeptime = HOOK_BEEPTIME,
        .zl_flag_start = ZL_FLAG_START,
        .xg_flag_start = XG_FLAG_START,
        .yl_flag_start = YL_FLAG_START,
        .avg_flag_start = AVG_FLAG_START,
        .testX_beep_th = TESTX_BEEP_TH,
        .testY_beep_th = TESTY_BEEP_TH,
        .testZ_beep_th = TESTZ_BEEP_TH,
        .testA_beep_th = TESTA_BEEP_TH,
        .test_high_beep_th = TEST_HIGH_BEEP_TH,
        .test_high_beep_th = TEST_LOW_BEEP_TH,
        .cr_val = CR_VAL,
        .acr_val = ACR_VAL,
        .bhd_val = BHD_VAL,
        .buz_beep_cnt = BUZ_BEEP_CNT,
        .lockth = {0, 0, 0, 0, 0, 0}};

static fds_record_t m_cfg_record =
    {
        .file_id = CONFIG_FILE,
        .key = CONFIG_REC_KEY,
        .data.p_data = &m_device_cfg,
        /* The length of a record is always expressed in 4-byte units (words). */
        .data.length_words = (sizeof(m_device_cfg) + 3) / sizeof(uint32_t),
};

log_configuration_t m_log_cfg =
    {
        .log_count = 0x0,
        .curr_log = 0x0,
};

static fds_record_t m_log_cfg_record =
    {
        .file_id = LOG_CONFIG_FILE,
        .key = LOG_CONFIG_KEY,
        .data.p_data = &m_log_cfg,
        /* The length of a record is always expressed in 4-byte units (words). */
        .data.length_words = (sizeof(m_log_cfg) + 3) / sizeof(uint32_t),
};

rotation_configuration_t m_cnt_cfg =
    {
        .cl = 0,
        .acl = 0,
        .user_time = time_user,
};
static fds_record_t m_cnt_cfg_record =
    {
        .file_id = ROTATION_CONFIG_FILE,
        .key = ROTATION_CONFIG_KEY,
        .data.p_data = &m_cnt_cfg,
        /* The length of a record is always expressed in 4-byte units (words). */
        .data.length_words = (sizeof(m_cnt_cfg) + 3) / sizeof(uint32_t),
};

/* Array to map FDS events to strings. */
static char const *fds_evt_str[] =
    {
        "FDS_EVT_INIT",
        "FDS_EVT_WRITE",
        "FDS_EVT_UPDATE",
        "FDS_EVT_DEL_RECORD",
        "FDS_EVT_DEL_FILE",
        "FDS_EVT_GC",
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next; //!< Delete next record.
    bool pending;     //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.

} m_delete_all;

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const *err_str[] =
        {
            "FDS_ERR_OPERATION_TIMEOUT",
            "FDS_ERR_NOT_INITIALIZED",
            "FDS_ERR_UNALIGNED_ADDR",
            "FDS_ERR_INVALID_ARG",
            "FDS_ERR_NULL_ARG",
            "FDS_ERR_NO_OPEN_RECORDS",
            "FDS_ERR_NO_SPACE_IN_FLASH",
            "FDS_ERR_NO_SPACE_IN_QUEUES",
            "FDS_ERR_RECORD_TOO_LARGE",
            "FDS_ERR_NOT_FOUND",
            "FDS_ERR_NO_PAGES",
            "FDS_ERR_USER_LIMIT_REACHED",
            "FDS_ERR_CRC_CHECK_FAILED",
            "FDS_ERR_BUSY",
            "FDS_ERR_INTERNAL",
        };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

static void fds_evt_handler(fds_evt_t const *p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        // NRF_LOG_GREEN("Event: %s received (NRF_SUCCESS)",
        //               fds_evt_str[p_evt->id]);
    }
    else
    {
        // NRF_LOG_GREEN("Event: %s received (%s)",
        //               fds_evt_str[p_evt->id],
        //               fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
    case FDS_EVT_INIT:
        if (p_evt->result == NRF_SUCCESS)
        {
            m_fds_initialized = true;
        }
        break;

    case FDS_EVT_WRITE:
    {
        if (p_evt->result == NRF_SUCCESS)
        {
            // NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
            // NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
            // NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
        }
    }
    break;

    case FDS_EVT_UPDATE:
    {
        if (p_evt->result == NRF_SUCCESS)
        {
            // NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
            // NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
            // NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
        }
    }
    break;

    case FDS_EVT_DEL_RECORD:
    {
        if (p_evt->result == NRF_SUCCESS)
        {
            // NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
            // NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
            // NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
        }
        m_delete_all.pending = false;
    }
    break;

    default:
        break;
    }
    if (p_evt->write.file_id == CONFIG_FILE)
    {
        if (restart == true)
        {
            rebootDevice();
        }
        time_written = true;
    }
}

/**
 * @brief   Begin deleting all records, one by one. */
void delete_all_begin(void)
{
    m_delete_all.delete_next = true;
}

/**
 * @brief   Process a delete all command.
 *
 * Delete records, one by one, until no records are left.
 */
void delete_all_process(void)
{
    if (m_delete_all.delete_next & !m_delete_all.pending)
    {
        NRF_LOG_INFO("Deleting next record.");

        m_delete_all.delete_next = record_delete_next();
        if (!m_delete_all.delete_next)
        {
            NRF_LOG_CYAN("No records left to delete.");
        }
    }
}

/*
 * @brief   Sleep until an event is received.   
 */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void)sd_app_evt_wait();
#else
    __WFE();
#endif
}

/**
 * @brief   Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        power_manage();
    }
}

uint32_t flash_init(void)
{

    ret_code_t err_code;

    (void)fds_register(fds_evt_handler);

    NRF_LOG_INFO("Initializing fds...");
    err_code = fds_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("FDS Init Failed");
    }
    wait_for_fds_ready();

    fds_stat_t stat = {0};

    err_code = fds_stat(&stat);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    NRF_LOG_INFO("Found %d Pages available.", stat.pages_available);

    if (stat.dirty_records > 0)
    {
        fds_gc();
    }

    return err_code;
}

void motor_reset()
{
    ret_code_t err_code;
    m_device_cfg.motorclk_flag = 0;
    m_device_cfg.motoraclk_flag = 0;
    nrf_gpio_pin_clear(MOTOR_PIN1); // MOTOR_PIN1 Low
    nrf_gpio_pin_clear(MOTOR_PIN2); // MOTOR_PIN2 Low
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cfg_record);
        if (err_code == FDS_ERR_NO_SPACE_IN_FLASH)
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

void reset_device_state(void)
{
    ret_code_t err_code;
    static uint16_t tm_cur_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
    m_device_cfg.hook_mode = 0;
    m_device_cfg.hm_stopflag = 0;
    m_device_cfg.hm_startflag = 0;

    m_device_cfg.nm_lan_flag = 0;
    scan_hook_stats = false;

    hm_stop_cnt = 0;
    buz_en = 0;
    delay_counter = 0;
    nrf_gpio_pin_clear(BUZZER_PIN);
    nrf_gpio_pin_set(BUZZER_LED); // Buzzer LED off

    // memset(temp2, 0, sizeof(temp2));

    // sprintf(temp2, "{ct:%s,alerthm:STOP}", timespm); // save hook history log in mem after disconn.
    sprintf(AlertStr, "{mac:%X%X%X,time:%s,alerthm:STOP}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm);
    AddLog(AlertStr);
    live_alert_update(AlertStr);
    if (conn_flag)
    {
        // sprintf(temp2, "{ct:%s,alerthm:STOP}", timespm); // save hook history log in mem after disconn.
        // live_alert_update(AlertStr);
        // AddLog(temp2);
        nrf_gpio_pin_clear(BATTERY_ALERT_PIN); // blue LED on
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    if (!his_notify)
    {
        // sprintf(temp2, "{ct:%s,alerthm:STOP}", timespm); // save hook history log in mem after disconn.
        // AddLog(AlertStr);
        tm_cur_conn_handle = BLE_CONN_HANDLE_INVALID;
    }

    uart_trasmit_str(AlertStr);

    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cfg_record);
        if (err_code == FDS_ERR_NO_SPACE_IN_FLASH)
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

void hmstop_timer_handler(void *p_context)
{
    if (m_device_cfg.hook_mode == 1 && m_device_cfg.hm_stopflag == 1 || m_device_cfg.slave_device_flag == 1 && m_device_cfg.hm_stopflag == 1 || m_device_cfg.master_device_flag == 1 && m_device_cfg.hm_stopflag == 1)
    {
        hm_stop_cnt++;
        printf("hm_stop_cnt: %d\n", hm_stop_cnt);

        if (hm_stop_cnt >= m_device_cfg.hm_stopth)
        {
            reset_device_state();
            // rebootDevice();
            printf("<<----- HM_STOP! ----->>\n");
        }
    }
}

void reset_timer_handler(void *p_context)
{
    printf("Rebooting device...\n");
    rebootDevice();
}

void schedule_reset()
{
    app_timer_start(reset_timer_id, APP_TIMER_TICKS(3000), NULL); // 3s delay
}

uint32_t config_init(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    uint8_t bit_0, bit_1, bit_2, bit_3, bit_4, bit_5;

    err_code = app_timer_create(&TIMER_HMSTOP, APP_TIMER_MODE_REPEATED, hmstop_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&reset_timer_id, APP_TIMER_MODE_SINGLE_SHOT, reset_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(TIMER_HMSTOP, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        err_code = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(err_code);

        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(&m_device_cfg, config.p_data, sizeof(configuration_t));

        //  NRF_LOG_INFO("CONFIG\r\nFall Arrest Threshold:%d\r\nFF Threshold:%d\r\nTest Threshold:%d\r\nWake Threshold:%d\r\n",m_device_cfg.fall_threshold,m_device_cfg.ff_threshold,m_device_cfg.test_threshold,m_device_cfg.wake_threshold,m_device_cfg.th1_threshold);
        // NRF_LOG_INFO("CONFIG\r\nWake Threshold:%d\r\n", m_device_cfg.wake_threshold);
        // printf("<<----- buzzer_beeptimer is: %dseconds ----->>\n", m_device_cfg.buzzertimer_threshold);
        // printf("<<----- buzzerhook_Delaytimer is: %dseconds ----->>\n", m_device_cfg.delaytimer_threshold);
        // printf("<<----- hm_stopth is: %dseconds ----->>\n", m_device_cfg.hm_stopth);
        printf("Slave_Sensor_name:%s\n", m_device_cfg.sensor_name_set);
        printf("MASTER_DEVICE_STATE:%d\n", m_device_cfg.master_device_flag);
        printf("SLAVE_DEVICE_STATE:%d\n", m_device_cfg.slave_device_flag);
        // printf("slave_name:%s\n", m_device_cfg.slave_name);
        // printf("Battery Low Thrs is:%d\n", m_device_cfg.battery_perc_value);
        // printf(" <<----- Beacon_UUID: ");
        // for (int i = 0; i < 16; i++)
        // {
        //     printf("%02X", m_device_cfg.beacon_uuid[i]);
        // }
        // printf(" ----->>\n");

        // printf("\n===== Device Threshold Configuration =====\n");
        // printf("lock_test_th         : %d\n", m_device_cfg.lock_test_th);
        // printf("fall_test_th         : %d\n", m_device_cfg.fall_test_th);
        // printf("battery_perc_value   : %d\n", m_device_cfg.battery_perc_value);

        // printf("x_axis_lt            : %d\n", m_device_cfg.x_axis_lt);
        // printf("x_axis_gt            : %d\n", m_device_cfg.x_axis_gt);
        // printf("y_axis_lt            : %d\n", m_device_cfg.y_axis_lt);
        // printf("y_axis_gt            : %d\n", m_device_cfg.y_axis_gt);
        // printf("z_axis_lt            : %d\n", m_device_cfg.z_axis_lt);
        // printf("z_axis_gt            : %d\n", m_device_cfg.z_axis_gt);

        // printf("xyz_avg_lt           : %d\n", m_device_cfg.xyz_avg_lt);
        // printf("xyz_avg_gt           : %d\n", m_device_cfg.xyz_avg_gt);

        // printf("rssi_lt              : %d\n", m_device_cfg.rssi_lt);
        // printf("rssi_gt              : %d\n", m_device_cfg.rssi_gt);

        // printf("hook_beeptime        : %d\n", m_device_cfg.hook_beeptime);
        // printf("wake_threshold       : %d\n", m_device_cfg.wake_threshold);
        // printf("buzzertimer_threshold: %d\n", m_device_cfg.buzzertimer_threshold);
        // printf("delaytimer_threshold : %d\n", m_device_cfg.delaytimer_threshold);
        // printf("hm_stopth            : %d\n", m_device_cfg.hm_stopth);

        // printf("mclk_en              : %d\n", m_device_cfg.mclk_en);
        // printf("manc_en              : %d\n", m_device_cfg.manc_en);
        // printf("mstop_en             : %d\n", m_device_cfg.mstop_en);

        // printf("scanwait_time        : %d\n", m_device_cfg.scanwait_time);
        // printf("slave_name           : %s\n", m_device_cfg.slave_name);

        // printf("UUID (beacon_uuid)   : ");
        // for (int i = 0; i < 16; i++)
        //     printf("%02X", m_device_cfg.beacon_uuid[i]);
        // printf("\n");
        // printf("==========================================\n\n");

        bit_0 = m_device_cfg.th_status & 0x01; // fall arrest
        bit_1 = m_device_cfg.th_status & 0x02; // test
        bit_2 = m_device_cfg.th_status & 0x04; // free fall

        /* changes */

        bit_3 = m_device_cfg.th_status & 0x08; // th1
        bit_4 = m_device_cfg.th_status & 0x10; // th2
        bit_5 = m_device_cfg.th_status & 0x20; // th3

        if (bit_0 == 1)
            NRF_LOG_INFO("\r\nFall Arrest Threshold:%d\r\n", m_device_cfg.fall_threshold);
        if (bit_1 == 2)
            NRF_LOG_INFO("\r\nTest Threshold:%d\r\n", m_device_cfg.test_threshold);
        if (bit_2 == 4)
            NRF_LOG_INFO("\r\nFF Threshold:%d\r\n", m_device_cfg.ff_threshold);

        /* changes */

        if (bit_3 == 8)
            NRF_LOG_INFO("\r\nTH1=%d\r\n", m_device_cfg.th1_threshold);
        if (bit_4 == 16)
            NRF_LOG_INFO("\r\nTH2=%d\r\n", m_device_cfg.th2_threshold);
        if (bit_5 == 32)
            NRF_LOG_INFO("\r\nTH3=%d\r\n", m_device_cfg.th3_threshold);

        if (m_device_cfg.rtc_time != 0)
        {
            nrf_cal_set_time_t(m_device_cfg.rtc_time);
            NRF_LOG_INFO("Calibrated time:\t%s\r\n", nrf_cal_get_time_string(false));
        }
        /* Close the record when done reading. */
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");

        err_code = fds_record_write(NULL, &m_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }

    memset(&desc, 0, sizeof(desc));
    memset(&tok, 0, sizeof(tok));
    err_code = fds_record_find(LOG_CONFIG_FILE, LOG_CONFIG_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t log_config = {0};

        /* Open the record and read its contents. */
        err_code = fds_record_open(&desc, &log_config);
        APP_ERROR_CHECK(err_code);

        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(&m_log_cfg, log_config.p_data, sizeof(log_configuration_t));

        NRF_LOG_INFO("Log Count:%d Curr Log:%d", m_log_cfg.log_count, m_log_cfg.curr_log);

        /* Close the record when done reading. */
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_INFO("Writing log config file...");

        err_code = fds_record_write(NULL, &m_log_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the log config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }

    memset(&desc, 0, sizeof(desc));
    memset(&tok, 0, sizeof(tok));
    err_code = fds_record_find(ROTATION_CONFIG_FILE, ROTATION_CONFIG_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t cnt_config = {0};

        /* Open the record and read its contents. */
        err_code = fds_record_open(&desc, &cnt_config);
        APP_ERROR_CHECK(err_code);

        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(&m_cnt_cfg, cnt_config.p_data, sizeof(rotation_configuration_t));

        NRF_LOG_INFO("***Num of Count: cl: %d , acl: %d", m_cnt_cfg.cl, m_cnt_cfg.acl);
        RotationF = m_cnt_cfg.cl;
        RotationR = m_cnt_cfg.acl;
        fix_time = m_cnt_cfg.user_time;
        /* Close the record when done reading. */
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_INFO("Writing log config file...");

        err_code = fds_record_write(NULL, &m_cnt_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the log config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    return err_code;
}

bool parse_uuid_string(const char *uuid_str, uint8_t *uuid_out)
{
    // Validate length (32 hex chars)
    if (strlen(uuid_str) != 32)
    {
        printf("Error: UUID must be 32 hex characters\n");
        return false;
    }

    // Convert each byte
    for (int i = 0; i < 16; i++)
    {
        char byte_str[3] = {uuid_str[i * 2], uuid_str[i * 2 + 1], '\0'};
        char *endptr;

        long val = strtol(byte_str, &endptr, 16);
        if (*endptr != '\0' || val < 0 || val > 255)
        {
            printf("Error: Invalid hex at position %d\n", i * 2);
            return false;
        }

        uuid_out[i] = (uint8_t)val;
    }
    return true;
}

void setThreshold(uint8_t *threshold)
{

    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    // uint8_t th[100];

    NRF_LOG_INFO("Threshold:%s", threshold);

    uint8_t *p = strstr(threshold, ":");
    *p++;

    if (strstr(threshold, "SET_FA_1"))
    {
        m_device_cfg.fall_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status | 0X01;
    }
    else if (strstr(threshold, "SET_TS_1"))
    {
        m_device_cfg.test_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status | 0X02;
    }
    else if (strstr(threshold, "SET_FF_1"))
    {
        m_device_cfg.ff_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status | 0X04;
    }
    else if (strstr(threshold, "SET_FA_0"))
    {
        m_device_cfg.fall_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status & ~0X01;
    }
    else if (strstr(threshold, "SET_TS_0"))
    {
        m_device_cfg.test_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status & ~0X02;
    }
    else if (strstr(threshold, "SET_FF_0"))
    {
        m_device_cfg.ff_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status & ~0X04;
    }
    else if (strstr(threshold, "SET_WUP"))
    {
        uint8_t wake_th = atoi(p);
        if (wake_th < 8 && wake_th > 0)
        {
            NRF_LOG_INFO("WAKE_UP:%s", p);
            m_device_cfg.wake_threshold = atoi(p);
        }
    }
    else if (strstr(threshold, "SET_BT"))
    {
        uint8_t buzzer_th = atoi(p);

        if (buzzer_th <= 10)
        {
            m_device_cfg.buzzertimer_threshold = buzzer_th;
            printf("<<----- BT_SET_SUCCESSFULLY TO:%s ----->>\n", p);
        }
    }
    else if (strstr(threshold, "SET_HT"))
    {
        m_device_cfg.delaytimer_threshold = atoi(p);
        // printf("<<----- HT_SET_SUCCESSFULLY TO:%s ----->>\n", p);
        m_device_cfg.hm_startflag = 1;
        buz_hmcmd_ex = true;

        m_device_cfg.nm_lan_flag = 1;

        m_device_cfg.hook_mode = 1;
        nrf_gpio_pin_clear(BUZZER_PIN);
        nrf_gpio_pin_set(BUZZER_LED); // Buzzer LDE off
        last_time = t1;
        FTCT = 0;
        buz_cnt = 0;
        buz_10s_cnt = 0;
        buz_stop = 0;
        sensor_detect = 0;
        buz_en = 0;
        m_device_cfg.buzzer_onoff_enable = 0;

        memset(temp2, 0, sizeof(temp2));
        sprintf(temp2, "{mac:%X%X%X,time:%s,alerthm:SET_HT-%s}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, p);
        AddLog(temp2);
        live_alert_update(temp2);
        uart_trasmit_str(temp2);

        if (conn_flag)
        {
            // live_alert_update(temp2);
            // AddLog(temp2);

            nrf_gpio_pin_set(BATTERY_ALERT_PIN); // blue led off
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
        }
        // if (!his_notify)
        // {
        // AddLog(temp2);
        // }
    }
    else if (strstr(threshold, "SET_ST"))
    {
        m_device_cfg.hm_stopth = atoi(p);
        // printf("<<----- ST_SET_SUCCESSFULLY TO:%s ----->>\n", p);
        m_device_cfg.hm_stopflag = 1;

        memset(temp2, 0, sizeof(temp2));
        sprintf(temp2, "{mac:%X%X%X,time:%s,alerthm:SET_ST-%s}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, p);
        uart_trasmit_str(temp2);
        AddLog(temp2);
        live_alert_update(temp2);

        // if (conn_flag)
        // {
        //     live_alert_update(temp2);
        //     // AddLog(temp2);
        // }
        // if (!his_notify)
        // {
        //     AddLog(temp2);
        // }
    }
    else if (strstr(threshold, "SET_MC"))
    {
        m_device_cfg.mclk_en = atoi(p);
        printf("motor moves in clockwise mode set for %s seconds\n", p);
        m_device_cfg.motorclk_flag = 1;
    }
    else if (strstr(threshold, "SET_MAC"))
    {
        m_device_cfg.manc_en = atoi(p);
        printf("motor moves in anti-clockwise mode set for %s seconds\n", p);
        m_device_cfg.motoraclk_flag = 1;
    }
    else if (strstr(threshold, "SET_MS"))
    {
        m_device_cfg.mstop_en = atoi(p);
        if (m_device_cfg.mstop_en == 1)
        {
            motor_reset();
            printf("motor stopped...\n");
        }
    }
    else if (strstr(threshold, "SET_WT"))
    {
        m_device_cfg.scanwait_time = atoi(p);
        printf("scan_wait_time set successfully to: %s\n", p);
    }
    else if (strstr(threshold, "SET_SN"))
    {
        strncpy((char *)m_device_cfg.slave_name, p, sizeof(m_device_cfg.slave_name) - 1);
        m_device_cfg.slave_name[sizeof(m_device_cfg.slave_name) - 1] = '\0';
        printf("slave_name set successfully to: %s\n", m_device_cfg.slave_name);
    }
    // else if (strstr(threshold, "SET_LT"))
    // {
    //     m_device_cfg.lock_test_th = atoi(p);
    //     printf("lock test thrs set to %s successfully\n", p); // lock test threshold set
    //     // schedule_reset();
    // }
    // else if (strstr(threshold, "SET_FT"))
    // {
    //     m_device_cfg.fall_test_th = atoi(p);
    //     printf("fall test thrs set to %s successfully\n", p); // fall test threshold set
    //     // schedule_reset();
    // }
    else if (strstr(threshold, "SET_BP"))
    {
        m_device_cfg.battery_perc_value = atoi(p);
        printf("battery low threshold set to %s successfully\n", p); // battery percent threshold set
        // schedule_reset();
    }
    else if (strstr(threshold, "SET_XL")) // x axis less than threshold set
    {
        xless_flag = true;
        printf("xl_flag_status:%s\n", xless_flag ? "TRUE" : "FALSE");
        m_device_cfg.x_axis_lt = atoi(p);
        printf("xl_val is set to %s successfully\n", p);

        if (m_device_cfg.x_axis_lt == 0)
        {
            xless_flag = false;
            printf("xl_flag_status:%s\n", xless_flag ? "TRUE" : "FALSE");
        }
    }
    else if (strstr(threshold, "SET_XG")) // x axis greater than threshold set
    {
        xg_flag = true;
        printf("xg_flag_status:%s\n", xg_flag ? "TRUE" : "FALSE");
        m_device_cfg.x_axis_gt = atoi(p);
        printf("xg_val is set to %s successfully\n", p);
        m_device_cfg.xg_flag_start = 1;

        if (m_device_cfg.x_axis_gt == 0)
        {
            xg_flag = false;
            printf("xg_flag_status:%s\n", xg_flag ? "TRUE" : "FALSE");
            m_device_cfg.xg_flag_start = 0;
        }
    }
    else if (strstr(threshold, "SET_YL")) // y axis less than threshold set
    {
        // yl_flag = true;
        // printf("yl_flag_status:%s\n", yl_flag ? "TRUE" : "FALSE");
        m_device_cfg.y_axis_lt = atoi(p);
        printf("yl_val is set to %s successfully\n", p);
        // m_device_cfg.yl_flag_start = 1;

        // if (m_device_cfg.y_axis_lt == 0)
        // {
        //     m_device_cfg.yl_flag_start = 0;
        //     yl_flag = false;
        //     printf("yl_flag_status:%s\n", yl_flag ? "TRUE" : "FALSE");
        // }
    }
    else if (strstr(threshold, "SET_YG")) // y axis greater than threshold set
    {
        // yg_flag = true;
        // printf("yg_flag_status:%s\n", yg_flag ? "TRUE" : "FALSE");
        m_device_cfg.y_axis_gt = atoi(p);
        printf("yg_val is set to %s successfully\n", p);

        // if (m_device_cfg.y_axis_gt == 0)
        // {
        //     yg_flag = false;
        //     printf("yg_flag_status:%s\n", yg_flag ? "TRUE" : "FALSE");
        // }
    }
    else if (strstr(threshold, "SET_ZL")) // z axis less than threshold set
    {
        zl_flag = true;
        printf("zl_flag_status:%s\n", zl_flag ? "TRUE" : "FALSE");
        m_device_cfg.z_axis_lt = atoi(p);
        printf("zl_val is set to %s successfully\n", p);
        m_device_cfg.zl_flag_start = 1;

        if (m_device_cfg.z_axis_lt == 0)
        {
            zl_flag = false;
            printf("zl_flag_status:%s\n", zl_flag ? "TRUE" : "FALSE");
            m_device_cfg.zl_flag_start = 0;
        }
    }
    else if (strstr(threshold, "SET_ZG")) // z axis greater than threshold set
    {
        zg_flag = true;
        printf("zg_flag_status:%s\n", zg_flag ? "TRUE" : "FALSE");
        m_device_cfg.z_axis_gt = atoi(p);
        printf("zg_val is set to %s successfully\n", p);

        if (m_device_cfg.z_axis_gt == 0)
        {
            zg_flag = false;
            printf("zg_flag_status:%s\n", zg_flag ? "TRUE" : "FALSE");
        }
    }
    else if (strstr(threshold, "SET_AL")) // average less than threshold set
    {
        avg_lt_flag = true;
        printf("al_flag_status:%s\n", avg_lt_flag ? "TRUE" : "FALSE");
        m_device_cfg.xyz_avg_lt = atoi(p);
        printf("al_val is set to %s successfully\n", p);

        if (m_device_cfg.xyz_avg_lt == 0)
        {
            avg_lt_flag = false;
            printf("al_flag_status:%s\n", avg_lt_flag ? "TRUE" : "FALSE");
        }
    }
    else if (strstr(threshold, "SET_AG")) // average greater than threshold set
    {
        avg_gt_flag = true;
        printf("ag_flag_status:%s\n", avg_gt_flag ? "TRUE" : "FALSE");
        m_device_cfg.xyz_avg_gt = atoi(p);
        printf("ag_val is set to %s successfully\n", p);
        m_device_cfg.avg_flag_start = 1;

        if (m_device_cfg.xyz_avg_gt == 0)
        {
            avg_gt_flag = false;
            printf("ag_flag_status:%s\n", avg_gt_flag ? "TRUE" : "FALSE");
            m_device_cfg.avg_flag_start = 0;
        }
    }
    else if (strstr(threshold, "SET_RL")) // rssi less than threshold set
    {
        rssi_lt_flag = true;
        printf("rl_flag_status:%s\n", rssi_lt_flag ? "TRUE" : "FALSE");
        m_device_cfg.rssi_lt = atoi(p);
        printf("rg_val is set to %s successfully\n", p);

        if (m_device_cfg.rssi_lt == 0)
        {
            rssi_lt_flag = false;
            printf("rl_flag_status:%s\n", rssi_lt_flag ? "TRUE" : "FALSE");
        }
    }
    else if (strstr(threshold, "SET_RG")) // rssi greater than threshold set
    {
        rssi_gt_flag = true;
        printf("rg_flag_status:%s\n", rssi_gt_flag ? "TRUE" : "FALSE");
        m_device_cfg.rssi_gt = atoi(p);
        printf("rg_val is set to %s successfully\n", p);

        if (m_device_cfg.rssi_gt == 0)
        {
            rssi_gt_flag = false;
            printf("rg_flag_status:%s\n", rssi_gt_flag ? "TRUE" : "FALSE");
        }
    }
    else if (strstr(threshold, "SET_HBT")) // hook beep time threshold set
    {
        m_device_cfg.hook_beeptime = atoi(p);
        printf("HBT is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_TX")) // x_val high
    {
        m_device_cfg.testX_beep_th = atoi(p);
        printf("TX is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_TY")) // y_val high
    {
        m_device_cfg.testY_beep_th = atoi(p);
        printf("TY is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_TZ")) // z_val high
    {
        m_device_cfg.testZ_beep_th = atoi(p);
        printf("MV is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_TA")) // mean_val high
    {
        m_device_cfg.testA_beep_th = atoi(p);
        printf("TA is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_THT")) // high/max_th val
    {
        m_device_cfg.test_high_beep_th = atoi(p);
        printf("tht is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_TLT")) // low/min_th val
    {
        m_device_cfg.test_low_beep_th = atoi(p);
        printf("tlt is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_CR")) // clock rotation
    {
        m_device_cfg.cr_val = atoi(p);
        rcc_cnt = m_device_cfg.cr_val;
        printf("cr_val is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_ACR")) // anticlock rotation
    {
        m_device_cfg.acr_val = atoi(p);
        rac_cnt = m_device_cfg.acr_val;
        printf("acr_val is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_BDV")) // both disconnect value update
    {
        m_device_cfg.bhd_val = atoi(p);
        both_hook_disconnect = m_device_cfg.bhd_val;
        printf("bhd_val is set to %s successfully\n", p);
    }
    else if (strstr(threshold, "SET_FST")) // fall sound threshold value
    {
        m_device_cfg.buz_beep_cnt = atoi(p);
        printf("buz_beep_cnt is set to %s successfully\n", p);
    }
    else if (strstr((char *)threshold, "SET_AX"))
    {
        char *p = strchr((char *)threshold, ':');
        if (p != NULL)
        {
            p++; // move to first number after ':'

            int i = 0;
            char *token = strtok(p, ",");
            while (token != NULL && i < MAX_THRESHOLDS)
            {
                m_device_cfg.lockth[i] = atoi(token);
                i++;
                token = strtok(NULL, ",");
                // printf("AV[%d]=%d", i, m_device_cfg.lockth[i]);
            }
        }
    }
    else if (strstr(threshold, "SET_UUID"))
    {
        if (strlen(p) == 32)
        {
            uint8_t uuid[16];
            if (parse_uuid_string(p, uuid))
            {
                if (m_device_cfg.slave_device_flag == 0)
                {
                    m_device_cfg.master_device_flag = 1;

                    memset(m_device_cfg.beacon_uuid, 0, sizeof(m_device_cfg.beacon_uuid));
                    memcpy(m_device_cfg.beacon_uuid, uuid, 16);
                    printf("<<----- UUID SET SUCCESSFULLY TO: %s ----->>\n", p);

                    sprintf(AlertStr,
                            "{mac:%X%X%X,KEY_ID:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X}",
                            gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0],
                            m_device_cfg.beacon_uuid[0], m_device_cfg.beacon_uuid[1],
                            m_device_cfg.beacon_uuid[2], m_device_cfg.beacon_uuid[3],
                            m_device_cfg.beacon_uuid[4], m_device_cfg.beacon_uuid[5],
                            m_device_cfg.beacon_uuid[6], m_device_cfg.beacon_uuid[7],
                            m_device_cfg.beacon_uuid[8], m_device_cfg.beacon_uuid[9],
                            m_device_cfg.beacon_uuid[10], m_device_cfg.beacon_uuid[11],
                            m_device_cfg.beacon_uuid[12], m_device_cfg.beacon_uuid[13],
                            m_device_cfg.beacon_uuid[14], m_device_cfg.beacon_uuid[15]);

                    AddLog(AlertStr);
                }
            }
            else
            {
                printf("<<----- INVALID UUID FORMAT ----->>\n");
            }
        }
        else
        {
            printf("<<----- UUID LENGTH MUST BE 32 HEX CHARACTERS ----->>\n");
        }
    }

    /* changes */

    else if (strstr(threshold, "SET_TH1_1"))
    {
        m_device_cfg.th1_threshold = atoi(p);
        // m_device_cfg.th_status = m_device_cfg.th_status | 0X08;
        // NRF_LOG_INFO("ON TH1");
        // printf("th1 enabled\n");
        // bit1_on = true;

        if (m_device_cfg.th1_threshold == 1)
        {
            nrf_gpio_pin_set(BUZZER_PIN);
            nrf_gpio_pin_clear(BUZZER_LED);
        }
        else if (m_device_cfg.th1_threshold == 0)
        {
            nrf_gpio_pin_clear(BUZZER_PIN);
        }
        printf("Received command: %s, Value: %d\n", threshold, m_device_cfg.th1_threshold);
    }

    else if (strstr(threshold, "SET_TH1_0"))
    {
        m_device_cfg.th1_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status & ~0X08;
        NRF_LOG_INFO("OFF TH1");
        printf("th1 disabled\n");
        bit1_on = false;
        printf("Received command: %s, Value: %d\n", threshold, m_device_cfg.th1_threshold);
    }

    else if (strstr(threshold, "SET_TH2_1"))
    {
        m_device_cfg.th2_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status | 0X10;
        NRF_LOG_INFO("ON TH2");
        printf("th2 enabled\n");
        bit2_on = true;
        printf("Received command: %s, Value: %d\n", threshold, m_device_cfg.th2_threshold);
    }

    else if (strstr(threshold, "SET_TH2_0"))
    {
        m_device_cfg.th2_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status & ~0X10;
        NRF_LOG_INFO("OFF TH2");
        printf("th2 disabled\n");
        bit2_on = false;
        printf("Received command: %s, Value: %d\n", threshold, m_device_cfg.th2_threshold);
    }

    else if (strstr(threshold, "SET_TH3_1"))
    {
        m_device_cfg.th3_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status | 0X20;
        NRF_LOG_INFO("ON TH3");
        printf("th3 enabled\n");
        bit3_on = true;
        printf("Received command: %s, Value: %d\n", threshold, m_device_cfg.th3_threshold);
    }

    else if (strstr(threshold, "SET_TH3_0"))
    {
        m_device_cfg.th3_threshold = atoi(p);
        m_device_cfg.th_status = m_device_cfg.th_status & ~0X20;
        NRF_LOG_INFO("OFF TH3");
        printf("th3 disabled\n");
        bit3_on = false;
        printf("Received command: %s, Value: %d\n", threshold, m_device_cfg.th3_threshold);
    }

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }

    return err_code;
}

void dfu_process(void)
{
    printf("fw mode\n");
    sd_power_gpregret_clr(0, 0xFF);                 // GPREGRET[0]
    sd_power_gpregret_set(0, BOOTLOADER_DFU_START); // GPREGRET[0]
}

/**
 * @brief Set hook mode command and buzzer status.
 *
 * @param hook pointer to the hook mode command string.
 */
void set_hook_mode(char *hook)
{

    // set hook mode cmd and buzzer status

    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    uint8_t *p = strstr(hook, ":");
    *p++;
    if (strstr(hook, "SET_HM"))
    {
        m_device_cfg.hook_mode = atoi(p);
        // if (m_device_cfg.hook_mode == 1)
        // {
        //     NRF_LOG_INFO("hook_mode: ON");
        //     printf("hook_mode: ON\n");
        //     nrf_gpio_pin_clear(BUZZER_PIN);
        //     nrf_gpio_pin_set(BUZZER_LED); // Buzzer LDE off
        //     last_time = t1;
        //     // buz_en = 1;
        //     FTCT = 0;
        //     buz_cnt = 0;
        //     buz_10s_cnt = 0;
        //     buz_stop = 0;
        //     sensor_detect = 0;
        //     buz_en = 0;
        //     m_device_cfg.buzzer_onoff_enable = 0;
        // }
        if (m_device_cfg.hook_mode == 0)
        {
            NRF_LOG_INFO("hook_mode: OFF");
            printf("hook_mode: OFF\n");
            nrf_gpio_pin_clear(BUZZER_PIN);
            nrf_gpio_pin_set(BUZZER_LED); // Buzzer LDE off
            buz_en = 0;
            m_device_cfg.hm_startflag = 0;
            delay_counter = 0;
            printf("<<----- BUZZER DELAY COUNTER DEACTIVATED! ----->>\n");

            if (conn_flag)
                nrf_gpio_pin_clear(BATTERY_ALERT_PIN); // blue led(green) on
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
        }
    }
    else if (strstr(hook, "SET_BE"))
    {
        m_device_cfg.buzzer_onoff_enable = atoi(p);
        if (m_device_cfg.buzzer_onoff_enable == 1)
        {
            NRF_LOG_INFO("Buzzer enable: ON");
            nrf_gpio_pin_clear(BUZZER_PIN);
        }
        if (m_device_cfg.buzzer_onoff_enable == 0)
        {
            NRF_LOG_INFO("Buzzer enable: OFF");
            nrf_gpio_pin_clear(BUZZER_PIN);
            // hook_test_ok = false;

            x_min = false;
            x_max = false;
            y_min = false;
            y_max = false;
            z_min = false;
            z_max = false;
            net_min = false;
            net_max = false;
            pattern_flag = false;
            ymax_ans_cond = false;
        }
    }
    else if (strstr(hook, "STEP_EN"))
    {
        m_device_cfg.step_en = atoi(p);
        if (m_device_cfg.step_en == 1)
        {
            printf("step_en on\n");
        }
        else if (m_device_cfg.step_en == 0)
        {
            printf("step_en off\n");
        }
    }
    else if (strstr(hook, "GET_STR")) // to get axis threshold string packet
    {
        // m_device_cfg.th_data = atoi(p);
        // if (m_device_cfg.th_data == 1)
        // {
        memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
        sprintf(threshold_data_packet, "{bp:%d,bt:%d,wp:%d,MC:%d,MA:%d,xl:%d,xg:%d,yl:%d,yg:%d,zl:%d,zg:%d,al:%d,ag:%d,rl:%d,rg:%d}\n",
                m_device_cfg.battery_perc_value, m_device_cfg.hook_beeptime, m_device_cfg.wake_threshold,
                m_device_cfg.mclk_en, m_device_cfg.manc_en, m_device_cfg.x_axis_lt,
                m_device_cfg.x_axis_gt, m_device_cfg.y_axis_lt, m_device_cfg.y_axis_gt, m_device_cfg.z_axis_lt,
                m_device_cfg.z_axis_gt, m_device_cfg.xyz_avg_lt, m_device_cfg.xyz_avg_gt, m_device_cfg.rssi_lt,
                m_device_cfg.rssi_gt);

        // printf("%s\n", threshold_data_packet);
        live_alert_update(threshold_data_packet);
        uart_trasmit_str(threshold_data_packet);
        // }
    }
    else if (strstr(hook, "GET_KEY")) // to get key id
    {
        // printf(" <<----- Beacon_UUID: ");
        // for (int i = 0; i < 16; i++)
        // {
        //     printf("%02X", m_device_cfg.beacon_uuid[i]);
        // }
        // printf(" ----->>\n");

        memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
        sprintf(threshold_data_packet, "{KEY_ID:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X}",
                m_device_cfg.beacon_uuid[0], m_device_cfg.beacon_uuid[1],
                m_device_cfg.beacon_uuid[2], m_device_cfg.beacon_uuid[3],
                m_device_cfg.beacon_uuid[4], m_device_cfg.beacon_uuid[5],
                m_device_cfg.beacon_uuid[6], m_device_cfg.beacon_uuid[7],
                m_device_cfg.beacon_uuid[8], m_device_cfg.beacon_uuid[9],
                m_device_cfg.beacon_uuid[10], m_device_cfg.beacon_uuid[11],
                m_device_cfg.beacon_uuid[12], m_device_cfg.beacon_uuid[13],
                m_device_cfg.beacon_uuid[14], m_device_cfg.beacon_uuid[15]);

        live_alert_update(threshold_data_packet);
        uart_trasmit_str(threshold_data_packet);
    }
    else if (strstr(hook, "1"))
    {
        dfu_process();
    }
    else if (strstr(hook, "GET_DS")) // to get device stats
    {
        if (m_device_cfg.master_device_flag == 1)
        {
            // printf("Configured as Master\n");
            memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
            sprintf(threshold_data_packet, "{mac:%X%X%X,DS_Master:%d,key:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X}",
                    gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], m_device_cfg.master_device_flag,
                    m_device_cfg.beacon_uuid[0], m_device_cfg.beacon_uuid[1],
                    m_device_cfg.beacon_uuid[2], m_device_cfg.beacon_uuid[3],
                    m_device_cfg.beacon_uuid[4], m_device_cfg.beacon_uuid[5],
                    m_device_cfg.beacon_uuid[6], m_device_cfg.beacon_uuid[7],
                    m_device_cfg.beacon_uuid[8], m_device_cfg.beacon_uuid[9],
                    m_device_cfg.beacon_uuid[10], m_device_cfg.beacon_uuid[11],
                    m_device_cfg.beacon_uuid[12], m_device_cfg.beacon_uuid[13],
                    m_device_cfg.beacon_uuid[14], m_device_cfg.beacon_uuid[15]);
            live_alert_update(threshold_data_packet);
            uart_trasmit_str(threshold_data_packet);
        }
        else if (m_device_cfg.slave_device_flag == 1)
        {
            // printf("Configured as slave:%s\n", m_device_cfg.sensor_name_set);
            memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
            sprintf(threshold_data_packet, "{mac:%X%X%X,DS_Slave:%d,SN:%s}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], m_device_cfg.slave_device_flag, m_device_cfg.sensor_name_set);
            live_alert_update(threshold_data_packet);
            uart_trasmit_str(threshold_data_packet);
        }
        else
        {
            // printf("MASTER_DEVICE_STATE:%d\n", m_device_cfg.master_device_flag);
            // printf("SLAVE_DEVICE_STATE:%d\n", m_device_cfg.slave_device_flag);
            memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
            sprintf(threshold_data_packet, "{mac:%X%X%X,MF:%d,SF:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], m_device_cfg.master_device_flag, m_device_cfg.slave_device_flag);
            live_alert_update(threshold_data_packet);
            uart_trasmit_str(threshold_data_packet);
        }
    }
    else if (strstr(hook, "GET_HS")) // to get hook stats
    {
        if (m_device_cfg.hook_mode == 1)
        {
            // printf("Hook Stats is 1\n");
            memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
            sprintf(threshold_data_packet, "{mac:%X%X%X,HS:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], m_device_cfg.hook_mode);
            live_alert_update(threshold_data_packet);
            uart_trasmit_str(threshold_data_packet);
        }
        else if (m_device_cfg.hook_mode == 0)
        {
            // printf("Hook Stats is 0\n");
            memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
            sprintf(threshold_data_packet, "{mac:%X%X%X,HS:%d}", gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], m_device_cfg.hook_mode);
            live_alert_update(threshold_data_packet);
            uart_trasmit_str(threshold_data_packet);
        }
    }
    else if (strstr(hook, "GET_THV")) // get threshold values
    {
        memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
        snprintf(threshold_data_packet, sizeof(threshold_data_packet),
                 "{mac:%X%X%X,time:%s,xv:%d,yv:%d,zv:%d,av:%d}",
                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
                 m_device_cfg.testX_beep_th,
                 m_device_cfg.testY_beep_th, m_device_cfg.testZ_beep_th, m_device_cfg.testA_beep_th);
        live_alert_update(threshold_data_packet);
        uart_trasmit_str(threshold_data_packet);
    }
    else if (strstr(hook, "GET_RD")) // to get rotation data (clockwise record & anticlock wise record)
    {
        memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
        snprintf(threshold_data_packet, sizeof(threshold_data_packet),
                 "{mac:%X%X%X,time:%s,ccr:%d,acr:%d,cr_val:%d,acr_val:%d}",
                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, rcc_cnt, rac_cnt,
                 m_device_cfg.cr_val, m_device_cfg.acr_val);
        live_alert_update(threshold_data_packet);
        uart_trasmit_str(threshold_data_packet);
    }
    else if (strstr(hook, "GET_LD")) // to get lanyard data
    {
        memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
        snprintf(threshold_data_packet, sizeof(threshold_data_packet),
                 "{mac:%X%X%X,time:%s,hcc:%d,hdc:%d,bhd:%d}",
                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm, hc_cnt, hd_cnt, both_hook_disconnect);
        live_alert_update(threshold_data_packet);
        uart_trasmit_str(threshold_data_packet);
    }
    else if (strstr(hook, "GET_STV")) // to get set threshold values
    {
        // dht means deltas high threshold values
        // fbt means fall buzzer time
        memset(threshold_data_packet, 0x00, sizeof(threshold_data_packet));
        snprintf(threshold_data_packet, sizeof(threshold_data_packet),
                 "{mac:%X%X%X,time:%s,[LOCKTH:%d,%d,%d,%d,%d,%d],tht:%d,fst:%d,TX:%d,TY:%d,TZ:%d,TA:%d}",
                 gap_addr.addr[2], gap_addr.addr[1], gap_addr.addr[0], timespm,
                 m_device_cfg.lockth[0],
                 m_device_cfg.lockth[1],
                 m_device_cfg.lockth[2], m_device_cfg.lockth[3],
                 m_device_cfg.lockth[4], m_device_cfg.lockth[5],
                 m_device_cfg.test_high_beep_th, m_device_cfg.buz_beep_cnt, m_device_cfg.testX_beep_th,
                 m_device_cfg.testY_beep_th, m_device_cfg.testZ_beep_th, m_device_cfg.testA_beep_th);

        live_alert_update(threshold_data_packet);
        uart_trasmit_str(threshold_data_packet);
    }
    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**
 * @brief Save rotation count mode
 *
 * @param a clockwise count
 * @param b anticlockwise count
 */
void save_cnt_mode(uint16_t a, uint16_t b)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    m_cnt_cfg.cl = a;
    m_cnt_cfg.acl = b;
    err_code = fds_record_find(ROTATION_CONFIG_FILE, ROTATION_CONFIG_KEY, &desc, &tok);
    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cnt_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}
/*
 * @brief This function is used to set the time.
 *
 * @details This function sets the time in the device configuration structure
 * and updates the record in flash memory.
 */
void setTime(void)
{

    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }

    return err_code;
}

/**
 * @brief Save a log entry to flash.
 *
 * @param[in] log The log entry, null-terminated string.
 *
 * @details
 * This function saves a log entry to flash, it will overwrite the oldest log
 * when the log count reaches MAX_LOG. The log is aligned to a 32-bit word
 * boundary.
 *
 * @return None
 */
void AddLog(char *log)
{
    if (log == NULL || strlen(log) == 0)
    {
        NRF_LOG_ERROR("Invalid log entry");
        return;
    }

    // Align length to word boundary
    uint16_t log_len = strlen(log) + 1;
    uint16_t padded_len = ((log_len + 3) / 4) * 4;
    uint16_t length_words = padded_len / sizeof(uint32_t);

    // Pick next key in circular fashion
    uint16_t key = (m_log_cfg.log_count % MAX_LOG) + 1;

    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    // If record with same key exists, delete it first
    if (fds_record_find(LOG_FILE, key, &desc, &tok) == NRF_SUCCESS)
    {
        ret_code_t err = fds_record_delete(&desc);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to delete old log: 0x%X", err);
            return;
        }
        // Wait until FDS confirms delete in event handler before writing again
    }

    fds_record_t record = {
        .file_id = LOG_FILE,
        .key = key,
        .data.p_data = (uint32_t *)log, // ✅ cast to aligned type
        .data.length_words = length_words};

    ret_code_t err_code = fds_record_write(NULL, &record);
    if (err_code == NRF_SUCCESS)
    {
        m_log_cfg.log_count++;
        if (m_log_cfg.log_count >= MAX_LOG)
        {
            // m_log_cfg.log_count = 0; // wrap around
        }
        NRF_LOG_INFO("Log saved key=%d: %s", key, log);
    }
    else if (err_code == FDS_ERR_BUSY)
    {
        NRF_LOG_WARNING("FDS busy, retry later");
        // could queue for retry here
    }
    // else if (err_code == FDS_ERR_RECORD_ALREADY_EXISTS) {
    //     NRF_LOG_WARNING("Key exists, delete before write");
    // }
    else
    {
        NRF_LOG_ERROR("Log write failed: 0x%X", err_code);
    }
}

/*
 * @brief This function is used to retrieve a log entry from flash.
 *
 * @param log pointer to the buffer where the log entry will be copied.
 *
 * @details This function retrieves a log entry from flash memory and copies it into the provided buffer.
 * It also updates the current log index.
 *
 * @return The length of the retrieved log entry, or 0 if no more logs are available.
 */
uint8_t GetLog(char *log)
{
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    fds_flash_record_t log_record = {0};

    if (m_log_cfg.log_count == 0 || m_log_cfg.curr_log == m_log_cfg.log_count)
    {
        return 0;
    }

    ret_code_t err_code = fds_record_find(LOG_FILE, m_log_cfg.curr_log + 1, &desc, &tok);
    if (err_code != NRF_SUCCESS)
        return 0;

    err_code = fds_record_open(&desc, &log_record);
    if (err_code != NRF_SUCCESS)
        return 0;

    size_t len = strnlen((char *)log_record.p_data, MAX_LOG_LEN - 1);
    memcpy(log, log_record.p_data, len);
    log[len] = '\0';

    fds_record_close(&desc);

    m_log_cfg.curr_log++;
    return (uint8_t)len;
}
/*
    * @brief This function is used to reset the log read pointer.
    *
    * @details This function resets the current log index to zero and updates the log configuration
    * record in flash memory.
*/
uint8_t GetLogReset()
{
    ret_code_t err_code;
    fds_record_t record;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    /* A config file is in flash. Let's update it. */
    fds_flash_record_t log_record = {0};

    m_log_cfg.curr_log = 0;
    /* Write the updated record to flash. */
    err_code = fds_record_update(&desc, &m_log_cfg_record);
    if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }
    return NRF_SUCCESS;
}
/*
 * @brief This function is used to clear the log data.
 *
 * @details This function clears the log data by deleting all log records from flash memory
 * and resetting the log configuration.
 */
uint8_t ClearLog(void)
{

    // to clear log data

    ret_code_t err_code;
    fds_record_t record;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    m_log_cfg.curr_log = 0;
    m_log_cfg.log_count = 0;
    /* Write the updated record to flash. */
    err_code = fds_record_update(&desc, &m_log_cfg_record);
    if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = fds_file_delete(LOG_FILE);
    return NRF_SUCCESS;
}
/*
 * @brief This function is used to set the device name prefix.
 *
 * @param prefix pointer to the device name prefix string.
 * @param len length of the device name prefix string.
 *
 * @details This function sets the device name prefix in the device configuration structure
 * and updates the record in flash memory.
 */
uint8_t setPrefix(uint8_t *prefix, size_t len)
{

    ret_code_t err_code;
    fds_record_t record;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    memset(m_device_cfg.name_prefix, 0, sizeof(m_device_cfg.name_prefix));

    strncpy(m_device_cfg.name_prefix, prefix, len);

    err_code = fds_record_update(&desc, &m_cfg_record);
    if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }
    return NRF_SUCCESS;
}
/*
 * @brief This function is used to set the sensor prefix.
 *
 * @param sensor_prefix pointer to the sensor prefix string.
 * @param len length of the sensor prefix string.
 *
 * @details This function sets the sensor prefix in the device configuration structure
 * and updates the record in flash memory.
 */
uint8_t setsensorPrefix(uint8_t *sensor_prefix, size_t len)
{

    ret_code_t err_code;
    fds_record_t record;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    memset(m_device_cfg.sensor_name_set, 0, sizeof(m_device_cfg.sensor_name_set));

    strncpy(m_device_cfg.sensor_name_set, sensor_prefix, len);
    printf("set successfully:%s\n", m_device_cfg.sensor_name_set);

    err_code = fds_record_update(&desc, &m_cfg_record);
    if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }
    return NRF_SUCCESS;
}
/*
 * @brief This function is used to set the block serial number.
 *
 * @param serial pointer to the serial number string.
 * @param len length of the serial number string.
 *
 * @details This function sets the block serial number in the device configuration structure
 * and updates the record in flash memory.
 */
uint8_t setBlockSerial(uint8_t *serial, size_t len)
{

    ret_code_t err_code;
    fds_record_t record;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    strncpy(m_device_cfg.block_serial, serial, len);

    err_code = fds_record_update(&desc, &m_cfg_record);
    if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }
    return NRF_SUCCESS;
}
/*
 * @brief This function is used to perform a factory reset.
 *
 * @details This function deletes the configuration file from flash memory,
 * effectively resetting the device to its factory settings.
 */
void factoryReset()
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    err_code = fds_file_delete(CONFIG_FILE);
}
/*
 * @brief This function is used to clear the rotation counter.
 *
 * @details This function clears the rotation counter by resetting the values in the configuration structure
 * and updating the record in flash memory.
 */
void clear_rotation_counter(void)
{

    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    m_cnt_cfg.cl = 0;
    m_cnt_cfg.acl = 0;
    RotationF = 0;
    RotationR = 0;
    err_code = fds_record_find(ROTATION_CONFIG_FILE, ROTATION_CONFIG_KEY, &desc, &tok);
    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cnt_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

/*
 * @brief This function is used to check the command string and perform the corresponding action.
 *
 * @param cl pointer to the command string.
 * @param len length of the command string.
 *
 * @details This function takes a pointer to a command string and its length. It checks the command string
 * and performs the corresponding action. The command can be "CLEAR_LOG", "CLEAR_RT_CNT", or "SET_TIME".
 */
void cl_check(uint8_t *cl, size_t len)
{
    uint8_t str[20];
    strncpy(str, cl, len);
    if (strstr(str, "CLEAR_LOG"))
    {
        ClearLog();
        NRF_LOG_INFO("Clear Log");
    }
    else if (strstr(str, "CLEAR_RT_CNT"))
    {
        clear_rotation_counter();
        NRF_LOG_INFO("Clear rotaion counter");
    }
    else if (strstr(str, "SET_TIME"))
    {
        set_user_time(str);
    }
}
/*
// Save anticlockwise rotation count
// b = anticlockwise count
*/
void set_user_time(char *st)
{
    uint16_t val = 0;
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};
    uint8_t *p = strstr(st, ":");
    *p++;
    val = atoi(p);
    val = val / 30;
    NRF_LOG_INFO("set_user_time: %d", val);
    if (val <= 1)
    {
        val = 0;
    }
    m_cnt_cfg.acl = val;
    err_code = fds_record_find(ROTATION_CONFIG_FILE, ROTATION_CONFIG_KEY, &desc, &tok);
    if (err_code == NRF_SUCCESS)
    {
        err_code = fds_record_update(&desc, &m_cnt_cfg_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}
