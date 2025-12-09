#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0x0001)
#define CONFIG_REC_KEY  (0x0001)

#define LOG_CONFIG_FILE (0x0002)
#define LOG_CONFIG_KEY  (0x0001)

#define ROTATION_CONFIG_FILE (0x0003)
#define ROTATION_CONFIG_KEY  (0x0001)

#define LOG_FILE        (0x0006)
#define FACTORY_RESET   (0x0004)

#define MAX_LOG          (0x3E8)
#define CLR_DIRTY_LOG1   (0xC8)
#define CLR_DIRTY_LOG2   (0x190)
#define CLR_DIRTY_LOG3   (0x258)
#define CLR_DIRTY_LOG4   (0x320)

#define MAX_THRESHOLDS 6   // or more if needed

extern uint16_t buz_en;
extern uint16_t FTCT;
extern uint16_t buz_cnt;
extern time_t t1, last_time;
extern uint16_t buz_10s_cnt;
extern uint16_t buz_stop;
extern uint16_t sensor_detect;
extern uint16_t RotationF,RotationR;
extern uint16_t fix_time ;
extern bool buz_hmcmd_ex;
extern uint8_t temp2[250];
extern uint8_t timespm[20];
extern char AlertStr[250];  // External declaration (no memory allocation here)

typedef struct
{
    uint8_t     block_serial[12];
    uint8_t     name_prefix[10];
    uint8_t     sensor_name_set[13];
    uint32_t    fall_threshold;
    uint32_t    ff_threshold;
    uint32_t    test_threshold;
    uint32_t    delaytimer_threshold;
    uint32_t    hm_stopth;
    uint32_t     mclk_en;
    uint32_t     manc_en;
    // uint16_t    th1_threshold;
    int16_t     th1_threshold;
    // uint16_t    th2_threshold;
    int16_t     th2_threshold;
    // uint16_t    th3_threshold;
    int16_t     th3_threshold;
    uint8_t     th_status;
    uint8_t     wake_threshold; 
    time_t      rtc_time;
    uint8_t     cal_mode;
    uint8_t     hook_mode;
    uint8_t     buzzer_onoff_enable;
    uint8_t     buzzertimer_threshold; 
    uint8_t     hm_startflag;
    uint8_t     hm_stopflag;
    uint8_t     step_en;
    uint8_t     mstop_en;
    uint8_t     motorclk_flag;
    uint8_t     motoraclk_flag;
    uint8_t     beacon_uuid[16]; 
    uint8_t     master_device_flag;
    uint8_t     slave_device_flag;
    uint8_t     scanwait_time;
    uint8_t     slave_name[15];  // Can hold up to 9 characters + null terminator
    uint8_t     nm_lan_flag;
    uint8_t     th_data;
    uint8_t     fall_test_th;
    uint16_t    lock_test_th;
    uint8_t     battery_perc_value;
    uint16_t    x_axis_lt;
    uint16_t    x_axis_gt;
    uint16_t    y_axis_lt;
    uint16_t    y_axis_gt;
    uint16_t    z_axis_lt;
    uint16_t    z_axis_gt;
    uint16_t    xyz_avg_lt;
    uint16_t    xyz_avg_gt;
    uint16_t    rssi_lt;
    uint16_t    rssi_gt;
    uint8_t     hook_beeptime;
    uint8_t     zl_flag_start;
    uint8_t     xg_flag_start;
    uint8_t     yl_flag_start;
    uint8_t     avg_flag_start;
    uint8_t     buz_beep_cnt;
    int         testX_beep_th;
    int         testY_beep_th;
    int         testZ_beep_th;
    int         testA_beep_th;
    uint16_t    test_high_beep_th;
    uint16_t    test_low_beep_th;
    int         lockth[MAX_THRESHOLDS];   // thresholds array
    uint16_t    cr_val;
    uint16_t    acr_val;
    uint16_t    bhd_val;

} configuration_t;

typedef struct{
    uint32_t    log_count;
    uint32_t    curr_log;
}log_configuration_t;

typedef struct{
    uint16_t    cl;
    uint16_t    acl;
    uint16_t    user_time;
}rotation_configuration_t;


uint32_t flash_init(void);
uint32_t config_init(void);
void     AddLog(char *log);
uint8_t  GetLog(char *log);
uint8_t GetLogReset();
uint8_t  ClearLog(void);
void setTime(void);
uint8_t setPrefix(uint8_t *prefix, size_t len);
uint8_t setBlockSerial(uint8_t *serial, size_t len);
void factoryReset();
void set_hook_mode(char *hook);
void save_cnt_mode( uint16_t a ,uint16_t b);
void clear_rotation_counter(void);
void cl_check(uint8_t *cl , size_t len);
void System_OFF_Mode(void);
void set_user_time(char * st);
void motor_reset();