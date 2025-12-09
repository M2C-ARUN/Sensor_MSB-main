#define USE_BOOTLOADER_FOTA    0

#define FALL_THRESHOLD         1500
#define TEST_THRESHOLD         1200
#define FF_THRESHOLD           100
#define THRESHOLD1             1700
#define THRESHOLD2             1900
#define THRESHOLD3             2200
#define TH_STATUS              7
#define HOOK_STATUS            0
#define BUZZER_STATUS          1
#define time_user              9
#define step_en_status         0
#define MCLK_EN                0
#define MANC_EN                0


#define DELAYTIMERTHRESHOLD     0   // by default zero
#define DELAYTIMER_FLAG         0 
#define HM_STOPFLAG             0
#define MASTER_DEVICE_FLAG      0
#define SLAVE_DEVICE_FLAG       0
#define HM_STOPTH               120
#define SCANWAIT_TIME           30
#define NM_LAN_FLAG             0

#define WAKE_THRESHOLD          7
#define BUZZERTIMERTHRESHOLD    10 // how much sec beep after disconnect
#define HOOK_BEEPTIME           3  // how much time wait to start beep after disconnect
#define BATTERY_PERC_VALUE      25
#define FT_THRESHOLD            100
#define LT_THRESHOLD            800

#define  TH_DATA                0
#define  X_AXIS_LT              0
#define  X_AXIS_GT              0
#define  Y_AXIS_LT              0
#define  Y_AXIS_GT              0
#define  Z_AXIS_LT              0
#define  Z_AXIS_GT              0
#define  RSSI_LT                0
#define  RSSI_GT                0
#define  XYZ_AVG_LT             0
#define  XYZ_AVG_GT             0
#define  ZL_FLAG_START          0
#define  XG_FLAG_START          0
#define  YL_FLAG_START          0
#define  AVG_FLAG_START         0
#define  TESTX_BEEP_TH          0
#define  TESTY_BEEP_TH          0
#define  TESTZ_BEEP_TH          0
#define  TESTA_BEEP_TH          0
#define  TEST_LOW_BEEP_TH       0
#define  CR_VAL                 0
#define  ACR_VAL                0
#define  BHD_VAL                0
#define  BUZ_BEEP_CNT           5
#define  TEST_HIGH_BEEP_TH      20

#define DEVICE_NAME             "FallArrest"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME       "KARAM"                                 /**< Manufacturer. Will be passed to Device Information Service. */
#define DEVICE_NAME_PREFIX      "KARE"
#define SENSOR_NAME_PREFIX      "KARE-ABCDEF"
#define SLAVE_NAME              "KARE-000001"
#define FW_VERSION              "LN_V1.2.0" // ib