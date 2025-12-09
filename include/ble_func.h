#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_cts_c.h"
#include "ble_cus_data.h"
#include "ble_cus_settings.h"
#include "ble_nus.h"

#define ble_adv_start   advertising_start

void ble_init(void);
void advertising_start(bool erase_bonds);
void battery_level_update(uint8_t battery_level);
ret_code_t live_alert_update(uint8_t *alert);
void th_settings_update(uint8_t *th, size_t len);
void get_cts();
void sendMean(uint8_t *mean);
void wakeUpBLE(void);
void raw_data_update(uint8_t *alert);
void update_advertising_data(uint8_t new_hook_conn);
void disconnect(uint16_t conn_handle, void *p_context);
void step_data_update(uint8_t *alert);
