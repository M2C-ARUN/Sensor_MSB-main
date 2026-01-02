#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_cus_settings.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "acc.h"
#include "main.h"
#include "flash_func.h"

extern bool restart;
extern configuration_t m_device_cfg;
uint8_t th[60] = "";
/**
 * @brief Function for handling the Connect event.
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_settings_t *p_cus, ble_evt_t const *p_ble_evt)
{
    ret_code_t err_code;
    ble_cus_settings_evt_t evt;

    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    memset(&evt, 0, sizeof(ble_cus_settings_evt_t));
    evt.evt_type = BLE_CUS_SETTINGS_EVT_CONNECTED;
    p_cus->evt_handler(p_cus, &evt);
}

/**
 * @brief Function for handling the Disconnect event.
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_settings_t *p_cus, ble_evt_t const *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**
 * @brief Function for handling the Write event.
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_settings_t *p_cus, ble_evt_t const *p_ble_evt)
{

    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if (p_evt_write->handle == p_cus->th_handles.value_handle)
    {
        memset(th, 0, 50);
        memcpy(th, p_evt_write->data, p_evt_write->len);
        NRF_LOG_INFO("received th=>%s,%d,%d", th, p_evt_write->len, sizeof(p_evt_write->data));
        printf("received th=>%s,%d,%d\n", th, p_evt_write->len, sizeof(p_evt_write->data));
        setThreshold(th);
        // printf("Received data: %s, Length: %d\n", th, p_evt_write->len);
        // Put specific task here.
    }
    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if ((p_evt_write->handle == p_cus->th_cal_handles.value_handle) && (p_evt_write->len == 1))
    {
        NRF_LOG_INFO("received TH Cal");
        m_device_cfg.cal_mode = !m_device_cfg.cal_mode;
        setTime();
        restart = true;
        // Put specific task here.
    }
    if ((p_evt_write->handle == p_cus->device_prefix_handles.value_handle))
    {
        NRF_LOG_INFO("received device prefix");
        setPrefix(p_evt_write->data, p_evt_write->len);
        setTime();
        restart = true;
    }
    // if ((p_evt_write->handle == p_cus->block_serial_handles.value_handle))
    // {
    //     NRF_LOG_INFO("received block serial");
    //     setBlockSerial(p_evt_write->data, p_evt_write->len);
    //     setTime();
    //     restart = true;
    // }
    if ((p_evt_write->handle == p_cus->sensorname_prefix_handles.value_handle))
    {
        if (m_device_cfg.master_device_flag == 0)
        {
            m_device_cfg.slave_device_flag = 1;
            printf("slave flag:%d\n",  m_device_cfg.slave_device_flag);
            // printf("received sensor name\n");
            setsensorPrefix(p_evt_write->data, p_evt_write->len);
            setTime();
        }
        // schedule_reset();
        // restart = true;
    }
    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if (p_evt_write->handle == p_cus->clear_log_handles.value_handle)
    {
        NRF_LOG_INFO("received clear log");
        printf("received clear log\n");
        cl_check(p_evt_write->data, p_evt_write->len);
        // Put specific task here.
    }
    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if ((p_evt_write->handle == p_cus->reboot_handles.value_handle) && (p_evt_write->len == 1))
    {
        NRF_LOG_INFO("received reboot");
        printf("received reboot\n");
        setTime();
        // restart = true;
        rebootDevice();
        // Put specific task here.
    }
    if ((p_evt_write->handle == p_cus->factory_reset_handles.value_handle) && (p_evt_write->len == 1))
    {
        NRF_LOG_INFO("received factory reset");
        printf("received factory reset\n");
        factoryReset();
        setTime();
        restart = true;
        // Put specific task here.
    }
    if ((p_evt_write->handle == p_cus->hook_mode_handles.value_handle))
    {
        NRF_LOG_INFO("hook and buzzer setting");
        memset(th, 0, 50);
        memcpy(th, p_evt_write->data, p_evt_write->len);
        NRF_LOG_INFO("received mode=>%s,%d,%d", th, p_evt_write->len, sizeof(p_evt_write->data));
        printf("received mode=>%s,%d,%d\n", th, p_evt_write->len, sizeof(p_evt_write->data));
        set_hook_mode(th);
        // Put specific task here.
    }
}
/**
 * @brief Function for handling the Read event.
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void on_read(ble_cus_settings_t *p_cus, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_read_t *p_evt_read = &p_ble_evt->evt.gatts_evt.params.authorize_request.request.read;

    NRF_LOG_INFO("TH READ");

    if (p_evt_read->handle == p_cus->th_handles.value_handle)
    {
        NRF_LOG_INFO("TH READ");
    }
}

/**
 * @brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Custom Service structure.
 */
void ble_cus_settings_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{

    // ble customiztaion setting on event

    ble_cus_settings_t *p_cus = (ble_cus_settings_t *)p_context;

    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_cus, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_cus, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_cus, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_read(p_cus, p_ble_evt);

    default:
        // No implementation needed.
        break;
    }
}
/**
 * @brief Function for initializing the Custom Service.
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_settings_init(ble_cus_settings_t *p_cus, const ble_cus_settings_init_t *p_cus_init)
{

    // this function is for ble custom setting initializtion of uuid, type, len
    // like : reboot setting / device prefix setting / factory resetting / hook mode setting etc...

    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure
    p_cus->evt_handler = p_cus_init->evt_handler;
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {SETTINGS_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = SETTINGS_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = TH_SETTINGS_VALUE_CHAR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = 55;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.char_props.read = 1;
    add_char_params.is_var_len = true;

    add_char_params.read_access = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->th_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the RX Characteristic.
    // memset(&add_char_params, 0, sizeof(add_char_params));
    // add_char_params.uuid = TH_CAL_SETTINGS_VALUE_CHAR_UUID;
    // add_char_params.uuid_type = p_cus->uuid_type;
    // add_char_params.max_len = sizeof(uint8_t);
    // add_char_params.init_len = sizeof(uint8_t);
    // add_char_params.char_props.write = 1;
    // add_char_params.char_props.write_wo_resp = 1;

    // add_char_params.write_access = SEC_OPEN;

    // err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->th_cal_handles);
    // if (err_code != NRF_SUCCESS)
    // {
    //     return err_code;
    // }

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = DEVICE_PREFIX_SETTINGS_VALUE_CAHR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = 10;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->device_prefix_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the RX Characteristic.
    // memset(&add_char_params, 0, sizeof(add_char_params));
    // add_char_params.uuid = BLOCK_SERIAL_SETTINGS_VALUE_CHAR_UUID;
    // add_char_params.uuid_type = p_cus->uuid_type;
    // add_char_params.max_len = 12;
    // add_char_params.init_len = sizeof(uint8_t);
    // add_char_params.char_props.write = 1;
    // add_char_params.char_props.write_wo_resp = 1;

    // add_char_params.write_access = SEC_OPEN;

    // err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->block_serial_handles);
    // if (err_code != NRF_SUCCESS)
    // {
    //     return err_code;
    // }

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = SENSOR_NAME_SETTINGS_VALUE_CHAR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = 15;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->sensorname_prefix_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    //...................FACTORY_RESET_stop..............................//
    // Add the TX Characteristic.
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = CL_SETTINGS_VALUE_CHAR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = 20;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->clear_log_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = REBOOT_SETTINGS_VALUE_CHAR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = sizeof(uint8_t);
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->reboot_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    //...................FACTORY_RESET_start..............................//
    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = FACTORY_RESET_SETTINGS_VALUE_CHAR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = sizeof(uint8_t);
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->factory_reset_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    //   ...................HOOK_MODE_start..............................//
    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = HOOK_MODE_SETTINGS_VALUE_CHAR_UUID;
    add_char_params.uuid_type = p_cus->uuid_type;
    add_char_params.max_len = 10;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.is_var_len = true;

    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->hook_mode_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
}
/**
 * @brief Function for updating the Custom Value characteristic.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   th          New Custom Value characteristic value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_settingss_th_value_update(ble_cus_settings_t *p_cus, uint8_t *th, size_t len)
{
    NRF_LOG_INFO("In ble_settingss_th_value_update. \r\n");
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = len;
    gatts_value.offset = 0;
    gatts_value.p_value = th;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->th_handles.value_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->th_handles.value_handle;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}