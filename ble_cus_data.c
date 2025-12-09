#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_cus_data.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_data_t * p_cus, ble_evt_t const * p_ble_evt) 
{
    ret_code_t                 err_code;
    ble_cus_data_evt_t              evt;
    ble_gatts_value_t          gatts_val[3];
    uint8_t                    cccd_value[3];

    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val[0], 0, sizeof(ble_gatts_value_t));
    gatts_val[0].p_value = cccd_value;
    gatts_val[0].len     = sizeof(cccd_value);
    gatts_val[0].offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_cus->live_handles.cccd_handle,
                                      &gatts_val[0]);

    memset(&gatts_val[1], 0, sizeof(ble_gatts_value_t));
    gatts_val[1].p_value = cccd_value;
    gatts_val[1].len     = sizeof(cccd_value);
    gatts_val[1].offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_cus->history_handles.cccd_handle,
                                      &gatts_val[1]);

    memset(&gatts_val[1], 0, sizeof(ble_gatts_value_t));
    gatts_val[1].p_value = 1;
    gatts_val[1].len     = sizeof(cccd_value);
    gatts_val[1].offset  = 0;

    err_code = sd_ble_gatts_value_set(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_cus->history_handles.cccd_handle,
                                      &gatts_val[1]);

    memset(&gatts_val[2], 0, sizeof(ble_gatts_value_t));
    gatts_val[2].p_value = cccd_value;
    gatts_val[2].len     = sizeof(cccd_value);
    gatts_val[2].offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_cus->acc_gyr_raw_handles.cccd_handle,
                                      &gatts_val[2]);

    // memset(&gatts_val[3], 0, sizeof(ble_gatts_value_t));
    // gatts_val[3].p_value = cccd_value;
    // gatts_val[3].len     = sizeof(cccd_value);
    // gatts_val[3].offset  = 0;

    // err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
    //                                   p_cus->new_char_handles.cccd_handle,
    //                                   &gatts_val[3]);


    if ((err_code == NRF_SUCCESS) && ble_srv_is_notification_enabled(gatts_val[0].p_value) )//&& ble_srv_is_notification_enabled(gatts_val[1].p_value))
    {

        memset(&evt, 0, sizeof(ble_cus_data_evt_t));
        evt.evt_type        = BLE_CUS_DATA_EVT_CONNECTED;
        p_cus->evt_handler(p_cus, &evt);
    }
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_data_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_data_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if (p_evt_write->handle == p_cus->live_handles.value_handle)
    {
        // Put specific task here. 
    }
    if ((p_evt_write->handle == p_cus->live_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            ble_cus_data_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_DATA_LIVE_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_DATA_LIVE_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }
    if (p_evt_write->handle == p_cus->history_handles.value_handle)
    {
        // Put specific task here. 
        ble_cus_data_evt_t evt;
        evt.evt_type = BLE_CUS_DATA_HISTORY_EVT_WRITE;
        p_cus->evt_handler(p_cus, &evt);
    }
    if ((p_evt_write->handle == p_cus->history_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            ble_cus_data_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_DATA_HISTORY_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_DATA_HISTORY_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }
    if (p_evt_write->handle == p_cus->acc_gyr_raw_handles.value_handle)
    {
        // Put specific task here. 
    }
    if ((p_evt_write->handle == p_cus->acc_gyr_raw_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            ble_cus_data_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_ACC_GYR_RAW_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_ACC_GYR_RAW_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }

    // if (p_evt_write->handle == p_cus->new_char_handles.value_handle)
    // {
    // // Handle the write to the new characteristic.
    // }
    // if ((p_evt_write->handle == p_cus->new_char_handles.cccd_handle) && (p_evt_write->len == 2))
    // {
    //     // CCCD written, call application event handler
    //     if (p_cus->evt_handler != NULL)
    //     {
    //         ble_cus_data_evt_t evt;

    //         if (ble_srv_is_notification_enabled(p_evt_write->data))
    //         {
    //             evt.evt_type = BLE_CUS_NEW_CHAR_EVT_NOTIFICATION_ENABLED;
    //         }
    //         else
    //         {
    //             evt.evt_type = BLE_CUS_NEW_CHAR_EVT_NOTIFICATION_DISABLED;
    //         }
    //         // Call the application event handler.
    //         p_cus->evt_handler(p_cus, &evt);
    //     }
    // }


}

void ble_cus_data_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_data_t * p_cus = (ble_cus_data_t *) p_context;
    
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

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_cus_data_init(ble_cus_data_t * p_cus, const ble_cus_data_init_t * p_cus_init)
{
        // this function is for customize data initialization
        // like : live data , history data, accelerometer raw data etc...

    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {DATA_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = DATA_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = DATA_LIVE_VALUE_CHAR_UUID;
    add_char_params.uuid_type                = p_cus->uuid_type;
    add_char_params.max_len                  = 100;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.notify        = 1;
    add_char_params.char_props.read          = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->live_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // return err_code;

    // Add the TX Characteristic.
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                      = DATA_HISTORY_VALUE_CHAR_UUID;
    add_char_params.uuid_type                 = p_cus->uuid_type;
    add_char_params.max_len                   = 100;
    add_char_params.init_len                  = sizeof(uint8_t);
    add_char_params.is_var_len                = true;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.write          = 1;
    add_char_params.char_props.write_wo_resp  = 1;
    add_char_params.char_props.read           = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->history_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

     // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = ACC_GYR_RAW_VALUE_CHAR_UUID;
    add_char_params.uuid_type                = p_cus->uuid_type;
    add_char_params.max_len                  = 100;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.notify        = 1;
    add_char_params.char_props.read          = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    // return characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->acc_gyr_raw_handles);
   err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->acc_gyr_raw_handles);
    
   if (err_code != NRF_SUCCESS)
   {
    return err_code;
   }

//    memset(&add_char_params, 0, sizeof(add_char_params));
//    add_char_params.uuid                     = NEW_CHAR_UUID;
//    add_char_params.uuid_type                = p_cus->uuid_type;
//    add_char_params.max_len                  = 10;  // Adjust as needed
//    add_char_params.init_len                 = sizeof(uint8_t);
//    add_char_params.is_var_len               = true;
//    add_char_params.char_props.notify        = 1;
//    add_char_params.char_props.read          = 1;

//    add_char_params.read_access              = SEC_OPEN;
//    add_char_params.write_access             = SEC_OPEN;
//    add_char_params.cccd_write_access        = SEC_OPEN;

//    err_code = characteristic_add(p_cus->service_handle, &add_char_params, &p_cus->new_char_handles);

//    if (err_code != NRF_SUCCESS)
//    {
//     return err_code;
//    }

    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}

uint32_t ble_data_live_value_update(ble_cus_data_t * p_cus, uint8_t *live_data)
{
        // this function update the ble live data value 


    NRF_LOG_INFO("In ble_data_live_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        NRF_LOG_INFO("NRF_ERROR_NULL-> ");
        return NRF_ERROR_NULL;

    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = strlen(live_data);
    gatts_value.offset  = 0;
    gatts_value.p_value = live_data;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->live_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("sd_ble_gatts_value_not_set_error");
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->live_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    }
    else
    {
        NRF_LOG_INFO("BLE_NOT_CONN_STATE");
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_data_history_value_update(ble_cus_data_t * p_cus, uint8_t *history_value)
{

    // this function update the ble history data value
    
    NRF_LOG_INFO("In ble_data_history_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = strlen(history_value);
    gatts_value.offset  = 0;
    gatts_value.p_value = history_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->history_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->history_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_data_acc_gya_row_value_update(ble_cus_data_t * p_cus, uint8_t *raw)
{
    // this function update the ble accelerometer data value

    NRF_LOG_INFO("In ble_acc_gyr_raw_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        NRF_LOG_INFO("NRF_ERROR_NULL-> ");
        return NRF_ERROR_NULL;

    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = strlen(raw);
    gatts_value.offset  = 0;
    gatts_value.p_value = raw;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->acc_gyr_raw_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("sd_ble_gatts_value_not_set_error");
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->acc_gyr_raw_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    }
    else
    {
        NRF_LOG_INFO("BLE_NOT_CONN_STATE");
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

// uint32_t ble_new_char_value_update(ble_cus_data_t * p_cus, uint8_t *new_value)
// {
//     if (p_cus == NULL)
//     {
//         return NRF_ERROR_NULL;
//     }

//     uint32_t err_code;
//     ble_gatts_value_t gatts_value;

//     // Initialize value struct.
//     memset(&gatts_value, 0, sizeof(gatts_value));

//     gatts_value.len     = strlen(new_value); // Adjust for actual length
//     gatts_value.offset  = 0;
//     gatts_value.p_value = new_value;

//     // Update database.
//     err_code = sd_ble_gatts_value_set(p_cus->conn_handle, p_cus->new_char_handles.value_handle, &gatts_value);
//     if (err_code != NRF_SUCCESS)
//     {
//         return err_code;
//     }

//     // Send value if connected and notifying.
//     if (p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)
//     {
//         ble_gatts_hvx_params_t hvx_params;
//         memset(&hvx_params, 0, sizeof(hvx_params));

//         hvx_params.handle = p_cus->new_char_handles.value_handle;
//         hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
//         hvx_params.offset = 0;
//         hvx_params.p_len  = &gatts_value.len;
//         hvx_params.p_data = gatts_value.p_value;

//         err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
//     }

//     return err_code;
// }
