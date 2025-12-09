#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define DATA_SERVICE_UUID_BASE          {0x60, 0x1C, 0x06, 0x24, 0x11, 0xE2, 0xD4, 0xA8,\
                                         0xAE, 0x44, 0x1F, 0x1B, 0x6E, 0x9C, 0x44, 0x82}   /**< Settings UUID 82449c6e-1b1f-44ae-a8d4-e21124061c60**/

#define DATA_SERVICE_UUID                   0x0000
#define DATA_LIVE_VALUE_CHAR_UUID           0x0001
#define DATA_HISTORY_VALUE_CHAR_UUID        0X0002
#define ACC_GYR_RAW_VALUE_CHAR_UUID         0X0003
#define NEW_CHAR_UUID                       0x0004

/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_DATA_DEF(_name)                                                                          \
static ble_cus_data_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_data_on_ble_evt, &_name)

typedef enum
{
    BLE_CUS_DATA_LIVE_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_CUS_DATA_LIVE_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_CUS_DATA_HISTORY_EVT_NOTIFICATION_ENABLED,                          /**< Custom value notification enabled event. */
    BLE_CUS_DATA_HISTORY_EVT_NOTIFICATION_DISABLED,                         /**< Custom value notification disabled event. */
    BLE_CUS_ACC_GYR_RAW_EVT_NOTIFICATION_ENABLED,
    BLE_CUS_ACC_GYR_RAW_EVT_NOTIFICATION_DISABLED,
    BLE_CUS_NEW_CHAR_EVT_NOTIFICATION_ENABLED,
    BLE_CUS_NEW_CHAR_EVT_NOTIFICATION_DISABLED,
    BLE_CUS_DATA_HISTORY_EVT_WRITE,
    BLE_CUS_DATA_EVT_DISCONNECTED,
    BLE_CUS_DATA_EVT_CONNECTED
} ble_cus_data_evt_type_t;

/**@brief Custom Service event. */
typedef struct
{
    ble_cus_data_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_data_evt_t;

// Forward declaration of the ble_cus_data_t type.
typedef struct ble_cus_data_s ble_cus_data_t;

/**@brief Custom Service event handler type. */
typedef void (*ble_cus_data_evt_handler_t) (ble_cus_data_t * p_cus, ble_cus_data_evt_t * p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_cus_data_evt_handler_t    evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                       live_data[50];                  /**< Initial custom value */
    uint8_t                       acc_gyr_raw[50];                /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;      /**< Initial security level for Custom characteristics attribute */
} ble_cus_data_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_data_s
{
    ble_cus_data_evt_handler_t    evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      live_handles;                   /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      history_handles;                /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      acc_gyr_raw_handles;                /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      new_char_handles; /**< Handles related to the new characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};



/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_data_init(ble_cus_data_t * p_cus, const ble_cus_data_init_t * p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Custom Service structure.
 */
void ble_cus_data_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_cus          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_data_live_value_update(ble_cus_data_t * p_cus, uint8_t *live_data);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_cus          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_data_history_value_update(ble_cus_data_t * p_cus, uint8_t *history_value);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_cus          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_data_acc_gya_row_value_update(ble_cus_data_t * p_cus, uint8_t *live_data);

uint32_t ble_new_char_value_update(ble_cus_data_t * p_cus, uint8_t *new_value);