#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define SETTINGS_SERVICE_UUID_BASE      {0x27, 0x85, 0x28, 0xF7, 0xe3, 0xc7, 0x6b, 0xbe,\
                                         0xC2, 0x4F, 0xE1, 0x16, 0x9A, 0xA9, 0x87, 0x36}  /**< Settings UUID 3687a99a-16e1-4fc2-be6b-c7e3f7288527**/

#define SETTINGS_SERVICE_UUID                   0x0000
#define TH_SETTINGS_VALUE_CHAR_UUID             0x0001
#define TH_CAL_SETTINGS_VALUE_CHAR_UUID         0x0002
#define DEVICE_PREFIX_SETTINGS_VALUE_CAHR_UUID  0X0003
// #define BLOCK_SERIAL_SETTINGS_VALUE_CHAR_UUID   0X0004
#define SENSOR_NAME_SETTINGS_VALUE_CHAR_UUID   0X0004
#define CL_SETTINGS_VALUE_CHAR_UUID             0x0005
#define REBOOT_SETTINGS_VALUE_CHAR_UUID         0x0006
#define FACTORY_RESET_SETTINGS_VALUE_CHAR_UUID  0x0007
#define HOOK_MODE_SETTINGS_VALUE_CHAR_UUID      0x0008

/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_SETTINGS_DEF(_name)                                                                          \
static ble_cus_settings_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_settings_on_ble_evt, &_name)

typedef enum
{
    BLE_CUS_SETTINGS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_CUS_SETTINGS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_CUS_SETTINGS_EVT_DISCONNECTED,
    BLE_CUS_SETTINGS_EVT_CONNECTED,
    BLE_CUS_SETTINGS_EVT_TH,
    BLE_CUS_SETTINGS_EVT_TH_CAL,
    BLE_CUS_SETTINGS_EVT_CL,
    BLE_CUS_SETTINGS_EVT_REBOOT,
    BLE_CUS_SETTINGS_EVT_FACTORY_RESET
} ble_cus_settings_evt_type_t;

/**@brief Custom Service event. */
typedef struct
{
    ble_cus_settings_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_settings_evt_t;

// Forward declaration of the ble_cus_settings_t type.
typedef struct ble_cus_settings_s ble_cus_settings_t;

/**@brief Custom Service event handler type. */
typedef void (*ble_cus_settings_evt_handler_t) (ble_cus_settings_t * p_cus, ble_cus_settings_evt_t * p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_cus_settings_evt_handler_t    evt_handler;                   /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                           initial_custom_value;          /**< Initial custom value */
    ble_srv_cccd_security_mode_t      custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_settings_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_settings_s
{
    ble_cus_settings_evt_handler_t    evt_handler;                  /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                          service_handle;               /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t          th_handles;                   /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t          th_cal_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t          device_prefix_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t          block_serial_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t          clear_log_handles;            /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t          reboot_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t          factory_reset_handles;
    ble_gatts_char_handles_t          hook_mode_handles;
    ble_gatts_char_handles_t          sensorname_prefix_handles;
    uint16_t                          conn_handle;                  /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                           uuid_type; 
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
uint32_t ble_cus_settings_init(ble_cus_settings_t * p_cus, const ble_cus_settings_init_t * p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Custom Service structure.
 */
void ble_cus_settings_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

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

uint32_t ble_settingss_th_value_update(ble_cus_settings_t * p_cus, uint8_t *th, size_t len);