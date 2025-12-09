#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"

typedef struct{
    bool pwrSenseEn;
    float inputPwr;
    uint8_t SOC;
}pwrSense_t;

extern pwrSense_t pwrSense;
extern bool btr_pwrdn;

ret_code_t pwr_sense_init();
ret_code_t getBattLevel();
ret_code_t DisableBattLevel();
ret_code_t getCurrBattLevel();