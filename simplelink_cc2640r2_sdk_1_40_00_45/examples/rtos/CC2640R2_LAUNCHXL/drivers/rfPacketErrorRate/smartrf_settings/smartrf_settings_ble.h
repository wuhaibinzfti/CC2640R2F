#ifndef _SMARTRF_SETTINGS_BLE_H_
#define _SMARTRF_SETTINGS_BLE_H_


//*********************************************************************************
// These settings have been generated for use with TI-RTOS and cc13xxware
//
// Generated by SmartRF Studio version 2.4.0
// Tested for TI-RTOS version tirtos_simplelink_2_20_xx
// Device: CC1350 Rev. 2.1
//
//*********************************************************************************
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_ble_cmd.h)

#include <ti/drivers/rf/RF.h>

// TI-RTOS RF Mode Object
extern RF_Mode *RF_pModeBle;


// RF Core API commands
extern rfc_CMD_RADIO_SETUP_t *RF_ble_pCmdRadioSetup;
extern rfc_CMD_FS_t *RF_ble_pCmdFs;
extern rfc_CMD_BLE_ADV_NC_t *RF_ble_pCmdBleAdvNc;
extern rfc_CMD_BLE_GENERIC_RX_t *RF_ble_pCmdBleGenericRx;




#endif // _SMARTRF_SETTINGS_BLE_H_