/******************************************************************************

 @file       simple_central.c

 @brief This file contains the Simple Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2013-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include "bcomdef.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "central.h"
#include "simple_gatt_profile.h"

#include "board_key.h"
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SBC_STATE_CHANGE_EVT                  0x0001
#define SBC_KEY_CHANGE_EVT                    0x0002
#define SBC_RSSI_READ_EVT                     0x0004
#define SBC_PAIRING_STATE_EVT                 0x0008
#define SBC_PASSCODE_NEEDED_EVT               0x0010

// Simple Central Task Events
#define SBC_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBC_START_DISCOVERY_EVT               Event_Id_00

#define SBC_ALL_EVENTS                        (SBC_ICALL_EVT           | \
                                               SBC_QUEUE_EVT           | \
                                               SBC_START_DISCOVERY_EVT)

// Default PHY preference
// Note: BLE_V50_FEATURES is always defined and long range phy (PHY_LR_CFG) is
//       defined in build_config.opt
#if (BLE_V50_FEATURES & PHY_LR_CFG)
  #define NUM_PHY                           5
#else // !PHY_LR_CFG
  #define NUM_PHY                           3
#endif // PHY_LR_CFG

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

/* Scan duration in ms */
#define DEFAULT_SCAN_DURATION                 4000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// After the connection is formed, the central will accept connection parameter
// update requests from the peripheral
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define SBC_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define SBC_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define SBC_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // Display_DISABLE_ALL
  #define SBC_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// Key states for connections
typedef enum {
  GATT_RW,                 // Perform GATT Read/Write
  RSSI,                    // Toggle RSSI updates
  CONN_UPDATE,             // Send Connection Parameter Update
  SET_PHY,                 // Set PHY preference
  DISCONNECT               // Disconnect
} keyPressConnOpt_t;

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Broadsens Central";

// Number of scan results and scan result index
static uint8_t scanRes = 0;
static int8_t scanIdx = -1;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

// Key option state.
static keyPressConnOpt_t keyPressConnOpt = DISCONNECT;

static const char searchStr[] =
{
    0x12, /* length of this data */
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'T', 'h', 'r', 'o', 'u', 'g', 'h', 'p', 'u', 't', ' ',
    'P', 'e', 'r', 'i', 'p', 'h',
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period);
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle);
static void SimpleBLECentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

#ifdef FPGA_AUTO_CONNECT
static void SimpleBLECentral_startGapDiscovery(void);
static void SimpleBLECentral_connectToFirstDevice(void);
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Central GAPRole Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // GAPRole Event Callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  (pfnPasscodeCB_t)SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB                  // Pairing / Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#ifdef FPGA_AUTO_CONNECT
/*********************************************************************
 * @fn      SimpleBLECentral_startGapDiscovery
 *
 * @brief   Start discovering devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_startGapDiscovery(void)
{
  // Start discovery
  if ((state != BLE_STATE_CONNECTED) && (!scanningStarted))
  {
    scanningStarted = TRUE;
    scanRes = 0;

    Display_print0(dispHandle, 2, 0, "Discovering...");
    Display_clearLines(dispHandle, 3, 4);

    GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                  DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                  DEFAULT_DISCOVERY_WHITE_LIST);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_connectToFirstDevice
 *
 * @brief   Connect to first device in list of discovered devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_connectToFirstDevice(void)
{
  uint8_t addrType;
  uint8_t *peerAddr;

  scanIdx = 0;

  if (state == BLE_STATE_IDLE)
  {
    // connect to current device in scan result
    peerAddr = devList[scanIdx].addr;
    addrType = devList[scanIdx].addrType;

    state = BLE_STATE_CONNECTING;

    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, peerAddr);

    Display_print0(dispHandle, 2, 0, "Connecting");
    Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
    Display_clearLine(dispHandle, 4);
  }
}
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_init
 Description  :
 Input        : void
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_init(void)
{
    uint8_t i;

    /**
    * Register the current thread as an ICall dispatcher application,
    * so that the application can send and receive messages.
    **/
    ICall_registerApp(&selfEntity, &syncEvent);

    /* Create an RTOS queue for message from profile to be sent to app. */
    appMsgQueue = Util_constructQueue(&appMsg);

    /* Setup discovery delay as a one-shot timer */
    Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                        DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

    Board_initKeys(SimpleBLECentral_keyChangeHandler);

    /* Initialize display device */
    dispHandle = Display_open(SBC_DISPLAY_TYPE, NULL);

    /* Initialize internal data */
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
        readRssi[i].pClock = NULL;
    }

    /* Setup the Central GAPRole Profile */
    {
        uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
        GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t), &scanRes);
    }

    /* Set GAP Parameters to set the discovery duration */
    {
        GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
        GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
        GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *)attDeviceName);
    }

    /* Initialize GATT Client */
    VOID GATT_InitClient();

    /* Register to receive incoming ATT Indications/Notifications */
    GATT_RegisterForInd(selfEntity);

    /* Initialize GATT attributes */
    GGS_AddService(GATT_ALL_SERVICES);         // GAP

    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    // Initialize GATT Client
    GATT_InitClient();

    GGS_SetParamValue(GGS_DISABLE_RPAO_CHARACTERISTIC);
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

    /* Start the Device */
    VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

    /* Register with GAP for HCI/Host messages (for RSSI) */
    GAP_RegisterForMsgs(selfEntity);

    /* Register for GATT local events and ATT Responses pending for transmission */
    GATT_RegisterForMsgs(selfEntity);


    /* Initialize Two-Button Menu module */
    TBM_SET_TITLE(&sbcMenuMain, "Broadsens BLE Central");
    tbm_setItemStatus(&sbcMenuMain, TBM_ITEM_0, TBM_ITEM_1 | TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4);
    tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_0 | TBM_ITEM_1);
    tbm_initTwoBtnMenu(dispHandle, &sbcMenuMain, 4, NULL);

    /* Get current data length */
    HCI_LE_ReadMaxDataLenCmd();
    /* By default allow central to support any and all phys */
    HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_ANY_PHY, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED);
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);

    Display_print1(dispHandle, SBC_ROW_MTU, 0, "MTU Size: %dB", ATT_MTU_SIZE);

    return;
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_taskFxn
 Description  :
 Input        : UArg a0
                UArg a1
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
    /* Initialize application */
    SimpleBLECentral_init();

    /* Application main loop */
    for (;;)
    {
        uint32_t events;

        events = Event_pend(syncEvent, Event_Id_NONE, SBC_ALL_EVENTS, ICALL_TIMEOUT_FOREVER);
        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if (ICall_fetchServiceMsg(&src, &dest, (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
            {
                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    /* Process inter-task message */
                    SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            /* If RTOS queue is not empty, process app message */
            if (events & SBC_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        /* Process message */
                        SimpleBLECentral_processAppMsg(pMsg);

                        /* Free the space from the message */
                        ICall_free(pMsg);
                    }
                }
            }

            if (events & SBC_START_DISCOVERY_EVT)
            {
                SimpleBLECentral_startDiscovery();
            }
        }
    }

    return;
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_processStackMsg
 Description  :
 Input        : ICall_Hdr *pMsg
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
    switch (pMsg->event)
    {
        case GAP_MSG_EVENT:
        {
            SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
            break;
        }

        case GATT_MSG_EVENT:
        {
            SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
            break;
        }

        case HCI_GAP_EVENT_EVENT:
        {
            /* Process HCI message */
            switch (pMsg->status)
            {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                {
                    SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
                }
                break;

                case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
                {
                    AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
                }
                break;

                case HCI_LE_EVENT_CODE:
                {
                    hciEvt_BLEPhyUpdateComplete_t *pPUC = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

                    if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
                    {
                        if (pPUC->status != SUCCESS)
                        {
                            Display_print0(dispHandle, SBC_ROW_RESULT, 0, "PHY Change failure");
                        }
                        else
                        {
                            Display_print0(dispHandle, SBC_ROW_RESULT, 0, "PHY Update Complete");
                            /* figure out whitch phy is being used */
                            uint8_t temp = 0;
                            switch (pPUC->txPhy)
                            {
                                case HCI_PHY_1_MBPS:
                                {
                                    phyIndex = 0; break;
                                }
                                case HCI_PHY_2_MBPS:
                                {
                                    phyIndex = 1; break;
                                }
                                case HCI_PHY_CODED:
                                {
                                    phyIndex = 4; break;
                                }
                            }

                            if (false == phyConfirm)
                            {
                                UInt key = Hwi_disable();
                                phyConfirm = true;
                                Hwi_restore(key);
                            }
                            else
                            {
                                phyIndex = temp;
                            }
                            currentPHY1M = 0;
                            currentPHY2M = 0;

                            if (0 == phyIndex)
                            {
                                currentPHY1M = 1;
                            }
                            else if (1 == phyIndex)
                            {
                                currentPHY2M = 1;
                            }

                            Display_print1(dispHandle, SBC_ROW_PHY, 0, "Current PHY: %s", phyName[phyIndex]);
                        }
                    }

                    if (pPUC->BLEEventCode == HCI_BLE_DATA_LENGTH_CHANGE_EVENT)
                    {
                        hciEvt_BLEDataLengthChange_t *dleEvt = (hciEvt_BLEDataLengthChange_t *)pMsg;
                        Display_print1(dispHandle, SBC_ROW_PDU, 0, "Device RX PDU Size: %dB", dleEvt->maxRxOctets);
                        /* GUI composer */
                        currentPeerTxPDUSize = dleEvt->maxRxOctets;
                    }
                }
                break;
                default:
                {
                    /* do nothing */
                }
                break;
            }
        }
        break;

        default:
        {
            /* do nothing */
        }
        break;
    }
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_processAppMsg
 Description  :
 Input        : sbcEvt_t *pMsg
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
    switch (pMsg->hdr.event)
    {
        case SBC_STATE_CHANGE_EVT:
        {
            SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);

            /* Free the stack message */
            ICall_freeMsg(pMsg->pData);
            break;
        }

        case SBC_KEY_CHANGE_EVT:
        {
            SimpleBLECentral_handleKeys(0, pMsg->hdr.state);
            break;
        }

        case SBC_RSSI_READ_EVT:
        {
             readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

             /* If link is up and RSSI reads active */
             if (pRssi->connHandle != GAP_CONNHANDLE_ALL && linkDB_Up(pRssi->connHandle))
             {
                 /* Restart timer */
                 Util_restartClock(pRssi->pClock, pRssi->period);

                 /* Read RSSI */
                 VOID HCI_ReadRssiCmd(pRssi->connHandle);
             }
         }
         break;

         case SBC_PDU_UPDATE_EVT:
         {
             attWriteReq_t writeReq;
             uint8_t pduSize = (uint8_t) *(pMsg->pData);

             /* populate the request structure */
             writeReq.cmd = 0;
             writeReq.handle = throughputHandles[THROUGHPUT_SERVICE_UPDATE_PDU].charHdl;
             writeReq.len = THROUGHPUT_SERVICE_UPDATE_PHY_LEN;
             writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_UPDATE_PHY_LEN, NULL);
             memcpy(writeReq.pValue, &pduSize, THROUGHPUT_SERVICE_UPDATE_PHY_LEN);
             writeReq.sig = 0;

             /* perform a GATT write + check status */
             uint8_t status;
             status = GATT_WriteCharValue(connHandle, &writeReq, selfEntity);

             if (SUCCESS != status)
             {
                 GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);
                 SimpleBLECentral_enqueueMsg(SBC_PDU_UPDATE_EVT, SUCCESS, pMsg->pData);
             }
             else
             {
                 Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Requested Peer Change TX PDU Size to %dB", pduSize);
                 if (pMsg->pData)
                 {
                     ICall_free(pMsg->pData);
                 }
             }
             break;
         }

         case SBC_PHY_UPDATE_EVT:
         {
             attWriteReq_t writeReq;
             UInt key = Hwi_disable();

             phyIndex = (uint8_t) *(pMsg->pData);
             phyConfirm = false;
             Hwi_restore(key);

             writeReq.cmd = 0;
             writeReq.handle = throughputHandles[THROUGHPUT_SERVICE_UPDATE_PHY].charHdl;
             writeReq.len = THROUGHPUT_SERVICE_UPDATE_PHY_LEN;
             writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_UPDATE_PHY_LEN, NULL);
             memcpy(writeReq.pValue, &phyIndex, THROUGHPUT_SERVICE_UPDATE_PHY_LEN);
             writeReq.sig = 0;

             /* perform a GATT write + check status */
             uint8_t status;
             status = GATT_WriteCharValue(connHandle, &writeReq, selfEntity);

             if (SUCCESS != status)
             {
                 GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);
                 SimpleBLECentral_enqueueMsg(SBC_PHY_UPDATE_EVT, SUCCESS, pMsg->pData);
             }
             else
             {
                 Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Requested Peer Change PHY to %s", phyName[phyIndex]);

                 if (pMsg->pData)
                 {
                     ICall_free(pMsg->pData);
                 }
             }
             break;
         }

         case SBC_MEASURE_INST_SPEED_EVT:
         {
             uint32_t *temp = = (uint32_t*)(pMsg->pData);

             uint32_t bitsReceived = *temp;
             bitsReceived = 8 * bitsReceived;

             Display_print2(dispHandle, SBC_ROW_INST_THROUGHPUT, 0, "Instant Rate (kb/s): %d.%d", (bitsReceived/1000),(bitsReceived % 1000));

             instantRate = bitsReceived / 1000;
             break;
         }

         default:
         {
             /* do nothing */
             break;
         }
    }
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_processRoleEvent
 Description  :
 Input        : gapCentralRoleEvent_t *pEvent
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
    switch (pEvent->gap.opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
        {
            maxPduSize = pEvent->initDone.dataPktLen;

            Display_print1(dispHandle, SBC_ROW_BDADDR, 0, "Device's address : %s ",
                           Util_convertBdAddr2Str(pEvent->initDone.devAddr));

            /* Auto connect */
            /* SimpleBLECentral_startGapDiscovery(); */
        }
        break;

        case GAP_DEVICE_INFO_EVENT:
        {
            /* if filtering device discovery results based on service UUID */
            if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
            {
                if (SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                                 pEvent->deviceInfo.pEvtData,
                                                 pEvent->deviceInfo.dataLen))
                {
                    SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                                   pEvent->deviceInfo.addrType);
                }
            }
        }
        break;

        case GAP_DEVICE_DISCOVERY_EVENT:
        {
            /* discovery complete */
            scanningStarted = FALSE;

            /* if not filtering device discovery results based on service UUID */
            if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
            {
                /* copy results */
                scanRes = pEvent->discCmpl.numDevs;
                memcpy(devList, pEvent->discCmpl.pDevList, (sizeof(gapDevRec_t) * scanRes));
            }

            Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);

            if (scanRes > 0)
            {
                /* enable all menu functions */
                tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_NONE);
            }
            else
            {
                /* no result, re-enable scaning only */
                tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_0 | TBM_ITEM_1);
            }
        }
        break;

        case GAP_LINK_ESTABLISHED_EVENT:
        {
            if (pEvent->gap.hdr.status == SUCCESS)
            {
                state = BLE_STATE_CONNECTED;
                connHandle = pEvent->linkCmpl.connectionHandle;

                /* go to main menu */
                tbm_goTo(&sbcMenuMain);
                /* disable scan connect menu, enable everything else */
                tbm_setItemStatus(&sbcMenuMain, TBM_ITEM_ALL, TBM_ITEM_0);
                /* forget about the scan results */
                SBC_ClearDeviceList();

                /* If service discovery not performed initiate service discovery */
                if (charHdl == 0)
                {
                    Util_startClock(&startDiscClock);
                }
                /* update display */
                Display_print1(dispHandle, SBC_ROW_PEER_DEVICE, 0, "Peer Device : %s", Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
                Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Connected, Exchanging MTU");
                Display_print0(dispHandle, SBC_ROW_PHY, 0, "PHY: 1 Mbps");
                /* start rssi collection */
                SimpleBLECentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
            }
            else
            {
                state = BLE_STATE_IDLE;
                connHandle = GAP_CONNHANDLE_INIT;
                discState = BLE_DISC_STATE_IDLE;

                Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Connect failed, reason: %d", pEvent->gap.hdr.status);
            }
        }
        break;

        case GAP_LINK_TERMINATED_EVENT:
        {
            state = BLE_STATE_IDLE;
            connHandle = GAP_CONNHANDLE_INIT;
            discState = BLE_DISC_STATE_IDLE;
            charHdl = 0;
            procedureInProgress = FALSE;
            keyPressConnOpt = DISCONNECT;
            scanIdx = -1;

            /* cancel rssi reads */
            SimpleBLECentral_CancelRssi(pEvent->linkTerminate.connectionHandle);
            Util_stopClock(&speedClock);
            /* update display */
            Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Reason: %d", pEvent->linkTerminate.reason);
            Display_clearLine(dispHandle, SBC_ROW_PEER_DEVICE);
            Display_clearLine(dispHandle, SBC_ROW_PHY);
            Display_clearLine(dispHandle, SBC_ROW_INST_THROUGHPUT);
            Display_clearLine(dispHandle, SBC_ROW_AVG_THROUGHPUT);
            Display_clearLine(dispHandle, SBC_ROW_RSSI);

            /* go to main menu */
            tbm_goTo(&sbcMenuMain);
            tbm_setItemStatus(&sbcMenuMain, TBM_ITEM_0, TBM_ITEM_1 | TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4);
            tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_0 | TBM_ITEM_1);
        }
        break;

        case GAP_LINK_PARAM_UPDATE_EVENT:
        {
            Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Param Update: %d", pEvent->linkUpdate.status);
        }
        break;

        default:
        {
            /* do nothing */
        }
        break;
    }
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_handleKeys
 Description  :
 Input        : uint8_t shift
                uint8_t keys
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
    /* Check if the key is still pressed. WA for possible bouncing. */
    if (keys & KEY_LEFT)
    {
        if (0 == PIN_getInputValue(Board_PIN_BUTTON0))
        {
            tbm_buttonLeft();
        }
    }

    if (keys & KEY_RIGHT)
    {
        /* Check if the key is still pressed. WA for possible bouncing. */
        if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
        {
            tbm_buttonRight();
        }
    }

    return;
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_processGATTMsg
 Description  :
 Input        : gattMsgEvent_t *pMsg
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
    if (state == BLE_STATE_CONNECTED)
    {
        /* see if GATT server was unable to transmit an ATT response */
        if (pMsg->hdr.status == blePending)
        {
            /**
            * no HCI buffer was available. app can try to retransmit the response
            * on the next connection event. drop it for now.
            */
            Display_print1(dispHandle, SBC_ROW_GATT_RESULT, 0, "ATT Rsp dropped %d", pMsg->method);
        }
        else if ((pMsg->method == ATT_READ_RSP)   ||
                 ((pMsg->method == ATT_ERROR_RSP) &&
                  (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
        {
            if (pMsg->method == ATT_ERROR_RSP)
            {
                Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                /* After a successful read, display the read value */
                Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
            }
        }

        else if ((pMsg->method == ATT_WRITE_RSP)  ||
                 ((pMsg->method == ATT_ERROR_RSP) &&
                  (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
        {
            if (pMsg->method == ATT_ERROR_RSP)
            {
                Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a successful write, display the value that was written and
                // increment value
                Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal++);
            }

            procedureInProgress = FALSE;

        }
        else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
        {
            /* Display the opcode of the message that caused the violation. */
            Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
        }
        else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
        {
            /* MTU size updated */
            Display_print0(dispHandle, SBC_ROW_RESULT, 0, "MTU Exchanged");
            Display_print1(dispHandle, SBC_ROW_MTU, 0, "MTU Size: %dB", pMsg->msg.mtuEvt.MTU);

            currentMTUSize = pMsg->msg.mtuEvt.MTU;
        }
        else if (discState != BLE_DISC_STATE_IDLE)
        {
            SimpleBLECentral_processGATTDiscEvent(pMsg);
        }
    } /* else - in case a GATT message came after a connection has dropped, ignore it. */

    /* Needed only for ATT Protocol messages */
    GATT_bm_free(&pMsg->msg, pMsg->method);
}


/*****************************************************************************
 Fuction      : SimpleBLECentral_processCmdCompleteEvt
 Description  :
 Input        : hciEvt_CmdComplete_t *pMsg
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
    switch (pMsg->cmdOpcode)
    {
        case HCI_READ_RSSI:
        {
            int8 rssi = (int8)pMsg->pReturnParam[3];
            Display_print1(dispHandle, SBC_ROW_RSSI, 0, "RSSI -dBm: %d", (uint32_t)(-rssi));
        }
        break;
        case HCI_LE_READ_MAX_DATA_LENGTH:
        {
            typedef struct
            {
                uint8_t status;
                uint8_t maxTxBytes[2];
                uint8_t maxTxTime[2];
                uint8_t maxRxBytes[2];
                uint8_t maxRxTime[2];
            } HCI_Read_Max_Data_Length_Event_t;

            HCI_Read_Max_Data_Length_Event_t* temp = (HCI_Read_Max_Data_Length_Event_t*) pMsg->pReturnParam;
            uint16_t ourRxPDUsize = 0;
            ourRxPDUsize = BUILD_UINT16(temp->maxRxBytes[0], temp->maxRxBytes[1]);
            Display_print1(dispHandle, SBC_ROW_PDU, 0, "Device RX PDU Size: %dB", ourRxPDUsize);
        }
        break;
        default:
        {
            /* do nothing */
        }
        break;
    }

    return;
}

/*********************************************************************
 * @fn      SimpleBLECentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period)
{
  readRssi_t *pRssi;

  // Verify link is up
  if (!linkDB_Up(connHandle))
  {
    return bleIncorrectMode;
  }

  // If already allocated
  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    pRssi->period = period;
  }
  // Allocate structure
  else if ((pRssi = SimpleBLECentral_RssiAlloc(connHandle)) != NULL)
  {
    pRssi->period = period;
  }
  // Allocate failed
  else
  {
    return bleNoResources;
  }

  // Start timer
  Util_restartClock(pRssi->pClock, period);

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleBLECentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle)
{
  readRssi_t *pRssi;

  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    // Free RSSI structure
    SimpleBLECentral_RssiFree(connHandle);

    return SUCCESS;
  }

  // Not found
  return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiAlloc
 *
 * @brief   Allocate an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if allocation failed.
 */
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
    {
      readRssi_t *pRssi = &readRssi[i];

      pRssi->pClock = (Clock_Struct *)ICall_malloc(sizeof(Clock_Struct));
      if (pRssi->pClock)
      {
        Util_constructClock(pRssi->pClock, SimpleBLECentral_readRssiHandler,
                            0, 0, false, i);
        pRssi->connHandle = connHandle;

        return pRssi;
      }
    }
  }

  // No free structure found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      return &readRssi[i];
    }
  }

  // Not found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void SimpleBLECentral_RssiFree(uint16_t connHandle)
{
  uint8_t i;

  // Find RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      readRssi_t *pRssi = &readRssi[i];
      if (pRssi->pClock)
      {
        Clock_destruct(pRssi->pClock);

        // Free clock struct
        ICall_free(pRssi->pClock);
        pRssi->pClock = NULL;
      }

      pRssi->connHandle = GAP_CONNHANDLE_ALL;
      break;
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}


/*****************************************************************************
 Fuction      : SimpleBLECentral_startDiscovery
 Description  :
 Input        : void
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
static void SimpleBLECentral_startDiscovery(void)
{
    attExchangeMTUReq_t req;

    /* Initialize cached handles */
    svcStartHdl = svcEndHdl = charHdl = 0;

    if (throughputHandles)
    {
        ICall_free(throughputHandles);
    }

    discState = BLE_DISC_STATE_MTU;

    /* Discover GATT Server's Rx MTU size */
    req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

    /* ATT MTU size should be set to the minimum of the Client Rx MTU */
    /* and Server Rx MTU values */
    VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);

    return;
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      // Just in case we're using the default MTU size (23 octets)
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", ATT_MTU_SIZE);

      discState = BLE_DISC_STATE_SVC;

      // Discovery simple service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);

      Display_print0(dispHandle, 2, 0, "Simple Svc Found");
      procedureInProgress = FALSE;
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) ||
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;

        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
    uint8_t i;

    /* If result count not at max */
    if (scanRes < DEFAULT_MAX_SCAN_RES)
    {
        /* Check if device is already in scan results */
        for (i = 0; i < scanRes; i++)
        {
            if (0 == memcmp(pAddr, devList[i].addr , B_ADDR_LEN))
            {
                return;
            }
        }

        /* Add addr to scan result list */
        memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
        devList[scanRes].addrType = addrType;

        /* Increment scan result count */
        scanRes++;
    }

    return;
}

/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  Event_post(syncEvent, SBC_START_DISCOVERY_EVT);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleBLECentral_readRssiHandler(UArg a0)
{
  SimpleBLECentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*****************************************************************************
 Fuction      : SBC_ClearDeviceList
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
void SBC_ClearDeviceList(void)
{
    int i = 0;

    /* Go through Device List and Clear out the ICALL Heap Allocs */
    for (i = 0; i < scanRes; i++)
    {
        if (devList[i].advertData != NULL)
        {
            ICall_free(devList[i].advertData);
            devList[i].advertLen = 0;
        }
        if (devList[i].scanRsp != NULL)
        {
            ICall_free(devList[i].scanRsp);
            devList[i].scanLen = 0;
        }
    }

    /* Clear the Device Display */
    Display_clearLine(dispHandle, SBC_ROW_STATUS_2);
    Display_clearLine(dispHandle, SBC_ROW_STATUS_3);
    Display_clearLine(dispHandle, SBC_ROW_STATUS_4);
    /* Reset Scan Res indicating No Valid Scan data on Device List */
    scanRes = 0;  scanIdx = -1;

    return;
}

/*****************************************************************************
 Fuction      : SBC_NextDevice
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
void SBC_SelectNextDevice(void)
{
    /* increment scanIndex to the next valid entry */
    ((scanIdx + 1) == scanRes) ? scanIdx = 0 : scanIdx++;

    /* Print the Device pointed to by the Index */
    Display_print1(dispHandle, SBC_ROW_STATUS_2, 0, "Scanned Device %d", (scanIdx+1));
    Display_print0(dispHandle, SBC_ROW_STATUS_3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));

    /* is next device a throughput peripheral? */
    if (memcmp(searchStr, devList[scanIdx].scanRsp, sizeof(searchStr)) == 0)
    {
        Display_print0(dispHandle, SBC_ROW_STATUS_4, 0, "-- Throughput Profile Supported --");
    }
    else
    {
        Display_clearLine(dispHandle, SBC_ROW_STATUS_4);
    }

    return;
}

/*****************************************************************************
 Fuction      : SBC_ConnectToDevice
 Description  : Connect to device
 Input        : void
 Output       : None
 Return Value : void
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
void SBC_ConnectToDevice(void)
{
    /* connect to current device selected by scanIdx */
    uint8_t *peerAddr = devList[scanIdx].addr;
    uint8_t addrType = devList[scanIdx].addrType;

    /* GAP role to connecting */
    state = BLE_STATE_CONNECTING;
    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, peerAddr);

    return;
}

/*****************************************************************************
 Fuction      : SimpleBLECentral_doScanAndConnect
 Description  :
 Input        : uint8 index
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
  History        :
  1.Date         : 2018/4/13
    Author       : wuhaibin
    Modification : Created function

*****************************************************************************/
bool SimpleBLECentral_doScanAndConnect(uint8 index)
{
    switch (index)
    {
        case 0:
        {
            /* select next device on scan list */
            SBC_SelectNextDevice();
        }
        break;

        case 1:
        {
            /* connect to select device */
            SBC_ConnectToDevice();
        }
        break;

        case 2:
        {
            /* scan for devices */
            /* disable scanning until completed */
            tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_NONE, TBM_ITEM_ALL);
            Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Scanning...");

            /* clear the device list */
            SBC_ClearDeviceList();

            /* command to tell gap role to start scanning */
            GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST);
        }
        break;

        default:
        {
            /* do nothing */
        }
        break;
    }

    return true;
}