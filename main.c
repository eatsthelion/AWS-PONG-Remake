//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************
//
// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "stdio.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

//Common interface includes
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "common.h"
#include "time.h"

//Imported from Lab 3
#include "spi.h"
#include "gpio.h"
#include "glcdfont.h"
#include "i2c_if.h"
#include "uart_if.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//define OLED pins and properties
#define OLED_RESET  0x10
#define OLED_DC     0x2
#define OLED_CS     0x80

#define WIDTH 128
#define HEIGHT 128

#define INITIALPLYHEIGHT 5
#define INITIALPLYLENGTH 27

#define VERSUS 1
#define ENDURANCE 2

#define LV1 40
#define LV2 50
#define LV3 56

#define WIN 2
#define LOSE 1

#define MAXSPEEDSCALE  4

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define PINK            0xF8F0
#define PURPLE          0x7070
#define ORANGE          0xFB00

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//End of Lab 3 imports

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1

#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2020"
#define SERVER_NAME                "apkl44r6y00sg-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT             8443

#define SL_SSL_CA_CERT "/cert/rootCA.der"
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                13    /* Current Date */
#define MONTH               5     /* Month 1-12 */
#define YEAR                2021  /* Current year */
#define HOUR                12    /* Time - hours */
#define MINUTE              1    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/Ethan_CC3200Board/shadow HTTP/1.1\n\r"
#define GETHEADER  "GET /things/Ethan_CC3200Board/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: apkl44r6y00sg-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define HELLO   "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Hello player!\"}\n\r}}}\n\r\n\r"
#define GOODBYE "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Goodbye.\"}\n\r}}}\n\r\n\r"
#define ENDING  "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Game Over! Try again? Y/N\"}\n\r}}}\n\r\n\r"
#define MODE    "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Please select a mode: V/E\"}\n\r}}}\n\r\n\r"
#define LEVEL   "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Please select a level: 1/2/3\"}\n\r}}}\n\r\n\r"
#define ERINPUT "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Please try again\"}\n\r}}}\n\r\n\r"
#define WELCOME "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \"Hello player!\"}\n\r}}}\n\r\n\r"
#define DATA2   "{\"state\": {\n\r\"desired\" : {\n\r\"settings\" : {\"mode\" : \"V\", \"level\" : \"1\"}\n\r}}}\n\r\n\r"
#define DATA3   "{\"state\": {\n\r\"game\" : {\n\r\"score\" : "
#define INPUTCLEAR  "{\"state\": {\n\r\"desired\" : {\n\r\"input\" : \"0\"}}}\n\r\n\r"

#define JSONSTART   "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : {\"sms\" : \""
#define JSONEND     "\"}\n\r}}}\n\r\n\r"

#define GAMEOVER "Game Over!"
#define LEVELTEXT1 "Level 1"
#define LEVELTEXT2 "Level 2"
#define LEVELTEXT3 "Level 3"

#define YES 100
#define NO  200
#define RETRY 300

#define CPULV1SLOWON    10
#define CPULV1SLOWOFF   50
#define CPULV2SLOWON    10
#define CPULV2SLOWOFF   20
#define CPULV3SLOWON    10
#define CPULV3SLOWOFF   5
// Application specific status/error codes
typedef enum
{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

typedef struct
{
    /* time */
    unsigned long tm_sec;
    unsigned long tm_min;
    unsigned long tm_hour;
    /* date */
    unsigned long tm_day;
    unsigned long tm_mon;
    unsigned long tm_year;
    unsigned long tm_week_day; //not required
    unsigned long tm_year_day; //not required
    unsigned long reserved[3];
} SlDateTime;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long g_ulStatus = 0; //SimpleLink Status
unsigned long g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char g_ucConnectionSSID[SSID_LEN_MAX + 1]; //Connection SSID
unsigned char g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (*const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//-----------------------------------------------------------------------------
//My Global Variables
char globalacRecvbuff[1460];
long lRetVal = -1;

time_t startTime, endTime;
time_t playTime=0;

int maxRally=0;
int currentRally=0;

int mode = VERSUS;
int level = LV1;

// ball properties
int ballSize = 4;
int ballColor = CYAN;
int ballColorCycle = 1;

int ballspdVector = LV1;
int ballhsp = 15;
int ballvsp = 25;
int speedMag=100;

// initial ball location at middle of board
int ballX = WIDTH / 2;
int ballY = HEIGHT / 8;


int plyHeight= INITIALPLYHEIGHT;
int plyLength=INITIALPLYLENGTH;
int plyX = (WIDTH / 2) - (INITIALPLYLENGTH / 2);
int plyY = (HEIGHT - 2.5 * INITIALPLYHEIGHT);
int plyColor = WHITE;

int cpuFollowCount=10;
int cpuFollowOffCount=10;
int cpuSlow = false;
int cpuX = (WIDTH / 2) - (INITIALPLYLENGTH / 2);
int cpuY = (1.5 * INITIALPLYHEIGHT);
int cpuColor = RED;

int paddleNineth,
    leftPaddle1,leftPaddle2,leftPaddle3,leftPaddle4,
    rightPaddle1,rightPaddle2,rightPaddle3,rightPaddle4;


int speedScale = 5; //determines how fast the objects travel
int cpuLevelScale=1;
int wall = 0;
int nextStep = 0;

int wallColor = WHITE;
int backgroundColor = BLACK;
int gameOver = false;
int endTurn=false;
int winCount=0;
int loseCount=0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int, char*);
long printErrConvenience(char *msg, long retVal);
//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if (!pWlanEvent)
    {
        return;
    }

    switch (pWlanEvent->Event)
    {
    case SL_WLAN_CONNECT_EVENT:
    {
        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

        //
        // Information about the connected AP (like name, MAC etc) will be
        // available in 'slWlanConnectAsyncResponse_t'.
        // Applications can use it if required
        //
        //  slWlanConnectAsyncResponse_t *pEventData = NULL;
        // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
        //

        // Copy new connection SSID and BSSID to global parameters
        memcpy(g_ucConnectionSSID,
               pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_name,
               pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
        memcpy(g_ucConnectionBSSID,
               pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
               SL_BSSID_LENGTH);

        UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                   "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                   g_ucConnectionSSID, g_ucConnectionBSSID[0],
                   g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                   g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                   g_ucConnectionBSSID[5]);
    }
        break;

    case SL_WLAN_DISCONNECT_EVENT:
    {
        slWlanConnectAsyncResponse_t *pEventData = NULL;

        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

        pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

        // If the user has initiated 'Disconnect' request,
        //'reason_code' is SL_USER_INITIATED_DISCONNECTION
        if (SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
        {
            UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                       "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                       g_ucConnectionSSID, g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        else
        {
            UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                       "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                       g_ucConnectionSSID, g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
        memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
    }
        break;

    default:
    {
        UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                   pWlanEvent->Event);
    }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if (!pNetAppEvent)
    {
        return;
    }

    switch (pNetAppEvent->Event)
    {
    case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
    {
        SlIpV4AcquiredAsync_t *pEventData = NULL;

        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

        //Ip Acquired Event Data
        pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

        //Gateway IP address
        g_ulGatewayIP = pEventData->gateway;

        UART_PRINT(
                "[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                "Gateway=%d.%d.%d.%d\n\r",
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 3),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 2),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 1),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 0),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 3),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 2),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 1),
                SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 0));
    }
        break;

    default:
    {
        UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                   pNetAppEvent->Event);
    }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if (!pDevEvent)
    {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if (!pSock)
    {
        return;
    }

    switch (pSock->Event)
    {
    case SL_SOCKET_TX_FAILED_EVENT:
        switch (pSock->socketAsyncEvent.SockTxFailData.status)
        {
        case SL_ECLOSE:
            UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                       "failed to transmit all queued packets\n\n",
                       pSock->socketAsyncEvent.SockTxFailData.sd);
            break;
        default:
            UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                       "(%d) \n\n",
                       pSock->socketAsyncEvent.SockTxFailData.sd,
                       pSock->socketAsyncEvent.SockTxFailData.status);
            break;
        }
        break;

    default:
        UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n", pSock->Event);
        break;
    }
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull ver = { 0 };
    _WlanRxFilterOperationCommandBuff_t RxFilterIdMask = { 0 };

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while (!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
                _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                        &ucConfigLen, (unsigned char*) (&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r", SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
               ver.NwpVersion[0], ver.NwpVersion[1], ver.NwpVersion[2],
               ver.NwpVersion[3], ver.ChipFwAndPhyVersion.FwVersion[0],
               ver.ChipFwAndPhyVersion.FwVersion[1],
               ver.ChipFwAndPhyVersion.FwVersion[2],
               ver.ChipFwAndPhyVersion.FwVersion[3],
               ver.ChipFwAndPhyVersion.PhyVersion[0],
               ver.ChipFwAndPhyVersion.PhyVersion[1],
               ver.ChipFwAndPhyVersion.PhyVersion[2],
               ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                               SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if (0 == lRetVal)
    {
        // Wait
        while (IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE, 1, 1, &ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN, ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
    WLAN_GENERAL_PARAM_OPT_STA_TX_POWER,
                         1, (unsigned char*) &ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM, SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8*) &RxFilterIdMask,
                                 sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long) &g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = { 0 };
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");

    // Wait for WLAN Event
    while ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        //GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        //GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time()
{
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
    SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                       sizeof(SlDateTime), (unsigned char*) (&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect()
{
    SlSockAddrIn_t Addr;
    int iAddrSize;
    unsigned char ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP,
            uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char*) g_Host),
                                        (unsigned long*) &uiIP, SL_AF_INET);

    if (lRetVal < 0)
    {
        return printErrConvenience(
                "Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket 
    //
    iSockID = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
    if (iSockID < 0)
    {
        return printErrConvenience("Device unable to create secure socket \n\r",
                                   lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,
                            sizeof(ucMethod));
    if (lRetVal < 0)
    {
        return printErrConvenience("Device couldn't set socket options \n\r",
                                   lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK,
                            &uiCipher, sizeof(uiCipher));
    if (lRetVal < 0)
    {
        return printErrConvenience("Device couldn't set socket options \n\r",
                                   lRetVal);
    }

    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET,
    SL_SO_SECURE_FILES_CA_FILE_NAME,
                            SL_SSL_CA_CERT, strlen(SL_SSL_CA_CERT));

    if (lRetVal < 0)
    {
        return printErrConvenience("Device couldn't set socket options \n\r",
                                   lRetVal);
    }

    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET,
    SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME,
                            SL_SSL_CLIENT, strlen(SL_SSL_CLIENT));

    if (lRetVal < 0)
    {
        return printErrConvenience("Device couldn't set socket options \n\r",
                                   lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET,
    SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME,
                            SL_SSL_PRIVATE, strlen(SL_SSL_PRIVATE));

    if (lRetVal < 0)
    {
        return printErrConvenience("Device couldn't set socket options \n\r",
                                   lRetVal);
    }

    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, (SlSockAddr_t*) &Addr, iAddrSize);

    if (lRetVal < 0)
    {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r",
                                   lRetVal);
    }
    else
    {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }

    //GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}

long printErrConvenience(char *msg, long retVal)
{
    UART_PRINT(msg);
    //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}

int connectToAccessPoint()
{
    long lRetVal = -1;
    //GPIO_IF_LedConfigure(LED1|LED3);

    //GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    //GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if (lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
            UART_PRINT(
                    "Failed to configure the device in its default state \n\r");

        return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal)
    {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}
//-----------------------------------------------------------------------------
//My Functions

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID, char* datapact){

    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(datapact);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, datapact);
    pcBufHeaders += strlen(datapact);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static int http_get(int iTLSSockID){
    char acSendBuff[512];
//    char acRecvbuff[1460];
//    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);

    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &globalacRecvbuff[0], sizeof(globalacRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        globalacRecvbuff[lRetVal+1] = '\0';
 //       globalacRecvbuff = acRecvbuff;
        UART_PRINT(globalacRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

void delay(unsigned long ulCount)
{
    int i;

    do
    {
        ulCount--;
        for (i = 0; i < 65535; i++)
            ;
    }
    while (ulCount);
}

//displays text in middle of screen
void centerText(char text[], int textColor, int backColor)
{
    int i, x;
    x = (WIDTH / 2) - (6 * strlen(text) - 1) / 2;
    for (i = 0; i < strlen(text); i++)
    {
        drawChar(x, (HEIGHT / 2) - 3, text[i], textColor, backColor, 1);
        x += 6; //moves horizontal location of next char
    }
}

//draws boarder walls of the screen
void drawWalls(int wallColor)
{
    drawRect(0, 0, WIDTH, 1, wallColor);                        //draws Top wall
    drawRect(0, 0, 1, HEIGHT, wallColor);                      //draws Left wall
    drawRect(WIDTH - 1, 0, 1, HEIGHT, wallColor);             //draws Right wall
    drawRect(0, HEIGHT - 1, WIDTH, 1, wallColor);            //draws Bottom wall
}

//redraws a selected wall
void redrawWall(int wall, int wallColor)
{
    switch (wall)
    {
    case 1:
        drawRect(0, 0, WIDTH, 1, wallColor);
        break;         //Top wall
    case 2:
        drawRect(0, 0, 1, HEIGHT, wallColor);
        break;        //Left wall
    case 3:
        drawRect(WIDTH - 1, 0, 1, HEIGHT, wallColor);
        break;    //Right wall
    }
}

//------------------------------------------------------------------------------
//Game State Functions

//displays Title Screen
void titleScreen()
{
    int colorCycle = 1;
    fillScreen(backgroundColor);
    centerText("PONG", WHITE, backgroundColor);
    drawCircle(WIDTH/2, HEIGHT/2, 50, WHITE);
    drawCircle((WIDTH/2)+25, (HEIGHT/2)-25, 10, WHITE);
    while (GPIOPinRead(GPIOA1_BASE, 0x20) != 0x20)
    {
        switch (colorCycle)
        {
        case 1:
            drawWalls(RED);
            break;
        case 2:
            drawWalls(ORANGE);
            break;
        case 3:
            drawWalls(YELLOW);
            break;
        case 4:
            drawWalls(GREEN);
            break;
        case 5:
            drawWalls(BLUE);
            break;
        case 6:
            drawWalls(PURPLE);
            break;
        case 7:
            drawWalls(PINK);
            break;
        }

        colorCycle += 1;
        if (colorCycle > 7)
        {
            colorCycle = 1;
        }
    }

    centerText("PONG", WHITE, backgroundColor);
    centerText("PONG", backgroundColor, backgroundColor);
    centerText("PONG", WHITE, backgroundColor);
    centerText("PONG", backgroundColor, backgroundColor);
    centerText("PONG", WHITE, backgroundColor);

}

int getAnswer(char* state){
    delay(500);
    http_get(lRetVal);
    Report("\n\r------------------\n\r");

    char temp1,temp2;
    int end;
    int statelen=strlen(state);
    int i;
    int j=0;
    for(i = 0; i < 1460; i++){
        temp1 = globalacRecvbuff[i];
        if (temp1==state[j]){
            j+=1;
            if (j==statelen){
                temp2=globalacRecvbuff[i+1];

                //Reads the first character of the input value

                if ((temp2=='Y')||(temp2=='y')){
                    end=YES;
                }else if ((temp2=='N')||(temp2=='n')){
                    end=NO;
                }else if ((temp2=='E')||(temp2=='e')){
                    end=ENDURANCE;
                }else if ((temp2=='V')||(temp2=='v')){
                    end=VERSUS;
                }else if ((temp2=='R')||(temp2=='r')){
                    end=RETRY;
                }else if (temp2=='1'){
                    end=LV1;
                }else if (temp2=='2'){
                    end=LV2;
                }else if (temp2=='3'){
                    end=LV3;
                }else {
                    end=false;
                }
                break;
            }
        }else{
            j=0;
        }
    }

    return end;
}

//Selects game mode
void modeSelection(){
    /*fillScreen(backgroundColor);
    int modeCount = 1;
    mode = VERSUS;

    drawRect(0, 0, WIDTH, 1, RED);                        //draws Top wall
    drawRect(0, 0, 1, HEIGHT, GREEN);                      //draws Left wall
    drawRect(WIDTH - 1, 0, 1, HEIGHT, RED);             //draws Right wall
    drawRect(0, HEIGHT - 1, WIDTH, 1, GREEN);
    centerText("Versus Mode", WHITE, backgroundColor);
    drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)+40, (HEIGHT - 2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, GREEN);
    drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)-40, (2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, RED);

    while (GPIOPinRead(GPIOA1_BASE, 0x20) != 0x20)
    {
        if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
        {
            modeCount += 1;
            if (modeCount > 2)
            {
                modeCount = 1;
            }*/
    fillScreen(backgroundColor);
    centerText("Mode Select", WHITE, backgroundColor);
    //Getting the mode setting from AWS

    http_post(lRetVal, MODE);
    //http_post(lRetVal, INPUTCLEAR);
    while(1){
        mode=getAnswer("\"input\":\"");
        if ((mode!=ENDURANCE)&&(mode!=VERSUS)){
           // http_post(lRetVal, ERINPUT);
        }else{
            break;
        }
    }
    centerText("Mode Select", backgroundColor, backgroundColor);

    switch (mode)
    {
    case VERSUS:
        /*drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)-30, (HEIGHT - 2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, backgroundColor);
        drawCircle((WIDTH / 2)+20, HEIGHT / 8, 4, backgroundColor);
        //mode = VERSUS;
        centerText("Endurance Mode", backgroundColor, backgroundColor);*/
        drawRect(0, 0, WIDTH, 1, RED);                        //draws Top wall
        drawRect(0, 0, 1, HEIGHT, GREEN);                      //draws Left wall
        drawRect(WIDTH - 1, 0, 1, HEIGHT, RED);             //draws Right wall
        drawRect(0, HEIGHT - 1, WIDTH, 1, GREEN);
        centerText("Versus Mode", WHITE, backgroundColor);
        drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)+40, (HEIGHT - 2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, GREEN);
        drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)-40, (2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, RED);
        break;
    case ENDURANCE:
        /*drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)+40, (HEIGHT - 2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, backgroundColor);
        drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)-40, (2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, backgroundColor);
        //mode = ENDURANCE;
        centerText("Versus Mode", backgroundColor, backgroundColor);*/
        drawWalls(WHITE);
        centerText("Endurance Mode", WHITE, backgroundColor);
        drawRect((WIDTH / 2) - (INITIALPLYLENGTH / 2)-30, (HEIGHT - 2.5 * INITIALPLYHEIGHT), INITIALPLYLENGTH, INITIALPLYHEIGHT, WHITE);
        drawCircle((WIDTH / 2)+20, HEIGHT / 8, 4, BLUE);
        break;
    }

    delay(200);
            /*while (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
            {
            }
        }
    }*/
}

//Selects game level
void levelSelection()
{
    /*fillScreen(backgroundColor);
    drawWalls(BLUE);
    int levelCount = 1;
    level = LV1;
    centerText("Level 1", WHITE, backgroundColor);
    while (GPIOPinRead(GPIOA1_BASE, 0x20) != 0x20)
    {
        if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
        {
            levelCount += 1;
            if (levelCount > 3)
            {
                levelCount = 1;
            }*/
    fillScreen(backgroundColor);
    centerText("Level Select", WHITE, backgroundColor);
    //Getting the mode setting from AWS

    http_post(lRetVal, LEVEL);
    //http_post(lRetVal, INPUTCLEAR);
    while(1){
        level=getAnswer("\"input\":\"");
        if ((level!=LV1)&&(level!=LV2)&&(level!=LV3)){
            //http_post(lRetVal, ERINPUT);
        }else{
            break;
        }
    }
    centerText("Level Select", backgroundColor, backgroundColor);
    switch (level)
    {
    case LV1:
        centerText("Level 1", WHITE, backgroundColor);
        drawWalls(BLUE);
        break;
    case LV2:
        centerText("Level 2", WHITE, backgroundColor);
        drawWalls(GREEN);
        break;
    case LV3:
        centerText("Level 3", WHITE, backgroundColor);
        drawWalls(PINK);
        break;
    }
    delay(200);

            /*while (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
            {
            }
        }
    }*/
}

//Starts the game
void gameStart(int levelSelect)
{
    ballColorCycle = 1;
    winCount=0;
    loseCount=0;\
    maxRally=0;
    currentRally=0;
    playTime=0;
    switch (levelSelect)
    {
    // object properties for levels
    case LV1:
        ballSize = 4;
        ballColor = BLUE;
        plyHeight = 5;
        plyLength = 40;
        plyColor = BLUE;
        speedScale = 10*speedMag;
        ballhsp=20;
        cpuLevelScale=2;
        break;
    case LV2:
        ballSize = 4;
        ballColor = GREEN;
        plyHeight = 5;
        plyLength = 30;
        plyColor = GREEN;
        speedScale = 7*speedMag;
        ballhsp=25;
        cpuLevelScale=2;
        break;
    case LV3:
        ballSize = 4;
        ballColor = PINK;
        plyHeight = 5;
        plyLength = 20;
        plyColor = PINK;
        speedScale = 4*speedMag;
        ballhsp=24;
        cpuLevelScale=2;
        break;
    }


    ballvsp=levelSelect-ballhsp;
    //Initial Object Locations
    ballX = WIDTH / 2;
    if (mode==VERSUS){
        ballY = HEIGHT / 2;
    }else{
        ballY = HEIGHT / 8;
    }

    plyX = (WIDTH / 2) - (plyLength / 2);
    plyY = (HEIGHT - 2.5 * INITIALPLYHEIGHT);

    cpuX = (WIDTH / 2) - (INITIALPLYLENGTH / 2);
    cpuY = (1.5 * INITIALPLYHEIGHT);

    fillScreen(backgroundColor);

    if (mode==VERSUS){
        ballColor=WHITE;
        drawRect(0, 0, WIDTH, 1, CYAN);                        //draws Top wall
        drawRect(0, HEIGHT - 1, WIDTH, 1, CYAN);            //draws Bottom wall
        drawRect(0, 0, 1, HEIGHT, WHITE);                      //draws Left wall
        drawRect(WIDTH - 1, 0, 1, HEIGHT, WHITE);             //draws Right wall
        delay(50);
        drawRect(0,(HEIGHT/4),WIDTH,3,BLACK);
        delay(50);
        drawRect(0,(HEIGHT/2),WIDTH,3,BLACK);
        delay(50);
        drawRect(0,(3*HEIGHT/4),WIDTH,3,BLACK);
        delay(50);
        drawRect(cpuX,cpuY,plyLength,plyHeight,RED);
    }else{
        plyColor=WHITE;
        drawWalls(wallColor);
    }
    drawCircle(ballX, ballY, ballSize, ballColor);      //displays ball on board
    drawRect(plyX, plyY, plyLength, plyHeight, plyColor);    //draws player
    gameOver = false;
    delay(100);
    time(&startTime);
}

//Starts next turn in Versus Mode
void nextTurn(int levelSelect)
{
    playTime+=(endTime-startTime);

    //Erases all object images
    drawRect(cpuX, cpuY, plyLength, plyHeight, backgroundColor);
    drawCircle(ballX, ballY, ballSize, backgroundColor);
    drawRect(0, 0, WIDTH, 1, CYAN);                        //draws Top wall
    drawRect(0, HEIGHT - 1, WIDTH, 1, CYAN);            //draws Bottom wall
    drawRect(plyX, plyY, plyLength, plyHeight, backgroundColor);

    ballColorCycle = 1;
    if (currentRally>maxRally){
        maxRally=currentRally;
    }
    currentRally=0;
    switch (levelSelect)
    {
    // object properties for levels
    case LV1:
        ballSize = 4;
        ballColor = BLUE;
        plyHeight = 5;
        plyLength = 36;
        plyColor = BLUE;
        speedScale = 10*speedMag;
        ballhsp=20;
        cpuFollowCount=CPULV1SLOWON;
        cpuFollowOffCount=CPULV1SLOWON;
        cpuSlow=false;
        break;
    case LV2:
        ballSize = 4;
        ballColor = GREEN;
        plyHeight = 5;
        plyLength = 27;
        plyColor = GREEN;
        speedScale = 7*speedMag;
        ballhsp=25;
        cpuFollowCount=CPULV2SLOWON;
        cpuFollowOffCount=CPULV2SLOWON;
        cpuSlow=false;
        break;
    case LV3:
        ballSize = 4;
        ballColor = PINK;
        plyHeight = 5;
        plyLength = 18;
        plyColor = PINK;
        speedScale = 4*speedMag;
        ballhsp=24;
        cpuFollowCount=CPULV3SLOWON;
        cpuFollowOffCount=CPULV3SLOWON;
        cpuSlow=false;
        break;
    }

    ballvsp=levelSelect-ballhsp;
    //Initial Object Locations
    ballX = WIDTH / 2;
    ballY = HEIGHT / 2;

    plyX = (WIDTH / 2) - (plyLength / 2);
    plyY = (HEIGHT - 2.5 * INITIALPLYHEIGHT);

    cpuX = (WIDTH / 2) - (INITIALPLYLENGTH / 2);
    cpuY = (1.5 * INITIALPLYHEIGHT);

    drawRect(cpuX,cpuY,plyLength,plyHeight,RED);
    drawCircle(ballX, ballY, ballSize, ballColor);      //displays ball on board
    drawRect(plyX, plyY, plyLength, plyHeight, plyColor);    //draws player
    if (endTurn==LOSE){
        ballvsp=-ballvsp;
    }
    endTurn=false;
    delay(100);
    time(&startTime);
}

//updates Score Board
void scoreBoard(){
    delay(100);
    switch(endTurn){
    case WIN:
        drawRect(0, 0, 1,  (winCount*(HEIGHT/4)), GREEN);
        break;
    case LOSE:
        drawRect(WIDTH - 1, 0, 1, (loseCount*(HEIGHT/4)), RED);
        break;
    }

    if (winCount>=4){
        gameOver=WIN;
        endTurn=false;
    }else if(loseCount>=4){
        gameOver=LOSE;
        endTurn=false;
    }
}

//-------------------------------------------------------------------------------
//Object Functions

//gets acceleration data from accelerometer for the x axis
signed char getAccelerationX()
{
    unsigned char address = 0x18;
    unsigned char regOffset = 0x3;
    unsigned char readDataBuf[16];
    signed char result = 0;

    RET_IF_ERR(I2C_IF_Write(address, &regOffset, 1, 0));
    RET_IF_ERR(I2C_IF_Read(address, &readDataBuf[0], 1));
    result = (signed char) readDataBuf[0];

    return result;
}

//calculates the horizontal speed of the ball
int cal_hsp()
{
    signed char xAccel = getAccelerationX();
    int hsp;
    hsp = (xAccel*speedMag) / speedScale; //horizontal speed calculation
    return hsp;
}

//determines how the ball will bounce back from paddles
void paddleLogic(int objX){
    /*paddleNineth=plyLength/9;
    leftPaddle1 = (objX + paddleNineth);
    leftPaddle2 = leftPaddle1 + paddleNineth;
    leftPaddle3 = leftPaddle2 + paddleNineth;
    leftPaddle4 = leftPaddle3 + paddleNineth;
    rightPaddle1 = (objX+(2*plyLength/3));
    rightPaddle2 = rightPaddle1+paddleNineth;
    rightPaddle3 = rightPaddle2+paddleNineth;
    rightPaddle4 = rightPaddle3+paddleNineth;

    //saves direction ball is traveling
    int ballDir;
    if (ballhsp>0){
        ballDir=1;
    }else{
        ballDir=-1;
    }

    /determines how the ball bounces off of the paddle
    if (((ballX>=(objX-ballSize))&&(ballX<leftPaddle1)&&(ballDir>0))
       ||((ballX>=rightPaddle3)&&(ballX<(plyLength+objX+ballSize))&&(ballDir<0)))
    {
       ballhsp=-ballhsp;
    } else if (((ballX>=leftPaddle1)&&(ballX<leftPaddle2))
        ||((ballX>=rightPaddle2)&&(ballX<rightPaddle3)))
    {
        if (ballDir>0){
            ballhsp=level/3;
        }else{
            ballhsp=2*level/3;
        }
    } else if (((ballX>=leftPaddle2)&&(ballX<leftPaddle3))
        || ((ballX>=rightPaddle1)&&(ballX<rightPaddle2)))
    {
        if (ballDir>0){
            ballhsp=2*level/3;
        }else{
            ballhsp=level/3;
        }
    }
    */
    int paddleFifth=plyLength/5;
    leftPaddle1 = (objX + paddleFifth);
    leftPaddle2 = leftPaddle1 + paddleFifth;
    rightPaddle1 = leftPaddle2 + paddleFifth;
    rightPaddle2 = rightPaddle1 + paddleFifth;

    if ((ballX>=(objX-ballSize)&&(ballX<leftPaddle1))){
        ballhsp=-level/3;
    } else if ((ballX>=leftPaddle1)&&(ballX<leftPaddle2)){
        ballhsp=-2*level/3;
    }else if ((ballX>=leftPaddle2)&&(ballX<rightPaddle1)){
        ballhsp=ballhsp;
    }else if ((ballX>=rightPaddle1)&&(ballX<rightPaddle2)){
        ballhsp=2*level/3;
    }else if ((ballX>=rightPaddle2)&&(ballX<(objX+plyLength+ballSize))){
        ballhsp=level/3;
    }

}

//draws player paddle on screen
void drawPly()
{
    int hsp = cal_hsp();


    //Erases image of previous player image
    drawRect(plyX, plyY, plyLength, plyHeight, backgroundColor);

    //updates ball positions
    plyX += hsp;

    //wall collisions
    if (plyX > (WIDTH - plyLength - 1))
    {
        plyX = WIDTH - plyLength - 1;
    }
    else if (plyX < 1)
    {
        plyX = 1;
    }

    //Redraws ply at new location
    drawRect(plyX, plyY, plyLength, plyHeight, plyColor);

}

//draws cpu paddle on screen
void drawCPU(){
    if (mode==VERSUS){

        int hsp;

        if (!cpuSlow){
            hsp = (ballhsp*speedMag) / (speedScale);
        }else{
            hsp = (ballhsp*speedMag) / (speedScale*cpuLevelScale);
        }

        if (cpuFollowCount>0){
            cpuFollowCount-=1;
            cpuSlow=false;
        } else if (cpuFollowOffCount>0){
            cpuFollowOffCount-=1;
            cpuSlow=true;
        } else {
            switch (level){
            case LV1:
                cpuFollowCount=CPULV1SLOWON;
                cpuFollowOffCount=CPULV1SLOWON;
                break;
            case LV2:
                cpuFollowCount=CPULV2SLOWON;
                cpuFollowOffCount=CPULV2SLOWON;
                break;
            case LV3:
                cpuFollowCount=CPULV3SLOWON;
                cpuFollowOffCount=CPULV3SLOWON;
                break;
            }
        }


        //Erases image of previous cpu image
        drawRect(cpuX, cpuY, plyLength, plyHeight, backgroundColor);

        //updates ball positions
        cpuX += hsp;

        //wall collisions
        if (cpuX > (WIDTH - plyLength - 1))
        {
            cpuX = WIDTH - plyLength - 1;
        }
        else if (cpuX < 1)
        {
            cpuX = 1;
        }

        //Redraws cpu at new location
        drawRect(cpuX, cpuY, plyLength, plyHeight, cpuColor);
    }
}

//draws ball on screen
void drawBall()
{
    int vsp = (ballvsp*speedMag) / speedScale;
    int hsp = (ballhsp*speedMag) / speedScale;

    speedScale-=1;
    if (speedScale<MAXSPEEDSCALE*speedMag){
        speedScale=MAXSPEEDSCALE*speedMag;
    }
    if ((ballhsp != 0) || (ballvsp != 0))
    {
        //Erases image of previous ball
        drawCircle(ballX, ballY, ballSize, backgroundColor);

        //updates ball positions
        ballX += hsp;
        ballY += vsp;

        //Collisions

        //collides with Right wall
        if (ballX > (WIDTH - ballSize - 2))
        {
            ballX = WIDTH - ballSize - 2;
            ballhsp = -ballhsp;
            //wall=3;
            //nextStep=1;
            //collides with Left wall
        }
        else if (ballX < ballSize + 1)
        {

            ballX = ballSize + 1;
            ballhsp = -ballhsp;
            //wall=2;
            //nextStep=1;
        }

        //collides with bottom wall
        if (ballY > (HEIGHT - ballSize))
        {
            ballY = HEIGHT - ballSize;
            if (mode==VERSUS){
                endTurn=LOSE;
                loseCount+=1;
            }else{
                gameOver=true;
            }
            drawRect(plyX, plyY, plyLength, plyHeight, plyColor);
            time(&endTime);

        }

        //collides with CPU
        else if ((mode==VERSUS)&&(vsp < 0) && (ballY >= cpuY)
                && (ballY <= (cpuY+plyHeight))
                && (ballX > (cpuX - (ballSize / 2)))
                && (ballX < (cpuX + plyLength + (ballSize / 2))))
        {
            paddleLogic(cpuX);
            ballY=cpuY+plyHeight+(ballSize/4);
            ballvsp = -ballvsp;
        }

        //collides with top wall
        else if (ballY < ballSize + 1)
        {
            if (mode==VERSUS){
                ballY=0;
                endTurn=WIN;
                drawRect(cpuX, cpuY, plyLength, plyHeight, cpuColor);
                time(&endTime);
                winCount+=1;
            }
            ballY = ballSize + 1;
            ballvsp = -ballvsp;
            //wall=1;
            //nextStep=1;

        //collides with player
        }
        else if ((vsp > 0) && (ballY > plyY-ballSize)
                && (ballY < (plyY + plyHeight))
                && (ballX > (plyX - (ballSize / 2)))
                && (ballX < (plyX + plyLength + (ballSize / 2))))
        {
            paddleLogic(plyX);
            currentRally+=1;
            ballY=plyY-ballSize/4;
            ballvsp = -ballvsp;
        }


        //Rainbow Effect
        if ((speedScale==MAXSPEEDSCALE*speedMag))
        {
            switch (ballColorCycle)
            {
            case 1: ballColor = RED;    break;
            case 2: ballColor = ORANGE; break;
            case 3: ballColor = YELLOW; break;
            case 4: ballColor = GREEN;  break;
            case 5: ballColor = BLUE;   break;
            case 6: ballColor = PURPLE; break;
            case 7: ballColor = PINK;   break;
            }
            ballColorCycle += 1;
            if (ballColorCycle > 7)
            {
                ballColorCycle = 1;
            }
        }

        //Redraws ball at new location
        drawCircle(ballX, ballY, ballSize, ballColor);
        //redrawWall(wall,BLUE);
    }
}

//-------------------------------------------------------------------------------

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main()
  {
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();

    //Imported from Lab 3
    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    // sets up SPI
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
    SPI_IF_BIT_RATE,
                           SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVELOW |
                           SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);

    // Enable Chip select
    MAP_SPICSEnable(GSPI_BASE);

    //initiates board
    Adafruit_Init();

    titleScreen();
    UART_PRINT("Hello player!\n\r");
    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        fillScreen(RED);
        LOOP_FOREVER();
    }
     //Connect to the website with TLS encryption
        lRetVal = tls_connect();
        if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }//

    http_post(lRetVal, HELLO);
    modeSelection();
    levelSelection();
    gameStart(level);

    while (1)
    {
        //game plays
        drawPly();
        drawCPU();
        drawBall();
        if ((wall != 0) && (nextStep == 2))
        {
            redrawWall(wall, wallColor);
            wall = 0;
            nextStep = 0;
        }
        if (nextStep == 1)
        {
            nextStep += 1;
        }
        if (endTurn){
            scoreBoard();
            if (!gameOver){
                nextTurn(level);
            }
        }

        //game ends
        if (gameOver)
        {
            if (currentRally>maxRally){
                maxRally=currentRally;
            }
            playTime+=(endTime-startTime);

            char* gameoverMSG;
            switch (gameOver){
            //if the player loses in any mode
            case LOSE:
                drawWalls(RED);
                centerText("Game Over!", WHITE, backgroundColor);
                gameoverMSG=", Game Over! Try again? Y/N/R";
                break;
            //if the player wins in Versus Mode
            case WIN:
                drawWalls(GREEN);
                centerText("You Win!", WHITE, backgroundColor);
                gameoverMSG=", You win! Play again? Y/N/R";
                break;
            }

            // Creates string for JSON and UART Report
            char* messagecpy;
            char messageSMS[500];
            char* modeMSG;
            char* levelMSG;
            char timeOutput[50];
            char rallyOutput[4];

            snprintf(timeOutput,5,"%d",playTime);
            snprintf(rallyOutput,4,"%d",maxRally);

            messagecpy=messageSMS;
            strcpy(messagecpy,JSONSTART);
            messagecpy+=strlen(JSONSTART);
            strcpy(messagecpy,"Mode: ");
            messagecpy+=strlen("Mode: ");

            Report("------------------\n\r");
            Report("Mode: ");
            switch (mode){
            case VERSUS:
                Report("Versus\n\r");
                modeMSG="Versus, ";
                break;
            case ENDURANCE:
                Report("Endurance\n\r");
                modeMSG="Endurance, ";
                break;
            }
            strcpy(messagecpy,modeMSG);
            messagecpy+=strlen(modeMSG);
            strcpy(messagecpy,"Level: ");
            messagecpy+=strlen("Level: ");
            Report("Level: ");
            switch (level){
            case LV1:
                Report("1\n\r");
                levelMSG="1";
                break;
            case LV2:
                Report("2\n\r");
                levelMSG="2";
                break;
            case LV3:
                Report("3\n\r");
                levelMSG="3";
                break;
            }
            strcpy(messagecpy,levelMSG);
            messagecpy+=strlen(levelMSG);
            strcpy(messagecpy,", Time: ");
            messagecpy+=strlen(", Time: ");
            strcpy(messagecpy,timeOutput);
            messagecpy+=strlen(timeOutput);
            strcpy(messagecpy," seconds, Rally: ");
            messagecpy+=strlen(" seconds, Rally: ");
            strcpy(messagecpy,rallyOutput);
            messagecpy+=strlen(rallyOutput);
            strcpy(messagecpy,gameoverMSG);
            messagecpy+=strlen(gameoverMSG);
            strcpy(messagecpy,JSONEND);
            Report("Time: %ld seconds\n\r",playTime);
            Report("Longest Rally: %d\n\r",maxRally);
            Report("----------------------------------\n\r");
            Report(messageSMS);
            Report("----------------------------------\n\r");

            http_post(lRetVal,messageSMS);

            //LOOP_FOREVER();
            while (gameOver)
            {
                int ending;
                http_post(lRetVal, INPUTCLEAR);
                while(1){
                    ending=getAnswer("\"input\":\"");
                    if ((ending!=YES)&&(ending!=NO)&&(ending!=RETRY)){
                        //http_post(lRetVal, ERINPUT);
                    }else{
                        break;
                    }
                }
                switch (ending){
                case NO:
                //Return to Title Screen
                /*if ((GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
                        && (GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20))
                {*/
                    http_post(lRetVal, GOODBYE);
                    sl_Stop(SL_STOP_TIMEOUT);
                    /*while (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
                    {
                    }*/
                    titleScreen();

                    lRetVal = -1;
                    lRetVal = connectToAccessPoint();
                     //Set time so that encryption can be used
                    lRetVal = set_time();
                    if(lRetVal < 0) {
                        UART_PRINT("Unable to set time in the device");
                        fillScreen(RED);
                        LOOP_FOREVER();
                    }
                     //Connect to the website with TLS encryption
                    lRetVal = tls_connect();
                    if(lRetVal < 0) {
                        ERR_PRINT(lRetVal);
                    }
                    modeSelection();
                    levelSelection();
                    gameStart(level);
                    break;
                case YES:
                    //Reselects mode and level
                /*else if (GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20)
                {*/
                    modeSelection();
                    levelSelection();
                    gameStart(level);
                    break;
                case RETRY:
                //Retry the same mode and level
               /* else if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
                {*/
                    gameStart(level);
                    break;
                }
            }
        }
    }
}

