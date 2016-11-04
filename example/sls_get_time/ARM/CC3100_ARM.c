
/*******************************************************************************
* Title                 :   CC3100 Test Routines
* Filename              :   CC3100_ARM.c
* Author                :   RBL
* Origin Date           :   09/02/2016
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  09/02/16          .1             RBL     Module Created.
*  15/03/16          .2             RBL     Interrupt control moved to user space
*
*******************************************************************************/
/**
 * @file CC3100_ARM.c
 * @brief This module contains the test routines for both SPI and Radio function
 *
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "simplelink.h"
#include <stddef.h>
#include <stdint.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define APPLICATION_VERSION  "1.2.0"
#define SL_STOP_TIMEOUT      0xFF

#define MAX_BUF_SIZE         48

#define TIME2013             3565987200 /* 43 years + 11 days of leap years */
#define YEAR2013             2013
#define SEC_IN_MIN           60
#define SEC_IN_HOUR          3600
#define SEC_IN_DAY           86400

/*
 * Values for below macros shall be modified for setting the time-zone
 */
#define GMT_TIME_ZONE_HR     0
#define GMT_TIME_ZONE_MIN    00

#define SUCCESS                         0
#define STATUS_BIT_IP_ACQUIRED          2
#define STATUS_BIT_CONNECTION           0
#define STATUS_BIT_STA_CONNECTED        1
#define STATUS_BIT_IP_ACQUIRED          2
#define STATUS_BIT_IP_LEASED            3
#define STATUS_BIT_CONNECTION_FAILED    4
#define STATUS_BIT_P2P_NEG_REQ_RECEIVED 5
#define STATUS_BIT_SMARTCONFIG_DONE     6
#define STATUS_BIT_SMARTCONFIG_STOPPED  7

#define SSID_NAME       "MikroE Public"       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2  /* Security type of the Access piont */
#define PASSKEY         "mikroe.guest"        /* Password in case of secure AP */
#define PASSKEY_LEN     strlen(PASSKEY)       /* Password length in case of secure AP */

/* Configuration of the device when it comes up in AP mode */
#define SSID_AP_MODE       "MIKROE"           /* SSID of the CC3100 in AP mode */
#define PASSWORD_AP_MODE   "gotovo"           /* Password of CC3100 AP */
#define SEC_TYPE_AP_MODE   SL_SEC_TYPE_OPEN   /* Can take SL_SEC_TYPE_WEP or
                                                 SL_SEC_TYPE_WPA as well */

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#define CLI_WRITE( txt ) UART_Write_Text( txt )

#define ASSERT_ON_ERROR( error_code ) \
        { char tmp_txt[80];           \
          if( error_code < 0 )        \
          {                           \
               sprinti( tmp_txt, "Error %d\r\n", error_code ); \
               CLI_WRITE( tmp_txt );  \
          }                           \
        }

#define LOOP_FOREVER() { while(1); }

#define SET_STATUS_BIT(status_variable, bit)    status_variable |= ((unsigned long)1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)    status_variable &= ~((unsigned long)1<<(bit))
#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & ((unsigned long)1<<(bit))))
#define IS_CONNECTED(status_variable)             GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTION)
#define IS_STA_CONNECTED(status_variable)         GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_STA_CONNECTED)
#define IS_IP_ACQUIRED(status_variable)           GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_IP_ACQUIRED)
#define IS_IP_LEASED(status_variable)             GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_IP_LEASED)
#define IS_CONNECTION_FAILED(status_variable)     GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTION_FAILED)
#define IS_P2P_NEG_REQ_RECEIVED(status_variable)  GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_P2P_NEG_REQ_RECEIVED)
#define IS_SMARTCONFIG_DONE(status_variable)      GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_SMARTCONFIG_DONE)
#define IS_SMARTCONFIG_STOPPED(status_variable)   GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_SMARTCONFIG_STOPPED)
#define pal_Memcpy(x,y,z)   memcpy( (void *)x, (const void *)y, z)
#define pal_Strlen(x)       strlen((const char *)x)
#define pal_Memset(x,y,z)   memset((void *)x,y,z)


/******************************************************************************
* Module Typedefs
*******************************************************************************/
/* Application specific status/error codes */
typedef enum
{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap w/ host-driver's error codes */
    SNTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    SNTP_RECV_ERROR = SNTP_SEND_ERROR - 1,
    SNTP_SERVER_RESPONSE_ERROR = SNTP_RECV_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
// Pin and interrupt definitions
#if defined( STM32 )
sbit CC3100_HIB_PIN at GPIOA_ODR.B0;//Hibernate Pin
sbit CC3100_CS_PIN at GPIOD_ODR.B13;//Chip Select Pin
sbit CC3100_RST_PIN at GPIOC_ODR.B2;//Reset Pin
#elif defined( TI )
sbit CC3100_CS_PIN  at GPIO_PORTB_AHB_DATA.B0;
sbit CC3100_RST_PIN at GPIO_PORTE_AHB_DATA.B0;
sbit CC3100_HIB_PIN  at GPIO_PORTD_AHB_DATA.B0;

sbit CC3100_CS_PIN_Direction  at GPIO_PORTB_AHB_DIR.B0;
sbit CC3100_RST_PIN_Direction at GPIO_PORTE_AHB_DIR.B0;
sbit CC3100_HIB_PIN_Direction  at GPIO_PORTD_AHB_DIR.B0;
#endif
P_EVENT_HANDLER isr_event;          //Function pointer to handler for interrupt
// Status
_u32  g_Status = 0;

struct
{
    _u32   DestinationIP;
    _u32   elapsedSec;
    _u32   uGeneralVar;
    _u32   uGeneralVar1;

    _u16   ccLen;

    _i32   SockID;
    _i32   sGeneralVar;

    _u8 time[30];
    _u8 *ccPtr;

} app_data_t;

const _u8 sntp_server[30] = "pool.ntp.org"; /* NTP Time server */

const _u8 days_week[7][4] = { {"Tue"},
                              {"Wed"},
                              {"Thu"},
                              {"Fri"},
                              {"Sat"},
                              {"Sun"},
                              {"Mon"} };

const _u8 month_of_year[12][4] = { {"Jan"},
                                   {"Feb"},
                                   {"Mar"},
                                   {"Apr"},
                                   {"May"},
                                   {"Jun"},
                                   {"Jul"},
                                   {"Aug"},
                                   {"Sep"},
                                   {"Oct"},
                                   {"Nov"},
                                   {"Dec"} };

const _u8 num_of_days_month[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

const _u8 digits[] = "0123456789";

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static _i32 establish_connection();
static _i32 disconnect();
static _i32 configure_to_defaults();

static _i32 initialize_variables();
static void display_banner();

static _i32 get_host_IP();
static _i32 create_connection();
static _i32 get_sntp_time(_i16 gmt_hr, _i16 gmt_min);
static _u16 itoa(_i16 cNum, _u8 *cString);

/******************************************************************************
* Function Definitions
*******************************************************************************/
/**
 * @brief Convert integer to ASCII in decimal base
 *
 * @param[in]      cNum - integer number to convert
 * @param[OUT]     cString - output string
 * @return         number of ASCII characters
 */
static _u16 itoa(_i16 cNum, _u8 *cString)
{
    _u16 length = 0;
    _u8* ptr = NULL;
    _i16 uTemp = cNum;

    /* value 0 is a special case */
    if (cNum == 0)
    {
        length = 1;
        *cString = '0';

        return length;
    }

    /* Find out the length of the number, in decimal base */
    length = 0;
    while (uTemp > 0)
    {
        uTemp /= 10;
        length++;
    }

    /* Do the actual formatting, right to left */
    uTemp = cNum;
    ptr = cString + length;
    while (uTemp > 0)
    {
        --ptr;
        *ptr = digits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}

/**
 * @brief Get the required data from the server.
 *
 * @param[in]      gmt_hr - GMT offset hours
 * @param[in]      gmt_min - GMT offset minutes
 * @return         0 on success, -ve otherwise
 */
static _i32 get_sntp_time(_i16 gmt_hr, _i16 gmt_min)
{
    /*
                                NTP Packet Header:
           0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9  0  1
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |LI | VN  |Mode |    Stratum    |     Poll      |   Precision    |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                          Root  Delay                           |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                       Root  Dispersion                         |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                     Reference Identifier                       |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                    Reference Time-stamp (64)                    |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                    Originate Time-stamp (64)                    |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                     Receive Time-stamp (64)                     |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                     Transmit Time-stamp (64)                    |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                 Key Identifier (optional) (32)                 |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                 Message Digest (optional) (128)                |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    */

    SlSockAddrIn_t  local_addr;
    SlSockAddr_t    address;

    _u8  data_buffer[MAX_BUF_SIZE];
    _i32 ret_value = -1;
    _i16 addr_size = 0;

    /* For time zone with negative GMT value, change minutes to negative for
     * computation */
    if(gmt_hr < 0 && gmt_min > 0)
        gmt_min = gmt_min * (-1);

    sl_Memset(data_buffer, 0, sizeof(data_buffer));
    data_buffer[0] = '\x1b';

    address.sa_family = AF_INET;
    /* the source port */
    address.sa_data[0] = 0x00;
    address.sa_data[1] = 0x7B;    /* 123 */
    address.sa_data[2] = (_u8)((app_data_t.DestinationIP >> 24) & 0xff);
    address.sa_data[3] = (_u8)((app_data_t.DestinationIP >> 16) & 0xff);
    address.sa_data[4] = (_u8)((app_data_t.DestinationIP >> 8) & 0xff);
    address.sa_data[5] = (_u8) (app_data_t.DestinationIP & 0xff);

    ret_value = sl_SendTo(app_data_t.SockID, data_buffer, sizeof(data_buffer), 0,
                     &address, sizeof(address));
    if (ret_value != sizeof(data_buffer))
    {
        /* could not send SNTP request */
        CLI_WRITE((_u8 *)" Device couldn't send SNTP request\n\r\n\r");
        ASSERT_ON_ERROR(SNTP_SEND_ERROR);
    }

    addr_size = sizeof(SlSockAddrIn_t);
    local_addr.sin_family = SL_AF_INET;
    local_addr.sin_port = 0;
    local_addr.sin_addr.s_addr = 0;

    ret_value = sl_Bind(app_data_t.SockID,(SlSockAddr_t *)&local_addr, addr_size);
    if(ret_value < 0)
        ASSERT_ON_ERROR(ret_value);

    ret_value = sl_RecvFrom(app_data_t.SockID, data_buffer, sizeof(data_buffer), 0,
                       (SlSockAddr_t *)&local_addr,  (SlSocklen_t*)&addr_size);
    if (ret_value <= 0)
    {
        CLI_WRITE((_u8 *)" Device couldn't receive time information \n\r");
        ASSERT_ON_ERROR(SNTP_RECV_ERROR);
    }

    if ((data_buffer[0] & 0x7) != 4)    /* expect only server response */
    {
        /* MODE is not server, abort */
        CLI_WRITE((_u8 *)" Device is expecting response from server only!\n\r");
        ASSERT_ON_ERROR(SNTP_SERVER_RESPONSE_ERROR);
    }
    else
    {
        _u8 index;

        app_data_t.elapsedSec = data_buffer[40];
        app_data_t.elapsedSec <<= 8;
        app_data_t.elapsedSec += data_buffer[41];
        app_data_t.elapsedSec <<= 8;
        app_data_t.elapsedSec += data_buffer[42];
        app_data_t.elapsedSec <<= 8;
        app_data_t.elapsedSec += data_buffer[43];

        app_data_t.elapsedSec -= TIME2013;

        /* correct the time zone */
        app_data_t.elapsedSec += (gmt_hr * SEC_IN_HOUR);
        app_data_t.elapsedSec += (gmt_min * SEC_IN_MIN);

        app_data_t.ccPtr = &app_data_t.time[0];

        /* day */
        app_data_t.sGeneralVar = app_data_t.elapsedSec/SEC_IN_DAY;
        pal_Memcpy(app_data_t.ccPtr, days_week[app_data_t.sGeneralVar%7], 3);
        app_data_t.ccPtr += 3;
        *app_data_t.ccPtr++ = '\x20';

        /* month */
        app_data_t.sGeneralVar %= 365;
        for (index = 0; index < 12; index++)
        {
            app_data_t.sGeneralVar -= num_of_days_month[index];
            if (app_data_t.sGeneralVar < 0)
                break;
        }

        pal_Memcpy( app_data_t.ccPtr, month_of_year[index], 3 );
        app_data_t.ccPtr += 3;
        *app_data_t.ccPtr++ = '\x20';

        /* date */
        /* restore the day in current month*/
        app_data_t.sGeneralVar += num_of_days_month[index];
        app_data_t.ccLen = itoa(app_data_t.sGeneralVar + 1, app_data_t.ccPtr);
        app_data_t.ccPtr += app_data_t.ccLen;
        *app_data_t.ccPtr++ = '\x20';

        /* year */
        /* number of days since beginning of 2013 */
        app_data_t.uGeneralVar = app_data_t.elapsedSec/SEC_IN_DAY;
        app_data_t.uGeneralVar /= 365;
        app_data_t.ccLen = itoa(YEAR2013 + app_data_t.uGeneralVar , app_data_t.ccPtr);
        app_data_t.ccPtr += app_data_t.ccLen;
        *app_data_t.ccPtr++ = '\x20';

        /* time */
        app_data_t.uGeneralVar = app_data_t.elapsedSec%SEC_IN_DAY;
        /* number of seconds per hour */
        app_data_t.uGeneralVar1 = app_data_t.uGeneralVar%SEC_IN_HOUR;
        app_data_t.uGeneralVar /= SEC_IN_HOUR;               /* number of hours */
        app_data_t.ccLen = itoa(app_data_t.uGeneralVar, app_data_t.ccPtr);
        app_data_t.ccPtr += app_data_t.ccLen;
        *app_data_t.ccPtr++ = ':';
        /* number of minutes per hour */
        app_data_t.uGeneralVar = app_data_t.uGeneralVar1/SEC_IN_MIN;
        /* number of seconds per minute */
        app_data_t.uGeneralVar1 %= SEC_IN_MIN;
        app_data_t.ccLen = itoa(app_data_t.uGeneralVar, app_data_t.ccPtr);
        app_data_t.ccPtr += app_data_t.ccLen;
        *app_data_t.ccPtr++ = ':';
        app_data_t.ccLen = itoa(app_data_t.uGeneralVar1, app_data_t.ccPtr);
        app_data_t.ccPtr += app_data_t.ccLen;
        *app_data_t.ccPtr++ = '\x20';

        *app_data_t.ccPtr++ = '\0';

        CLI_WRITE((_u8 *)"\r\n Server ");
        CLI_WRITE((_u8 *)sntp_server);
        CLI_WRITE((_u8 *)" has responded with time information");
        CLI_WRITE((_u8 *)"\n\r\r\n ");
        CLI_WRITE((_u8 *)app_data_t.time);
        CLI_WRITE((_u8 *)"\n\r\r\n");
    }

    return SUCCESS;
}

/**
 * @brief Create UDP socket to communicate with server.
 *
 * @param[in]      none
 * @return         Socket descriptor for success otherwise negative
 */
static _i32 create_connection()
{
    _i32 sd = 0;

    sd = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, IPPROTO_UDP);
    if( sd < 0 )
        CLI_WRITE((_u8 *)"Error creating socket\n\r\n\r");

    return sd;
}

/**
 * @brief Gets the Server IP address
 *
 * @param[in]      none
 * @return         zero for success and -1 for error
 */
static _i32 get_host_IP()
{
    _i32 status = 0;
    app_data_t.DestinationIP = 0;

    status = sl_NetAppDnsGetHostByName( (_i8*)sntp_server, strlen(sntp_server),
                                       &app_data_t.DestinationIP, SL_AF_INET);
    ASSERT_ON_ERROR(status);

    return SUCCESS;
}

/**
 * @brief This function configure the SimpleLink device in its default state. It:
 *            - Sets the mode to STATION
 *            - Configures connection policy to Auto and AutoSmartConfig
 *            - Deletes all the stored profiles
 *            - Enables DHCP
 *            - Disables Scan policy
 *            - Sets Tx power to maximum
 *            - Sets power policy to normal
 *            - Unregisters mDNS services
 *            - Remove all filters
 *
 * @param[in]      none
 *
 * @return         On success, zero is returned. On error, negative is returned
 */
static _i32 configure_to_defaults()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        /* If the device is in AP mode, we need to wait for this event before
           doing anything */
        if (ROLE_AP == mode)
            while(!IS_IP_ACQUIRED(g_Status))
                _SlNonOsMainLoopTask();

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initialize_variables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/**
 * @brief Connecting to a WLAN Access point
 *
 * This function connects to the required AP (SSID_NAME).
 * The function will return once we are connected and have acquired IP address
 *
 * @param[in]  None
 *
 * @return     0 on success, negative error-code on error
 * @warning    If the WLAN connection fails or we don't acquire an IP address,
 *             We will be stuck in this function forever.
*/
static _i32 establish_connection()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;
    _i16 length = strlen( SSID_NAME );

    secParams.Key = PASSKEY;
    secParams.KeyLen = strlen( secParams.Key );
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect( SSID_NAME, length, 0, &secParams, 0 );
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while( ( !IS_CONNECTED( g_Status ) ) || ( !IS_IP_ACQUIRED( g_Status ) ) )
        _SlNonOsMainLoopTask();

    return SUCCESS;
}

/**
 * @brief Disconnecting from a WLAN Access point
 *
 * This function disconnects from the connected AP
 *
 * @param[in]      None
 * @return         none
 * @warning        If the WLAN disconnection fails, we will be stuck in this function forever.
 */
static _i32 disconnect()
{
    _i32 retVal = -1;

    /*
     * The function returns 0 if 'Disconnected done', negative number if
     * already disconnected, wait for 'disconnection' event if 0 is returned,
     * ignore other return-codes.
     */
    retVal = sl_WlanDisconnect();

    if(0 == retVal)
        while(IS_CONNECTED(g_Status))
            _SlNonOsMainLoopTask();

    return SUCCESS;
}

/**
 * @brief This function initializes the application variables
 *
 * @param[in]  None
 * @return     0 on success, negative error-code on error
 */
static _i32 initialize_variables()
{
    g_Status = 0;
    pal_Memset(&app_data_t, 0, sizeof(app_data_t));

    return SUCCESS;
}

/**
 * @brief This function displays the application's banner
 */
static void display_banner()
{
    CLI_WRITE((_u8 *)"\n\r\n\r");
    CLI_WRITE((_u8 *)" Get time application - Version ");
    CLI_WRITE(APPLICATION_VERSION);
    CLI_WRITE((_u8 *)"\n\r*******************************************************************************\n\r");
}

/******************************************************************************
 ******************** Required USER Defined Functions *************************
 *****************************************************************************/
void cc3100_wlan_event_handler( SlWlanEvent_t *p_wlan_event )
{
    if(p_wlan_event == NULL)
    {
        CLI_WRITE(" [WLAN EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch(p_wlan_event->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &p_wlan_event->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &p_wlan_event->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_WRITE((_u8 *)" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_WRITE((_u8 *)" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_WRITE((_u8 *)" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void cc3100_net_app_event_handler( SlNetAppEvent_t *p_net_app_event )
{
    if(p_net_app_event == NULL)
    {
        CLI_WRITE(" [NETAPP EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch(p_net_app_event->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &p_net_app_event->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_WRITE((_u8 *)" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void cc3100_http_server_callback( SlHttpServerEvent_t *p_http_event,
                           SlHttpServerResponse_t *p_http_response )
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_WRITE((_u8 *)" [HTTP EVENT] Unexpected event \n\r");
}

void cc3100_general_event_handler( SlDeviceEvent_t *p_device_event )
{
    /*
     * Most of the general errors are not FATAL are to be handled
     * appropriately by the application
     */
    CLI_WRITE((_u8 *)" [GENERAL EVENT] \n\r");
}

void cc3100_socket_event_handler( SlSockEvent_t *p_socket )
{
    if(p_socket == NULL)
    {
        CLI_WRITE(" [SOCK EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch( p_socket->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
           /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_u *pEventData = NULL;
            * pEventData = & p_socket->socketAsyncEvent;
            */
            switch( p_socket->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    CLI_WRITE((_u8 *)" [SOCK EVENT] Close socket operation \
                                       failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_WRITE((_u8 *)" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_WRITE((_u8 *)" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}

void cc3100_interrupt_enable(void)
{
    #if defined( STM32 )
    NVIC_IntEnable( IVT_INT_EXTI15_10 ); // Enable External interrupt
    #elif defined( TI )
    NVIC_IntEnable( IVT_INT_GPIOF );
    #endif
}

void cc3100_interrupt_disable()
{
    #if defined( STM32 )
    NVIC_IntDisable( IVT_INT_EXTI15_10 ); // Disable External interrupt
    #elif defined( TI )
    NVIC_IntDisable( IVT_INT_GPIOF );
    #endif
}

/******************************************************************************
 ******************** END Required USER Defined *******************************
 *****************************************************************************/

/*
 * System initialize
 */
void system_init()
{
    DisableInterrupts();
    #if defined( STM32 )
    GPIO_Digital_Output( &GPIOC_BASE, _GPIO_PINMASK_2 );
    GPIO_Digital_Output( &GPIOA_BASE, _GPIO_PINMASK_0 );
    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    GPIO_Digital_Input( &GPIOD_BASE, _GPIO_PINMASK_10 );

    SPI3_Init_Advanced( _SPI_FPCLK_DIV16,
                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                        _SPI_SECOND_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );

    RCC_APB2ENR.AFIOEN = 1;          // Enable clock for alternate pin functions
    AFIO_EXTICR3 = ( 1 << EXTI100 ) | ( 1 << EXTI101 ); // Mask interrupt
    EXTI_RTSR |= ( 1 << TR10 );      // Set interrupt on Rising edge
    EXTI_IMR |= ( 1 << MR10 );       // Set mask

    // UART
    UART1_Init_Advanced( 115200,
                         _UART_8_BIT_DATA,
                         _UART_NOPARITY,
                         _UART_ONE_STOPBIT,
                         &_GPIO_MODULE_USART1_PA9_10 );
    #elif defined( TI )
    GPIO_Digital_Output( &GPIO_PORTB_AHB, _GPIO_PINMASK_0 ); // CC3100 CS
    GPIO_Digital_Output( &GPIO_PORTE_AHB, _GPIO_PINMASK_0 ); // CC3100 RST Dummy
    GPIO_Digital_Output( &GPIO_PORTD_AHB, _GPIO_PINMASK_0 ); // CC3100 HIB
    GPIO_Digital_Input( &GPIO_PORTF_AHB, _GPIO_PINMASK_4 );  // CC3100 INT

    /* Set SPI1 to the Master Mode, data length is 8-bit, clock = 15MHz, clock
    IDLE state high and data transmitted at the first clock edge transition*/
    SPI0_Init_Advanced( 15000000, _SPI_MASTER, _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                       _SPI_SECOND_CLK_EDGE_TRANSITION,
                       &_GPIO_MODULE_SPI0_A245_AHB );

    GPIO_PORTF_AHB_AFSEL |= ( 1 << _GPIO_PINCODE_4 );  // Enable alternate pin functions
    GPIO_PORTF_AHB_IM    |= ( 1 << _GPIO_PINCODE_4 );  // Set mask to pin 2
    GPIO_PORTF_AHB_IEV   |= ( 1 << _GPIO_PINCODE_4 );  // Rising Edge will trigger interrupt
    //GPIO_PORTB_AHB_IS    |= ( 1 << _GPIO_PINCODE_2 );  // Interrupt sense on Edge = 0, level = 1
    //GPIO_PORTB_AHB_IBE   |= ( 1 << _GPIO_PINCODE_2 );  // Edge/Level detection triggers interrupt



    UART7_Init_Advanced( 115200, 115200, _UART_8_BIT_DATA, _UART_NOPARITY,
                        _UART_ONE_STOPBIT, _UART_HIGH_SPEED,
                        &_GPIO_MODULE_UART7_C45_AHB );

    #endif
    Delay_ms( 100 );
    UART_Write_Text( "System Initialized\r\n" );
    EnableInterrupts();
}

void main(void)
{
    _i32 retVal = -1;

    system_init();

    retVal = initialize_variables();
    ASSERT_ON_ERROR(retVal);

    display_banner();

    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */
    retVal = configure_to_defaults();

    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            CLI_WRITE((_u8 *)" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_WRITE((_u8 *)" Device is configured in default state \n\r");

    /*
     * Initializing the CC3100 device
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    retVal = sl_Start(0, 0, 0);

    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_WRITE((_u8 *)" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_WRITE((_u8 *)" Device started as STATION \n\r");

    /* Connecting to WLAN AP */
    retVal = establish_connection();
    if(retVal < 0)
    {
        CLI_WRITE((_u8 *)" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_WRITE(" Connection established w/ AP and IP is acquired \n\r");

    retVal = get_host_IP();

    if(retVal < 0)
    {
        CLI_WRITE((_u8 *)" Unable to get host IP\n\r\n\r");
        LOOP_FOREVER();
    }

    app_data_t.SockID = create_connection();

    if(app_data_t.SockID < 0)
        LOOP_FOREVER();

    retVal = get_sntp_time(GMT_TIME_ZONE_HR,GMT_TIME_ZONE_MIN);

    if(retVal < 0)
        LOOP_FOREVER();

    retVal = sl_Close(app_data_t.SockID);
    if(retVal < 0)
        LOOP_FOREVER();

    retVal = disconnect();
    if(retVal < 0)
    {
        CLI_WRITE((_u8 *)" Failed to disconnect from the AP \n\r");
        LOOP_FOREVER();
    }
}

/**
 * @brief register an interrupt handler for the host IRQ
 *
 */
#if defined( STM32 )
void irq_handler() iv IVT_INT_EXTI15_10 ics ICS_AUTO
{
    EXTI_PR |= ( 1 << PR10 );
    ( *isr_event )( 0 );
}
#elif defined( TI )
void irq_handler() iv IVT_INT_GPIOF ics ICS_AUTO
{
    if( GPIO_PORTF_AHB_RIS & ( 1 << _GPIO_PINCODE_4 ))
    {
        GPIO_PORTF_AHB_ICR |= ( 1 << _GPIO_PINCODE_4 );
        ( *isr_event )( 0 );
    }
}
#endif