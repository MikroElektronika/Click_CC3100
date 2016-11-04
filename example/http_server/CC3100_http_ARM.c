/*
 * Title                 :   CC3100 Get Weather Example
 * Filename              :   CC3100_http_ARM.c
 * Author                :   Milos Vidojevic
 * Origin Date           :   31. Oct 2016.
 * Notes                 :   None
 */
/******************************************************************************
* Includes
*******************************************************************************/

#include "simplelink.h"
#include "resources.h"
#include <stddef.h>
#include <stdint.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

#define APPLICATION_VERSION  "1.2.0"
#define SL_STOP_TIMEOUT      0xFF

#define CITY_NAME       "Belgrade"
#define WEATHER_SERVER  "api.apixu.com"
#define PREFIX_BUFFER   "GET /v1/current.json?key=90763912dcfb4bb8a7e01104163010&q="
#define POST_BUFFER     " HTTP/1.1\r\n"
#define POST_BUFFER2    "Host:api.apixu.com\r\n\r\n"

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   2048

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

/* Station mode configuration */
#define SSID_NAME       "MikroE Public"                                         /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2                                    /* Security type of the Access piont */
#define PASSKEY         "mikroe.guest"                                          /* Password in case of secure AP */
#define PASSKEY_LEN     strlen(PASSKEY)                                         /* Password length in case of secure AP */

/* AP mode configuration */
#define SSID_AP_MODE       "MIKROE"                                             /* SSID of the CC3100 in AP mode */
#define PASSWORD_AP_MODE   "gotovo"                                             /* Password of CC3100 AP */
#define SEC_TYPE_AP_MODE   SL_SEC_TYPE_OPEN                                     /* Can take SL_SEC_TYPE_WEP or SL_SEC_TYPE_WPA as well */

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

#define CLI_Write( txt ) UART_Write_Text( txt )

#define ASSERT_ON_ERROR( error_code ) \
        { char tmp_txt[80];           \
          if( error_code < 0 )        \
          {                           \
               sprinti( tmp_txt, "Error %d\r\n", error_code ); \
               CLI_Write( tmp_txt );  \
          }                           \
        }

#define LOOP_FOREVER() { while(1); }

#define SET_STATUS_BIT(status_variable, bit)    \
        status_variable |= ((unsigned long)1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)    \
        status_variable &= ~((unsigned long)1<<(bit))
#define GET_STATUS_BIT(status_variable, bit)    \
        (0 != (status_variable & ((unsigned long)1<<(bit))))

#define IS_CONNECTED(status_variable)             \
        GET_STATUS_BIT(status_variable, STATUS_BIT_CONNECTION)
#define IS_STA_CONNECTED(status_variable)         \
        GET_STATUS_BIT(status_variable, STATUS_BIT_STA_CONNECTED)
#define IS_IP_ACQUIRED(status_variable)           \
        GET_STATUS_BIT(status_variable, STATUS_BIT_IP_ACQUIRED)
#define IS_IP_LEASED(status_variable)             \
        GET_STATUS_BIT(status_variable, STATUS_BIT_IP_LEASED)
#define IS_CONNECTION_FAILED(status_variable)     \
        GET_STATUS_BIT(status_variable, STATUS_BIT_CONNECTION_FAILED)
#define IS_P2P_NEG_REQ_RECEIVED(status_variable)  \
        GET_STATUS_BIT(status_variable, STATUS_BIT_P2P_NEG_REQ_RECEIVED)
#define IS_SMARTCONFIG_DONE(status_variable)      \
        GET_STATUS_BIT(status_variable, STATUS_BIT_SMARTCONFIG_DONE)
#define IS_SMARTCONFIG_STOPPED(status_variable)   \
        GET_STATUS_BIT(status_variable, STATUS_BIT_SMARTCONFIG_STOPPED)

#define pal_Memcpy(x,y,z)   memcpy( (void *)x, (const void *)y, z)
#define pal_Strlen(x)       strlen((const char *)x)
#define pal_Memset(x,y,z)   memset((void *)x,y,z)
#define pal_Strcpy(x,y)     strcpy((unsigned char *)x,(unsigned char *)y)
#define pal_Strstr(x,y)     strstr((char *)x,(char *)y)

/******************************************************************************
* Module Typedefs
*******************************************************************************/

typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

#ifdef STM32
sbit CC3100_HIB_PIN at GPIOA_ODR.B0;                                            //Hibernate Pin
sbit CC3100_CS_PIN at GPIOD_ODR.B13;                                            //Chip Select Pin
sbit CC3100_RST_PIN at GPIOC_ODR.B2;                                            //Reset Pin
#endif
#ifdef TI
sbit CC3100_CS_PIN  at GPIO_PORTB_AHB_DATA.B0;
sbit CC3100_RST_PIN at GPIO_PORTE_AHB_DATA.B0;
sbit CC3100_HIB_PIN  at GPIO_PORTD_AHB_DATA.B0;

sbit CC3100_CS_PIN_Direction  at GPIO_PORTB_AHB_DIR.B0;
sbit CC3100_RST_PIN_Direction at GPIO_PORTE_AHB_DIR.B0;
sbit CC3100_HIB_PIN_Direction  at GPIO_PORTD_AHB_DIR.B0;
#endif

unsigned int TFT_DataPort at GPIOE_ODR;
sbit TFT_RST at GPIOE_ODR.B8;
sbit TFT_RS at GPIOE_ODR.B12;
sbit TFT_CS at GPIOE_ODR.B15;
sbit TFT_RD at GPIOE_ODR.B10;
sbit TFT_WR at GPIOE_ODR.B11;
sbit TFT_BLED at GPIOE_ODR.B9;

P_EVENT_HANDLER isr_event;
_u32  g_Status = 0;

struct{

    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];
    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];
    _u32 DestinationIP;
    _i16 SockID;

}g_AppData;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static _i32 establishConnectionWithAP();
static _i32 disconnectFromAP();
static _i32 configureSimpleLinkToDefaultState();

static _i32 initializeAppVariables();
static void  displayBanner();

static _i32 getWeather();
static _i32 getHostIP();
static _i32 createConnection();
static _i32 getData();

static void display_init();

/******************************************************************************
* Function Definitions
*******************************************************************************/

static void display_init()
{
    TFT_Init_ILI9341_8bit( 320, 240 );
    TFT_BLED = 1;
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Brush( 1,CL_WHITE,0,0,0,0 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );
    TFT_Fill_Screen( CL_WHITE );
    TFT_Set_Pen( CL_BLACK, 1 );
    TFT_Line(  20,  46, 300,  46 );
    TFT_Line(  20,  70, 300,  70 );
    TFT_Line(  20, 196, 300, 196 );;
    TFT_Line(  20, 220, 300, 220 );
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "CC3100 click", 100, 14 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLUE, FO_HORIZONTAL );
    TFT_Write_Text( "Texas Instruments SimpleLink", 65, 50 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "EasyMx PRO v7 for STM32", 19, 223);
    TFT_Set_Font( &Verdana12x13_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "www.mikroe.com", 200, 223 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "City :", 40, 76 );
    TFT_Write_Text( "Temperature (C) :", 40, 96 );
    TFT_Write_Text( "Weather : ", 40, 116 );
    TFT_Write_Text( "Wind (KPH) :", 40, 136 );
    TFT_Write_Text( "Wind direction:", 40, 156 );
    TFT_Write_Text( "Feels Like (C): ", 40, 176 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
}

static _i32 getData()
{
    _u8 *p_startPtr = NULL;
    _u8 *p_endPtr = NULL;
    _u8* p_bufLocation = NULL;
    _i32 retVal = -1;

    pal_Memset(g_AppData.Recvbuff, 0, sizeof(g_AppData.Recvbuff));

    p_bufLocation = g_AppData.SendBuff;
    pal_Strcpy(p_bufLocation, PREFIX_BUFFER);

    p_bufLocation += pal_Strlen(PREFIX_BUFFER);
    pal_Strcpy(p_bufLocation, g_AppData.CityName);

    p_bufLocation += pal_Strlen(g_AppData.CityName);
    pal_Strcpy(p_bufLocation, POST_BUFFER);

    p_bufLocation += pal_Strlen(POST_BUFFER);
    pal_Strcpy(p_bufLocation, POST_BUFFER2);
    
    retVal = sl_Send(g_AppData.SockID, g_AppData.SendBuff, pal_Strlen(g_AppData.SendBuff), 0);
    if(retVal != pal_Strlen(g_AppData.SendBuff))
        ASSERT_ON_ERROR(HTTP_SEND_ERROR);

    retVal = sl_Recv(g_AppData.SockID, &g_AppData.Recvbuff[0], MAX_SEND_RCV_SIZE, 0);
    if(retVal <= 0)
        ASSERT_ON_ERROR(HTTP_RECV_ERROR);

    g_AppData.Recvbuff[pal_Strlen(g_AppData.Recvbuff)] = '\0';

    p_startPtr = (_u8 *)pal_Strstr(g_AppData.Recvbuff, "name\":");
    
    if( NULL != p_startPtr )
    {
        p_startPtr = p_startPtr + pal_Strlen("name\":") + 1;
        p_endPtr = (_u8 *)pal_Strstr(p_startPtr, "\"");
        if( NULL != p_endPtr )
        {
            *p_endPtr = 0;
        }
    }

    CLI_Write((_u8 *)"\n ************************ \n\r\n\r");
    CLI_Write((_u8 *)" City: ");
    TFT_Write_Text( p_startPtr, 200, 76 );
    if(p_startPtr == NULL)
    {
        CLI_Write((_u8 *)"N/A\n\r\n\r");
    }
    else
    {
        CLI_Write((_u8 *)p_startPtr);
        CLI_Write((_u8 *)"\n\r\n\r");
    }

    if(p_endPtr == NULL)
    {
        CLI_Write((_u8 *)" Error during parsing the data.\r\n");
        ASSERT_ON_ERROR(HTTP_INVALID_RESPONSE);
    }

    p_startPtr = (_u8 *)pal_Strstr(p_endPtr+1, "temp_c\":");
    if( NULL != p_startPtr )
    {
        p_startPtr = p_startPtr + pal_Strlen("temp_c\":");
        p_endPtr = (_u8 *)pal_Strstr(p_startPtr, "\"") - 1;
        if( NULL != p_endPtr )
        {
            *p_endPtr = 0;
        }
    }

    CLI_Write((_u8 *)" Temperature: ");
    TFT_Write_Text( p_startPtr, 200, 96 );
    if(p_startPtr == NULL)
    {
        CLI_Write((_u8 *)"N/A\n\r\n\r");
    }
    else
    {
        CLI_Write((_u8 *)p_startPtr);
        CLI_Write((_u8 *)" C\n\r\n\r");
    }
    
    p_startPtr = (_u8 *)pal_Strstr(p_endPtr+1, "text\":");
    if( NULL != p_startPtr )
    {
        p_startPtr = p_startPtr + pal_Strlen("text\":") + 1;
        p_endPtr = (_u8 *)pal_Strstr(p_startPtr + 1, "\"");
        if( NULL != p_endPtr )
        {
            *p_endPtr = 0;
        }
    }
    
    CLI_Write((_u8 *)" Weather: ");
    TFT_Write_Text( p_startPtr, 200, 116 );
    if(p_startPtr == NULL)
    {
        CLI_Write((_u8 *)"N/A\n\r\n\r");
    }
    else
    {
        CLI_Write((_u8 *)p_startPtr);
        CLI_Write((_u8 *)"\n\r\n\r");
    }
    
    p_startPtr = (_u8 *)pal_Strstr(p_endPtr+1, "wind_kph\":");
    if( NULL != p_startPtr )
    {
        p_startPtr = p_startPtr + pal_Strlen("wind_kph\":");
        p_endPtr = (_u8 *)pal_Strstr(p_startPtr, "\"") - 1;
        if( NULL != p_endPtr )
        {
            *p_endPtr = 0;
        }
    }

    CLI_Write((_u8 *)" Wind : ");
    TFT_Write_Text( p_startPtr, 200, 136 );
    if(p_startPtr == NULL)
    {
        CLI_Write((_u8 *)"N/A\n\r\n\r");
    }
    else
    {
        CLI_Write((_u8 *)p_startPtr);
        CLI_Write((_u8 *)" KPH\n\r\n\r");
    }
    
    p_startPtr = (_u8 *)pal_Strstr(p_endPtr+1, "wind_dir\":");
    if( NULL != p_startPtr )
    {
        p_startPtr = p_startPtr + pal_Strlen("wind_dir\":") + 1;
        p_endPtr = (_u8 *)pal_Strstr(p_startPtr + 1, "\"");
        if( NULL != p_endPtr )
        {
            *p_endPtr = 0;
        }
    }

    CLI_Write((_u8 *)" Wind Direction: ");
    TFT_Write_Text( p_startPtr, 200, 156 );
    if(p_startPtr == NULL)
    {
        CLI_Write((_u8 *)"N/A\n\r\n\r");
    }
    else
    {
        CLI_Write((_u8 *)p_startPtr);
        CLI_Write((_u8 *)"\n\r\n\r");
    }
    
    p_startPtr = (_u8 *)pal_Strstr(p_endPtr+1, "feelslike_c\":");
    if( NULL != p_startPtr )
    {
        p_startPtr = p_startPtr + pal_Strlen("feelslike_c\":");
        p_endPtr = (_u8 *)pal_Strstr(p_startPtr, "\"") - 1;
        if( NULL != p_endPtr )
        {
            *p_endPtr = 0;
        }
    }

    CLI_Write((_u8 *)" Feels Like: ");
    TFT_Write_Text( p_startPtr, 200, 176 );
    if(p_startPtr == NULL)
    {
        CLI_Write((_u8 *)"N/A\n\r\n\r");
    }
    else
    {
        CLI_Write((_u8 *)p_startPtr);
        CLI_Write((_u8 *)" C\n\r\n\r");
    }

    CLI_Write((_u8 *)"\n ************************ \n\r\n\r");

    return SUCCESS;
}

static _i32 createConnection()
{
    SlSockAddrIn_t  Addr;

    _i16 sd = 0;
    _i16 AddrSize = 0;
    _i32 ret_val = 0;

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(80);
    Addr.sin_addr.s_addr = sl_Htonl(g_AppData.DestinationIP);
    AddrSize = sizeof(SlSockAddrIn_t);

    sd = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( sd < 0 )
    {
        CLI_Write((_u8 *)" Error creating socket\n\r\n\r");
        ASSERT_ON_ERROR(sd);
    }

    ret_val = sl_Connect(sd, ( SlSockAddr_t *)&Addr, AddrSize);
    if( ret_val < 0 )
    {
        CLI_Write((_u8 *)" Error connecting to server\n\r\n\r");
        ASSERT_ON_ERROR(ret_val);
    }

    return sd;
}

static _i32 getHostIP()
{
    _i32 status = -1;

    status = sl_NetAppDnsGetHostByName((_i8 *)g_AppData.HostName,
                                       pal_Strlen(g_AppData.HostName),
                                       &g_AppData.DestinationIP, SL_AF_INET);
    ASSERT_ON_ERROR(status);

    return SUCCESS;
}

static _i32 getWeather()
{
    _i32 retVal = -1;

    pal_Strcpy((char *)g_AppData.HostName, WEATHER_SERVER);

    retVal = getHostIP();
    if(retVal < 0)
    {
        CLI_Write((_u8 *)" Unable to reach Host\n\r\n\r");
        ASSERT_ON_ERROR(retVal);
    }

    g_AppData.SockID = createConnection();
    ASSERT_ON_ERROR(g_AppData.SockID);

    pal_Memset(g_AppData.CityName, 0x00, sizeof(g_AppData.CityName));
    pal_Memcpy(g_AppData.CityName, CITY_NAME, pal_Strlen(CITY_NAME));
    g_AppData.CityName[pal_Strlen(CITY_NAME)] = '\0';

    retVal = getData();
    ASSERT_ON_ERROR(retVal);


    retVal = sl_Close(g_AppData.SockID);
    ASSERT_ON_ERROR(retVal);

    return 0;
}

static _i32 configureSimpleLinkToDefaultState()
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

    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }


        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        if (ROLE_STA != retVal)
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
    }

    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    retVal = sl_WlanDisconnect();
    if(0 == retVal)
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }

    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal;
}

static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

static _i32 disconnectFromAP()
{
    _i32 retVal = -1;

    retVal = sl_WlanDisconnect();
    
    if(0 == retVal)
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" Get weather application - Version ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r******************************************************\n\r");
}

/******************************************************************************
 ******************** Required USER Defined Functions *************************
 *****************************************************************************/
 
void cc3100_wlan_event_handler( SlWlanEvent_t *p_wlan_event )
{
    if(p_wlan_event == NULL)
    {
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch(p_wlan_event->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &p_wlan_event->EventData.STAandP2PModeDisconnected;

            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write((_u8 *)" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write((_u8 *)" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write((_u8 *)" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void cc3100_net_app_event_handler( SlNetAppEvent_t *p_net_app_event )
{
    if(p_net_app_event == NULL)
    {
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch(p_net_app_event->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
        }
        break;

        default:
        {
            CLI_Write((_u8 *)" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void cc3100_http_server_callback( SlHttpServerEvent_t *p_http_event,
                                  SlHttpServerResponse_t *p_http_response )
{
    CLI_Write((_u8 *)" [HTTP EVENT] Unexpected event \n\r");
}

void cc3100_general_event_handler( SlDeviceEvent_t *p_device_event )
{
    CLI_Write((_u8 *)" [GENERAL EVENT] \n\r");
}

void cc3100_socket_event_handler( SlSockEvent_t *p_socket )
{
    if(p_socket == NULL)
    {
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch( p_socket->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            switch( p_socket->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    CLI_Write((_u8 *)" [SOCK EVENT] Close socket operation \
                                       failed to transmit all queued packets\n\r");
                break;
                default:
                    CLI_Write((_u8 *)" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write((_u8 *)" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}

void cc3100_interrupt_enable(void)
{
#ifdef STM32
    NVIC_IntEnable( IVT_INT_EXTI15_10 );
#endif
#ifdef TI
    NVIC_IntEnable( IVT_INT_GPIOF );
#endif
}

void cc3100_interrupt_disable()
{
#ifdef STM32
    NVIC_IntDisable( IVT_INT_EXTI15_10 );
#endif
#ifdef TI
    NVIC_IntDisable( IVT_INT_GPIOF );
#endif
}

/******************************************************************************
 ******************** System Specific Functions *******************************
 *****************************************************************************/

void system_init()
{
    DisableInterrupts();
#ifdef STM32
    GPIO_Digital_Output( &GPIOC_BASE, _GPIO_PINMASK_2 );
    GPIO_Digital_Output( &GPIOA_BASE, _GPIO_PINMASK_0 );
    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    GPIO_Digital_Input( &GPIOD_BASE, _GPIO_PINMASK_10 );

    SPI3_Init_Advanced( _SPI_FPCLK_DIV16,
                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                        _SPI_SECOND_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );

    RCC_APB2ENR.AFIOEN = 1;
    AFIO_EXTICR3 = ( 1 << EXTI100 ) | ( 1 << EXTI101 );
    EXTI_RTSR |= ( 1 << TR10 );
    EXTI_IMR |= ( 1 << MR10 );

    UART1_Init_Advanced( 115200,
                         _UART_8_BIT_DATA,
                         _UART_NOPARITY,
                         _UART_ONE_STOPBIT,
                         &_GPIO_MODULE_USART1_PA9_10 );
#endif
#ifdef TI
    GPIO_Digital_Output( &GPIO_PORTB_AHB, _GPIO_PINMASK_0 );
    GPIO_Digital_Output( &GPIO_PORTE_AHB, _GPIO_PINMASK_0 );
    GPIO_Digital_Output( &GPIO_PORTD_AHB, _GPIO_PINMASK_0 );
    GPIO_Digital_Input( &GPIO_PORTF_AHB, _GPIO_PINMASK_4 );

    SPI0_Init_Advanced( 15000000, _SPI_MASTER, _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                       _SPI_SECOND_CLK_EDGE_TRANSITION,
                       &_GPIO_MODULE_SPI0_A245_AHB );

    GPIO_PORTF_AHB_AFSEL |= ( 1 << _GPIO_PINCODE_4 );
    GPIO_PORTF_AHB_IM    |= ( 1 << _GPIO_PINCODE_4 );
    GPIO_PORTF_AHB_IEV   |= ( 1 << _GPIO_PINCODE_4 );

    UART7_Init_Advanced( 115200, 115200, _UART_8_BIT_DATA, _UART_NOPARITY,
                        _UART_ONE_STOPBIT, _UART_HIGH_SPEED,
                        &_GPIO_MODULE_UART7_C45_AHB );

#endif
    Delay_ms( 100 );
    UART_Write_Text( "System Initialized\r\n" );
    EnableInterrupts();
}

void main( void )
{
    _i32 retVal = -1;
    system_init();
    display_init();

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    displayBanner();

    retVal = configureSimpleLinkToDefaultState();

    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            CLI_Write(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");
    TFT_Rectangle( 10, 200, 310, 219 );
    TFT_Write_Text( "Device is configured in default state", 30, 200 );

    retVal = sl_Start(0, 0, 0);

    if ((retVal < 0) || (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        TFT_Rectangle( 10, 200, 310, 219 );
        TFT_Write_Text( "Failed to start the device", 30, 200 );
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");
    TFT_Rectangle( 10, 200, 310, 219 );
    TFT_Write_Text( "Device started as STATION", 30, 200 );

    retVal = establishConnectionWithAP();

    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        TFT_Rectangle( 10, 200, 310, 219 );
        TFT_Write_Text( "Failed to establish connection", 30, 200 );
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");
    TFT_Rectangle( 10, 200, 310, 219 );
    TFT_Write_Text( "Connection established", 30, 200 );
    
    retVal = getWeather();

    if(retVal < 0)
    {
        CLI_Write(" Failed to get weather information \n\r");
        TFT_Rectangle( 10, 200, 310, 219 );
        TFT_Write_Text( "Failed to get weather information", 30, 200 );
        LOOP_FOREVER();
    }

    retVal = disconnectFromAP();

    if(retVal < 0)
    {
        CLI_Write(" Failed to disconnect from AP \n\r");
        TFT_Rectangle( 10, 200, 310, 219 );
        TFT_Write_Text( "Failed to disconnect from AP", 30, 200 );
        LOOP_FOREVER();
    }
}

#ifdef STM32
void irq_handler() iv IVT_INT_EXTI15_10 ics ICS_AUTO
{
    EXTI_PR |= ( 1 << PR10 );
    ( *isr_event )( 0 );
}
#endif
#ifdef TI
void irq_handler() iv IVT_INT_GPIOF ics ICS_AUTO
{
    if( GPIO_PORTF_AHB_RIS & ( 1 << _GPIO_PINCODE_4 ))
    {
        GPIO_PORTF_AHB_ICR |= ( 1 << _GPIO_PINCODE_4 );
        ( *isr_event )( 0 );
    }
}
#endif