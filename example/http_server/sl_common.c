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