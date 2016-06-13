/*******************************************************************************
* Title                 :   CC3100 Hardware Implementation Layer
* Filename              :   cc3100_hal.c
* Author                :   RBL
* Origin Date           :   06/02/2016
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  06/02/16         .1              RBL     Module Created.
*
*******************************************************************************/
/**
 * @file cc3100_hal.c
 * @brief This is the hardware porting layer for the TI CC3100 SDK
 */
/******************************************************************************
* Includes
*******************************************************************************/
#if defined( __GNUC__ )
#include <stdio.h>
#endif
#include "cc3100_hal.h"
#include "simplelink.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define SPI_MODE      1
#define UART_MODE     2
#define CC3100_ERROR -1
#define CC3100_OK     0

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32 )
static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
static unsigned int ( *uart_data_ready_p )( void );
static unsigned int ( *uart_read_p )( void );
static void ( *uart_write_p )( unsigned int _data );

#elif defined( TI )
static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
static unsigned int ( *uart_data_ready_p )( void );
static unsigned int ( *uart_read_p )( void );
static void ( *uart_write_p )( unsigned int _data );

#endif
#elif defined( __MIKROC_PRO_FOR_AVR__ )
static unsigned char ( *spi_read_p )( unsigned char data_in );
static void ( *spi_write_p )( unsigned char data_out );
static char ( *uart_data_ready_p )( void );
static char ( *uart_read_p )( void );
static void ( *uart_write_p )( char _data );

#elif defined( __MIKROC_PRO_FOR_PIC__ )
static unsigned char( *spi_read_p )( unsigned char _data );
static void ( *spi_write_p )( unsigned char data_out );
static unsigned char ( *uart_data_ready_p )( void );
static unsigned char ( *uart_read_p )( void );
static void ( *uart_write_p )( unsigned char _data );

#elif defined( __MIKROC_PRO_FOR_PIC32__ )
static unsigned long( *spi_read_p )( unsigned long buffer );
static void ( *spi_write_p )( unsigned long data_out );
static unsigned int ( *uart_data_ready_p )( void );
static unsigned int ( *uart_read_p )( void );
static void ( *uart_write_p )( unsigned int _data );

#elif defined( __MIKROC_PRO_FOR_DSPIC__ )
static unsigned int( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
static char ( *uart_data_ready_p )( void );
static unsigned int ( *uart_read_p )( void );
static void ( *uart_write_p )( unsigned int _data );

#elif defined( __MIKROC_PRO_FOR_8051__ )


#elif defined( __MIKROC_PRO_FOR_FT90x__ )
static unsigned char( *spi_read_p )( unsigned char dummy );
static void( *spi_write_p )( unsigned char dataOut );
static unsigned char ( *uart_data_ready_p )( void );
static unsigned char ( *uart_read_p )( void );
static void ( *uart_write_p )( unsigned char _data );

#elif defined ( __GNUC__ )

#define spi_write_p 1
#define spi_read_p 1
#define uart_data_ready_p 1
#define uart_read_p 1
#define uart_write_p 1

#endif

#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
    defined( __MIKROC_PRO_FOR_AVR__ )   || \
    defined( __MIKROC_PRO_FOR_PIC__ )   || \
    defined( __MIKROC_PRO_FOR_PIC32__ ) || \
    defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
    defined( __MIKROC_PRO_FOR_8051__ )  || \
    defined( __MIKROC_PRO_FOR_FT90x__ )
    #define MIKROC
extern sfr sbit CC3100_HIB_PIN;
extern sfr sbit CC3100_CS_PIN;
extern sfr sbit CC3100_RST_PIN;
#endif

extern P_EVENT_HANDLER isr_event;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void cs_low( void );
static void cs_high( void );
static void rst_high( void );
static void rst_low( void );
static void hibernate_enable( void );
static void hibernate_disable( void );
static void delay_us( uint16_t ns );

/******************************************************************************
* Function Definitions
*******************************************************************************/
static void delay_us( uint16_t ns )
{
   while( ns-- )
       Delay_1us();
}

static void cs_low()
{
#if defined( __GNUC__ )
    printf( "CS Low\n" );
#else
    CC3100_CS_PIN = 0;
#endif
    delay_us(1);
    //cc3100_hal_delay( 1 );
}

static void cs_high()
{
    delay_us(1);
    //cc3100_hal_delay( 1 );
#if defined( __GNUC__ )
    printf( "CS High\n" );
#else
    CC3100_CS_PIN = 1;
#endif
}

static void rst_high()
{
#if defined( __GNUC__ )
    printf( "RST High\n" );
#else
    CC3100_RST_PIN = 1;
#endif
    cc3100_hal_delay( 25 );
}

static void rst_low()
{
#if defined( __GNUC__ )
    printf( "RST Low\n" );
#else
    CC3100_RST_PIN = 0;
#endif
    cc3100_hal_delay( 3 );
}

static void hibernate_enable()
{
#if defined( __GNUC__ )
    printf( "Hibernating\n" );
#else
    CC3100_HIB_PIN = 0;
#endif
    cc3100_hal_delay( 10 );
}

static void hibernate_disable()
{
#if defined( __GNUC__ )
    printf( "Wake up\n" );
#else
    CC3100_HIB_PIN = 1;
    cc3100_hal_delay( 50 );
#endif
}

/****************************************************************************
 ******************* Public Implementations *********************************
 ***************************************************************************/
Fd_t cc3100_hal_init( char *interface_name, unsigned long flags )
{
#if defined( __GNUC__ )
    printf( "Interface name: %s, Flags %d\n", interface_name, flags );
#endif
    if( interface_name != 0 )
    {
        if( strcmp( interface_name, "CC3100_UART" ) == 0 )
        {
#if defined( MIKROC )
            uart_data_ready_p = UART_Data_Ready;
            uart_read_p  = UART_Read;
            uart_write_p = UART_Write;
#endif
            hibernate_disable();
            cc3100_hal_reset();
            cc3100_hal_delay( 1400 );
            return ( uart_data_ready_p == NULL || uart_read_p == NULL ||
                     uart_write_p == NULL )
                    ? CC3100_ERROR
                    : UART_MODE;
        } else {
#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
    defined( __MIKROC_PRO_FOR_AVR__ )   || \
    defined( __MIKROC_PRO_FOR_PIC__ )   || \
    defined( __MIKROC_PRO_FOR_PIC32__ ) || \
    defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
    defined( __MIKROC_PRO_FOR_8051__ )
            spi_read_p = SPI_Rd_Ptr;
            spi_write_p = SPI_Wr_Ptr;
#elif defined( __MIKROC_PRO_FOR_FT90x__ )
            spi_read_p = SPIM_Rd_Ptr;
            spi_write_p = SPIM_Wr_Ptr;
#endif
            cs_high();
            hibernate_disable();
            cc3100_hal_reset();
            cc3100_hal_delay( 1400 );
            return ( spi_read_p == NULL || spi_write_p == NULL )
                    ? CC3100_ERROR
                    : SPI_MODE;
        }
    } else {
#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
    defined( __MIKROC_PRO_FOR_AVR__ )   || \
    defined( __MIKROC_PRO_FOR_PIC__ )   || \
    defined( __MIKROC_PRO_FOR_PIC32__ ) || \
    defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
    defined( __MIKROC_PRO_FOR_8051__ )
        spi_read_p = SPI_Rd_Ptr;
        spi_write_p = SPI_Wr_Ptr;
#elif defined( __MIKROC_PRO_FOR_FT90x__ )
        spi_read_p = SPIM_Rd_Ptr;
        spi_write_p = SPIM_Wr_Ptr;
#endif
        cc3100_interrupt_disable();
        cs_high();
        hibernate_disable();
        cc3100_hal_reset();
        cc3100_hal_delay( 1400 );
        return ( spi_read_p == NULL || spi_write_p == NULL )
                ? CC3100_ERROR
                : SPI_MODE;
    }

    return CC3100_ERROR;
}

int cc3100_hal_uninit( Fd_t fd )
{
    switch( fd )
    {
    case SPI_MODE:
        return CC3100_OK;
        break;
    case UART_MODE:
        return CC3100_OK;
        break;
    default:
        return CC3100_ERROR;
    }
}

void cc3100_hal_reset()
{
   rst_low();
   rst_high();
}


int cc3100_hal_read( Fd_t fd, uint8_t *p_buffer, int len )
{    
    uint8_t *ptr = p_buffer;
    int ret_len = len;

    switch( fd )
    {
    case SPI_MODE:
        cs_low();
#if defined( __GNUC__ )
        printf( "File Des: %d\nBuffer address: %p\nLength: %d\n" );
#else
        while( len-- )
            *( ptr++ ) = spi_read_p( 0x03 );
        cs_high();
#endif
        return ret_len;
        break;
    case UART_MODE:
        return CC3100_OK;
        break;
    default:
        return ret_len;
    }
}

int cc3100_hal_write( Fd_t fd, uint8_t *p_buffer, int len)
{
    int ret_len = len;
    
    switch( fd )
    {
    case SPI_MODE:
#if defined( __GNUC__ )
        printf( "File Des: %d\nWriting: %d\nLength: %d\n" );
#else
        cs_low();
        while( len-- )
            spi_write_p( *( p_buffer++ ) );
        cs_high();
#endif
        return ret_len;
        break;
    case UART_MODE:
        return CC3100_OK;
        break;
    default:
        return ret_len;
    }
}


int cc3100_hal_register_int_handler( P_EVENT_HANDLER isr_handler, void* p_value )
{
    isr_event = isr_handler;
    cc3100_interrupt_enable();

    return ( isr_event == 0 ) ? -1 : CC3100_OK;
}

void cc3100_hal_disable()
{
    hibernate_enable();
}

void cc3100_hal_enable()
{
    hibernate_enable();
    cc3100_hal_delay( 10 ); // Minimum time to wake from hibernation
    hibernate_disable();
    cc3100_hal_delay( 50 ); // Hardware wakeup time plus firmware init time
}

void cc3100_hal_delay(unsigned long interval)
{
    #if defined( __GNUC__ )
    printf( "Delay for %d ms\n", interval );
    #endif
    while( interval-- )
    {
        #if defined( __GNUC__ )

        #else
            Delay_1ms();
        #endif
    }
}