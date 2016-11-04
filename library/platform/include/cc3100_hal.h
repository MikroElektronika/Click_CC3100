/****************************************************************************
* Title                 :   ADC Application
* Filename              :   cc3100_hal.h
* Author                :   JWB
* Origin Date           :   06/07/2012
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  08/17/13    XXXXXXXXXXX         JWB      Interface Created.
*
*****************************************************************************/
/** @file file_here.h
 *  @brief What is does
 *
 *  @date 25 Aug 2015
 *  @author Richard Lowe
 *  @copyright GNU Public License
 *
 *  @version .1 - Initial testing and verification
 *
 *  @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    GPS Click
 *   SW:              ARM 4.5.2
 *
 */

#ifndef CC3100_HAL_H
#define CC3100_HAL_H

/******************************************************************************
* Includes
*******************************************************************************/

#include <stdint.h>
#include "datatypes.h"
#include "simplelink.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/

typedef void ( *P_EVENT_HANDLER )( void *p_pvalue );
typedef unsigned int Fd_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

/**
 * @brief HAL initialization
 *
 * @param[in] interface_name
 * @param[in] flags
 * @return
 */
Fd_t cc3100_hal_init
(
                char *interface_name,
                unsigned long flags
);

/**
 * @brief HAL uninitialization
 *
 * @param[in] fd
 *
 * @return
 */
int cc3100_hal_uninit
(
                Fd_t fd
);

/**
 * @brief cc3100_hal_read
 * @param fd
 * @param p_buffer
 * @param len
 * @return
 */
int cc3100_hal_read
(
                Fd_t fd,
                uint8_t *p_buffer,
                int len
);

/**
 * @brief cc3100_hal_write
 * @param p_buffer
 * @param len
 * @return
 */
int cc3100_hal_write
(
                Fd_t fd,
                uint8_t *p_buffer,
                int len
);

/**
 *  @brief Register an Interrupt Handler For The Host IRQ
 *
 *  @note If there is already registered interrupt handler, the
 *  function will overwrite the old handler with the new one
 *
 *  @param[in] isr_handler - pointer to interrupt handler function
 *
 *  @param[in] p_value  - pointer to a memory structure that is
 *                        passed to the interrupt handler.
 *
 *  @retval 0 / -1
 */
int cc3100_hal_register_int_handler
(
     P_EVENT_HANDLER isr_handler,
        void* p_value
);

/**
 * @brief Enables the CC3100
 *
 * @note Uses the hibernate pin
 */
void cc3100_hal_enable
(
                void
);

/**
 * @brief Resets the module
 *
 * Used at startup sequence and when module is unresponsive.
 */
void cc3100_hal_reset
(
                void
);

/**
 * @brief Disables the CC3100
 *
 * @note Uses the hibernate pin
 */
void cc3100_hal_disable
(
                void
);

/**
 * @brief Enables the interrupt from the CC3100
 *
 * External function defined by user, enables the interrupt pin attached to the
 * units int pin.
 */
void cc3100_interrupt_enable
(
                void
);

/**
 * @brief Disables the interrupt from the CC3100
 *
 * Extern function defined by user, disables the interrupt pin attached to the
 * units int pin.
 */
void cc3100_interrupt_disable
(
                void
);

/**
 * @brief Delay in ms
 *
 * @param interval[in] - ms for delay
 */
void cc3100_hal_delay
(
                unsigned long interval
);

#endif
/*** End of File **************************************************************/