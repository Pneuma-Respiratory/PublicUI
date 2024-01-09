
#ifndef _SWI_PIO_PIC18_H
#define _SWI_PIO_PIC18_H

/**
  Section: Included Files
*/
//#include <stdbool.h>
//#include <stdint.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>

#include "stdint.h"


//#include "mcc_generated_files/mcc.h"
#include "atca_status.h"
//#include "delay.h"

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

/**
  Section: Macro Declarations
*/
#define RX_DELAY        10  // Delay before responses come
#define TX_DELAY        65  // Delay before new flag is sent

#define SWI_WAKE_TOKEN   ((uint8_t)0x00)    //!< flag preceding a command
#define SWI_FLAG_CMD     ((uint8_t)0x77)    //!< flag preceding a command
#define SWI_FLAG_TX      ((uint8_t)0x88)    //!< flag requesting a response
#define SWI_FLAG_IDLE    ((uint8_t)0xBB)    //!< flag requesting to go into Idle mode
#define SWI_FLAG_SLEEP   ((uint8_t)0xCC)    //!< flag requesting to go into Sleep mode

        
#define SDA_HIGH()      LATC5 = 1
#define SDA_LOW()       LATC5 = 0

#define SDA_INPUT()     TRISC5 = 1
#define SDA_OUTPUT()    TRISC5 = 0
#define SDA_STATE()     RC5


#define ATCA_RSP_SIZE_MIN           ((uint8_t)4)                        //!< minimum number of bytes in response
#define ATCA_RSP_SIZE_4             ((uint8_t)7)                        //!< size of response packet containing 4 bytes data
#define ATCA_RSP_SIZE_72            ((uint8_t)75)                       //!< size of response packet containing 64 bytes data
#define ATCA_RSP_SIZE_64            ((uint8_t)67)                       //!< size of response packet containing 64 bytes data
#define ATCA_RSP_SIZE_32            ((uint8_t)35)                       //!< size of response packet containing 32 bytes data
#define ATCA_RSP_SIZE_MAX           ((uint8_t)75)                       //!< maximum size of response packet (GenKey and Verify command)


//#define atca_delay_us(n)            delay_us(n)
//#define atca_delay_ms(n)            delay_ms(n)

/**
  Section: Data Type Definitions
*/

/**
 Section: Global variables
 */

/**
 * \brief Send byte(s) via SWI.
 * \param[in] txdata    pointer to bytes to send
 * \param[in] txlength  number of bytes to send
 * \return ATCA_STATUS
 */
void hal_swi_send(uint8_t *txdata, int txlength);

/**
 * \brief Receive byte(s) via SWI.
 * \param[in] rxdata    pointer to where bytes will be received
 * \param[in] rxlength  pointer to expected number of receive bytes to
 *                      request
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_swi_receive(uint8_t *rxdata, uint16_t rxlength);

/**
 * \brief Send Wake flag via SWI.
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_swi_wake();

/**
 * \brief Send Idle flag via SWI.
 * \return ATCA_STATUS
 */
void hal_swi_idle();

uint8_t at_I2CWrite( uint8_t cmd, uint8_t *buf, uint8_t len );
uint8_t at_I2CRead( uint8_t *buf, uint8_t len );

#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif  // _EUSART1_H
/**
 End of File
*/
