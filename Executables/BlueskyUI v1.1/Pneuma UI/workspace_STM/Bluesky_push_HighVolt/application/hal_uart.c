/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2015-09-21 $
  Revision:       $Revision: V0.1 $

  Description:    This file contains the interface to the HAL UART Service.


**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include <string.h>

#include "usart.h"
#include "hal_board_cfg.h"

#include "hal_uart.h"


/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define BAUD_RATE                 9600

/**************************************************************************************************
 *                                            MACROS
 **************************************************************************************************/


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        LOCAL VARIABLES
 **************************************************************************************************/
static uartDMACfg_t dmaCfg;
uint8_t receivedData = 0x00;
uint8_t transmitData = 0x00;

/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void HalUartInit( void );

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      Hal_UART_RxBufLen()
 *
 * @brief   Calculate Rx Buffer length - the number of bytes in the buffer.
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 **************************************************************************************************/
uint16_t sweepIdx = 0;

uint16_t Hal_UART_RxBufLen(void)
{
    return 0;
}

/**************************************************************************************************
 * @fn      Hal_UART_RxReset()
 *
 * @brief   Reset UART_RX_DMA and dmaCfg information
 *
 * @param   none
 *
 * @return  none
 **************************************************************************************************/
static void Hal_UART_RxReset( void )
{    
    dmaCfg.rxHead = 0;
    dmaCfg.rxTail = 0;
}

static void Hal_UART_TxReset( void )
{    
    dmaCfg.txHead = 0;
    dmaCfg.txTail = 0;
}

void Hal_UART_Reset( void )
{
    Hal_UART_RxReset();
    Hal_UART_TxReset();
}

#if 0
/******************************************************************************
 * @fn      HalUARTWrite
 *
 * @brief   Write a buffer to the UART, enforcing an all or none policy if the requested length
 *          exceeds the space available.
 *
 * @param   buf - pointer to the buffer that will be written, not freed
 *          len - length of
 *
 * @return  length of the buffer that was sent
 *****************************************************************************/
uint8_t HalUARTWrite( uint8_t *buf, uint8_t len )
{
    uint8_t cnt;

    if ( dmaCfg.txHead == dmaCfg.txTail ){
        dmaCfg.txHead = 0;
        dmaCfg.txTail = 0;
    }
    for ( cnt = 0; cnt < len; cnt++ ){
        *(dmaCfg.txBuf + dmaCfg.txTail) = *(buf + cnt);
        if ( ++dmaCfg.txTail >= HAL_UART_DMA_TX_MAX ){
            break;
        }
    }
    
    USCI_A_UART_transmitData( USCI_A0_BASE, *(dmaCfg.txBuf + dmaCfg.txHead) );
    dmaCfg.txHead++;
    
    return cnt;
}

static uint8_t HalUART_TxBuffer( void )
{
    if ( dmaCfg.txHead < dmaCfg.txTail ){
        USCI_A_UART_transmitData( USCI_A0_BASE, *(dmaCfg.txBuf + dmaCfg.txHead) );
        dmaCfg.txHead++;
        return TRUE;
    }
    else{
        dmaCfg.txHead = 0;
        dmaCfg.txTail = 0;
        return FALSE;
    }
}

/*****************************************************************************
 * @fn      HalUARTRead
 *
 * @brief   Read a buffer from the UART, 
 *          If read 3 bytes(SOF + LEN) once a time don't clear it, otherwise clear
 *          after read, and update the pointer of the buffer
 *          
 *
 * @param   
 *          buf  - valid data buffer at least 'len' bytes in size
 *          len  - max length number of bytes to copy to 'buf'
 *          flag - TRUE or FALSE, clear or not after read
 *
 * @return  length of buffer that was read
 *****************************************************************************/
uint16_t HalUARTRead(uint8_t *buf, uint16_t len, uint8_t flag)
{
    uint8_t data;
    uint16_t cnt, i, *pBuf;
    
    cnt = dmaCfg.rxTail - dmaCfg.rxHead;
    pBuf = dmaCfg.rxBuf + dmaCfg.rxHead;
    
    if ( cnt < len ){
        len = cnt;
    }
    for ( i = 0; i < cnt; i++ ){
        //*buf++ = (uint8_t)(*pBuf++);
        data = (uint8_t)(*pBuf);
        *pBuf++ = 0;
        *buf++ = data;
        dmaCfg.rxHead++;
    }
    
    if ( dmaCfg.rxHead == dmaCfg.rxTail ){
        Hal_UART_RxReset();
    }
    
    return (cnt-1);
}

uint16_t HalUART_RxBufLen( void )
{
    return ( dmaCfg.rxTail - dmaCfg.rxHead );
}

static void HalUART_RxBuffer( void )
{
    int8 rcvData;
    
    rcvData = USCI_A_UART_receiveData(USCI_A0_BASE);
#if 0
    if ( dmaCfg.rxHead == dmaCfg.rxTail ){
        Hal_UART_RxReset();
    }
#endif
    if ( dmaCfg.rxTail < HAL_UART_DMA_RX_MAX ){
        *(dmaCfg.rxBuf + dmaCfg.rxTail) = rcvData;  
        dmaCfg.rxTail++;
    }
}

/**************************************************************************************************
 * @fn      HalUartInit
 *
 * @brief   USART configuration 
 *
 * @param   none
 *          9600, 8, n, 1
 *
 * @return  None
 **************************************************************************************************/
void HalUartInit( void )
{
    //P3.3,4 = USCI_A0 TXD/RXD
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P3, GPIO_PIN3 + GPIO_PIN4 );
    
    //Baudrate = 9600, clock freq = 5MHz, 9600bps-->521, 2
    //UCBRx = 109, UCBRFx = 0, UCBRSx = 2, UCOS16 = 0
    // CLK = 20MHz, 115200bps-->173, 5 // 9600bps-->2083, 2
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 2083;
    param.firstModReg = 0;
    param.secondModReg = 2;
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if (STATUS_FAIL == USCI_A_UART_init(USCI_A0_BASE, &param)){
        return;
    }
    
    //Enable UART module for operation
    USCI_A_UART_enable(USCI_A0_BASE);

    //Enable Receive Interrupt
    USCI_A_UART_clearInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
    
    USCI_A_UART_clearInterrupt(USCI_A0_BASE, USCI_A_UART_TRANSMIT_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_TRANSMIT_INTERRUPT);
    // __enable_interrupt();
    
    (void)memset(dmaCfg.rxBuf, 0xFF00, HAL_UART_DMA_RX_MAX * sizeof(uint16_t));
    
    Hal_UART_TxReset();
    Hal_UART_RxReset();
}



//******************************************************************************
//This is the USCI_A0 interrupt vector service routine.
//******************************************************************************
__interrupt void USCI_A0_ISR (void)
{
    switch (__even_in_range(UCA0IV,4)){
        case 2:       //Vector 2 - RXIFG
             //receivedData = USCI_A_UART_receiveData(USCI_A0_BASE);
             HalUART_RxBuffer();
             if(!(receivedData == transmitData)){                   // Check value
             //    while(1);
             }
             break;
        case 4:       //Vector 4 - TXIFG
             HalUART_TxBuffer();
             break;
        default: break;
    }
}

#endif

/**************************************************************************************************
**************************************************************************************************/
