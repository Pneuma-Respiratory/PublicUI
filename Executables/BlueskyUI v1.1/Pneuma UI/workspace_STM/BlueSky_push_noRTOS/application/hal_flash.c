/***********************************************************************************
  Filename:     hal_virtual_i2c.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "projdefs.h"

#include "string.h"
#include "hal_flash.h"

/***********************************************************************************
 * CONSTANTS
 */
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_200   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_201 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */


/***********************************************************************************
 * MACROS
 */


/***********************************************************************************
* GLOBAL VARIABLES
*/

/***********************************************************************************
* LOCAL VARIABLES
*/



/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* GLOBAL FUNCTIONS
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/


/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}

/***********************************************************************************
*  @fn         info_mem_erase
*  @param      
*  @return   
*/  
uint8_t info_mem_erase( void )
{
	uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
	uint32_t PAGEError = 0;

	static FLASH_EraseInitTypeDef EraseInitStruct;

    /* Erase the user Flash area
      (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    /* Get the 1st page to erase */
    FirstPage = GetPage(FLASH_USER_START_ADDR);
    /* Get the number of pages to erase from 1st page */
    NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
    /* Get the bank */
    BankNumber = GetBank(FLASH_USER_START_ADDR);
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = BankNumber;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = NbOfPages;

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
       you have to make sure that these data are rewritten before they are accessed during code
       execution. If this cannot be done safely, it is recommended to flush the caches by setting the
       DCRST and ICRST bits in the FLASH_CR register. */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
    	return pdFALSE;
    }

    return pdTRUE;
}

void info_mem_read( uint8_t *data, uint16_t count )
{
    uint32_t Address = 0;
    
    Address = FLASH_USER_START_ADDR;
    while (count > 0)
    {
        *data++ = *(__IO uint32_t *)Address++;
        count--;
    }
}

uint8_t info_mem_write( uint8_t *buf, uint16_t count )
{
    uint8_t rslt = pdTRUE, i;
    uint32_t Address = 0;
    uint64_t data = 0;
    
    if ( count > (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) ){
    	return pdFALSE;
    }

    HAL_FLASH_Unlock();
    if ( info_mem_erase() == pdTRUE ){
    	Address = FLASH_USER_START_ADDR;

    	for ( i = 0; i < count; i++ ){
    		memcpy( (uint8_t *)(&data), (uint8_t *)buf, 8 );
    		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data) == HAL_OK){
    			Address += 8;
    			buf += 8;
    		}
    		else{
    			/* Error occurred while writing data in Flash memory. User can add here some code to deal with this error */
    			rslt = pdFALSE;
    			break;
    		}
    	}
    }
    HAL_FLASH_Lock();

    return rslt;
}

uint8_t flash_mem_erase( void )
{
    uint16_t status = pdTRUE;
   
    return status;
}

void flash_mem_write( uint8_t *data, uint32_t *flash_addr, uint16_t count )
{

}

uint8_t read_info_mem( uint8_t *data, uint8_t length )
{
    info_mem_read( data, length >> 2 );
    
    return pdTRUE;
}

uint8_t write_record( uint8_t *data, uint8_t length )
{
    return pdTRUE;
}

uint8_t hal_snv_load( uint8_t item, uint8_t *data, uint16_t sequence )
{
    if ( sequence > 1000 ){
        return pdFALSE;
    }
    
    if ( item == SNV_USER_RECORD_ITEM ){
    }
    else if ( item == SNV_USER_INFO_ITEM ){
        info_mem_read( data, 16 );
    }
    
    return pdTRUE;
}

uint8_t hal_snv_store( uint8_t item,  uint8_t *data, uint16_t sequence )
{
    if ( sequence > 1000 ){
        return pdFALSE;
    }
    
    if ( item == SNV_USER_RECORD_ITEM ){
    }
    else if ( item == SNV_USER_INFO_ITEM ){
        info_mem_write( (uint8_t *)data, 8 );
    }
    
    return pdTRUE;
}

uint8_t memory_test( void )
{
    uint8_t data[64], buffer[64];
    uint8_t i;
    
    for ( i = 0; i < 64; i++ ){
        data[i] = i + 35;
        buffer[i] = 0;
    }

    info_mem_write( data, sizeof(data) >> 3 );
    info_mem_read( buffer, sizeof(buffer) >> 3 );
    for ( i = 0; i < sizeof(data); i++ ){
    	if ( data[i] != buffer[i] ){
    		return pdFALSE;
    	}
    }
    return pdTRUE;
}


