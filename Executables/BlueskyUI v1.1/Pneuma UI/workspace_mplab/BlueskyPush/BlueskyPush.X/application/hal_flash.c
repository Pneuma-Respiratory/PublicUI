#include "hal_flash.h"
#include "../mcc_generated_files/memory.h"

#include "app_task.h"
#define FLASH_START_ADDR        0x3000;

static uint16_t cur_flash_address;

// Sets cur_flash_address to the end of the last recorded shot
// Returns the shot number of the last recorded shot
uint16_t flash_init(void){
    uint16_t last_shot_num = 0;
    uint16_t data;
	cur_flash_address = FLASH_START_ADDR;
	data = 0;
	while(1)
	{
		data = FLASH_ReadWord(cur_flash_address);
        if (data == 0x3FFF) break;
        last_shot_num = data;
		cur_flash_address += sizeof(info_shot_t);
	}
    return last_shot_num;
}

// Returns true or false depending on if the current flash address
// is past the maximum amount of shots.
uint8_t is_mem_full(){
    if (cur_flash_address >= END_FLASH - sizeof(info_shot_t)) return 1;
    else return 0;
}

// Flashes a shot structure to memory
void write_shot(info_shot_t *shot) 
{
    uint8_t *ptr = (uint8_t *) shot;
    uint16_t buf[ERASE_FLASH_BLOCKSIZE];
    for (uint16_t i = 0; i < sizeof(info_shot_t); i++)
    {
        FLASH_WriteWord(cur_flash_address, buf, (uint16_t) *ptr);
        cur_flash_address++;
        ptr++; 
    }
}

// Flashes the pressure buffer to memory
void write_pressure(uint16_t press_buff[]){
    uint16_t *ptr = (uint16_t *) &press_buff;
    for (uint8_t i = 0; i< sizeof(press_buff)/2; i++){
        FLASH_WriteWord(cur_flash_address, ptr, *ptr);
        ptr++;
    }
}