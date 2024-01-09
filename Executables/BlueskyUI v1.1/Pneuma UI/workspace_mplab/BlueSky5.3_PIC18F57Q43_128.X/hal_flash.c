#include "hal_flash.h"
#include "mcc_generated_files/nvm/nvm.h"

#include "application/app_task.h"
#define FLASH_START_ADDR        0x10000;

static uint24_t cur_flash_address;

// Sets cur_flash_address to the end of the last recorded shot
// Returns the shot number of the last recorded shot
uint16_t flash_init(void){
    uint16_t last_shot_num = 0;
    uint16_t data;
	cur_flash_address = FLASH_START_ADDR;
	data = 0;
	while(1)
	{
		data = FLASH_Read(cur_flash_address);
        data |= FLASH_Read(cur_flash_address + 1) << 8;
        if (data == 0xFFFF) break;
        last_shot_num = data;
		cur_flash_address += sizeof(info_shot_t);
	}
    return last_shot_num;
}

// Returns true or false depending on if the current flash address
// is past the maximum amount of shots.
uint8_t is_mem_full(){
    if (cur_flash_address >= PROGMEM_SIZE - sizeof(info_shot_t)) return 1;
    else return 0;
}

// Flashes a shot structure to memory
void write_shot(info_shot_t *shot) 
{
    uint16_t *ptr = (uint16_t *) shot;
    for (uint16_t i = 0; i < sizeof(info_shot_t) / 2; i++)
    {
        NVM_UnlockKeySet(0xAA55);
        if (FLASH_Write(cur_flash_address, *ptr) == NVM_ERROR){
            while (1);
        }
        NVM_UnlockKeyClear();
        cur_flash_address += 2;
        ptr++; 
    }
}