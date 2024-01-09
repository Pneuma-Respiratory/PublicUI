
#ifndef ATCA_MAIN_H
#define ATCA_MAIN_H

#include "atca_status.h"
#include "hal_board_cfg.h"



typedef struct
{
    // Slot0
    uint32_t bt_id;                 // Cartridge ID
    uint16_t bt_type;               // Drug type
    uint16_t per_sdp_time;          // Maximum ejection time allowed for SDP3X trigger each time, unit is 2ms
    uint32_t total_eje_time;        // Total ejection times, unit is 2ms
    uint16_t total_eje_dose;        // Total ejection dose
    uint16_t per_key_time;          // Ejection time after key is triggered, unit is 2ms
    uint8_t reserve_dat0[14];
    uint8_t slot0_crc[2];
    // Slot1
    UTCTime fill_time;
    UTCTime expire_time;
    uint8_t reserve_dat1[22];
    uint8_t slot1_crc[2];
    // Slot2
    uint16_t used_eje_dose;
    uint32_t used_eje_time;
    uint8_t reserve_dat2[24];
    uint8_t slot2_crc[2];
}cartidge_attr_t;

// Slot2
typedef struct
{
    uint16_t used_eje_dose;
    uint32_t used_eje_time;
    uint8_t reserve_dat2[24];
    uint8_t slot2_crc[2];
}cartidge_slot2_t;

uint8_t sha_detect( void );
ATCA_STATUS sha_update_slot2( uint16_t used_dose, uint32_t used_time );

#endif
