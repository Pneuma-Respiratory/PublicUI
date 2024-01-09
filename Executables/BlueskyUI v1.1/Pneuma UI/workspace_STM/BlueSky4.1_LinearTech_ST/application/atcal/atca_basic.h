/*
 * File:   mac.c
 * Author: A19582
 *
 * Created on August 14, 2018, 4:31 PM
 */
#ifndef __ATCA_BASICS_H
#define __ATCA_BASICS_H

#include "stdint.h"

#include "swi_pio_pic18.h"


/** \name Opcodes for Crypto Authentication device commands
   @{ */
#define ATCA_CHECKMAC     ((uint8_t)0x28)  //!< CheckMac command op-code
#define ATCA_DERIVE_KEY   ((uint8_t)0x1C)  //!< DeriveKey command op-code
#define ATCA_INFO         ((uint8_t)0x30)  //!< Info command op-code
#define ATCA_GENDIG       ((uint8_t)0x15)  //!< GenDig command op-code
#define ATCA_GENKEY       ((uint8_t)0x40)  //!< GenKey command op-code
#define ATCA_HMAC         ((uint8_t)0x11)  //!< HMAC command op-code
#define ATCA_LOCK         ((uint8_t)0x17)  //!< Lock command op-code
#define ATCA_MAC          ((uint8_t)0x08)  //!< MAC command op-code
#define ATCA_NONCE        ((uint8_t)0x16)  //!< Nonce command op-code
#define ATCA_PAUSE        ((uint8_t)0x01)  //!< Pause command op-code
#define ATCA_PRIVWRITE    ((uint8_t)0x46)  //!< PrivWrite command op-code
#define ATCA_RANDOM       ((uint8_t)0x1B)  //!< Random command op-code
#define ATCA_READ         ((uint8_t)0x02)  //!< Read command op-code
#define ATCA_SIGN         ((uint8_t)0x41)  //!< Sign command op-code
#define ATCA_UPDATE_EXTRA ((uint8_t)0x20)  //!< UpdateExtra command op-code
#define ATCA_VERIFY       ((uint8_t)0x45)  //!< GenKey command op-code
#define ATCA_WRITE        ((uint8_t)0x12)  //!< Write command op-code
#define ATCA_ECDH         ((uint8_t)0x43)  //!< ECDH command op-code
#define ATCA_COUNTER      ((uint8_t)0x24)  //!< Counter command op-code
#define ATCA_SHA          ((uint8_t)0x47)  //!< SHA command op-code
#define ATCA_AES          ((uint8_t)0x51)  //!< AES command op-code
#define ATCA_KDF          ((uint8_t)0x56)  //!< KDF command op-code
#define ATCA_SECUREBOOT   ((uint8_t)0x80)  //!< Secure Boot command op-code
#define ATCA_SELFTEST     ((uint8_t)0x77)  //!< Self test command op-code

/** \name Definitions of Data and Packet Sizes
   @{ */
#define ATCA_BLOCK_SIZE             (32)                                //!< size of a block
#define ATCA_WORD_SIZE              (4)                                 //!< size of a word
#define ATCA_PUB_KEY_PAD            (4)                                 //!< size of the public key pad
#define ATCA_SERIAL_NUM_SIZE        (9)                                 //!< number of bytes in the device serial number
#define ATCA_RSP_SIZE_VAL           ((uint8_t)7)                        //!< size of response packet containing four bytes of data
#define ATCA_KEY_COUNT              (16)                                //!< number of keys
#define ATCA_ECC_CONFIG_SIZE        (128)                               //!< size of configuration zone
#define ATCA_SHA_CONFIG_SIZE        (88)                                //!< size of configuration zone
#define ATCA_OTP_SIZE               (64)                                //!< size of OTP zone
#define ATCA_DATA_SIZE              (ATCA_KEY_COUNT * ATCA_KEY_SIZE)    //!< size of data zone

#define ATCA_COUNT_SIZE             ((uint8_t)1)                        //!< Number of bytes in the command packet Count
#define ATCA_CRC_SIZE               ((uint8_t)2)                        //!< Number of bytes in the command packet CRC
#define ATCA_PACKET_OVERHEAD        (ATCA_COUNT_SIZE + ATCA_CRC_SIZE)   //!< Number of bytes in the command packet

/** \name Definitions for Indexes Common to All Commands
   @{ */
#define ATCA_COUNT_IDX              (0)     //!< command packet index for count
#define ATCA_OPCODE_IDX             (1)     //!< command packet index for op-code
#define ATCA_PARAM1_IDX             (2)     //!< command packet index for first parameter
#define ATCA_PARAM2_IDX             (3)     //!< command packet index for second parameter
#define ATCA_DATA_IDX               (5)     //!< command packet index for data load
#define ATCA_RSP_DATA_IDX           (1)     //!< buffer index of data in response
/** @} */

/** \name Definitions for Zone and Address Parameters
   @{ */
#define ATCA_ZONE_CONFIG                ((uint8_t)0x00)         //!< Configuration zone
#define ATCA_ZONE_OTP                   ((uint8_t)0x01)         //!< OTP (One Time Programming) zone
#define ATCA_ZONE_DATA                  ((uint8_t)0x02)         //!< Data zone
#define ATCA_ZONE_MASK                  ((uint8_t)0x03)         //!< Zone mask
#define ATCA_ZONE_ENCRYPTED             ((uint8_t)0x40)         //!< Zone bit 6 set: Write is encrypted with an unlocked data zone.
#define ATCA_ZONE_READWRITE_32          ((uint8_t)0x80)         //!< Zone bit 7 set: Access 32 bytes, otherwise 4 bytes.
#define ATCA_ADDRESS_MASK_CONFIG        (0x001F)                //!< Address bits 5 to 7 are 0 for Configuration zone.
#define ATCA_ADDRESS_MASK_OTP           (0x000F)                //!< Address bits 4 to 7 are 0 for OTP zone.
#define ATCA_ADDRESS_MASK               (0x007F)                //!< Address bit 7 to 15 are always 0.
/** @} */

#pragma pack(1) 
typedef struct
{
    //uint8_t cmd;           //ss
    //--- start of packet i/o frame----
    uint8_t txsize;
    uint8_t opcode;
    uint8_t param1;     // often same as mode
    uint16_t param2;    // often same as address
    uint8_t data[130];  // includes 2-byte CRC.  data size is determined by largest possible data section of any
    // command + crc (see: x08 verify data1 + data2 + data3 + data4)
    // this is an explicit design trade-off (space) resulting in simplicity in use
    // and implementation
    //--- end of packet i/o frame

    // used for receive
    uint8_t execTime;       // execution time of command by opcode
    uint16_t rxsize;        // expected response size, response is held in data member

    // structure should be packed since it will be transmitted over the wire
    // this method varies by compiler.  As new compilers are supported, add their structure packing method here

} ATCAPacket;
#pragma pack()

ATCA_STATUS atcab_write_config_zone(const uint8_t *data);

ATCA_STATUS atcab_read_serial_number( uint8_t* serial_number );
ATCA_STATUS atcab_read_slot_data( uint8_t slot_seq, uint8_t *data );
ATCA_STATUS atcab_write_slot_data( uint8_t slot_seq, uint8_t *data );

ATCA_STATUS atcab_read_all( uint8_t *read_buf, uint8_t num );

#endif
