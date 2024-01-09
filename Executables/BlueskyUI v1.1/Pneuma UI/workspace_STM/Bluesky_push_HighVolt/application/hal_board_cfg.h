/************************************************************************************
 * Hardware Change Note
 * 20201206
 * Function    new(old)
 * I2C_SDA     PC14(PB11),     I2C_SCL     PC15(PB10)
 * IRQ         PA0(PA0),
 * AD_CURRENT  PA5/ADC1_IN10(PB0/ADC1_IN15),
 * AD_BAT      PB0/ADC1_IN15(PB1/ADC1_IN16)
 * PWM1        PA6(PA6),
 * VBUS_IN     PB1(PC14),      CHRG        PB2(PA1)
 * VBAT_CN     PB10(PA2),
 * SW_TEST     PB11(PC15)
 * SPI_CSN     PB12(PB12),     SPI_CLK     PB13(PB13),     SPI_MISO     PB14(PB14),     SPI_MOSI     PB15(PB15)
 * RST_N       PA8(PA8)
 * SHNDA       PA9(PA9)
 * UART_TXD    PB6(PB6),       UART_RXD    PB7(PB7)
 * LED1R       PA10(PA10),     LED1B       PA11(PA11),     LED1G       PA12(PA12)
 * LED2        PA15(PB5),      LED3        PB3(PB8),       LED4        PB8(PB9)
 * LED5        PA2(PA3),       LED6        PA4(PA4),       LED7        PA7(PA5)
 * LED8        PA3(PB2),
 *
 *
 * **********************************************************************************/

#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H


/***********************************************************************************
* INCLUDES
*/
#include <string.h>

#include "stm32l4xx_ll_utils.h"

#include "hal_adc.h"
#include "hal_led.h"
#include "hal_sdp.h"
#include "hal_flash.h"
#include "hal_key.h"
#include "hal_pwm.h"

#include "main.h"

#include "hal_board_cfg.h"


/*********************************************************************
 * TYPEDEFS
 */
typedef uint32_t UTCTime;

/***********************************************************************************
 * CONSTANTS
 */
#define WDT_EN                   0
#define USE_EXTAL2               1

#define WTD_ENABLE               1
#define POWER_KEY_LOW_LEVEL      false

// ADC
#define ADC_CUT_PORT             GPIOA
#define ADC_CUT_PIN              GPIO_PIN_7
#define ADC_CUT_CH               ADC_CHANNEL_12                 // ADC1_IN12

#define ADC_BAT_PORT             GPIOA
#define ADC_BAT_PIN              GPIO_PIN_6
#define ADC_BAT_CH               ADC_CHANNEL_11                 // ADC1_IN11

#define ADC_NTC1_PORT            GPIOA
#define ADC_NTC1_PIN             GPIO_PIN_1
#define ADC_NTC1_CH              ADC_CHANNEL_6

#define ADC_NTC2_PORT            GPIOA
#define ADC_NTC2_PIN             GPIO_PIN_4
#define ADC_NTC2_CH              ADC_CHANNEL_9

#define HEAT_PORT               GPIOB
#define HEAT_PIN                GPIO_PIN_3

// PWM
#define PWM_A_PORT               GPIOB
#define PWM_A_PIN                GPIO_PIN_4       // TIM3_CH1,
//#define PWM_B_PORT               GPIOA
//#define PWM_B_PIN                GPIO_PIN_7       // TIM3_CH2, TIM1_CH1N
// Turn ON Button
#define SW_ON_PORT               GPIOB
#define SW_ON_PIN                GPIO_PIN_12
// CHRG
#define CHARGE_PORT              GPIOB
#define CHARGE_PIN               GPIO_PIN_11
// 5VIN_TST
#define VBUS_PORT                GPIOB
#define VBUS_PIN                 GPIO_PIN_10
// VBAT_CN
#define VBAT_CN_PORT             GPIOA
#define VBAT_CN_PIN              GPIO_PIN_5

#define LED1R_PORT               GPIOA
#define LED1R_PIN                GPIO_PIN_10
#define LED1G_PORT               GPIOA
#define LED1G_PIN                GPIO_PIN_12
#define LED1B_PORT               GPIOA
#define LED1B_PIN                GPIO_PIN_11
#define LED2_PORT                GPIOA
#define LED2_PIN                 GPIO_PIN_15
#define LED3_PORT                GPIOB
#define LED3_PIN                 GPIO_PIN_9
#define LED4_PORT                GPIOB
#define LED4_PIN                 GPIO_PIN_8
#define LED5_PORT                GPIOA
#define LED5_PIN                 GPIO_PIN_2
#define LED6_PORT                GPIOB
#define LED6_PIN                 GPIO_PIN_0
#define LED7_PORT                GPIOB
#define LED7_PIN                 GPIO_PIN_2
#define LED8_PORT                GPIOA
#define LED8_PIN                 GPIO_PIN_3

//#define LED8A15_PORT             GPIOA
//#define LED8A15_PIN              GPIO_PIN_15
//#define LED7B3_PORT              GPIOB
//#define LED7B3_PIN               GPIO_PIN_3

//#define LT_SHNDA_PORT             GPIOA
//#define LT_SHNDA_PIN              GPIO_PIN_9
//#define LT_SHNDB_PORT             GPIOC
//#define LT_SHNDB_PIN              GPIO_PIN_13

#if 0
#define SDP_I2C_SDA_PORT         GPIO_PORT_P6
#define SDP_I2C_SDA_PIN          GPIO_PIN5
#define SDP_I2C_SCL_PORT         GPIO_PORT_P5
#define SDP_I2C_SCL_PIN          GPIO_PIN0

#define SHA204A_I2C_SDA_PORT     GPIO_PORT_P6
#define SHA204A_I2C_SDA_PIN      GPIO_PIN5
#define SHA204A_I2C_SCL_PORT     GPIO_PORT_P5
#define SHA204A_I2C_SCL_PIN      GPIO_PIN0
#else
// I2C, I2C2
#define I2C_PORT                 GPIOC
#define I2C_SCL_PIN              GPIO_PIN_15
#define I2C_SDA_PIN              GPIO_PIN_14
#endif
// UART, UART1
//#define UART_PORT                GPIOB
//#define UART_TXD_PIN             GPIO_PIN_6
//#define UART_RXD_PIN             GPIO_PIN_7

#define VOICE_BUSY_PORT          GPIOB
#define VOICE_BUSY_PIN           GPIO_PIN_5

#define VOICE_SDA_PORT           GPIOB
#define VOICE_SDA_PIN            GPIO_PIN_6

//-----------------------------------------------------------------------------

// Failure informations

// I2C
#define I2C_ERROR_DEVICE         0x10

#define ADC_FAIL                 0x51
#define ADC_BUSY                 0x01
#define ADC_IDLE                 0x00
     
#define VOICE_BUSY               0x60
     
// Key
#define SW_ON                    0x01
#define SW_OFF                   0x02


// Battery
#define BATT_THR_1               2451    // 3.95V  (4095 <--> 3.3V)
#define BATT_THR_2               2326    // 3.75V
#define BATT_THR_3               2233    // 3.6V
#define BATT_THR_4               2047    // 3.3V

#define BATT_THR_3V9             2419    // 要根据实际电压调整
#define BATT_THR_3V7             2295
#define BATT_THR_3V5             2171

#define BAT_VOL_FULL             0x05
#define BAT_VOL_NORMAL           0x04
#define BAT_VOL_LOW              0x03
#define BAT_VOL_BAD              0x02
#define BAT_VOL_OFF              0x01

#define BAT_CHARGE_OFF           0x00
#define BAT_CHARGE_ON            0x01
#define BAT_CHARGE_FULL          0x02
     
#define SDP_STOP_THR             10
#define SDP_RESTAR_THR           30


     
/***********************************************************************************
 * MACROS
 */
#define GPIO_setOutputHighOnPin( port, pin )   HAL_GPIO_WritePin( port, pin, GPIO_PIN_SET )
#define GPIO_setOutputLowOnPin( port, pin )    HAL_GPIO_WritePin( port, pin, GPIO_PIN_RESET )


#define VBAT_CN_ON()         HAL_GPIO_WritePin( VBAT_CN_PORT, VBAT_CN_PIN, GPIO_PIN_SET )
#define VBAT_CN_OFF()        HAL_GPIO_WritePin( VBAT_CN_PORT, VBAT_CN_PIN, GPIO_PIN_RESET )

#define LT_SHNDA_ON()      //  PWM_Start( TIM_CHANNEL_1 ) //  HAL_GPIO_WritePin( LT_SHNDA_PORT, LT_SHNDA_PIN, GPIO_PIN_SET )
#define LT_SHNDA_OFF()       PWM_Stop( TIM_CHANNEL_1 )    //  HAL_GPIO_WritePin( LT_SHNDA_PORT, LT_SHNDA_PIN, GPIO_PIN_RESET )

#define HEAT_ON()            HAL_GPIO_WritePin( HEAT_PORT, HEAT_PIN, GPIO_PIN_SET )
//#define HEAT_ON()           HAL_GPIO_WritePin( HEAT_PORT, HEAT_PIN, GPIO_PIN_RESET )
#define HEAT_OFF()           HAL_GPIO_WritePin( HEAT_PORT, HEAT_PIN, GPIO_PIN_RESET )

#define VOICE_BUSY_STA()     HAL_GPIO_ReadPin( VOICE_BUSY_PORT, VOICE_BUSY_PIN )

//#define LT_SHNDB_ON()        HAL_GPIO_WritePin( LT_SHNDB_PORT, LT_SHNDB_PIN, GPIO_PIN_SET )
//#define LT_SHNDB_OFF()       HAL_GPIO_WritePin( LT_SHNDB_PORT, LT_SHNDB_PIN, GPIO_PIN_RESET )

//#define BT_A_ON()            { LT_SHNDA_ON();  LT_SHNDB_OFF(); }
//#define BT_B_ON()            { LT_SHNDB_ON();  LT_SHNDA_OFF(); }
//#define BT_AB_OFF()          { LT_SHNDA_OFF(); LT_SHNDB_OFF(); }



/*******************************************************************************
*  Global Function
*/

/***********************************************************************************
* Global Variable
*/
extern uint16_t KEY_LP_TIME;     // unit 20ms ;¸´Î»Ê±¼ä3S
extern uint16_t SDP_TRIGER_THR;    //4 Pa ´¥·¢ÏÂÎçÃÅÏÞÖµ 6000/60=100Pa 100/60=1.33pa
extern uint16_t duty;

/*******************************************************************************
*  Global Function
*/


#endif
