

#ifndef LCD_H_
#define LCD_H_
#include "STM32fxx_gpio.h"
#include "STM32fxx.h"
#include <stdint.h>
//------------GPIO PINS USED FOR LCD------------
#define LCD_GPIO_PORT	GPIOC
#define LCD_GPIO_RS		GPIO_PIN_NO_0
#define LCD_GPIO_RW		GPIO_PIN_NO_2
#define LCD_GPIO_EN		GPIO_PIN_NO_1
#define LCD_GPIO_D4		GPIO_PIN_NO_4
#define LCD_GPIO_D5		GPIO_PIN_NO_5
#define LCD_GPIO_D6		GPIO_PIN_NO_6
#define LCD_GPIO_D7		GPIO_PIN_NO_7

//------------LCD_COMMANDS--------------
#define LCD_CMD_4DL_2N_5x8F				0x28
#define LCD_CMD_ENTRY_MODE				0x06	//0x06-->ADDRESS INC AFTER WRITE TO LCD ;DISP NO SHIFT
#define LCD_CMD_D_C_B_OFF				0x08
#define LCD_CMD_D_C_ON					0x0E
#define LCD_CMD_D_C_B_ON				0x0F
#define LCD_CMD_SHIFT_LEFT				0x18
#define LCD_CMD_SHIFT_RIGHT				0x1C
#define LCD_CMD_DIS_CLEAR				0x01
#define LCD_CMD_DIS_RETURN_HOME			0x02

#define ROLLING_DISPLAY_DELAY			200		//change the time delay (ms) if display needs to roll slower/faster


//-----------LCD FUNCTIONS ----------------
void LCD_INIT(void);
void LCD_SEND_CMD(uint8_t);
void LCD_DISPLAY_CLEAR(void);
void SET_CUR_POS(uint8_t ,uint8_t);
void LCD_SEND_CHAR(uint8_t);
void LCD_DISP_CUR_ON(void);
void LCD_DISP_CUR_BLINK_OFF(void);
void LCD_ENTRY_MODE_SET(void);
void LCD_SEND_STRING(char *);
void DISP_RETURN_HOME(void);
void LCD_DISP_CUR_BLINK_ON(void);
void LCD_ROLLING_DISP_RIGHT(uint32_t);
void LCD_ROLLING_DISP_LEFT(uint32_t);

#endif /* LCD_H_ */
