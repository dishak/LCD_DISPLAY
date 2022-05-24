/*
 * lcd.c
 *
 *  Created on: Apr 26, 2022
 *  by: Disha
 *  Functions:
 *   lcd_init()-->INITIALISATION STEPS for 4BIT MODE LCD
 *   lcd_config()--> CONFIGURE THE LCD PINS AS OUTPUT.
 *   write_4_bits(uint8_t)--> FUNCTION FOR SENDING 4BITS OF COMMAND AT A TIME
 *   LCD_SEND_CMD(uint8_t)
 *   LCD_DISP_CUR_ON()
 */

#include "lcd.h"
#include "delay.h"
#include <stdint.h>
#include <stdio.h>
GPIO_Handle_t LCD_Config;

//-----------LCD FUNCTIONS FOR INITIALISATION--------------
static void write_4_bits(uint8_t);		//-------------Function to send nibble for 4bit interface----------
static void LCD_ENABLE(void);			//-------------Function to Enable Instruction and Data write/Read----------
static void RESET_LCD_PINS();	        //-------------Function to RESET RS,RW & DB7-DB4 pins of LCD----------
static void LCD_PINS_CONFIG(void);		//-------------Function to CONFIGURE RS,RW & DB7-DB4 pins of LCD AS O/P----------
static void WAIT_UNTIL_LCD_BUSY(void);  //-------------Function to WAIT & CHECK THE DB7 OF LCD WHICH INDICATES BUSY STATUS OF LCD----------


void LCD_INIT()
{
	LCD_PINS_CONFIG();
	RESET_LCD_PINS(); 		//Reset all LCD pins
	//LCD INITIALISATION SEQUENCE FOR 4BIT AS GIVEN IN DATASHEET
	delay_ms(50);

	write_4_bits(0x3);
	delay_ms(10);

	write_4_bits(0x3);
	delay_us(100);

	write_4_bits(0x3);
	write_4_bits(0x2);

	//SEND FUNCTION SET CMD
	LCD_SEND_CMD(LCD_CMD_4DL_2N_5x8F);	//Cmd for selecting 4bit Data Length ,2Line & 5*8 pixel font

	//LCD Display cursor blink off
	LCD_DISP_CUR_BLINK_OFF();

	//display clear
	LCD_DISPLAY_CLEAR();

	//entry mode set
	LCD_ENTRY_MODE_SET();

	LCD_DISP_CUR_BLINK_ON();

}
static void LCD_PINS_CONFIG()
{
	LCD_Config.pGPIOx=LCD_GPIO_PORT;
	LCD_Config.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	LCD_Config.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	LCD_Config.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	for(int i=0;i<=7;i++)
	{
		LCD_Config.GPIO_PinConfig.GPIO_PinNumber=(uint8_t)i;
		GPIO_Init(&LCD_Config);
	}
}

//-----------Function to Send Instruction to LCD ---------------
void LCD_SEND_CMD(uint8_t cmd)
{

	// RS=0,RW=0 for Command Write Operation to LCD
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, RESET);
	write_4_bits(cmd>>4);		//send higher nibble
	write_4_bits(cmd&0x0F);		//send lower nibble
	WAIT_UNTIL_LCD_BUSY();		//BUSY FLAG CHECK , Wait until LCD is busy DB7 bit is used to check busy flag status
	//delay_ms(30);
}

//-------------Function to send nibble for 4bit interface----------
static void write_4_bits(uint8_t in)
{

  GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4, ((in>>0)& 0x1));
  GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5, ((in>>1)& 0x1));
  GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6, ((in>>2)& 0x1));
  GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7, ((in>>3)& 0x1));

  LCD_ENABLE();
}

//----------------DISPLAY and CURSON ON ----------------
void LCD_DISP_CUR_ON()
{
    //Display on cursor on
	LCD_SEND_CMD(LCD_CMD_D_C_ON);
}

//----------------DISPLAY CURSOR & BLINK ON ----------------
void LCD_DISP_CUR_BLINK_ON()
{
    //Display on cursor on
	LCD_SEND_CMD(LCD_CMD_D_C_B_ON);

}

//----------------DISPLAY CURSOR & BLINK OFF----------------
void LCD_DISP_CUR_BLINK_OFF()
{
	LCD_SEND_CMD(LCD_CMD_D_C_B_OFF);

}

void LCD_ENTRY_MODE_SET()
{

	LCD_SEND_CMD(LCD_CMD_ENTRY_MODE);

}

//-------------Function to make RS,RW & DB7-DB4 pins of LCD=0----------
static void RESET_LCD_PINS()
{
	for(uint8_t i=0;i<=7;i++)
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, i, RESET);

}


void LCD_SEND_CHAR(uint8_t data)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, RESET);
	write_4_bits(data>>4); 	   //higher nibble
	write_4_bits(data&0x0F);   //lower nibble
	delay_ms(50);

}
void LCD_SEND_STRING(char *message)
{
	do
	{
		LCD_SEND_CHAR((uint8_t)*message++);
	}while(*message!='\0');
}
void DISP_RETURN_HOME()
{
	LCD_SEND_CMD(LCD_CMD_DIS_RETURN_HOME);

}
void LCD_DISPLAY_CLEAR()
{
	LCD_SEND_CMD(LCD_CMD_DIS_CLEAR);
	delay_ms(2);
}
void LCD_ROLLING_DISP_LEFT(uint32_t freq)
{
	if(freq<255)						//If user entered value is less than 255 then,255 times rolling of diplay
		{
			while(freq<255)
			{
				for(int i=0;i<=39;i++)
				{
				LCD_SEND_CMD(LCD_CMD_SHIFT_LEFT);
				delay_ms(ROLLING_DISPLAY_DELAY);
				}
				freq--;
			}
		}
		else							//Else if freq>=255 forever rolling of display but,the functions after will never be executed
		{								//call freq>=255 only at end of all function execution.
			while(1)
			{
				for(int i=0;i<=39;i++)
					{
						LCD_SEND_CMD(LCD_CMD_SHIFT_RIGHT);
						delay_ms(ROLLING_DISPLAY_DELAY);
					}
			}
		}

}
void LCD_ROLLING_DISP_RIGHT(uint32_t freq)
{
	if(freq<255)						//If user entered value is less than 255 then,255 times rolling of diplay
	{
		while(freq<255)
		{
			for(int i=0;i<=39;i++)
			{
			LCD_SEND_CMD(LCD_CMD_SHIFT_RIGHT);
			delay_ms(ROLLING_DISPLAY_DELAY);
			}
			freq--;
		}
	}
	else							//Else if freq>=255 forever rolling of display but,the functions after will never be executed
	{								//call freq>=255 only at end of all function execution.
		while(1)
		{
			for(int i=0;i<=39;i++)
				{
					LCD_SEND_CMD(LCD_CMD_SHIFT_RIGHT);
					delay_ms(ROLLING_DISPLAY_DELAY);
				}
		}
	}
}
void WAIT_UNTIL_LCD_BUSY()
{
	/*
	 * 1.Make LCD_GPIO_PORT pin 7 as i/p pin
	 * 2.RW=1 (Read operation)				Hint:Check with making En high to low if doesnt work
	 * 3.wait until D7 bit is low
	 * 4. make RW=0(write operation)
	 */
	RESET_LCD_PINS();
	//Step1
	LCD_Config.pGPIOx=LCD_GPIO_PORT;
	LCD_Config.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	LCD_Config.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PD;
	LCD_Config.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D7;
	LCD_Config.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&LCD_Config);
	//Step2
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, SET);  //En=HIGH
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, RESET);//delay_us(1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, SET);//delay_us(1);
	//Step3
	while(GPIO_ReadFromInputPin(LCD_GPIO_PORT, LCD_GPIO_D7))//Wait until D7 bit is reset
	{
		LCD_ENABLE();

	}
	//Step 4
	LCD_Config.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	LCD_Config.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	LCD_Config.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D7;
	GPIO_Init(&LCD_Config);
	RESET_LCD_PINS();

}
static void LCD_ENABLE()
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, SET);
	delay_us(5);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, RESET);
	delay_us(5);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, SET);

}
//----FUNCTION TO SET CUR POSITION AT (ROW,COLUMN) POSITION OF LCD ROW=0,COL = 0-0X27 & ROW=0X40 COL=0X67---------
void SET_CUR_POS(uint8_t r,uint8_t c)
{
	uint8_t ADDcmd;
	if( (r==0 && c<=(uint8_t) 0x27) )
	{
		ADDcmd=(uint8_t)(0x80)+c;
	}
	else if( (r==1 && c<=(uint8_t) 0x27) )
	{
		ADDcmd=(uint8_t)(0x80)+c+(uint8_t)(0x40);
	}
	LCD_SEND_CMD(ADDcmd);
}


