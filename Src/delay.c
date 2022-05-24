
#include "delay.h"
#include <stdint.h>
typedef struct
		{
			uint32_t CSR;
			uint32_t RVR;
			uint32_t CVR;
			uint32_t CALIB;
		}SYST_t;

SYST_t *SYSTick = (SYST_t*)0xE000E010;

//Initialise systick timer
void systickinit()
{
	SYSTick->CSR|=(1<<2);  //take processor as clock source for timer
}

//delay function for 1us delay
void delay_us(uint32_t blinktime)
{

	systickinit();
	for(int i=0;i<blinktime;i++)
	{
	SYSTick->RVR = 16;  //reload value for 1us
	SYSTick->CSR|=(1<<0);//enable the timer countdown
	while(!(SYSTick->CSR & (1<<16) )); //count down flag until set
	//SYSTick->CSR &=~(1<<16);  //reset the countdown flag
	SYSTick->CSR&=~(1<<0); //disable the timer

	}
}

void delay_ms(uint32_t blinktime)
{
		blinktime*=1000;
			delay_us(blinktime);
}

void delay(uint32_t count)			//provides for loopdelyin ms
{
		int i,j;
		count=count*1000;
		for(i=0;i<count;i++)
		for(j=0;j<16;j++);		//gives 1us delay
}

void delay_MS(uint32_t cnt)
{
	for(uint32_t i=0;i<(cnt*1000);i++);
}

void delay_US(uint32_t cnt)
{
	for(uint32_t i=0;i<(cnt*1);i++);
}
