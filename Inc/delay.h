

#ifndef DELAY_H_
#define DELAY_H_
#include <stdint.h>

void delay_us(uint32_t );
void delay_ms(uint32_t );
void delay(uint32_t);
void delay_MS(uint32_t);
void delay_US(uint32_t);
void systickinit(void);
typedef struct
{
	uint32_t CSR;
	uint32_t RVR;
	uint32_t CVR;
	uint32_t CALIB;
}SYST;


#endif /* DELAY_H_ */
