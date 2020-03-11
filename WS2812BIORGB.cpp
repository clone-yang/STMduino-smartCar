/**
*    文件: WS2812BIORGB.cpp      by 零知实验室(www.lingzhilab.com)
*    -^^- 零知开源，让电子制作变得更简单！ -^^-
*    时间: 2019/12/03 16:34
*    说明: 
**/

#include "WS2812BIORGB.h"

#define __nop() asm volatile("nop")

void ws281X_delay_xx(unsigned int delay_num)
{
	volatile unsigned int timeouts = delay_num;
	while(timeouts--);
}

/********************************************************/
//
/********************************************************/
static void RGB_LED_Write0(void)
{
	RGB_LED_HIGH;
	__nop();__nop();__nop();__nop();
	__nop();__nop();
	RGB_LED_LOW;
	ws281X_delay_xx(2);
}

/********************************************************/
//
/********************************************************/

static void RGB_LED_Write1(void)
{
	RGB_LED_HIGH;
	ws281X_delay_xx(2);
	RGB_LED_LOW;
	__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
}

static void RGB_LED_Reset(void)
{
	RGB_LED_LOW;
	delayMicroseconds(80);
}

static void RGB_LED_Write_Byte(uint8_t byte)
{
	uint8_t i;

	for(i=0;i<8;i++)
		{
			if(byte&0x80)
			{
					RGB_LED_Write1();
			}
			else
			{
					RGB_LED_Write0();
			}
		byte <<= 1;
	}

}

static void RGB_LED_Write_24Bits(uint32_t color){
	RGB_LED_Write_Byte(color>>8);
	RGB_LED_Write_Byte(color>>16);
	RGB_LED_Write_Byte(color);
}

void WS2812BRGB_init()
{
	pinMode(7,OUTPUT);
	digitalWrite(7,HIGH);

	RGB_LED_Reset();
}

void WS2812BRGB_setColor(uint8_t ledNum, uint32_t *colorArray)
{
	
	for(int i=0; i<ledNum; i++){
		RGB_LED_Write_24Bits(colorArray[i]);
	}
}



void WS2812BRGB_setAll(uint8_t ledNum, uint32_t color)

{

	for(int i=0; i<ledNum; i++){

		RGB_LED_Write_24Bits(color);

	}

}

void WS2812BRGB_setOne(uint8_t ledNums, uint8_t led, uint32_t color)
{
	
	for(int i=0; i<ledNums; i++){
		
		if(i == led){
			RGB_LED_Write_24Bits(color);
		}else{
			RGB_LED_Write_24Bits(0x00);
		}
	}
}

void WS2812BRGB_breath(int freq, uint32_t color)
{
	for(int i=0; i<0xff; i++){
		WS2812BRGB_setAll(LED_NUMS,i);
		delay(freq);
	}
	for(int i=0xff; i>0; i--){
		WS2812BRGB_setAll(LED_NUMS,i);
		delay(freq);
	}
}

void WS2812BRGB_flow(int freq, int timeout, uint32_t color, bool reverse)
{
	
	if(reverse){
		for(int i=LED_NUMS-1; i>=0; i--){
			WS2812BRGB_setOne(LED_NUMS, i,color);
			delay(freq);
		}
		// delay(timeout);
	}
	else{
		for(int i=0; i<LED_NUMS; i++){
			WS2812BRGB_setOne(LED_NUMS, i,color);
			delay(freq);
		}
		// delay(timeout);
	}
}
