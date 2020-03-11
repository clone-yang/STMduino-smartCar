/*
 * @文件: \WS2812BIORGB.h
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 13:51:25
 * @说明: 
 */

#ifndef _WS2812B_IO_RGB_H_
#define _WS2812B_IO_RGB_H_

#include <Arduino.h>

//#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
//#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
//#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
//#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
//#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
//#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
//#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
//#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
//#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
//#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
//#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
//#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
//#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
//#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
//#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
//#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */



//GPIOA->regs->GSRR=PIN_num
//GPIOA,GPIO_PIN_4

#define		RGB_LED_HIGH	GPIOA->regs->BSRR=0x10
#define 	RGB_LED_LOW		GPIOA->regs->BRR=0x10

#define LED_NUMS 8

void WS2812BRGB_init();
void WS2812BRGB_setColor(uint8_t ledNum, uint32_t *colorArray);

void WS2812BRGB_setAll(uint8_t ledNum, uint32_t color);
void WS2812BRGB_setOne(uint8_t ledNums, uint8_t led, uint32_t color);

void WS2812BRGB_flow(int freq, int timeout, uint32_t color, bool reverse);
void WS2812BRGB_breath(int freq, uint32_t color);

#endif

