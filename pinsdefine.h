/*
 * @文件: \pinsdefine.h
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 14:06:46
 * @说明: 
 */
#ifndef __PINS_DEFINE_H_
#define __PINS_DEFINE_H_

///////////////// 引脚说明

#include <Arduino.h>

//蓝牙模块
#define BTSerial 			Serial2

//电机模块
#define MotorIN1			2
#define MotorIN2			3
#define MotorIN3			4
#define MotorIN4			5

//#define MotorIN1			8
//#define MotorIN2			9
//#define MotorIN3			10
//#define MotorIN4			11

#define MotorIN5			8
#define MotorIN6			9
#define MotorIN7			10
#define MotorIN8			11

#define BATADC0				6

//RGB led
#define WS2812				7

//循迹
#define TracePin1			18
#define TracePin2			17
#define TracePin3			12
#define TracePin4			13
#define TracePin5			14

//舵机
#define ServoPWM1			15
#define ServoPWM2			16
#define ServoPWM3			21
#define ServoPWM4			24

//风扇
#define FAN_PIN				22

//火焰传感器				
#define FLAME_PIN			23

//温度
#define DHT11_PIN			27

//迷你传感器
#define MiniSensorWire		Wire

//测速模块
#define SM1Pin				29
#define SM2Pin				28

//超声波
#define UltraTrigPin		31
#define UltraEchoPin		30

#endif
