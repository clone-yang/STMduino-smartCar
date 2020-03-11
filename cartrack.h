/*
 * @文件: \cartrack.h
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 15:45:27
 * @说明: 
 */
#ifndef __CAR_TRACK_H_
#define __CAR_TRACK_H_

#include <Arduino.h>
#include "pinsdefine.h"

#define WHITE 1
#define BLACK 0

//#define ninetyLeft    -9
//#define ninetyRight    9
//#define	biggestLeft   -10
//#define	biggerLeft    -7
//#define	bigLeft       -5
//#define	smallLeft     -3
//#define	smallerLeft   -2
//#define	straight       0
//#define	smallerRight   2
//#define	smallRight     3
//#define	bigRight       5
//#define	biggerRight    7
//#define	biggestRight   10
                            
#define ninetyLeft    -9
#define ninetyRight    9
#define	biggestLeft   -10
#define	biggerLeft    -7
#define	bigLeft       -5
#define	smallLeft     -3
#define	smallerLeft   -2
#define	straight       0
#define	smallerRight   2
#define	smallRight     3
#define	bigRight       5
#define	biggerRight    7
#define	biggestRight   10

class CarTracking
{
private:
    int _numLines;
    uint8_t pinBuff[5]={TracePin1, TracePin2, TracePin3, TracePin4, TracePin5};
    byte Ground = WHITE;
    byte Line = BLACK;
	float decide;
public:

    CarTracking(int numLines = 5);

    //把获取到的循迹数据存放到buff
    void getLineData(byte *buff);

    void setGround(byte groundColor);

    void setLine(byte lineColor);

    float getDecide(byte *buff);

    ~CarTracking();
};

#endif
