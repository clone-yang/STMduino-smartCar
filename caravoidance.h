/*
 * @文件: \caravoidance.h
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 15:47:07
 * @说明: 
 */
#ifndef __CAR_AVOIDANCE_H_
#define __CAR_AVOIDANCE_H_

#include <Arduino.h>

#include "pinsdefine.h"

#include <Servo.h>

#define ServoDefaultPos	80

class CarAvoidance
{
private:
    Servo UltraServo;
public:
    CarAvoidance();

    //获取超声波测距数据
    int getDistance();

    //红外避障数据
    void getIRAvoidData(bool *isLeftAvoid, bool *isRightAvoid);

    //控制超声波的舵机运转
    void servoRun(int from, int to, int time);

    ~CarAvoidance();
};

#endif
