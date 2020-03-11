/*
 * @文件: \motor.h
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 13:54:02
 * @说明: 
 */
#ifndef __MOTOR_H_
#define __MOTOR_H_

#include <Arduino.h>

#include "pinsdefine.h"

//小车运行方向
enum carDirec{
	carForward = 1,
	carForwardSlow,
	carBack,
	carLeft,
	carRight,
	carForLeft,
	carForRight,
	carBacLeft,
	carBacRight,
	carRightFast,
	carRightSlow,
	carLeftFast,
	carLeftSlow,
	carStop
};

#define MotorSpeedTrace                 15
#define MotorSpeedRoSlow                75
#define MotorSpeedRoFast                115

#define MotorSpeedRotate        20
#define MotorSpeedNormal        75 //正常前进速度
#define MotorSpeedFast          140 //
#define MotorSpeedSlow          60

#define Button            0
#define Gravity           1
#define JoyStick          2
#define Rocker            3

class LZSmartCarMotor
{
private:
    byte _wheelNum = 2;

	int pos1, pos2, pos3;

	byte _mode = Button;

	int cmd[2];

	int x, y, xMap, yMap;

	int speed;

	int speedLeft, speedRight;
public:
	
    //两轮 or 四轮
    LZSmartCarMotor(bool is2WD = true);

    //运行motor，电机的编号、速度、运行的方向是否反向
    void speedGo(byte number, int speed, bool reverse = false);

	void motorRun(int leftPwm, int rightPwm);

    void goWithDirec(carDirec cmd);

    //停止
    void stop();

	bool getCommand(int *buff);

	void setControlMode(byte mode);

	void gravityControl();

	void btnControl();
	
	void joyStickControl();
	
	void rockerControl();
	
	void bleControl();

    ~LZSmartCarMotor();
};

#endif
