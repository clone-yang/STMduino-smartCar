/*
 * @文件: \carsensor.h
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 14:31:17
 * @说明: 
 */
#ifndef __CAR_SENSOR_H_
#define __CAR_SENSOR_H_

#include <Arduino.h>

#include <SimpleDHT.h>

#include "pinsdefine.h"

#include <PID_v1.h>

#include "motor.h"

#define Pi    0
#define Pid   1

class CarSensor
{
private:
    SimpleDHT11 *sensorDHT11;

    unsigned long time1 = 0, old_time = 0; // 时间标记

    unsigned long timechange = 0;

	int sampletime = 100;

	bool controlType = Pi;

	double Kp, Ki, Kd;
		
	double outputHigh, outputLow;
		
	int outputStep = 30;
	
	double inputHighAll, inputHighNum, inputLowAll, inputLowNum;
		
	double inputHistory[20];
		
	uint8_t caculateFlag = 1;
	
	uint8_t atemp, btemp;

    static int leftCounter, rightCounter;

    static void RightCount_CallBack();

    static void LeftCount_CallBack();

public:
    CarSensor();
    ~CarSensor();

    //温湿度
    void getTempHumi(byte *temp, byte *humi);

    //火焰传感器数据
    int getFlameData();

    //是否打开风扇
    void fanOpen(bool open);

    //测速
    bool getMotorSpeed(double *left, double *right);
		
	double getMotorSpeed(byte SM_Pin);
		
	void setSampletime(int sampleTime);
	
	void setStep(uint16_t step);
		
	void setPIDType(byte mode);
		
	bool getAutoPid(LZSmartCarMotor Motor, PID pid, double *input, double *output, double *setpoint, bool noteFlag, bool isleft = true);

};

#endif
