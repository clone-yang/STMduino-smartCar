/*
* @文件: \carsensor.cpp
* @作者:零知实验室
* -^^- 零知开源，让电子制作变得更简单！ -^^-
* @时间: 2019-12-05 14:31:04
* @说明:
*/
#include "carsensor.h"

int CarSensor::leftCounter = 0;
int CarSensor::rightCounter = 0;

CarSensor::CarSensor()
{
	//DHT
	sensorDHT11 = new SimpleDHT11(DHT11_PIN);
	
	//flame
	pinMode(FLAME_PIN, INPUT_PULLUP);
	
	//fan
	pinMode(FAN_PIN, OUTPUT);
	digitalWrite(FAN_PIN, 0); //关闭风扇
	
	//hall
	pinMode(SM1Pin, INPUT);
	pinMode(SM2Pin, INPUT);
}

void CarSensor::getTempHumi(byte *temp, byte *humi)
{
	byte temperature = 0;
	
	byte humidity = 0;
	
	int err = SimpleDHTErrSuccess;
	
	if ((err = sensorDHT11->read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess)
	{
		
		Serial.print("Read DHT11 failed, err=");
		Serial.println(err);
		
		*temp = 0;
		
		*humi = 0;
	}else{
		Serial.print("Sample OK: ");
		
		Serial.print((int)temperature);
		Serial.print(" *C, ");
		
		Serial.print((int)humidity);
		Serial.println(" H");
		
		*temp = temperature;
		
		*humi = humidity;
	}
	
}

int CarSensor::getFlameData()
{
	int hy_sig = digitalRead(FLAME_PIN);
	
	Serial.print("火焰传感器:");
	
	Serial.println(hy_sig);
	
	return hy_sig;
}

void CarSensor::fanOpen(bool open)
{
	if (open)
	{
		
		digitalWrite(FAN_PIN, 1);
	}
	else
	{
		
		digitalWrite(FAN_PIN, 0);
	}
}

bool CarSensor::getMotorSpeed(double *left, double *right)
{
	
	time1 = millis();
	timechange = time1 - old_time;
	if(timechange >= sampletime)
	{
		detachInterrupt(SM1Pin);
		detachInterrupt(SM2Pin);
		// 计算车轮转速，单位为rpm/min
		*left = (float)(leftCounter * 60000 / (20 * timechange));
		*right = (float)(rightCounter * 60000 / (20 * timechange));
		// 把脉冲计数值清零，以便计算下一秒的脉冲计数
		leftCounter = 0;
		rightCounter = 0;
		old_time = millis();
		attachInterrupt(SM1Pin, LeftCount_CallBack, FALLING);
		attachInterrupt(SM2Pin, RightCount_CallBack, FALLING);
		return true;
	}
	else
	return false;
	
}

double CarSensor::getMotorSpeed(byte SM_Pin)
{
	int Counter;
	long time2;
	long old_time2 = millis();
	time1 = millis();
	long timechange2 = time2 - old_time2;
	if(timechange >= sampletime)
	{
		detachInterrupt(SM_Pin);
		// 计算车轮转速，单位为rpm/min
		double speed = (float)(Counter * 60000 / (20 * timechange2));
		// 把脉冲计数值清零，以便计算下一秒的脉冲计数
		Counter = 0;
		attachInterrupt(SM_Pin, LeftCount_CallBack, FALLING);
		return speed;
	}
}

void CarSensor::RightCount_CallBack()
{
	rightCounter++;
}

void CarSensor::LeftCount_CallBack()
{
	leftCounter++;
}

void CarSensor::setPIDType(byte mode)
{
	controlType = mode;
}

void CarSensor::setStep(uint16_t step)
{
	outputStep = step;
}

void CarSensor::setSampletime(int sampleTime)
{
	sampletime = sampleTime;
}

bool CarSensor::getAutoPid(LZSmartCarMotor Motor, PID pid, double *input, double *output, double *setpoint, bool noteFlag, bool isleft)
{
	// 使用手动输入的不是很准确的PID参数使小车达到稳定状态
	if (millis() < 5000)
	{
		pid.Compute();
		if (isleft){
			Motor.speedGo(1, *output);
			Motor.speedGo(3, *output);
		}
		else{
			Motor.speedGo(2, *output);
			Motor.speedGo(4, *output);
		}
		
	}
	
	// 强行震荡，产生波形
	else if (millis() < (20 * sampletime + 6000))
	{
		outputLow = *output - outputStep;
		outputHigh = *output + outputStep;
		// 如果当前速度小于设定速度，输出高的PWM，产生波峰
		if (*input < *setpoint)
		{
			*output = outputHigh;
			if(outputLow < 0)
				outputLow = 0;
		}
		// 产生波谷
		else if (*input > *setpoint)
		{
			*output = outputLow;
			if(outputHigh > 255)
				outputLow = 255;
		}
		if (isleft){
			Motor.speedGo(1, *output);
			Motor.speedGo(3, *output);
		}
		else{
			Motor.speedGo(2, *output);
			Motor.speedGo(4, *output);
		}
		//记录波形
		if (noteFlag == 1)
		{
			for (int i = 1; i < 20; i++)
			{
				inputHistory[20 - i] = inputHistory[20 - i - 1];
			}
			inputHistory[0] = *input;
			noteFlag = 0;
		}
	}
	else
	{
		while(caculateFlag)
		{
			// 开始分析波形
			for (int i = 1; i < 20; i++)
			{
				// 波峰
				if (inputHistory[i] > inputHistory[i - 1] && inputHistory[i] > inputHistory[i + 1])
				{
					inputHighAll += inputHistory[i]; // 波峰数据累加
					inputHighNum++;					 // 波峰个数加1
					if (inputHighNum == 1)
					atemp = i; // 记录第一个波峰的位置
					btemp = i;	 // 记录最后一个波峰的位置
				}
				// 波谷
				else if (inputHistory[i] < inputHistory[i - 1] && inputHistory[i] < inputHistory[i + 1])
				{
					inputLowAll += inputHistory[i];
					inputLowNum++;
				}
			}
			float autoTuneA = (inputHighAll / inputHighNum) - (inputLowAll / inputLowNum); // 波峰与波谷之差的平均值
			float autoTunePu = (btemp - atemp) * (sampletime / 1000) / (inputHighNum - 1); // 两个波峰之间的时间间隔
			
			// 根据公式计算PID
			double Ku = 4 * outputStep / (autoTuneA * 3.14159);
			Kp = controlType == 1 ? 0.6 * Ku : 0.4 * Ku;
			Ki = controlType == 1 ? 1.2 * Ku / autoTunePu : 0.48 * Ku / autoTunePu;
			Kd = controlType == 1 ? 0.075 * Ku * autoTunePu : 0;
			
			caculateFlag = 0; // 关闭电机自整定
			
			// 串口打印出自整定计算得到的PID参数
			Serial.println("******************************");
			Serial.println("PID 自整定过程完成！");
			Serial.println("自整定的PID参数为： ");
			Serial.print("Kp：");
			Serial.println(Kp);
			Serial.print("Ki：");
			Serial.println(Ki);
			Serial.print("Kd：");
			Serial.println(Kd);
			Serial.println(" ");
			Serial.print("震荡过程中的峰峰值 A：");
			Serial.print(autoTuneA);
			Serial.println("rpm/min");
			Serial.print("两个波峰之间的时间间隔Pu：");
			Serial.print(autoTunePu);
			Serial.println("秒");
			Serial.println("******************************");
			Serial.println(" ");
			
			pid.SetTunings(Kp, Ki, Kd); // 导入新的PID参数
			//			delay(2000);
			return true;
		}
	}
}

CarSensor::~CarSensor()
{
}