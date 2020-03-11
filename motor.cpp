/*
 * @文件: \motor.cpp
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 13:53:55
 * @说明: 
 */
#include "motor.h"

LZSmartCarMotor::LZSmartCarMotor(bool is2WD)
{
    
        _wheelNum = 2;
        pinMode(MotorIN1, OUTPUT);
        pinMode(MotorIN2, OUTPUT);
        pinMode(MotorIN3, OUTPUT);
        pinMode(MotorIN4, OUTPUT);
    if (!is2WD)
    {
        _wheelNum = 4;
        pinMode(MotorIN5, OUTPUT);
        pinMode(MotorIN6, OUTPUT);
        pinMode(MotorIN7, OUTPUT);
        pinMode(MotorIN8, OUTPUT);
    }
}

void LZSmartCarMotor::speedGo(byte number, int speed, bool reverse)
{
    if (speed < 0)
    {
        speed = 0;
    }
    if (speed > 255)
    {
        speed = 255;
    }
    speed = 255 - speed;

    if(number > _wheelNum){
        // Serial.println("轮子个数是否正确？");
        return;
    }

    if (number == 1)
    {
        if (reverse)
        {
            analogWrite(MotorIN1, speed);
            analogWrite(MotorIN2, 255);
        }
        else
        {
			
            analogWrite(MotorIN1, 255);
            analogWrite(MotorIN2, speed);
        }
    }

    if (number == 2)
    {
        if (reverse)
        {
            analogWrite(MotorIN3, 255);
            analogWrite(MotorIN4, speed);
        }
        else
        {
            analogWrite(MotorIN3, speed);
            analogWrite(MotorIN4, 255);
        }
    }

    if (number == 3)
    {

        if (reverse)
        {
			analogWrite(MotorIN5, speed);

            analogWrite(MotorIN6, 255);
        }
        else
        {

			analogWrite(MotorIN5, 255);

            analogWrite(MotorIN6, speed);
        }
    }
    if (number == 4)
    {

        if (reverse)
        {

			analogWrite(MotorIN7, 255);

            analogWrite(MotorIN8, speed);
        }
        else
        {

            analogWrite(MotorIN7, speed);

            analogWrite(MotorIN8, 255);
        }
    }
}

void LZSmartCarMotor::motorRun(int leftPwm, int rightPwm)
{
    if(leftPwm > 0){
		speedGo(1, leftPwm);
		speedGo(3, leftPwm);
	}else{
		speedGo(1, abs(leftPwm), true);
		speedGo(3, abs(leftPwm), true);
	}
	if(rightPwm > 0){
		speedGo(2, rightPwm);
		speedGo(4, rightPwm);
	}else{
		speedGo(2, abs(rightPwm), true);
		speedGo(4, abs(rightPwm), true);
	}
}

void LZSmartCarMotor::stop()
{

    analogWrite(MotorIN1, 255);
    analogWrite(MotorIN2, 255);
    analogWrite(MotorIN3, 255);
    analogWrite(MotorIN4, 255);

    analogWrite(MotorIN5, 255);
    analogWrite(MotorIN6, 255);
    analogWrite(MotorIN7, 255);
    analogWrite(MotorIN8, 255);
}

void LZSmartCarMotor::goWithDirec(carDirec cmd)
{
    switch(cmd){
		case carForward:
		speedGo(1,MotorSpeedNormal);
		speedGo(2,MotorSpeedNormal);
		speedGo(3,MotorSpeedNormal);
		speedGo(4,MotorSpeedNormal);
		break;
		case carForwardSlow:
		speedGo(1,MotorSpeedSlow);
		speedGo(2,MotorSpeedSlow);
		speedGo(3,MotorSpeedSlow);
		speedGo(4,MotorSpeedSlow);
		break;
		case carBack:
		speedGo(1,MotorSpeedNormal,true);
		speedGo(2,MotorSpeedNormal,true);
		speedGo(3,MotorSpeedNormal,true);
		speedGo(4,MotorSpeedNormal,true);
		break;
		case carLeft:
		speedGo(1,MotorSpeedFast);
		speedGo(2,MotorSpeedRotate);
		speedGo(3,MotorSpeedFast);
		speedGo(4,MotorSpeedRotate);
		break;
		case carRight:
		speedGo(1,MotorSpeedRotate);
		speedGo(2,MotorSpeedFast);
		speedGo(3,MotorSpeedRotate);
		speedGo(4,MotorSpeedFast);
		break;
		case carForLeft:
		speedGo(1,MotorSpeedNormal);
		speedGo(2,MotorSpeedRotate);
		speedGo(3,MotorSpeedNormal);
		speedGo(4,MotorSpeedRotate);
		break;
		case carForRight:
		speedGo(1,MotorSpeedRotate);
		speedGo(2,MotorSpeedNormal);
		speedGo(3,MotorSpeedRotate);
		speedGo(4,MotorSpeedNormal);
		break;
		case carLeftFast:
		speedGo(1,MotorSpeedRoFast);
        speedGo(2,MotorSpeedTrace);
        speedGo(3,MotorSpeedRoFast);
        speedGo(4,MotorSpeedTrace);
        break;
        case carLeftSlow:
        speedGo(1,MotorSpeedRoSlow);
		speedGo(2,MotorSpeedTrace);
		speedGo(3,MotorSpeedRoSlow);
        speedGo(4,MotorSpeedTrace);
        break;
        case carRightFast:
		speedGo(1,MotorSpeedTrace);
        speedGo(2,MotorSpeedRoFast);
        speedGo(3,MotorSpeedTrace);
        speedGo(4,MotorSpeedRoFast);
        break;
        case carRightSlow:
		speedGo(2,MotorSpeedTrace);
        speedGo(1,MotorSpeedRoSlow);
        speedGo(4,MotorSpeedTrace);
        speedGo(3,MotorSpeedRoSlow);
        break;
		case carStop:
		stop();
		break;
		default:
		stop();
		break;
	}
}

bool LZSmartCarMotor::getCommand(int *buff)
{
	String str = "";
	if(_mode == Button){
		if(BTSerial.available()){
			char c = BTSerial.read();
			str += c;
			buff[0] = str.toInt();
//			Serial.println(str);
		}
	}
	else if(_mode == Gravity){
		while(BTSerial.available()){
			char c = BTSerial.read();
//			Serial.println(c);
			str += c;
		}
//		Serial.println(str);
		pos1 = str.indexOf("(");
		pos2 = str.indexOf(",");
		pos3 = str.indexOf(")");
		buff[0] = str.substring(pos1+1, pos2).toInt();
		buff[1] = str.substring(pos2+1, pos3).toInt();
	}
	else if(_mode == JoyStick){
		while(BTSerial.available()){
			char c = BTSerial.read();
//			Serial.println(c);
//			if(c == '['){
				//begin read
				str += c;
//				while(BTSerial.available()){
//					c = BTSerial.read();
//					str += c;
//					if(c == ']'){
//						break;
//					}
//				}
				
//				break;
//			}
//			Serial.println(str);
//			str += c;
		}
//		Serial.println(str);
		pos1 = str.indexOf("[");
		pos2 = str.indexOf(",");
		pos3 = str.indexOf("]");
		Serial.print("pos1:");Serial.print(pos1);
		Serial.print("pos2:");Serial.print(pos2);
		Serial.print("pos3:");Serial.println(pos3);
		if(pos1 != -1 && pos2 != -1 && pos3 != -1){
			buff[0] = str.substring(pos1+1, pos2).toInt();
			buff[1] = str.substring(pos2+1, pos3).toInt();
		}
	}
}

void LZSmartCarMotor::setControlMode(byte mode = 0)
{
	_mode = mode;
	switch(_mode)
	{
		case 0:
			Serial.println("按键控制模式");
		break;
		case 1:
			Serial.println("重力感应控制模式");
		break;
		case 2:
			Serial.println("零知星球摇杆控制模式");
		break;
		case 3:
			Serial.println("JoyStick摇杆控制模式");
		break;
	}
}

void LZSmartCarMotor::btnControl()
{
		if (cmd[0] == 1)
		{
			goWithDirec(carForward);
		}
		else if (cmd[0] == 2)
		{
			goWithDirec(carBack);
		}
		else if (cmd[0] == 3)
		{
			goWithDirec(carForLeft);
		}
		else if (cmd[0] == 4)
		{
			goWithDirec(carForRight);
		}
		else if (cmd[0] == 9)
		{
			goWithDirec(carStop);
		}
}

void LZSmartCarMotor::gravityControl()
{
		x = cmd[0];
		y = cmd[1];
		
		speed = map(x, 10, 90, 20, 255);
		if(y < 90 || y > -90){
			speedRight = map(y, 10, 90, speed, 20);
			speedLeft = map(y, -90, -10, 20, speed);
		}
		else{
			speedRight = map(y, 170, 90, speed, 0);
			speedLeft = map(y, -90, -170, 0, speed);
		}

		if (y > -10 && y < 10 && x > 10)
		{
			Serial.println("前进");
			motorRun(speed, speed);
		}
		else if ((y > 170 || y < -170) && x > 10)
		{
			Serial.println("后退");
			motorRun(-speed, -speed);
		}
		else if (y > 10 && y < 90 && x > 10)
		{
			Serial.println("右转弯");
			motorRun(speed, speedRight);
		}
		else if (y < -10 && y > -90 && x > 10)
		{
			Serial.println("左转弯");
			motorRun(speedLeft, speed);
		}
		else if (y > 90 && y < 170 && x > 10)
		{
			Serial.println("右后转弯");
			motorRun(speed, speedRight);
		}
		else if(y < -90 && y > -170 && x > 10)
		{
			Serial.println("左后转弯");
			motorRun(speedLeft, speed);
		}
		else{
			Serial.println("停车");
			stop();
		}
}

void LZSmartCarMotor::rockerControl()
{
		x = cmd[0];
		y = cmd[1];
		
		if(x > 5){
			speedLeft = map(x, 5, 100, 20, 255);
			speedRight = map(x, 5, 100, 20, 255);
		}
		else{
			speedLeft = 0;
			speedRight = 0;
		}
		
		if(y > 5 && y < 85){
			yMap = map(y, 5, 85, 20, 255);
			speedLeft = speedLeft + yMap;
			speedRight = speedRight - yMap;
		}
		else if(y < -5 && y > -85){
			yMap = map(y, -5, -85, 20, 255);
			speedLeft = speedLeft - yMap;
			speedRight = speedRight + yMap;
		}
		else if(y >95 && y < 175){
			yMap = map(y, 175, 85, 20, 255);
			speedLeft = -speedLeft - yMap;
			speedRight = -speedRight + yMap;
		}
		else if(y < -95 && y > -175){
			yMap = map(y, -175, -85, 20, 255);
			speedLeft = -speedLeft + yMap;
			speedRight = -speedRight - yMap;
		}
		motorRun(speedLeft, speedRight);
}

void LZSmartCarMotor::joyStickControl()
{
		x = cmd[0];
		y = cmd[1];
		
		if (y < 470){
			speedRight  = map(y, 470, 0, 0, -255);
			speedLeft   = map(y, 470, 0, 0, -255);
		}
		else if(y > 550){
			speedRight  = map(y, 550, 1023, 0, 255);
			speedLeft   = map(y, 550, 1023, 0, 255);
		}
		else{
			speedLeft = 0;
			speedRight = 0;
		}
		if(x < 470){
			xMap = map(x, 470, 0, 0, 255);
			speedLeft = speedLeft + xMap;
			speedRight = speedRight - xMap;
		}
		else if(x > 550){
			xMap = map(x, 550, 1023, 0, 255);
			speedLeft = speedLeft + xMap;
			speedRight = speedRight - xMap;
		}
		motorRun(speedLeft, speedRight);
}

void LZSmartCarMotor::bleControl()
{
	getCommand(cmd);
	if(_mode == Button)
	{
		btnControl();
	}
	else if(_mode == Gravity)
	{
		gravityControl();
	}
	else if(_mode == Rocker)
	{
		rockerControl();
	}
	else if(_mode == JoyStick)
	{
		joyStickControl();
	}
//	Serial.print("x:");Serial.println(x);
//	Serial.print("y:");Serial.println(y);
}

LZSmartCarMotor::~LZSmartCarMotor()
{
}