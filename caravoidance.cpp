/*
 * @文件: \caravoidance.cpp
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 15:46:50
 * @说明: 
 */
#include "caravoidance.h"

CarAvoidance::CarAvoidance()
{
	//超声波
	pinMode(UltraEchoPin, INPUT);
	pinMode(UltraTrigPin, OUTPUT);

	//超声波舵机
	UltraServo.attach(ServoPWM1);
	UltraServo.write(ServoDefaultPos); //正中间
}

int CarAvoidance::getDistance()
{
	digitalWrite(UltraTrigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(UltraTrigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(UltraTrigPin, LOW);

	// int distance = pulseIn(UltraEchoPin, HIGH);

	int previousMicros = 0;
	int timeout = 40000;

	previousMicros = micros();
	while(!digitalRead(UltraEchoPin) && (micros() - previousMicros) <= timeout); // wait for the echo pin HIGH or timeout
	previousMicros = micros();
	while(digitalRead(UltraEchoPin)  && (micros() - previousMicros) <= timeout); // wait for the echo pin LOW or timeout

	int durat = micros() - previousMicros; // duration

	int distance = durat / 58.0;

	return distance;
}

void CarAvoidance::servoRun(int from, int to, int time)
{
	if (from < to)
	{
		for (int i = from; i <= to; i++)
		{
			UltraServo.write(i);
			delay(time);
		}
	}else if (from > to)
	{
		for (int i = from; i >= to; i--)
		{
			UltraServo.write(i);
			delay(time);
		}
	}
	else{
		UltraServo.write(from);
	}
}

void CarAvoidance::getIRAvoidData(bool *isLeftAvoid, bool *isRightAvoid)
{
	if(digitalRead(ServoPWM3) == 0){
		*isLeftAvoid = true;
	}else{
		*isLeftAvoid = false;
	}

	if(digitalRead(ServoPWM4) == 0){
		*isRightAvoid = true;
	}else{
		*isRightAvoid = false;
	}
}

CarAvoidance::~CarAvoidance()
{
}
