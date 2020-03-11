/*
 * @文件: \cartrack.cpp
 * @作者:零知实验室
 * -^^- 零知开源，让电子制作变得更简单！ -^^-
 * @时间: 2019-12-05 15:43:58
 * @说明: 
 */
#include "cartrack.h"

CarTracking::CarTracking(int numLines)
{
    _numLines = numLines;
	for(int i=0; i<numLines; i++)
	{
		pinMode(pinBuff[i],INPUT);
	}
//	pinMode(TracePin1,INPUT);
//	pinMode(TracePin2,INPUT);
//	pinMode(TracePin3,INPUT);
//	pinMode(TracePin4,INPUT);
//	pinMode(TracePin5,INPUT);
}

void CarTracking::getLineData(byte *buff)
{
    for(int i=0; i<_numLines; i++)
	{
		int IR = digitalRead(pinBuff[i]);
		buff[i] = IR;
	}
//	int IR_1 = digitalRead(TracePin1);
//	int IR_2 = digitalRead(TracePin2);
//	int IR_3 = digitalRead(TracePin3);
//	int IR_4 = digitalRead(TracePin4);
//	int IR_5 = digitalRead(TracePin5);
//
//	buff[0]=IR_1;
//	buff[1]=IR_2;
//	buff[2]=IR_3;
//	buff[3]=IR_4;
//	buff[4]=IR_5;
	
	Serial.print("IR:");
	Serial.print(buff[0]);Serial.print("        ");
	Serial.print(buff[1]);Serial.print("        ");
	Serial.print(buff[2]);Serial.print("		 ");
	Serial.print(buff[3]);Serial.print("        ");
	Serial.print(buff[4]);Serial.println();
//	delay(300);
}

void CarTracking::setGround(byte groundColor)
{
	Ground = groundColor;
}

void CarTracking::setLine(byte lineColor)
{
	Line = lineColor;
}

float CarTracking::getDecide(byte *buff)
{
//	if(_numLines = 5){
		if(buff[4] == Line && buff[3] == Line && buff[2] == Line && buff[1] == Ground && buff[0] == Ground){
			Serial.println("左直角");
			decide = ninetyLeft;
		}
		else if(buff[4] == Line && buff[3] == Ground && buff[2] == Ground && buff[1] == Ground && buff[0] == Ground){
			Serial.println("左大转");
			decide = biggerLeft;
		}
		else if(buff[4] == Line && buff[3] == Line && buff[2] == Ground && buff[1] == Ground && buff[0] == Ground){
			Serial.println("左大锐角");
			decide = bigLeft;
		}
		else if(buff[4] == Ground && buff[3] == Line && buff[2] == Ground && buff[1] == Ground && buff[0] == Ground){
			Serial.println("左小转");
			decide = smallLeft;
		}
		else if(buff[4] == Ground && buff[3] == Line && buff[2] == Line && buff[1] == Ground && buff[0] == Ground){
			Serial.println("左小锐角");
			decide = smallerLeft;
		}
		else if(buff[4] == Line && buff[3] == Line && buff[2] == Line && buff[1] == Line && buff[0] == Line){
			Serial.println("直行");
			decide = straight;
		}
		else if(buff[4] == Ground && buff[3] == Line && buff[2] == Line && buff[1] == Line && buff[0] == Ground){
			Serial.println("直行");
			decide = straight;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Line && buff[1] == Ground && buff[0] == Ground){
			Serial.println("直行");
			decide = straight;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Line && buff[1] == Line && buff[0] == Ground){
			Serial.println("右小锐角");
			decide = smallerRight;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Ground && buff[1] == Line && buff[0] == Ground){
			Serial.println("右小转");
			decide = smallRight;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Ground && buff[1] == Line && buff[0] == Line){
			Serial.println("右大锐角");
			decide = bigRight;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Ground && buff[1] == Ground && buff[0] == Line){
			Serial.println("右大转");
			decide = biggerRight;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Line && buff[1] == Line && buff[0] == Line){
			Serial.println("右直角");
			decide = ninetyRight;
		}
		else if(buff[4] == Ground && buff[3] == Line && buff[2] == Line && buff[1] == Line && buff[0] == Line){
			Serial.println("右直角")	;
			decide = ninetyRight;
		}
		else if(buff[4] == Line && buff[3] == Line && buff[2] == Line && buff[1] == Line && buff[0] == Ground){
			Serial.println("左直角")	;
			decide = ninetyLeft;
		}
		else if(buff[4] == Ground && buff[3] == Ground && buff[2] == Ground && buff[1] == Ground && buff[0] == Ground){
			if(decide == -4)
				decide = biggestLeft;
			else if(decide == 4)
				decide = biggestRight;
		}
//	}
//	else if(_numLines = 4){
//		if(buff[3] = Line && buff[2] = Ground && buff[1] == Ground && buff[0] == Ground){
//			decide = biggerLeft;
//		}
//		else if(buff[3] = Line && buff[2] = Line && buff[1] == Ground && buff[0] == Ground){
//			decide = bigLeft;
//		}
//		else if(buff[3] = Ground && buff[2] = Line && buff[1] == Ground && buff[0] == Ground){
//			decide = smallLeft;
//		}
//		else if(buff[3] = Ground && buff[2] = Line && buff[1] == Line && buff[0] == Ground){
//			decide = straight;
//		}
//		else if(buff[3] = Line && buff[2] = Line && buff[1] == Line && buff[0] == Line){
//			decide = straight;
//		}
//		else if(buff[3] = Ground && buff[2] = Ground && buff[1] == Line && buff[0] == Ground){
//			decide = smallRight;
//		}
//		else if(buff[3] = Ground && buff[2] = Ground && buff[1] == Line && buff[0] == Line){
//			decide = bigRight;
//		}
//		else if(buff[3] = Ground && buff[2] = Ground && buff[1] == Ground && buff[0] == Line){
//			decide = biggerRight;
//		}
//		else if(buff[3] = Ground && buff[2] = Ground && buff[1] == Ground && buff[0] == Ground){
//			if(decide = smallLeft || decide == smallRight)
//				decide = straight;
//			else if(decide = biggerLeft)
//				decide = biggestLeft;
//			else if(decide = biggestRight)
//				decide = biggestRight;
//		}
//	}
	
	return decide;
}

CarTracking::~CarTracking()
{

}

