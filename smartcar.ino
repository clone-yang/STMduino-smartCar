/**********************************************************
*    文件: LZSmartCarMotor_v2.ino      by 零知实验室(www.lingzhilab.com)
*    -^^- 零知开源，让电子制作变得更简单！ -^^-
*    时间: 2019/11/30 09:37
*    说明: 零知智能小车-高级版，两轮/四轮-迷你板主控
************************************************************/

//所有功能开关，1开启，0关闭，选择需要的功能上传
#define MOTOR_TEST            0 // 电机测试

#define LIGHT                 0 // 跑马灯开关

#define BLE_CONTROL           0 // 蓝牙控制、默认按键控制

#define CONTROL_MODE          0 // 蓝牙控制模式，0-按键控制、1-零知星球重力感应控制、2-JoyStick摇杆控制、3-零知星球摇杆模式控制

#define FIRE_CAR              0 // 灭火小车

#define AVOIDANCE             0 // 避障小车

#define PID_SWITCH            0 // PID算法开关

#define TRACK                 0 // 循迹(不含PID算法)，PID循迹请开启PID_SWITCH

#define MAGIC_HAND            0 // 魔术手

#define FOLLOW_CAR            0 // 物体跟随

#define NEAR_WALL             0 // 贴边行驶，使用红外避障或开启PID_SWITCH使用超声波

#define PID_STRAIGHT          0 // PID算法走直线

#define AUTO_PID              0 // 电机PID自整定，需同时开启PID_STRAIGHT


#if PID_SWITCH && !(TRACK || NEAR_WALL)

#error Please trun on TRACK, MAGIC_HAND or NEAR_WALL!

#endif

#if CONTROL_MODE > 3

#error The value of the CONTROL_MODE is 0,1,2 or 3

#endif


#if AUTO_PID && !PID_STRAIGHT

#error Please trun on PID_STRAIGHT!

#endif

#include "pinsdefine.h"
#include "motor.h"

//两轮车,若是四轮车请注释此句
//#define MOTOR_2WD

#ifdef MOTOR_2WD
LZSmartCarMotor myMotor;
#else
LZSmartCarMotor myMotor(false);
#endif

#if FIRE_CAR || PID_STRAIGHT
#include "carsensor.h"

//传感器：温湿度、火焰、风扇
CarSensor carsensor;

CarSensor carsensor1;
#endif

#if LIGHT
#include "WS2812BIORGB.h"
#endif

#if AVOIDANCE || MAGIC_HAND || FOLLOW_CAR || NEAR_WALL
#include "caravoidance.h"

//超声波避障
CarAvoidance carAvoidance;
#endif

#if TRACK
#include "cartrack.h"

//红外循迹,5路
CarTracking carTrack(5);
#endif

#if PID_SWITCH || PID_STRAIGHT
#include <PID_v1.h>
#endif

#if BLE_CONTROL

void bleControl()
{
	myMotor.setControlMode(CONTROL_MODE);
	while(1){
		if(BTSerial.available())
			myMotor.bleControl();
	}
}
#endif

#if AVOIDANCE
void doAvoidance()
{
	int dt = 10;		  //舵机速度
	int maxAbstacle = 28; //障碍物检测最大距离

	//首先探测正前方
	int disFor = 0;
	for (int i = 0; i < 5; i++)
	{
		disFor += carAvoidance.getDistance();
		delay(50);
	}
	disFor = disFor / 5;

	Serial.print(" 前方距离：");
	Serial.println(disFor);

	if (disFor <= maxAbstacle) //正前方有障碍物
	{
		//停车
		myMotor.stop();

		int t = 0;
		int total = 0;

		//探测右边
		carAvoidance.servoRun(90, 0, dt);

		int disRight = 0;
		for (int i = 0; i < 5; i++)
		{
			disRight += carAvoidance.getDistance();
			delay(50);
		}
		disRight = disRight / 5;

		Serial.print(" 右侧距离：");
		Serial.println(disRight);

		carAvoidance.servoRun(0, 90, dt);

		//探测左边
		carAvoidance.servoRun(90, 200, dt);
		int disLeft = 0;
		for (int i = 0; i < 5; i++)
		{
			disLeft += carAvoidance.getDistance();
			delay(50);
		}
		disLeft = disLeft / 5;

		Serial.print(" 左侧距离：");
		Serial.println(disLeft);

		carAvoidance.servoRun(200, 90, dt);

		int rollTime = 720;

		//两边都有障碍物，掉头
		if ((disLeft <= maxAbstacle) && (disRight <= maxAbstacle))
		{
			myMotor.goWithDirec(carBack);
			Serial.println("--后退");
			delay(rollTime);
		}
		if (disLeft >= disRight)
		{
			myMotor.goWithDirec(carLeft); //左转
			Serial.println("--左转");
			delay(rollTime);
		}
		if (disLeft < disRight)
		{
			myMotor.goWithDirec(carRight); //右转
			Serial.println("--右转");
			delay(rollTime);
		}
	}

	//红外测量斜边
	int rotateSidetime = 90;
	bool isLeftIRAvoid = false;
	bool isRightIRAvoid = false;
	carAvoidance.getIRAvoidData(&isLeftIRAvoid, &isRightIRAvoid);
	if (isLeftIRAvoid)
	{
		Serial.println("left ir avoid");

		myMotor.goWithDirec(carRight);
		delay(rotateSidetime);
	}
	if (isRightIRAvoid)
	{
		Serial.println("right ir avoid");

		myMotor.goWithDirec(carLeft);
		delay(rotateSidetime);
	}

	myMotor.goWithDirec(carForwardSlow);
}
#endif

#if TRACK
//循迹的颜色，非黑即白
//#define BLACK 1
//#define WHITE 0

//黑底白线，若白底黑线请调换颜色
// #define LINE   WHITE
// #define GROUND BLACK

#if PID_SWITCH
// 设置PID参数，根据实际情况调整至最佳
//double Kp = 6.5, Ki = 1, Kd = 0;

double Kp = 6.5, Ki = 0.05, Kd = 0.5;
double Input, Output, Setpoint = 0;

static int speed = 90;

PID trackpid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
#endif

void doTrackLine()
{
	byte buff[5];
	carTrack.getLineData(buff);
	// 设置底的颜色和线的颜色，只有黑白，默认白底黑线
	carTrack.setGround(BLACK);
	carTrack.setLine(WHITE);

	int decide = carTrack.getDecide(buff);
//	Serial.print("decide:");Serial.println(decide);

#if PID_SWITCH
	while(decide == ninetyLeft){
		myMotor.motorRun(0,80);
		carTrack.getLineData(buff);
		if(carTrack.getDecide(buff) == smallLeft){
			return;
		}
	}
	while(decide == ninetyRight){
		myMotor.motorRun(80,0);
		carTrack.getLineData(buff);
		if(carTrack.getDecide(buff) == smallRight){
			return;
		}
	}
	Input = decide;
	trackpid.Compute();
	
	int leftspeed = speed - Output;
	int rightspeed = speed + Output;
	Serial.print("leftspeed:");
	Serial.print(leftspeed);
	Serial.print("  rightspeed:");
	Serial.println(rightspeed);
	
	myMotor.motorRun(leftspeed, rightspeed);

#else
	switch (decide)
	{
	case biggerLeft:
		myMotor.goWithDirec(carLeft);
		break;
	case smallLeft:
		myMotor.goWithDirec(carForLeft);
		break;
	case straight:
		myMotor.goWithDirec(carForwardSlow);
		break;
	case smallRight:
		myMotor.goWithDirec(carForRight);
		break;
	case biggerRight:
		myMotor.goWithDirec(carRight);
		break;
	case ninetyLeft:
		myMotor.goWithDirec(carLeft);
		delay(160);
		break;
	case ninetyRight:
		myMotor.goWithDirec(carRight);
		delay(160);
		break;
	default:
		break;
	}

	// byte buff[5];
	// carTrack.getLineData(buff);
	//路线元素，有其它元素可自行添加
	// if(buff[4] == LINE && buff[3] == GROUND && buff[2] == GROUND && buff[1]==GROUND && buff[0] == GROUND){
	// 	//左大转
	// 	myMotor.goWithDirec(carLeft);
	// 	Serial.println("左大转");
	// }
	// if(buff[4] == GROUND && buff[3] == GROUND && buff[2] == GROUND && buff[1]==GROUND && buff[0] == LINE){
	// 	//右大转
	// 	myMotor.goWithDirec(carRight);
	// 	Serial.println("右大转");
	// }
	// if(buff[4] == GROUND && buff[3] == LINE && buff[2] == GROUND && buff[1]==GROUND && buff[0] == GROUND){
	// 	//左小转
	// 	myMotor.goWithDirec(carForLeft);
	// 	Serial.println("左小转");
	// }
	// if(buff[4] == GROUND && buff[3] == GROUND && buff[2] == GROUND && buff[1]==LINE && buff[0] == GROUND){
	// 	//右小转
	// 	myMotor.goWithDirec(carForRight);
	// 	Serial.println("右小转");
	// }
	// if(buff[4] == GROUND && buff[3] == GROUND && buff[2] == LINE && buff[1]==LINE && buff[0] == LINE){
	// 	myMotor.goWithDirec(carRight);
	// 	delay(160);
	// 	Serial.println("右直角");
	// }
	// if(buff[4] == LINE && buff[3] == LINE && buff[2] == LINE && buff[1] == GROUND && buff[0] == GROUND){
	// 	myMotor.goWithDirec(carLeft);
	// 	delay(160);
	// 	Serial.println("左直角");
	// }
	// if(buff[4] == GROUND && buff[3] == LINE && buff[2] == LINE && buff[1] == LINE && buff[0] == GROUND){
	// 	myMotor.goWithDirec(carForwardSlow);
	// 	Serial.println("直行");
	// }
	// if(buff[4] == GROUND && buff[3] == GROUND && buff[2] == LINE && buff[1] == GROUND && buff[0] == GROUND){
	// 	myMotor.goWithDirec(carForwardSlow);
	// 	Serial.println("直行");
	// }
	//	else{
	//		myMotor.goWithDirec(carForwardSlow);
	//	}
#endif
}
#endif

#if MAGIC_HAND

void magicHand()
{
	int speed;
	int maxStopDis = 17, minStopDis = 13;
	int distance = carAvoidance.getDistance();
	
	if(distance > maxStopDis + minStopDis)
		distance = maxStopDis + minStopDis;
	Serial.print("距离：");Serial.println(distance);
	
	if (distance > maxStopDis)
	{
		speed = map(distance, maxStopDis, maxStopDis+minStopDis, 60, 120);
		myMotor.motorRun(speed,speed);
		Serial.println("前进");
	}
	else if (distance < minStopDis)
	{
		speed = map(distance, minStopDis, 0, -60, -120);
		myMotor.motorRun(speed,speed);
		Serial.println("后退");
	}
	else
	{
		myMotor.stop();
		Serial.println("停车");
	}
}
#endif

#if FOLLOW_CAR
void doFollow()
{
	bool leftIR, rightIR;
	carAvoidance.getIRAvoidData(&leftIR, &rightIR);
	Serial.print("leftIR: ");Serial.print(leftIR);
	Serial.print("   rightIR: ");Serial.println(rightIR);
	if (leftIR == 0 && rightIR == 0)
	{
		myMotor.goWithDirec(carForwardSlow);
		Serial.println("前进");
	}
	else if (leftIR == 1 && rightIR == 0)
	{
		myMotor.goWithDirec(carForLeft);
		Serial.println("左转");
	}
	else if (leftIR == 0 && rightIR == 1)
	{
		myMotor.goWithDirec(carForRight);
		Serial.println("右转");
	}
	else{
		myMotor.stop();
		Serial.println("停车");
	}
		
}
#endif

#if NEAR_WALL

#if PID_SWITCH
// PID参数根据小车实际行驶的情况调整至最佳
double Kp =4, Ki = 0.02, Kd = 0.2;
double Input, Output, Setpoint = 17;  // 设置想要贴近的距离

PID nearpid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int leftPwm, rightPwm, initialPwm = 70;
#endif

void nearWall()
{
#if !PID_SWITCH
	bool leftIR, rightIR;
	carAvoidance.getIRAvoidData(&leftIR, &rightIR);
	Serial.print("leftIR: ");Serial.print(leftIR);
	Serial.print("   rightIR: ");Serial.println(rightIR);
	if (leftIR == 0 && rightIR == 0)
	{
		myMotor.motorRun(80,80);
		Serial.println("前进");
	}
	else if (leftIR == 0 && rightIR == 1)
	{
		myMotor.motorRun(0,70);
		Serial.println("左转");
	}
	else if (leftIR == 1 && rightIR == 0)
	{
		myMotor.motorRun(70,0);
		Serial.println("右转");
	}
	else{
		myMotor.stop();
		Serial.println("停车");
	}
#else
	//检测车身左右两侧的距离，贴距离近的一边行驶
	carAvoidance.servoRun(90, 0, 0);
	int leftDis = carAvoidance.getDistance();
	carAvoidance.servoRun(0, 180, 10);
	int rightDis = carAvoidance.getDistance();
	if(leftDis <= rightDis)
	{
		carAvoidance.servoRun(180, 30, 10);  // 将超声波移动到左前方30°
	}else
		carAvoidance.servoRun(180, 150, 10);  // 将超声波移动到右前方30°
	
	//开始贴边行驶
	while(1){
		int interval = carAvoidance.getDistance();  // 检测距离
		Serial.println("距离:");Serial.println(interval);
		if(interval > Setpoint*2)
			interval = Setpoint*2;
		Input = (double)interval;
		nearpid.Compute();
		if(leftDis <= rightDis){
			leftPwm = initialPwm - Output;
			rightPwm = initialPwm + Output;
		}else{
			leftPwm = initialPwm + Output;
			rightPwm = initialPwm - Output;	
		}
		
		Serial.print("leftPwm:");Serial.print(leftPwm);
		Serial.print("   rightPwm:");Serial.println(rightPwm);
		
		myMotor.motorRun(leftPwm, rightPwm);
	}
#endif
}
#endif

#if PID_STRAIGHT

double kp = 0.5, ki = 0.01, kd = 0;
double leftSpeed, rightSpeed;
double leftOut, rightOut;
double Setpoint = 180;
int sampletime = 100;  // 采样时间

// 左右轮的PID实例
PID leftpid(&leftSpeed, &leftOut, &Setpoint, kp, ki, kd, DIRECT);
PID rightpid(&rightSpeed, &rightOut, &Setpoint, kp, ki, kd, DIRECT);

void goStraight()
{
	carsensor.setSampletime(sampletime);  // 设置测速的采样时间
	bool flag = carsensor.getMotorSpeed(&leftSpeed, &rightSpeed);  // 测速，单位为rpm/min
	Serial.print("leftspeed:");
	Serial.print(leftSpeed);
	Serial.print("   rightspeed:");
	Serial.println(rightSpeed);

#if AUTO_PID
	carsensor.setPIDType(Pi);  //设置PI控制或PID控制
	carsensor.getAutoPid(myMotor, leftpid, &leftSpeed, &leftOut, &Setpoint, flag);
	carsensor1.getAutoPid(myMotor, rightpid, &rightSpeed, &leftOut, &Setpoint, flag, false);
	while(1)
	{
		carsensor.getMotorSpeed(&leftSpeed, &rightSpeed);
		leftpid.Compute();
		rightpid.Compute();
		Serial.print("leftOut:");
		Serial.print(leftOut);
		Serial.print("   rightOut:");
		Serial.println(rightOut);
		myMotor.motorRun(leftOut, rightOut);
	}
#else
	leftpid.Compute();
	rightpid.Compute();
	Serial.print("leftOut:");
	Serial.print(leftOut);
	Serial.print("   rightOut:");
	Serial.println(rightOut);

	myMotor.motorRun(leftOut, rightOut);
#endif
}
#endif

#if FIRE_CAR
void outFire()
{
	if (carsensor.getFlameData() == 0)
	{
		Serial.println("起火了....");

		//左转
		myMotor.goWithDirec(carForLeft);
		delay(600);
		myMotor.stop();

		//开启电扇
		carsensor.fanOpen(true);
#if LIGHT
		WS2812BRGB_flow(50, 160, 0xff0000, true);
		WS2812BRGB_flow(50, 160, 0xff0000, true);
		WS2812BRGB_setAll(LED_NUMS, 0x00);
#endif
		delay(1000);
		carsensor.fanOpen(false);

		//右转，转回原方向
		myMotor.goWithDirec(carForRight);
		delay(600);
		myMotor.goWithDirec(carForwardSlow);
	}
}
#endif

// 复位或上电后运行一次:
void setup()
{
	//在这里加入初始化相关代码，只运行一次:
	Serial.begin(9600);
	Serial.println("开车啦...");

#if LIGHT
	WS2812BRGB_init();
#endif

#if BLE_CONTROL
	BTSerial.begin(9600);
	BTSerial.println("AT+NAMELZSmartCar");
#endif

#if PID_SWITCH

#if TRACK
	trackpid.SetMode(AUTOMATIC);
	trackpid.SetOutputLimits(speed - 255, 255 - speed);
#endif
	
#if NEAR_WALL
	nearpid.SetMode(AUTOMATIC);
	nearpid.SetOutputLimits(initialPwm - 255,255 - initialPwm);
#endif

#endif

#if PID_STRAIGHT
	leftpid.SetMode(AUTOMATIC);
	rightpid.SetMode(AUTOMATIC);
	leftpid.SetOutputLimits(0, 255);
	rightpid.SetOutputLimits(0, 255);
#endif

	myMotor.stop();

//	myMotor.goWithDirec(carForwardSlow);
}

//一直循环执行:
void loop()
{
	// 在这里加入主要程序代码，重复执行:
#if LIGHT
	//RGB测试
	//left
	WS2812BRGB_flow(80, 160, 0xff0000, false);
	delay(100);
	//right
	WS2812BRGB_flow(80, 160, 0x00ff00, true);
	delay(100);

	//breath
	for (int i = 0; i < 5; i++)
	{
		WS2812BRGB_breath(5, 0xff);
		delay(300);
	}
#endif

#if FIRE_CAR
	outFire();  // 灭火
#endif

#if BLE_CONTROL
	bleControl();  // 蓝牙控制
#endif

#if AVOIDANCE
	doAvoidance();  // 避障
#endif

#if TRACK
	doTrackLine();  // 循迹
#endif

#if MAGIC_HAND
	magicHand();  // 魔术手
#endif

#if FOLLOW_CAR
	doFollow();  // 物体跟随或贴边行驶
#endif

#if NEAR_WALL
	nearWall();  // 贴边行驶
#endif

#if PID_STRAIGHT
	goStraight();  // PID算法走直线
#endif

#if MOTOR_TEST
	// 串口发送命令测试电机转向
	if (Serial.available())
	{
		char cmd = Serial.read();
		Serial.print("cmd:");
		Serial.write(cmd);
		Serial.println();
		if (cmd == '1')
		{
			myMotor.goWithDirec(carForward);
		}
		else if (cmd == '2')
		{
			myMotor.goWithDirec(carBack);
		}
		else if (cmd == '3')
		{
			myMotor.goWithDirec(carForLeft);
		}
		else if (cmd == '4')
		{
			myMotor.goWithDirec(carForRight);
		}
		else
		{
			myMotor.goWithDirec(carStop);
		}
	}
#endif
}
