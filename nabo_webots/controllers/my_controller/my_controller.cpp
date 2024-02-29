/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/Device.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
//#include<chrono>
#include"nabo.h"
#include<cstring>

using namespace webots;
using namespace std;

inline void clip(double&x, double lim){
	if(lim<0){lim=-lim;}
	if(x<-lim){x=-lim;}
	else if(x>lim){x=lim;}
}

Nabo::inputStruct in;
Nabo::outputStruct out;
int main(int argc, char **argv){
	Supervisor		 sup;
	InertialUnit	 imu{*sup.getInertialUnit("imu")};
	Gyro			 gyro{*sup.getGyro("gyr")};
	Accelerometer	 accel{*sup.getAccelerometer("accel")};
	Node			*dummy{sup.getFromDef("robot")};
	Motor			*mt[NMot];
	PositionSensor	*ps[NMot];
	Keyboard		 kb{*sup.getKeyboard()};

	for(int i=0;i<NLegDof;i++){
		mt[i]=sup.getMotor("mt0"+to_string(i));
		mt[i+NLegDof]=sup.getMotor("mt1"+to_string(i));
	}

	int timeStep = (int)sup.getBasicTimeStep();
	double dt=timeStep/1000.0;
	Nabo::setDt(dt);
	for(int i=0;i<NMot;i++){
		//mt[i].setControlPID(5,0,0.2);
		ps[i]=mt[i]->getPositionSensor();
		ps[i]->enable(timeStep);
		mt[i]->enableTorqueFeedback(timeStep);
	}
	imu.enable(timeStep);
	gyro.enable(timeStep);
	accel.enable(timeStep);
	kb.enable(timeStep*10);

	//膝关节避免奇异（很奇怪，第一个关节不会执行，所以强行添加了mt[0]）
	// mt[0]->setPosition(-0.1);//不会执行
	// mt[3]->setPosition(-0.1);
	// mt[8]->setPosition(-0.1);

	// cout<<sup.getSelf()->getProtoField("physics")->getSFNode()->getField("density")->getSFFloat()<<endl;
	bool floatBaseFlag=0;
	if(sup.getSelf()->getProtoField("physics")->getSFNode()){
		floatBaseFlag=1;
	}
	int cnt=0;
	while(sup.step(timeStep) !=-1){
		in.cnt=cnt;
		cnt++;
		int key=kb.getKey();//点击场景后键盘生效，连续触发
		double kkk=0.005;
		if(key>0){
			switch(key){
			case ' ':
				in.cmdVx=0;in.cmdVy=0;in.cmdWz=0;break;
			case 'W':
				in.cmdVx= 0.5*kkk +in.cmdVx*(1-kkk);break;
			case 'S':
				in.cmdVx=-0.5*kkk +in.cmdVx*(1-kkk);break;
			case 'A':
				in.cmdVy= 0.2*kkk +in.cmdVy*(1-kkk);break;
			case 'D':
				in.cmdVy=-0.2*kkk +in.cmdVy*(1-kkk);break;
			case 'J':
				in.cmdWz= 0.8*kkk +in.cmdWz*(1-kkk);break;
			case 'L':
				in.cmdWz=-0.8*kkk +in.cmdWz*(1-kkk);break;
			}
			clip(in.cmdVx,0.5);
			clip(in.cmdVy,0.2);
			clip(in.cmdWz,0.5);
			if(abs(in.cmdVx)<1e-6){in.cmdVx=0;}
			if(abs(in.cmdVy)<1e-6){in.cmdVy=0;}
			if(abs(in.cmdWz)<1e-6){in.cmdWz=0;}
		}

		memcpy(in.supP, dummy->getPosition(), 24);
		if(floatBaseFlag){
			in.supP[2]-=0.04;//ankle到脚底高度0.04，W系
		}else{
			in.supP[2]=0.65;
		}
		memcpy(in.supV, dummy->getVelocity(), 24);

		memcpy(in.rpy, imu.getRollPitchYaw(), 24);
		memcpy(in.gyr, gyro.getValues(), 24);
		memcpy(in.acc, accel.getValues(), 24);
		for(int i=0;i<NMot;i++){
			double tmp=in.j[i];
			in.j[i]=ps[i]->getValue();
			in.w[i]=(in.j[i]-tmp)/dt;
			in.t[i]=mt[i]->getTorqueFeedback();
		}

		Nabo::run(in,out);

		for(int i=0;i<NMot;i++){
			// mt[i]->setPosition(out.j[i]);
			//mt[i]->setVelocity(out.w[i]);
			mt[i]->setTorque(out.t[i]);
		}
		// if(cnt%500==0){
		//	 // cout<<in.w[0]<<", "<<in.w[1]<<", "<<in.w[2]<<endl;
		//	 //cout<<in.j[0]<<", "<<out.w[0]<<endl;
		//	 cout<<"cmd: vx="<<in.cmdVx<<", vy="<<in.cmdVy<<", wz="<<in.cmdWz;
		// }
	};
	delete &sup;
	return 0;
}
