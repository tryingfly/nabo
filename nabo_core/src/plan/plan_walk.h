/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#pragma once
#include"plan.h"
#include"cpg.h"
#include"estimator.h"
#include"curve.h"
#include"com_data.h"
#include"mpc.h"
#include"cbic.h"

namespace Plan{
class walkPlanClass final:public basePlanClass{
public:
	walkPlanClass();
	bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	void log() override final;
private:
	Cpg::cpgClass &cpg=Cpg::cpgClass::instance();
	Est::estClass &est=Est::estClass::instance();
	Blc::mpcClass &mpc=Blc::mpcClass::instance();
	Blc::cbicClass &cbic=Blc::cbicClass::instance();
	Crv::swTrj6dClass swTrj6d[2];
	WBC::comDataStruct com;
	Blc::stateStruct stt;
	bool cbicFlag,cmdFocFlag;
	double pRate,vRate;

	mat3d cmdTipR[2];
	vec3d cmdTipP[2],stdTipP[2];
	vec6d cmdTipV6d[2],cmdTipF6d[2];

	struct{
		vec3d rpy,p,v2S;//p：W系，v：S系（坡面系，遥控指令）
		mat3d RxyS,RxyA2S,Rz,RS;//地形、主动（地形之外）、yaw、地形+yaw
		double wz;//F与W系等价
		double dVx,dVy,dWz;
		void init(){
			p.setZero();v2S.setZero();rpy.setZero();
			RxyS.setIdentity();RxyA2S.setIdentity();Rz.setIdentity();
			wz=0;
			dVx=0;dVy=0;dWz=0;
		}
	}des;//desire：期望平滑轨迹
	struct{
		mat3d Rxy,R;//地形+主动（B to W系）
		vec3d p,w,v,v2F;
		double zOff;
		void init(){
			Rxy.setIdentity();R.setIdentity();
			p.setZero();w.setZero();
			v.setZero();v2F.setZero();
			zOff=0;
		}
	}tgt;//target：叠加zmp等调整
	struct{
		double vx=0.5,vy=0.2,wz=0.8;
		double dVx,dVy;
		double dRol,dPit,dWz;
		void init(double dt){
			dVx=0.4*dt;
			dVy=0.3*dt;
			dRol=0.5*dt;
			dPit=0.5*dt;
			dWz=1*dt;
		}
	}lmt;//limit：限幅

	void init();
	void update();
	void plan();
	void balance();
	void dwdate(Nabo::outputStruct &outRef);
};
}//namespace
