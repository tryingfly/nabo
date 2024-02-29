/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

#include"robot.h"
	Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
=====================================================*/
#pragma once
#include"nabo_config.h"
#include"imu.h"
namespace Rbt{
class rbtClass{
public:
	double tgtT[NMot];
	static rbtClass& instance();
	imuClass imu;
	void update(const double(&j)[NMot], const double(&w)[NMot], const double(&t)[NMot]);
	void dwdate(double(&t)[NMot]);
	void dwdate(double(&jOut)[NMot], double(&wOut)[NMot], double(&tOut)[NMot]);
//--get--------------------------------
	double getMass(){return m;}
	const mat3d & getInertia(){return inertia;}
	bool isSwing(int legId){return swFlag[legId];}
	const mat3d& getAnkleR2B(int legId);
	const vec3d& getAnkleP2B(int legId);
	const vec6d& getAnkleV6d2B(int legId);
	const vec6d& getAnkleF6d2B(int legId);//不带动力学，直接雅可比
//--set--------------------------------
	void setDt(double dt);
	void setSwing(int legId,bool sw){swFlag[legId]=sw;}
	//void setTip2B(vec6d tpP[2]);
	//void setTip2B(vec6d tpP[2],vec6d tpV[2]);
	void setTip2B(const mat3d(&tpR)[2],const vec3d(&tpP)[2],const vec6d(&tpV6d)[2],const vec6d(&tpF6d)[2]);
//==private================================
private:
	rbtClass();
	rbtClass(const rbtClass&)=delete;
	rbtClass & operator=(const rbtClass&)=delete;
	//--结构参数
	double m;
	mat3d inertia;
	bool swFlag[2];

	class impClass;
	impClass &imp;
};
}//namespace
