/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

线性mpc，F系
	Blc::mpcClass &mpc=Blc::mpcClass::instance();
	Blc::stateStruct stt;
=====================================================*/
#pragma once
#include"eigen.h"
#include"nabo_config.h"

namespace Blc{
struct stateStruct{//全部为F系
	vec3d aDlt,pDlt;//A、p的相对差（act-tgt）
	vec3d w,v;
	vec3d leg0,leg1;
	double vx,vy,wz;//命令值
	void setZero(){
		aDlt.setZero();
		pDlt.setZero();
		w.setZero();
		v.setZero();
		vx=0;vy=0;wz=0;
		leg0<<0,-0.1,-BodyH;
		leg1<<0, 0.1,-BodyH;
	}
};
//=========================
class mpcClass{
public:
	static mpcClass& instance();
	~mpcClass(){stop();};
	bool start();
	bool stop();
	bool wake();
	bool sleep();
	void setCiParam(double mu,double muz,double lx,double ly);//摩擦系数mu，旋转摩擦系数muz，脚面半长lx，脚面半宽ly
	void setState(stateStruct &stt);
	void getResult(vec3d &p,mat3d &R,vec6d &wv,vec6d &mf0,vec6d &mf1);
	void getMF(vec6d &mf0,vec6d &mf1);//F系，出力方向
	void test();
	void testRun();
private:
	mpcClass();
	mpcClass(const mpcClass&)=delete;
	mpcClass& operator=(const mpcClass&)=delete;

	class impClass;
	impClass &imp;
};
}//namespace