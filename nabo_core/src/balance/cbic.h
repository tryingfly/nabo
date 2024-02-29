/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

质心平衡，相当于对mpc的结果进行补偿滤波
计算均在F系
	Blc::cbicClass &cbic=Blc::cbicClass::instance();
=====================================================*/
#pragma once
#include"eigen.h"

namespace Blc{
class cbicClass{
public:
	static cbicClass& instance();
	void init(double dt);
	void setZero();
	void setCiParam(double mu,double muz,double lx,double ly);//摩擦系数mu，旋转摩擦系数muz，脚面半长lx，脚面半宽ly
	void setFr(const vec6d (&mf)[2]);//F系，出力方向
	void run(vec3d &aAcc,vec3d &acc);//F系，根据相位分配力
	void run(vec3d &aAcc,vec3d &acc,const double(&stPgs)[2],bool isLock);//F系，根据相位分配力-支撑腿平滑
	void run(vec3d &aAcc,vec3d &acc,const double(&stPgs)[2],const double(&swPgs)[2]);//F系，根据相位分配力-支撑腿、空中腿分别平滑
	vec6d mf[2];//F系，出力方向
private:
	cbicClass();
	cbicClass(const cbicClass&)=delete;
	cbicClass& operator=(const cbicClass&)=delete;

	double maxFz=300;

	class impClass;
	impClass &imp;
};
}//namespace