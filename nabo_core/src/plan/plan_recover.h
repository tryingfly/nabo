/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#pragma once
#include"plan.h"
#include<array>

namespace Plan{

class rcPlanClass final:public basePlanClass{
public:
	rcPlanClass();
	bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	void log() override final;
private:
	void init();
	void plan();
	void dwdate(Nabo::outputStruct &outRef);
	array<double,NMot> j0,j1,j2;
	double errSum[NMot];
	double tSit,tWait,tStand;
	double kp,ki,kd;
};
}//namespace
