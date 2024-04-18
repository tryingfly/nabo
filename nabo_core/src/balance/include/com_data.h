/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#pragma once
#include "eigen.h"
namespace WBC{
struct comDataStruct{
	comDataStruct();
	vec3d acc,aAcc;
	void setZero();
	void pid(const vec3d &errA,const vec3d &errP,const vec3d &errW,const vec3d &errV);
	void filt();
private:
	vec3d kpP,kiP,kdP,kpA,kiA,kdA;
	vec3d errPSum,errASum;
	Alg::filterTwoClass fil[6];
	// Ei::filterOneClass filter[2];
	// vec3d accLast,aAccLast;
	// void smoothing(){
	// 	Ei::cmd2out1step(acc,accLast,0.03);
	// 	Ei::cmd2out1step(aAcc,aAccLast,0.06);
	// 	acc=accLast;
	// 	aAcc=aAccLast;
	// }
};
}//namespace