/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"plan.h"

namespace Plan{
	void basePlanClass::setDt(double dtt){
		dt=dtt;
		rbt.setDt(dt);
		logCnt=int(0.01/dt);//0.01s一帧log
		consoleCnt=int(0.5/dt);//0.5s一帧控制台print
	}
	void basePlanClass::baseInit(){
		tim=0;
		quitFlag=0;
		For(NMot){
			wFil[i].init(dt,500,1,0);
			tOutFil[i].init(dt,200,1,0);
		}
		rbt.imu.init(in.rpy);
		rbt.setSwing(0,0);
		rbt.setSwing(1,0);
	}
}//namespace

