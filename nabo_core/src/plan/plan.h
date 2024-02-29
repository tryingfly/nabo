/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

运动规划基类
继承提供统一接口，可方便做管理调度（todo：manager.cpp内实现系统调度）
继承会导致阅读不便，基类尽量不添加复杂逻辑
=====================================================*/
#pragma once
#include"iopack.h"
#include"robot.h"
#include"nabo_config.h"

namespace Plan{
class basePlanClass{
public:
	virtual void setDt(double dt)final;
	virtual bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)=0;
	virtual void log()=0;
protected:
//都会用到的变量，在基类定义。个体plan用到的变量在个体plan中定义即可
	Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
	double dt;
	double tim;
	bool quitFlag;
	bool logFlag,consoleFlag;
	int logCnt,consoleCnt;
	Nabo::inputStruct in;
	Nabo::outputStruct out;
	Alg::filterOneClass wFil[NMot],tOutFil[NMot];
//base开头的函数所有plan都会调用，标记为fianl
	virtual void baseInit()final;
};
}//namespace
