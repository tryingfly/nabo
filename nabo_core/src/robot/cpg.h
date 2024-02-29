/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

步态周期
缩写pgs=progress
	Cpg::cpgClass &cpg=Cpg::cpgClass::instance();
=====================================================*/
#pragma once

namespace Cpg{
static const int Num=2;

class cpgClass{
public:
	static cpgClass& instance();
	void init(const double dt,const double period, const double(&duty)[Num], const double(&offset)[Num]);//开始swing的pgs
	void setZero();
	void setParam(const double period,const double(&duty)[Num],const double(&offset)[Num]);//init为突变，此为平滑改变

	void update();
	void lock(){locking=1;};
	void unlock();
	bool isLock(){return locking;};//lock有一个过程，即便=1，有可能正在进行中
	bool isStop(){return locking & (!isSwAny);};//lock完毕

	bool isAllSt(){return !isSwAny;}
	bool isAllSw(double fT);//未来fT(future T)是否为all sw
	void isFutureSt(double fT,bool*isSt);
	void isFutureSt(double fT,int*stage);//sw=0，st=1，第二次st以后都为2
	double getNextEvent(double fT);//fT之后的下一次状态变换的时间，不包含fT，event无论抬或落

	const double &getPeriod(){return period;};
	const double getPgs(){if(pgs<0){return 0;}return pgs;};
	const double (&getSwPgs() )[Num]{return swPgs;};//返回数组引用，0～1
	const double (&getSwPgs2())[Num]{return swPgs2;};//返回数组引用，在sw结束后继续增长直至st结束，可用于触地下探
	const double (&getStPgs() )[Num]{return stPgs;};
	const double &getSwPgs(int i){return swPgs[i];};
	const double &getSwPgs2(int i){return swPgs2[i];};
	const double &getStPgs(int i){return stPgs[i];};
	const double &getSwTic(int i){return swTic[i];};
	const double &getSwTic2(int i){return swTic2[i];};
	const double &getStTic(int i){return stTic[i];};
	const double &getSwT(int i){return swT[i];};
	const double &getStT(int i){return stT[i];};
	const double getSwTicRm(int i){return swT[i]-swTic2[i];};//剩余时长
	const double getStTicRm(int i){return stT[i]-stTic[i];};//剩余时长
private:
	cpgClass();
	bool locking=1,shouldStart=0;//shouldStart仅用于在sw期间lock未结束又重新start
	double pgs=0;//~[0,1]
	double pgsEarly=-0.0;//负数(-1,0]，在unlock后等一段时间再动
	double swPgs[Num]{},swPgs2[Num]{},stPgs[Num]={1,1};//swPgs2在sw结束后继续增长直至st结束，可用于触地下探
	double swTic[Num]{},swTic2[Num]{},stTic[Num]{};
	double swT[Num],stT[Num];
	bool isSw[Num]{},isSwAny=0,isFirstStep[Num];
	double dt,period;
	double offset[Num],duty[Num];//~[0,1]
	struct{
		double period,offset[Num],duty[Num];
	}cmd;
	bool calSw(int i);
	bool calSt(int i);
};
}//namespace