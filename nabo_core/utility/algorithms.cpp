/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常用方法
=====================================================*/
#include"algorithms.h"
namespace Alg{
//==上下限区间，触发过边界return 1====================
	template<typename T1,typename T2,typename T3>
	bool clip(T1&x,const T2&a, const T3&b){
		if(a<=b){
			if(x<=a){x=(T1)a;return 1;}
			else if(x>b){x=(T1)b;return 1;}
		}else{
			if(x<=b){x=(T1)b;return 1;}
			else if(x>a){x=(T1)a;return 1;}
		}
		return 0;
	}
	template bool clip(int&x,const int&a, const int&b);
	template bool clip(int&x,const int&a, const float&b);
	template bool clip(int&x,const int&a, const double&b);
	template bool clip(int&x,const float&a, const int&b);
	template bool clip(int&x,const float&a, const float&b);
	template bool clip(int&x,const float&a, const double&b);
	template bool clip(int&x,const double&a, const int&b);
	template bool clip(int&x,const double&a, const float&b);
	template bool clip(int&x,const double&a, const double&b);
	template bool clip(float&x,const int&a, const int&b);
	template bool clip(float&x,const int&a, const float&b);
	template bool clip(float&x,const int&a, const double&b);
	template bool clip(float&x,const float&a, const int&b);
	template bool clip(float&x,const float&a, const float&b);
	template bool clip(float&x,const float&a, const double&b);
	template bool clip(float&x,const double&a, const int&b);
	template bool clip(float&x,const double&a, const float&b);
	template bool clip(float&x,const double&a, const double&b);
	template bool clip(double&x,const int&a, const int&b);
	template bool clip(double&x,const int&a, const float&b);
	template bool clip(double&x,const int&a, const double&b);
	template bool clip(double&x,const float&a, const int&b);
	template bool clip(double&x,const float&a, const float&b);
	template bool clip(double&x,const float&a, const double&b);
	template bool clip(double&x,const double&a, const int&b);
	template bool clip(double&x,const double&a, const float&b);
	template bool clip(double&x,const double&a, const double&b);

	template<typename T1,typename T2>
	bool clip(T1&x,T2 lim){
		if(lim<0){lim=-lim;}
		if(x<-lim){x=(T1)(-lim);return 1;}
		else if(x>lim){x=(T1)(lim);return 1;}
		return 0;
	}
	template bool clip(int&x, int lim);
	template bool clip(int&x, float lim);
	template bool clip(int&x, double lim);
	template bool clip(float&x, int lim);
	template bool clip(float&x, float lim);
	template bool clip(float&x, double lim);
	template bool clip(double&x, int lim);
	template bool clip(double&x, float lim);
	template bool clip(double&x, double lim);
//==一阶滤波 class============
	filterOneClass::filterOneClass(){
		init(0.001,50);
	}
	filterOneClass::filterOneClass(double dt,double cutF,double ki,double y0){
		init(dt,cutF,ki,y0);
	}
	double filterOneClass::init(double dt,double cutF,double ki,double y0){
		this->dt=dt;
		setCutF(cutF,ki);
		setBase(y0);
		return y;
	}
	double filterOneClass::reset(double cutF,double ki,double y0){
		setCutF(cutF,ki);
		setBase(y0);
		return y;
	}
	double filterOneClass::setBase(double y0){
		y=y0;
		errSum=0;
		return y;
	}
	void filterOneClass::setCutF(double cutF,double ki){
		k=cutF*dt;
		Alg::clip(k,0,1);
		ks=ki*dt;
		clip(ks,0,k/50);
	}
	const double &filterOneClass::filt(const double &x){
		y+=k*(x-y)-errSum;
		errSum+=ks*(y-x);
		return y;
	}
//==二阶低通滤波 class============
	filterTwoClass::filterTwoClass(){
		init(0.001,50);
	}
	filterTwoClass::filterTwoClass(double dt,double cutF,double ki,double y0){
		init(dt,cutF,ki,y0);
	}
	double filterTwoClass::init(double dt,double cutF,double ki,double y0){
		this->dt=dt;
		setCutF(cutF,ki);
		setBase(y0);
		return y;
	}
	double filterTwoClass::reset(double cutF,double ki,double y0){
		setCutF(cutF,ki);
		setBase(y0);
		return y;
	}
	double filterTwoClass::setBase(double y0){
		y=y0;
		yd=0;
		ydd=0;
		errSum=0;
		return y;
	}
	void filterTwoClass::setCutF(double cutF,double ki){
		w=cutF*Pi2;
		Alg::clip(w,0,1/dt/2);
		w1414=1.414*w;
		ww=w*w;
		clip(ki,0,1);
		ks=ki*dt;
	}
	const double &filterTwoClass::filt(const double &x){
		ydd=ww*(x-y)-w1414*yd;
		yd+=ydd*dt;
		y += yd*dt +errSum;
		errSum+=ks*(x-y);
		return y;
	}
	double filterTwoClass::getV(){return yd;}
}//namespace