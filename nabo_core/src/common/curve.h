/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常用曲线
#include"curve.h"
=====================================================*/
#pragma once
#include"eigen.h"

namespace Crv{
static const int Mem6d=48;
static const int Mem3d=24;


//==1维3次曲线，归一化==
class cubicClass{
public:
	cubicClass(){}
	void reset();
	void setP0(double pp){p0=pp;}
	void setP1(double pp){p1=pp;}
	void setV0(double vv){v0=vv;}
	void setV1(double vv){v1=vv;}
	void update(double pgs);
	double getP(){return p;}
	double getV(){return v;}
private:
	double p0,p1,v0,v1;
	double p,v;
};
//==swing trajectory==
class swTrj6dClass{
public:
	swTrj6dClass(){}
	void setRP0(const mat3d &RR,const vec3d &pp){R0=RR;p0=pp;}
	void setRP0now(){R0=R;p0=p;}
	void setRP1(const mat3d &RR,const vec3d &pp){R1=RR;p1=pp;}//一锤定音
	void setRP1(const mat3d &RR,const vec3d &pp,double pgs);//有权重，适合实时set
	void setH(const double h){this->h=h;}

	void update(const double pgs);
	void adjust(const double z,bool polish=1);
	void adjust(const double x,const double y,const double z,bool polish=1);
	void adjust(const vec3d &xyz,bool polish=1);

	void getR(mat3d &RR){RR=R;}
	void getP(vec3d &pp){pp=p;}
	mat3d &getR(){return R;}
	vec3d &getP(){return p;}
	void getRP(mat3d &RR,vec3d &pp){RR=R;pp=p;}
	void getW(vec3d &ww){ww=w;}
	void getV(vec3d &vv){vv=v;}
	void getWV6d(vec6d &wv){wv=this->wv;}
	void getRP0(mat3d &RR,vec3d &pp){RR=R0;pp=p0;}
	void getRP1(mat3d &RR,vec3d &pp){RR=R1;pp=p1;}
private:
	mat3d R0,R1,R;
	vec3d p0,p1,p,v,w;
	double h=0;
	vec6d wv;
};
}//namespace
